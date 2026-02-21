extern crate alloc;

use alloc::string::String;
use alloc::vec::Vec;

use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_net::{Stack, dns::DnsQueryType, tcp::TcpSocket};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::{Duration, Ticker, Timer};

use rust_mqtt::Bytes;
use rust_mqtt::buffer::BumpBuffer;
use rust_mqtt::client::event::Event;
use rust_mqtt::client::options::SubscriptionOptions;
use rust_mqtt::client::{Client, MqttError};
use rust_mqtt::types::{MqttString, TooLargeToEncode, TopicFilter, TopicName};
use static_cell::StaticCell;

use defmt::{error, info};

const MAX_SUBSCRIPTIONS: usize = 5;
const RECEIVE_MAXIMUM: usize = 1;
const SEND_MAXIMUM: usize = 1;

type MqttClient<'a, W, B> = Client<'a, W, B, MAX_SUBSCRIPTIONS, RECEIVE_MAXIMUM, SEND_MAXIMUM>;

#[derive(Clone)]
pub struct MqttMessage {
    pub topic: String,
    pub payload: Vec<u8>,
}

pub enum MqttCommand {
    Publish {
        topic: String,
        payload: Vec<u8>,
    },
    Subscribe {
        topic: String,
        sender: Sender<'static, NoopRawMutex, MqttMessage, 4>,
    },
    Unsubscribe {
        topic: String,
    },
    UnsubscribeAll,
}

static CMD_CHANNEL: StaticCell<Channel<NoopRawMutex, MqttCommand, 8>> = StaticCell::new();

// A pool of StaticCells to allocate subscription channels dynamically but with 'static lifetimes
static SUB_CHANNELS_POOL: [StaticCell<Channel<NoopRawMutex, MqttMessage, 4>>; MAX_SUBSCRIPTIONS] = [
    StaticCell::new(),
    StaticCell::new(),
    StaticCell::new(),
    StaticCell::new(),
    StaticCell::new(),
];

// A simple bump allocator to grab the next available channel
static mut SUB_CHANNEL_INDEX: usize = 0;

static TCP_RX_BUFFER: StaticCell<[u8; 4096]> = StaticCell::new();
static TCP_TX_BUFFER: StaticCell<[u8; 4096]> = StaticCell::new();
static MQTT_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();

pub struct MQTT {
    cmd_sender: Sender<'static, NoopRawMutex, MqttCommand, 8>,
}

impl MQTT {
    pub fn new(
        spawner: Spawner,
        stack: Stack<'static>,
        host: &'static str,
        port: u16,
        client_id: &'static str,
    ) -> Self {
        let cmd_channel: &mut Channel<NoopRawMutex, MqttCommand, 8> =
            CMD_CHANNEL.init_with(Channel::new);

        spawner
            .spawn(mqtt_background_task(
                stack,
                host,
                port,
                client_id,
                cmd_channel,
            ))
            .unwrap();

        Self {
            cmd_sender: cmd_channel.sender(),
        }
    }

    pub async fn publish(&self, topic: &str, payload: &[u8]) {
        self.cmd_sender
            .send(MqttCommand::Publish {
                topic: String::from(topic),
                payload: payload.to_vec(),
            })
            .await;
    }

    pub async fn subscribe(&self, topic: &str) -> Receiver<'static, NoopRawMutex, MqttMessage, 4> {
        // Safely allocate a new 'static channel from the pool
        let index = unsafe {
            // TODO: Why Unsafe?
            let i = SUB_CHANNEL_INDEX;
            SUB_CHANNEL_INDEX += 1;
            i
        };
        assert!(index < MAX_SUBSCRIPTIONS, "Out of subscription channels!"); // TODO: better error handling

        let channel = SUB_CHANNELS_POOL[index].init(Channel::new());
        let sender = channel.sender();
        let receiver = channel.receiver();

        self.cmd_sender
            .send(MqttCommand::Subscribe {
                topic: String::from(topic),
                sender,
            })
            .await;

        receiver
    }

    pub async fn unsubscribe(&self, topic: &str) {
        self.cmd_sender
            .send(MqttCommand::Unsubscribe {
                topic: String::from(topic),
            })
            .await;
    }

    pub async fn unsubscribe_all(&self) {
        self.cmd_sender.send(MqttCommand::UnsubscribeAll).await;
    }
}

#[embassy_executor::task]
async fn mqtt_background_task(
    stack: Stack<'static>,
    host: &'static str,
    port: u16,
    client_id: &'static str,
    cmd_channel: &'static Channel<NoopRawMutex, MqttCommand, 8>,
) {
    let tcp_rx = TCP_RX_BUFFER.init_with(|| [0; 4096]);
    let tcp_tx = TCP_TX_BUFFER.init_with(|| [0; 4096]);
    let mqtt_buf = MQTT_BUFFER.init_with(|| [0; 1024]);

    // A local vector to hold active subscriptions and their routing channels
    let mut router: Vec<(String, Sender<'static, NoopRawMutex, MqttMessage, 4>)> = Vec::new();

    loop {
        while !stack.is_link_up() {
            Timer::after(Duration::from_millis(500)).await;
        }

        info!("Resolving IP for broker...");
        let ip_address = match stack.dns_query(host, DnsQueryType::A).await {
            Ok(results) if !results.is_empty() => results[0],
            _ => {
                error!("DNS resolution failed");
                continue;
            }
        };

        info!("Resolved {} to {:?}", host, ip_address);

        let mut mqtt_buf = BumpBuffer::new(mqtt_buf);
        let mut socket = TcpSocket::new(stack, tcp_rx, tcp_tx);
        socket.set_timeout(Some(Duration::from_secs(10)));

        if socket.connect((ip_address, port)).await.is_ok() {
            info!("TCP Connected! Starting session...");
            socket.set_timeout(None); // MQTT is now responsible for keeping the socket alive
            let _ = mqtt_session(
                &mut socket,
                &mut mqtt_buf,
                client_id,
                cmd_channel,
                &mut router,
            )
            .await;
        }

        error!("Connection lost. Reconnecting in 5s...");
        Timer::after(Duration::from_secs(5)).await;
    }
}

#[derive(defmt::Format)]
enum PubSubError<'s> {
    InvalidTopic(TooLargeToEncode),
    Mqtt(MqttError<'s>),
}

impl<'s> From<MqttError<'s>> for PubSubError<'s> {
    fn from(error: MqttError<'s>) -> Self {
        PubSubError::Mqtt(error)
    }
}

impl<'s> From<TooLargeToEncode> for PubSubError<'s> {
    fn from(error: TooLargeToEncode) -> Self {
        PubSubError::InvalidTopic(error)
    }
}

// TODO externalize subscription options
async fn mqtt_subscribe<'a>(
    client: &mut MqttClient<'a, &mut TcpSocket<'a>, BumpBuffer<'a>>,
    topic: &str,
) -> Result<(), PubSubError<'a>> {
    let topic = MqttString::try_from(topic)?;
    let topic = unsafe { TopicFilter::new_unchecked(topic) };
    client
        .subscribe(
            topic,
            SubscriptionOptions {
                retain_as_published: false, //?
                retain_handling: rust_mqtt::client::options::RetainHandling::AlwaysSend, //?
                no_local: false,
                qos: rust_mqtt::types::QoS::AtLeastOnce,
            },
        )
        .await
        .map_err(|e| PubSubError::Mqtt(e.clone()))?;

    // TODO: poll for ack
    Ok(())
}

async fn mqtt_unsubscribe<'a>(
    client: &mut MqttClient<'a, &mut TcpSocket<'a>, BumpBuffer<'a>>,
    topic: &str,
) -> Result<(), PubSubError<'a>> {
    let topic = MqttString::try_from(topic)?;
    let topic = unsafe { TopicFilter::new_unchecked(topic) };
    client.unsubscribe(topic).await?;

    // TODO: poll for ack
    Ok(())
}

async fn mqtt_publish<'a>(
    client: &mut MqttClient<'a, &mut TcpSocket<'a>, BumpBuffer<'a>>,
    topic: &str,
    payload: Vec<u8>,
) -> Result<(), PubSubError<'a>> {
    let topic = MqttString::try_from(topic)?;
    let topic = unsafe { TopicName::new_unchecked(topic) };
    let payload = Bytes::from(payload.as_slice());
    // TODO: wait for ack
    let _ = client
        .publish(
            &rust_mqtt::client::options::PublicationOptions {
                retain: false,
                topic,
                qos: rust_mqtt::types::QoS::AtLeastOnce,
            },
            payload,
        )
        .await?;

    Ok(())
}

async fn mqtt_session<'a>(
    socket: &mut TcpSocket<'a>,
    mqtt_buffer: &'a mut BumpBuffer<'a>,
    client_id: &'static str,
    cmd_channel: &'static Channel<NoopRawMutex, MqttCommand, 8>,
    router: &mut Vec<(String, Sender<'static, NoopRawMutex, MqttMessage, 4>)>,
) -> Result<(), ()> {
    let mut client = MqttClient::new(mqtt_buffer);
    let options = rust_mqtt::client::options::ConnectOptions {
        clean_start: true,
        keep_alive: rust_mqtt::config::KeepAlive::Seconds(60),
        user_name: None,
        password: None,
        session_expiry_interval: rust_mqtt::config::SessionExpiryInterval::EndOnDisconnect,
        will: None,
    };
    client
        .connect(socket, &options, MqttString::try_from(client_id).ok())
        .await
        .inspect_err(|e| error!("Error connecting to MQTT broker: {:?}", e))
        .map_err(|_| ())?;

    // Resubscribe top topics after disconnection
    if !router.is_empty() {
        for (topic, _) in router.iter() {
            match mqtt_subscribe(&mut client, topic.as_str()).await {
                Ok(()) => info!("Resubscribed to {}", topic.as_str()),
                Err(e) => error!("Error resubscribing to {}: {:?}", topic.as_str(), e),
            }
        }
    }

    // need to ping the broker to avoid disconnection
    let mut ping_ticker = Ticker::every(Duration::from_secs(30));

    loop {
        // handle incoming messages, commands from users, and keep-alive pings
        match select3(
            client.poll_header(),
            cmd_channel.receive(),
            ping_ticker.next(),
        )
        .await
        {
            // 1. INCOMING MQTT MESSAGE
            Either3::First(event) => match event {
                Ok(header) => {
                    match client.poll_body(header).await {
                        Ok(Event::Publish(p)) => {
                            // Route the message to the correct channel
                            for (sub_topic, sender) in router.iter() {
                                let topic = p.topic.as_ref();
                                // TODO: possibly use rust-mqtt topic matching logic
                                if topic_matches(sub_topic, topic) {
                                    let _ = sender.try_send(MqttMessage {
                                        topic: String::from(p.topic.as_ref()),
                                        payload: p.message.to_vec(),
                                    });
                                }
                            }
                        }
                        Ok(event) => {
                            info!("Received event: {:?}", event);
                        }
                        Err(e) => {
                            error!("Error polling body: {:?}", e);
                            break Err(()); // TODO maybe not break
                        }
                    }
                }
                Err(e) => {
                    error!("Error polling body: {:?}", e);
                    break Err(()); // TODO maybe not break
                }
            },

            // 2. INCOMING COMMAND FROM APP
            Either3::Second(cmd) => match cmd {
                MqttCommand::Publish { topic, payload } => {
                    let _ = mqtt_publish(&mut client, topic.as_str(), payload)
                        .await
                        .inspect_err(|e| error!("Error publishing to {}: {:?}", topic.as_str(), e));
                }
                MqttCommand::Subscribe { topic, sender } => {
                    let _ = mqtt_subscribe(&mut client, topic.as_str())
                        .await
                        .inspect_err(|e| {
                            error!("Error subscribing to {}: {:?}", topic.as_str(), e)
                        });
                    router.push((topic, sender));
                }
                MqttCommand::Unsubscribe { topic } => {
                    let _ = mqtt_unsubscribe(&mut client, topic.as_str())
                        .await
                        .inspect_err(|e| {
                            error!("Error unsubscribing from {}: {:?}", topic.as_str(), e)
                        });
                    router.retain(|(t, _)| t != &topic);
                }
                MqttCommand::UnsubscribeAll => {
                    for (topic, _) in router.iter() {
                        mqtt_unsubscribe(&mut client, topic.as_str())
                            .await
                            .map_err(|e| {
                                error!("Error unsubscribing from {}: {:?}", topic.as_str(), e)
                            })?; // TODO log success?
                    }
                    router.clear();
                }
            },

            // 3. PING TICKER
            Either3::Third(_) => {
                info!("sending ping");
                let _ = client
                    .ping()
                    .await
                    .inspect_err(|e| error!("Error sending ping: {:?}", e));
            }
        }

        unsafe { client.buffer().reset() };
    }
}

// Checks if an incoming MQTT topic matches a subscription filter (which may contain wildcards)
fn topic_matches(filter: &str, topic: &str) -> bool {
    let mut f_iter = filter.split('/');
    let mut t_iter = topic.split('/');

    loop {
        match (f_iter.next(), t_iter.next()) {
            // '#' matches everything that follows, so it's an instant match
            (Some("#"), _) => return true,

            // '+' matches exactly one level, so as long as there is a level here, we continue
            (Some("+"), Some(_)) => continue,

            // Exact string match for this specific level
            (Some(f), Some(t)) if f == t => continue,

            // Both strings ended at the exact same time without mismatches
            (None, None) => return true,

            // Anything else (mismatched words, or different lengths) is a failure
            _ => return false,
        }
    }
}
