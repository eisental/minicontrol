extern crate alloc;

use alloc::string::String;
use alloc::vec::Vec;

use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_net::{Stack, dns::DnsQueryType, tcp::TcpSocket};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::{Duration, Ticker, Timer};

use rust_mqtt::client::client::MqttClient;
use rust_mqtt::client::client_config::{ClientConfig, MqttVersion};
use rust_mqtt::packet::v5::publish_packet::QualityOfService;
use rust_mqtt::packet::v5::reason_codes::ReasonCode;
use rust_mqtt::utils::rng_generator::CountingRng;
use static_cell::StaticCell;

use defmt::{error, info};

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
const MAX_SUBSCRIPTIONS: usize = 5;
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
static MQTT_RX_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
static MQTT_TX_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();

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
    let mqtt_rx = MQTT_RX_BUFFER.init_with(|| [0; 1024]);
    let mqtt_tx = MQTT_TX_BUFFER.init_with(|| [0; 1024]);

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

        let mut socket = TcpSocket::new(stack, tcp_rx, tcp_tx);
        socket.set_timeout(Some(Duration::from_secs(10)));

        if socket.connect((ip_address, port)).await.is_ok() {
            info!("TCP Connected! Starting session...");
            socket.set_timeout(None); // MQTT is now responsible for keeping the socket alive
            let _ = mqtt_session(
                socket,
                client_id,
                mqtt_rx,
                mqtt_tx,
                cmd_channel,
                &mut router,
            )
            .await;
        }

        error!("Connection lost. Reconnecting in 5s...");
        Timer::after(Duration::from_secs(5)).await;
    }
}

async fn mqtt_session<'a>(
    socket: TcpSocket<'a>,
    client_id: &'static str,
    mqtt_rx: &mut [u8],
    mqtt_tx: &mut [u8],
    cmd_channel: &'static Channel<NoopRawMutex, MqttCommand, 8>,
    router: &mut Vec<(String, Sender<'static, NoopRawMutex, MqttMessage, 4>)>,
) -> Result<(), ()> {
    let mut config = ClientConfig::new(MqttVersion::MQTTv5, CountingRng(20000));
    config.add_client_id(client_id);
    config.max_packet_size = 1024;
    config.keep_alive = 60;

    let mut client = MqttClient::<_, 5, _>::new(socket, mqtt_rx, 1024, mqtt_tx, 1024, config);
    client.connect_to_broker().await.map_err(|_| ())?;

    // Resubscribe top topics after disconnection
    if !router.is_empty() {
        for (topic, _) in router.iter() {
            info!("Resubscribing to: {}", topic.as_str());
            client.subscribe_to_topic(topic).await.map_err(|_| ())?;
        }
    }

    // need to ping the broker to avoid disconnection
    let mut ping_ticker = Ticker::every(Duration::from_secs(30));

    loop {
        // handle incoming messages, commands from users, and keep-alive pings
        match select3(
            client.receive_message(),
            cmd_channel.receive(),
            ping_ticker.next(),
        )
        .await
        {
            // 1. INCOMING MQTT MESSAGE
            Either3::First(rx_result) => {
                let (topic, payload) = rx_result.map_err(|e| error!("Error 1: {:?}", e))?; // TODO better err handling

                // Route the message to the correct channel
                for (sub_topic, sender) in router.iter() {
                    // Note: This is exact matching. For wildcards (+/#), you'd add logic here.
                    if topic_matches(sub_topic, topic) {
                        let _ = sender.try_send(MqttMessage {
                            topic: String::from(topic),
                            payload: payload.to_vec(),
                        });
                    }
                }
            }

            // 2. INCOMING COMMAND FROM APP
            Either3::Second(cmd) => match cmd {
                MqttCommand::Publish { topic, payload } => {
                    match client
                        .send_message(&topic, &payload, QualityOfService::QoS1, false)
                        .await
                    {
                        Ok(()) => {}
                        Err(ReasonCode::NoMatchingSubscribers) => {
                            info!(
                                "Published to {}, but broker reported no active subscribers.",
                                topic.as_str()
                            );
                        }
                        Err(e) => {
                            error!("failed to publish to {}: {:?}", topic.as_str(), e);
                        }
                    }
                }
                MqttCommand::Subscribe { topic, sender } => {
                    client
                        .subscribe_to_topic(&topic)
                        .await
                        .map_err(|e| error!("Error 3: {:?}", e))?; // TODO err handling
                    router.push((topic, sender));
                }
                MqttCommand::Unsubscribe { topic } => {
                    client
                        .unsubscribe_from_topic(&topic)
                        .await
                        .map_err(|e| error!("Error 4: {:?}", e))?; // TODO err handling
                    router.retain(|(t, _)| t != &topic);
                }
                MqttCommand::UnsubscribeAll => {
                    for (topic, _) in router.iter() {
                        client
                            .unsubscribe_from_topic(topic)
                            .await
                            .map_err(|e| error!("Error 5: {:?}", e))?; // TODO err handling
                    }
                    router.clear();
                }
            },

            // 3. PING TICKER
            Either3::Third(_) => {
                info!("sending ping");
                client
                    .send_ping()
                    .await
                    .map_err(|e| error!("Error 6: {:?}", e))?; // TODO err handling
            }
        }
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
