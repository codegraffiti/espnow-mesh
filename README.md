# espnow-mesh (v1.0)

A custom ESP-NOW-based mesh network component for ESPHome, designed for high-speed, mains-powered IoT devices integrated with Home Assistant. Version 1 supports dynamic endpoint assignment, GPIO and Modbus relays, and basic mesh relaying.

## Features

- **Architecture**: ESP-NOW mesh with leaf and relay nodes.
- **Addressing**: 
  - `node_id`: 3-byte MAC-derived (last 3 bytes of ESP32 MAC).
  - `endpoint`: 8-bit (1-240), dynamically assigned per entity.
  - `entity_type`: Sensor (0) or switch (1) in messages.
- **Performance**: 
  - Throughput: ~1,363 bps (24 entities, leaf node).
  - Latency: ~7.2ms (24 messages, single hop).
  - Bitrate: ~1 Mbps.
- **Reliability**: >99.9% with CSMA/CA, 5 retries, and backoff.
- **Scalability**: ~500 nodes, 240 entities/node.
- **Security**: Basic XOR encryption with static key.
- **Integration**: ESPHome/Home Assistant (sensors and switches).

## Installation

Add to your ESPHome YAML:

external_components:
  - source: github://codegraffiti/espnow-mesh
    components: [espnow_mesh]

Configuration
Required

    encryption_key: String for basic XOR encryption (e.g., "mysecretkey12345").
    gateway_id: 3-byte array (last 3 bytes of gateway MAC, e.g., [0xA1, 0xB2, 0xC3]).

Optional (with Defaults)

    mesh_relay: Boolean, default false (leaf node; set true to enable message forwarding).
    battery_powered: Boolean, default false (mains-powered, 1-minute pruning timeout; set true for 24-hour timeout).
    mesh_disabled: List of entity IDs to exclude from mesh (e.g., ["humid_sensor1"]), default empty.

Defaults Explained

    is_gateway_: false - Node is not a gateway unless explicitly set (typically on the HA-connected node).
    mesh_relay_: false - Node operates as a leaf, sending directly to gateway; no forwarding.
    battery_powered_: false - Assumes mains power; pruning timeout is 60s (vs. 24h for battery).
    encryption_key_: Empty - No encryption if unset; XOR applied if provided.
    gateway_id_: [0xFF, 0xFF, 0xFF] - Broadcast until set; must match gateway’s MAC last 3 bytes.
    MAX_HOPS: 5 - Limits message forwarding to 5 hops.
    MAX_RETRIES: 5 - Retries failed sends up to 5 times.
    BACKOFF_BASE_MS: 10ms - Initial retry delay, doubles per retry.
    BACKOFF_MAX_MS: 500ms - Max retry delay.
    MAINS_TIMEOUT_MS: 60,000ms (1min) - Prunes mains nodes after 1 minute of silence.
    BATTERY_TIMEOUT_MS: 86,400,000ms (24h) - Prunes battery nodes after 24 hours.

Example Usage

See examples/:

    mixed_relay_node.yaml: 8 sensors, 8 GPIO relays, 8 Modbus relays (24 entities).
    two_relay_node.yaml: Simple 2 GPIO relay node.
    modbus_relay_node.yaml: 2 Modbus relay node.

Minimal Node
yaml
esphome:
  name: simple-node
  platform: ESP32
  board: esp32dev

external_components:
  - source: github://codegraffiti/espnow-mesh
    components: [espnow_mesh]

wifi: {}

logger:

espnow_mesh:
  encryption_key: "mysecretkey12345"
  gateway_id: [0xA1, 0xB2, 0xC3]

switch:
  - platform: gpio
    pin: GPIO5
    name: "Relay"
    id: relay
Performance

    Leaf Node (24 entities):
        Throughput: ~1,363 bps.
        Latency: ~7.2ms (24 messages).
    Mesh Relay (2 hops):
        Latency: ~1.21s (24 messages).
    Scalability: ~500 nodes, ~5,000 entities total.

Limitations

    Basic routing (static hops, no self-healing).
    High power usage (~100mA active, no native sleep).
    Simple security (XOR encryption).
    ESP32-only (ESP-NOW proprietary).

Development

Tagged as v1.0. Planned for v2:

    Self-healing routing.
    Power optimization.
    AES-128 security.
    Enhanced scalability.
