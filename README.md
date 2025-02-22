# espnow-mesh (v2.0)

A Zigbee-like ESP-NOW mesh network component for ESPHome, designed for ESP32 in no-Wi-Fi mode. Supports Coordinator, Router, and End Device roles with AODV-like routing, RSSI-based optimization, and automatic HA integration.

## Features

- **Architecture**: ESP-NOW mesh with Coordinator, Router, End Device roles; self-healing routing.
- **Addressing**: 
  - `node_id`: 3-byte MAC-derived.
  - `short_addr`: 16-bit, assigned by Coordinator.
  - `endpoint`: 8-bit (1-240).
- **Performance**: 
  - Throughput: ~1,363 bps (24 entities, leaf).
  - Latency: ~150-200ms (2 hops), ~3-4s failover.
  - Bitrate: ~1 Mbps.
- **Reliability**: >99% (retries, failover).
- **Power**: Sleep for End Devices (~10µA idle).
- **Security**: AES-128 CCM with Network Key.
- **Scalability**: ~1,000 nodes, 240 entities/node.
- **HA Integration**: Auto RSSI sensors (`sensor.<node_id>_rssi`), no YAML required.

## Installation

```yaml
external_components:
  - source: github://codegraffiti/espnow-mesh@v2
    components: [espnow_mesh]
```
Configuration
Required

    None (Coordinator auto-generates Network Key).

Optional (Defaults)

    role: "end_device" - Options: coordinator, router, end_device.
    mesh_disabled: [] - List of entity IDs to exclude from mesh updates.

Defaults Explained

    role_: ROLE_END_DEVICE (2) - Leaf node unless set.
    network_key_: Random 16-byte key generated by Coordinator on boot.
    MAX_HOPS: 5 - Max hops for routing.
    MAX_RETRIES: 5 - Send retries (~1-2s total).
    BACKOFF_BASE_MS: 10ms - Initial retry delay.
    BACKOFF_MAX_MS: 500ms - Max retry delay.
    MAINS_TIMEOUT_MS: 60,000ms - Router pruning timeout.
    BATTERY_TIMEOUT_MS: 86,400,000ms - End Device pruning timeout (24h).
    ROUTE_EXPIRY_MS: 60,000ms - Route validity (60s).
    PERMIT_JOIN_TIMEOUT_MS: 120,000ms - Join window (2min).
    CHANNEL_SWITCH_TIMEOUT_MS: 5,000ms - Channel switch timeout (10s max with retries).
    COORDINATOR_FAILOVER_TIMEOUT_MS: 30,000ms - Router failover timeout (30s).
    ROUTE_UPDATE_INTERVAL_MS: 15,000ms - Route update interval (15s).
    RSSI_UPDATE_MIN_INTERVAL_MS: 60,000ms - RSSI update interval (60s).
    CHILD_RETRY_INTERVAL_MS: 1,000ms - Child retry interval (1s).
    JOIN_TOTAL_TIMEOUT_MS: 8,000ms - Total join timeout (8s).
    SCAN_CHANNEL_TIMEOUT_MS: 1,000ms - Per-channel scan timeout (1s).
    SCAN_RETRY_INTERVAL_MS: 10,000ms - Scan retry interval (10s).
    RSSI_CHANGE_THRESHOLD: 5 - RSSI change threshold (±5 dBm).
    mesh_disabled_: [] - All entities participate unless disabled.
    permit_join_: false - Joining disabled unless toggled via HA.

Example Usage

See examples/ directory for complete configurations:

    end_node_5_sensors.yaml: End Device with 5 sensors.
    router_node_2gpio_2modbus_3sensors.yaml: Router with 2 GPIO switches, 2 Modbus switches, 3 sensors.
    coordinator_node.yaml: Coordinator with detailed comments.

End Node with 5 Sensors
```
esphome:
  name: end-node-5-sensors
  platform: ESP32
  board: esp32dev

external_components:
  - source: github://codegraffiti/espnow-mesh@v2
    components: [espnow_mesh]

logger:

espnow_mesh:
  role: end_device

sensor:
  - platform: dht
    pin: GPIO4
    model: DHT22
    temperature:
      name: "Temperature"
    humidity:
      name: "Humidity"
    update_interval: 60s
  - platform: adc
    pin: GPIO33
    name: "Light Level"
    update_interval: 60s
  - platform: sht3xd
    address: 0x44
    temperature:
      name: "Outdoor Temp"
    humidity:
      name: "Outdoor Humidity"
    update_interval: 60s

deep_sleep:
  run_duration: 10s
  sleep_duration: 60s
```
Router Node with 2 GPIO Switches, 2 Modbus Switches, 3 Sensors

```
esphome:
  name: router-node
  platform: ESP32
  board: esp32dev

external_components:
  - source: github://codegraffiti/espnow-mesh@v2
    components: [espnow_mesh]

logger:

espnow_mesh:
  role: router

switch:
  - platform: gpio
    pin: GPIO5
    name: "Relay 1"
  - platform: gpio
    pin: GPIO18
    name: "Relay 2"
  - platform: modbus
    id: modbus_switch1
    register_type: coil
    address: 1
    name: "Modbus Switch 1"
  - platform: modbus
    id: modbus_switch2
    register_type: coil
    address: 2
    name: "Modbus Switch 2"

sensor:
  - platform: dht
    pin: GPIO4
    model: DHT22
    temperature:
      name: "Temperature"
    humidity:
      name: "Humidity"
    update_interval: 60s
  - platform: adc
    pin: GPIO33
    name: "Light Level"
    update_interval: 60s

modbus:
  id: modbus1
  uart_num: 1
  tx_pin: GPIO17
  rx_pin: GPIO16
```
Coordinator Node

```
# coordinator_node.yaml - Example Coordinator configuration for espnow-mesh
# This node manages the network, assigns addresses, and integrates with HA.

esphome:
  name: coordinator-node  # Unique name for the node
  platform: ESP32         # Target platform (ESP32-only)
  board: esp32dev         # Specific board type

# Include espnow-mesh component from GitHub release v2
external_components:
  - source: github://codegraffiti/espnow-mesh@v2
    components: [espnow_mesh]

# Enable logging for debugging (visible in HA or serial)
logger:

# Define espnow-mesh configuration
espnow_mesh:
  role: coordinator  # Set role to Coordinator (default: end_device)
  # No mesh_disabled specified, all entities participate by default

# Notes:
# - The Coordinator auto-generates a 16-byte AES-128 CCM network key on boot.
# - It creates two HA switches: "Permit Join" (2min timeout) and "Channel Switch".
# - RSSI is automatically exposed as sensor.<node_id>_rssi (e.g., sensor.coordinator_node_rssi) without additional configuration.
# - No sensors/switches defined here; the Coordinator manages network traffic and exposes remote devices (e.g., from End Devices/Routers).
```

HA Controlled Joining

    Switch: switch.<coordinator_id>_permit_join in HA.
    Behavior:
        Default off—new devices can’t join; known devices reconnect.
        Toggle on—2-minute join window, auto-disables.

Channel Management

    No Wi-Fi Mode:
        Starts on channel 1, scans 1-13 (~3-4s with Router beacons).
        HA triggers switches via switch.<coordinator_id>_channel_switch (~10s max with retries).
    Monitoring: Auto RSSI sensors (sensor.<node_id>_rssi) update on ±5 dBm change or topology shift (~60s minimum).

Performance

    Leaf (24 entities): ~1,363 bps, ~150-200ms (2 hops).
    Failover: ~4s (Router offline to new path).
    Scalability: ~1,000 nodes, ~5,000 entities.

Changes in v2.0

    Routing: RSSI-optimized AODV-like routing.
    Joining: Router-assisted, permit_join required for new devices.
    Failover: Router promotion (30s), multi-Router support.
    RSSI: Auto HA sensors, piggybacked on data (~0.08% airtime).

Limitations

    ESP32-only.
    No Coordinator failover beyond Router promotion.
    RSSI requires promiscuous mode workaround.
