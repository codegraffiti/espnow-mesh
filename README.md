# espnow-mesh
GrokAI generated espnow-mesh for esphome
# ESP-NOW Mesh for ESPHome

A custom ESPHome component implementing an ESP-NOW-based mesh network with self-healing, multi-hop routing, and switch control. Designed for mains-powered gateways/relays and battery-powered sensor nodes, with countermeasures for high traffic (e.g., 10 nodes at 1 Hz).

## Features
- **Self-Healing**: Dynamic routing table updates and route rediscovery.
- **Multi-Hop**: Up to 5 hops between nodes.
- **Switch Control**: Mains-powered nodes (gateway/relays) can control switches via Home Assistant.
- **Collision Avoidance**: Jitter, ACKs, backoff, and traffic shaping for reliable high-frequency messaging.
- **Encryption**: Simple XOR (extendable to AES).

## Installation
Add this component to your ESPHome YAML:

```yaml
external_components:
  - source: github://codegraffiti/espnow-mesh
    components: [espnow_mesh]
