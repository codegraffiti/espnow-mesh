#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <vector>
#include <map>
#include <set>
#include <array>

namespace esphome {
namespace espnow_mesh {

struct RouteEntry {
  uint8_t next_hop[6];  // MAC address of next hop
  uint8_t hops;         // Number of hops to gateway
  bool peer_added;      // Whether peer is added to ESP-NOW
  uint32_t last_seen;   // Last update time (ms)
  uint8_t node_type;    // 0 = mains, 1 = battery
};

struct MeshMessage {
  uint8_t node_id[3];   // 3-byte MAC-derived node ID
  uint8_t msg_type;     // 0 = JOIN, 1 = DATA, 2 = CONTROL
  uint8_t hops;         // Number of hops traversed
  uint8_t node_type;    // 0 = mains, 1 = battery
  uint8_t entity_type;  // 0 = sensor, 1 = switch
  uint8_t endpoint;     // Unique entity ID (1-240)
  uint8_t data_len;     // Length of data payload
  uint8_t data[16];     // Payload (e.g., float for sensor, bool for switch)
};

class ESPNowMesh : public Component {
 public:
  enum EntityType {
    ENTITY_SENSOR = 0,
    ENTITY_SWITCH = 1,
  };

  void setup() override;
  void loop() override;
  void send_message(const MeshMessage &msg);
  void send_data(uint8_t endpoint, const std::vector<float> &values);
  void send_switch_state(uint8_t endpoint, bool state);
  std::string get_last_message() const { return last_message_; }
  void set_is_gateway(bool is_gateway) { is_gateway_ = is_gateway; }
  void set_mesh_relay(bool relay) { mesh_relay_ = relay; }
  void set_encryption_key(const std::string &key) { encryption_key_ = key; }
  void set_battery_powered(bool powered) { battery_powered_ = powered; }
  void set_gateway_id(const std::vector<uint8_t> &id) { memcpy(gateway_id_.data(), id.data(), 3); }
  void write_switch_state(bool state, uint8_t endpoint);
  void register_node_sensor(const std::string &node_id, uint8_t endpoint, const std::string &unit, const std::string &device_class);
  void register_node_switch(const std::string &node_id, uint8_t endpoint);
  void send_join_request();
  void prune_stale_nodes();
  void set_on_message_received(std::function<void()> &&callback) { on_message_received_ = std::move(callback); }
  void set_on_message_sent(std::function<void()> &&callback) { on_message_sent_ = std::move(callback); }
  void set_mesh_disabled(const std::vector<std::string> &disabled) { 
    for (const auto &id : disabled) mesh_disabled_.insert(id); 
  }

 protected:
  void on_recv_callback_(const uint8_t *mac_addr, const uint8_t *data, int len);
  void on_sent_callback_(const uint8_t *mac_addr, esp_now_send_status_t status);
  void encrypt_data_(uint8_t *data, size_t len);
  void decrypt_data_(uint8_t *data, size_t len);
  void update_routing_table_(const uint8_t *source_id, const uint8_t *source_mac, uint8_t hops);
  bool forward_message_(const MeshMessage &msg);
  void process_switch_command_(const MeshMessage &msg);
  bool add_peer_if_needed_(const uint8_t *mac_addr);
  uint32_t calculate_backoff_(uint8_t retry_count);
  bool is_channel_busy_();

  // Default values explained:
  bool is_gateway_{false};         // False: Node is not a gateway unless specified
  bool mesh_relay_{false};         // False: Node is a leaf (no forwarding) unless set
  std::string node_id_str_;        // String representation of MAC for logging
  uint8_t node_id_[3];             // 3-byte MAC-derived ID, set in setup()
  std::string encryption_key_;     // Empty: No encryption unless set in YAML
  std::string last_message_;       // Empty: Last received message buffer
  std::map<std::array<uint8_t, 3>, RouteEntry> routing_table_;  // Empty: Builds dynamically
  static const uint8_t MAX_HOPS = 5;      // 5: Max hops before dropping message
  static const uint8_t MAX_RETRIES = 5;   // 5: Max send retries
  static const uint32_t BACKOFF_BASE_MS = 10;  // 10ms: Base retry delay
  static const uint32_t BACKOFF_MAX_MS = 500;  // 500ms: Max retry delay
  static const uint32_t MAINS_TIMEOUT_MS = 60000;  // 1min: Prune mains nodes
  static const uint32_t BATTERY_TIMEOUT_MS = 86400000;  // 24h: Prune battery nodes
  bool awaiting_ack_{false};       // False: No ACK pending unless sending
  std::vector<uint8_t> switch_pins_;  // Empty: Populated from GPIO switches
  volatile uint32_t channel_activity_count_{0};  // 0: Activity counter for CSMA/CA
  bool battery_powered_{false};    // False: Mains-powered unless set
  std::array<uint8_t, 3> gateway_id_{0xFF, 0xFF, 0xFF};  // FF:FF:FF: Broadcast until set
  std::set<std::string> mesh_disabled_;  // Empty: No entities disabled unless set
  std::map<std::string, sensor::Sensor *> dynamic_sensors_;  // Empty: Gateway HA sensors
  std::map<std::string, switch_::Switch *> dynamic_switches_;  // Empty: Gateway HA switches
  std::function<void()> on_message_received_;  // Null: Optional callback
  std::function<void()> on_message_sent_;      // Null: Optional callback
  std::vector<sensor::Sensor *> sensors_;      // Empty: Node sensors
  std::vector<switch_::Switch *> switches_;    // Empty: Node switches
};

}  // namespace espnow_mesh
}  // namespace esphome
