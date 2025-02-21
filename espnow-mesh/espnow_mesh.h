#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/switch/switch.h"
#include <esp_now.h>
#include <WiFi.h>
#include <vector>
#include <map>

namespace esphome {
namespace espnow_mesh {

struct RouteEntry {
  uint8_t next_hop[6];  // MAC address of the next hop toward the destination
  uint8_t hops;         // Number of hops to reach the destination
  bool peer_added;      // Flag indicating if this MAC is registered as an ESP-NOW peer
};

class ESPNowMesh : public Component {
 public:
  void setup() override;  // Initialize ESP-NOW and hardware
  void loop() override;   // Handle ongoing tasks (currently empty)
  void send_message(const std::string &data);  // Send a message through the mesh
  std::string get_last_message() const { return last_message_; }  // Retrieve the last received message
  void set_is_gateway(bool is_gateway) { is_gateway_ = is_gateway; }  // Mark node as gateway
  void set_is_relay(bool is_relay) { is_relay_ = is_relay; }          // Mark node as relay
  void set_encryption_key(const std::string &key) { encryption_key_ = key; }  // Set encryption key
  void set_node_id(const std::string &id) { node_id_ = id; }          // Set unique node identifier
  void set_switch_pin(uint8_t pin) { switch_pin_ = pin; }  // Set GPIO pin for switch
  void set_switch(switch_::Switch *sw) { switch_ = sw; }   // Link to ESPHome switch entity
  void write_switch_state(bool state);                     // Update switch state and notify

 protected:
  void on_recv_callback_(const uint8_t *mac_addr, const uint8_t *data, int len);  // Callback for received messages
  void on_sent_callback_(const uint8_t *mac_addr, esp_now_send_status_t status);  // Callback for sent messages
  void encrypt_data_(std::string &data);  // Encrypt data using XOR
  void decrypt_data_(std::string &data);  // Decrypt data
  void update_routing_table_(const std::string &source_id, const uint8_t *source_mac, uint8_t hops);  // Update routing
  bool forward_message_(const std::string &message, const std::string &dest_id);  // Forward message to next hop
  void process_switch_command_(const std::string &payload);  // Process switch commands
  bool add_peer_if_needed_(const uint8_t *mac_addr);  // Add ESP-NOW peer
  uint32_t calculate_backoff_(uint8_t retry_count);   // Calculate backoff delay

  bool is_gateway_{false};          // True if this is the gateway node
  bool is_relay_{false};            // True if this is a relay node
  std::string node_id_;             // Unique identifier (e.g., "batt1")
  std::string encryption_key_;      // Key for XOR encryption
  std::string last_message_;        // Last received message
  std::map<std::string, RouteEntry> routing_table_;  // Routing table
  static const uint8_t MAX_HOPS = 5;                 // Max hops allowed
  static const uint8_t MAX_RETRIES = 3;              // Max retransmissions
  static const uint32_t BACKOFF_BASE_MS = 10;        // Base backoff time (ms)
  bool awaiting_ack_{false};                         // Track ACK status
  uint8_t switch_pin_{255};                          // GPIO for switch
  switch_::Switch *switch_{nullptr};                 // ESPHome switch object
};

}  // namespace espnow_mesh
}  // namespace esphome
