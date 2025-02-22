// espnow_mesh.h - Header file for ESP-NOW mesh network component (Version 2.0)
// Provides Zigbee-like mesh functionality for ESP32, optimized for no-Wi-Fi mode.

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
#include <mbedtls/ccm.h>

namespace esphome {
namespace espnow_mesh {

struct RouteEntry {
  uint8_t next_hop[6];       // MAC address of next hop (6 bytes)
  uint16_t short_addr;       // 16-bit short address of destination
  uint8_t hops;              // Number of hops to destination
  uint32_t cost;             // Routing cost (hops * 100 - RSSI + retries * 50)
  bool peer_added;           // Flag if peer added to ESP-NOW
  uint32_t last_seen;        // Last time route was updated (ms)
  uint32_t expires_at;       // Route expiry time (ms)
  bool is_router;            // True if destination is a Router
  uint8_t retry_count;       // Number of failed send attempts
  int8_t rssi;               // Received Signal Strength Indicator (dBm)
};

struct MeshMessage {
  uint8_t src_node_id[3];    // Source node ID (3-byte MAC-derived)
  uint8_t dest_node_id[3];   // Destination node ID (3-byte MAC-derived)
  uint8_t msg_type;          // Message type (e.g., MSG_DATA)
  uint8_t hops;              // Current hop count
  uint8_t role;              // Node role (e.g., ROLE_END_DEVICE)
  uint8_t entity_type;       // Entity type (e.g., ENTITY_SENSOR)
  uint8_t endpoint;          // Endpoint ID (1-240)
  uint8_t data_len;          // Length of data payload (bytes)
  uint8_t data[24];          // Data payload (max 24 bytes)
  static const uint8_t MAX_DATA_LEN = 23;  // Max payload (23 bytes + 1 for RSSI)
};

class ESPNowMesh : public Component {
 public:
  enum RoleType {
    ROLE_COORDINATOR = 0,    // Network coordinator
    ROLE_ROUTER = 1,         // Routing node
    ROLE_END_DEVICE = 2,     // Leaf node (e.g., sensors)
  };
  enum EntityType {
    ENTITY_SENSOR = 0,       // Sensor entity
    ENTITY_SWITCH = 1,       // Switch entity
  };
  enum MessageType {
    MSG_JOIN = 0,            // Join request
    MSG_DATA = 1,            // Data message
    MSG_CONTROL = 2,         // Control message (e.g., permit_join toggle)
    MSG_RREQ = 3,            // Route request
    MSG_RREP = 4,            // Route reply
    MSG_KEY = 5,             // Network key exchange
    MSG_CHANNEL = 6,         // Channel switch command
    MSG_CHANNEL_ACK = 7,     // Channel switch acknowledgment
  };

  // Public methods for component lifecycle and user interaction
  void setup() override;     // Initialize node (e.g., ESP-NOW, RSSI sensor)
  void loop() override;      // Main loop for timeouts, retries, updates
  void send_message(const MeshMessage &msg);  // Send ESP-NOW message
  void send_data(uint8_t endpoint, const std::vector<float> &values);  // Send sensor data
  void send_switch_state(uint8_t endpoint, bool state);  // Send switch state
  std::string get_last_message() const { return last_message_; }  // Get last received message
  void set_role(uint8_t role) { role_ = role; }  // Set node role
  void set_network_key(const std::string &key) {  // Set encryption key
    network_key_ = key;
    if (key.length() < 16) network_key_.resize(16, 0);
  }
  void write_switch_state(bool state, uint8_t endpoint);  // Write switch state locally
  void register_node_sensor(const std::string &node_id, uint8_t endpoint, const std::string &unit, const std::string &device_class);  // Register remote sensor
  void register_node_switch(const std::string &node_id, uint8_t endpoint);  // Register remote switch
  void send_join_request();  // Request network join
  void send_route_request(const std::array<uint8_t, 3> &dest_id);  // Request route to destination
  void send_channel_switch(uint8_t new_channel);  // Initiate channel switch
  void send_channel_ack(uint8_t channel);  // Acknowledge channel switch
  void prune_stale_routes();  // Remove expired routes
  void set_on_message_received(std::function<void()> &&callback) { on_message_received_ = std::move(callback); }  // Callback for received messages
  void set_on_message_sent(std::function<void()> &&callback) { on_message_sent_ = std::move(callback); }  // Callback for sent messages
  void set_mesh_disabled(const std::vector<std::string> &disabled) {  // Disable specific entities from mesh
    for (const auto &id : disabled) mesh_disabled_.insert(id);
  }

 protected:
  // Protected methods for internal operations
  void on_recv_callback_(const uint8_t *mac_addr, const uint8_t *data, int len, int8_t rssi);  // Handle received ESP-NOW packets
  void on_sent_callback_(const uint8_t *mac_addr, esp_now_send_status_t status);  // Handle send status
  void encrypt_data_(uint8_t *data, size_t len, uint8_t *nonce);  // Encrypt message data
  void decrypt_data_(uint8_t *data, size_t len, uint8_t *nonce);  // Decrypt message data
  void update_routing_table_(const uint8_t *source_id, const uint8_t *source_mac, uint8_t hops, uint8_t role);  // Update routing table
  bool forward_message_(const MeshMessage &msg);  // Forward message to next hop
  void process_switch_command_(const MeshMessage &msg);  // Process switch commands
  void process_route_request_(const MeshMessage &msg, const uint8_t *mac_addr);  // Handle route requests
  void process_route_reply_(const MeshMessage &msg, const uint8_t *mac_addr);  // Handle route replies
  void process_join_request_(const MeshMessage &msg, const uint8_t *mac_addr);  // Handle join requests
  void process_key_message_(const MeshMessage &msg);  // Handle key exchange
  void process_channel_message_(const MeshMessage &msg);  // Handle channel switch
  void process_channel_ack_(const MeshMessage &msg);  // Handle channel ACK
  bool add_peer_if_needed_(const uint8_t *mac_addr);  // Add ESP-NOW peer if not present
  uint32_t calculate_backoff_(uint8_t retry_count);  // Calculate retry backoff
  bool is_channel_busy_();  // Check channel activity
  uint16_t assign_short_address();  // Assign short address to new node
  uint8_t get_wifi_channel();  // Get Wi-Fi channel (unused in no-Wi-Fi mode)
  uint8_t scan_channels();  // Scan for least busy channel
  void scan_for_coordinator_channel();  // Scan for Coordinator/Router channel
  void sync_child_table_(const std::array<uint8_t, 3> &dest_id);  // Sync child table with Coordinator

  // Node state and configuration
  uint8_t role_{ROLE_END_DEVICE};  // Default role: End Device
  std::string node_id_str_;        // Node ID as string (MAC-derived)
  uint8_t node_id_[3];             // Node ID (3-byte MAC-derived)
  uint16_t short_addr_{0xFFFF};    // Short address (0xFFFF = unjoined)
  std::string network_key_;        // AES-128 CCM encryption key
  std::string last_message_;       // Last received message content
  std::map<std::array<uint8_t, 3>, RouteEntry> routing_table_;  // Routing table
  static const uint8_t MAX_HOPS = 5;  // Maximum hops allowed
  static const uint8_t MAX_RETRIES = 5;  // Maximum send retries
  static const uint32_t BACKOFF_BASE_MS = 10;  // Base backoff time (ms)
  static const uint32_t BACKOFF_MAX_MS = 500;  // Max backoff time (ms)
  static const uint32_t MAINS_TIMEOUT_MS = 60000;  // Router timeout (60s)
  static const uint32_t BATTERY_TIMEOUT_MS = 86400000;  // End Device timeout (24h)
  static const uint32_t ROUTE_EXPIRY_MS = 60000;  // Route expiry (60s)
  static const uint32_t PERMIT_JOIN_TIMEOUT_MS = 120000;  // Permit join timeout (2min)
  static const uint32_t CHANNEL_SWITCH_TIMEOUT_MS = 5000;  // Channel switch timeout (5s)
  static const uint32_t COORDINATOR_FAILOVER_TIMEOUT_MS = 30000;  // Failover timeout (30s)
  static const uint32_t ROUTE_UPDATE_INTERVAL_MS = 15000;  // Route update interval (15s)
  static const uint32_t RSSI_UPDATE_MIN_INTERVAL_MS = 60000;  // RSSI update interval (60s)
  static const uint32_t CHILD_RETRY_INTERVAL_MS = 1000;  // Child retry interval (1s)
  static const uint32_t JOIN_TOTAL_TIMEOUT_MS = 8000;  // Total join timeout (8s)
  static const uint32_t SCAN_CHANNEL_TIMEOUT_MS = 1000;  // Per-channel scan timeout (1s)
  static const uint32_t SCAN_RETRY_INTERVAL_MS = 10000;  // Scan retry interval (10s)
  static const int8_t RSSI_CHANGE_THRESHOLD = 5;  // RSSI change threshold (±5 dBm)
  bool awaiting_ack_{false};      // Flag for send acknowledgment
  std::vector<uint8_t> switch_pins_;  // GPIO switch pins
  volatile uint32_t channel_activity_count_{0};  // Channel activity counter
  std::array<uint8_t, 3> coordinator_id_{0, 0, 0};  // Coordinator ID
  std::set<std::string> mesh_disabled_;  // Disabled entities
  std::map<std::string, sensor::Sensor *> dynamic_sensors_;  // Dynamic HA sensors
  std::map<std::string, switch_::Switch *> dynamic_switches_;  // Dynamic HA switches
  std::set<uint16_t> used_short_addresses_;  // Used short addresses
  std::function<void()> on_message_received_;  // Message received callback
  std::function<void()> on_message_sent_;  // Message sent callback
  std::vector<sensor::Sensor *> sensors_;  // Local sensors
  std::vector<switch_::Switch *> switches_;  // Local switches
  mbedtls_ccm_context ccm_ctx_;  // AES-128 CCM context
  uint32_t nonce_counter_{0};  // Encryption nonce counter
  bool permit_join_{false};  // Permit join flag
  switch_::Switch *permit_join_switch_{nullptr};  // HA permit join switch
  uint32_t permit_join_start_time_{0};  // Permit join start time
  uint8_t current_channel_{1};  // Current ESP-NOW channel
  switch_::Switch *channel_switch_{nullptr};  // HA channel switch
  bool use_wifi_channel_{false};  // Wi-Fi channel flag (false default)
  sensor::Sensor *rssi_sensor_{nullptr};  // Auto RSSI sensor
  uint32_t retry_failures_{0};  // Retry failure counter
  uint32_t channel_switch_start_time_{0};  // Channel switch start time
  std::set<std::array<uint8_t, 3>> pending_channel_acks_;  // Nodes awaiting channel ACK
  uint32_t last_channel_beacon_time_{0};  // Last channel beacon time
  bool channel_found_{false};  // Channel found flag
  int8_t parent_rssi_{-127};  // RSSI to parent (invalid default)
  int8_t last_published_rssi_{-127};  // Last published RSSI
  uint32_t last_rssi_update_{0};  // Last RSSI update time
  std::vector<MeshMessage> local_buffer_;  // Local message buffer
  std::map<std::array<uint8_t, 3>, RouteEntry> child_table_;  // Router child table
  std::map<std::array<uint8_t, 3>, uint32_t> child_last_seen_;  // Child activity times
  uint32_t last_child_retry_{0};  // Last child retry time
  uint32_t last_route_update_{0};  // Last route update time
  uint32_t last_scan_retry_{0};  // Last scan retry time
};
}  // namespace espnow_mesh
}  // namespace esphome
