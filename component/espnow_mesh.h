#ifndef ESPNOW_MESH_H
#define ESPNOW_MESH_H

#include "esphome.h"
#include <array>
#include <vector>
#include <queue>
#include <set>
#include <mbedtls/ccm.h>
#include <random>

#define MAX_HOPS 5            // Max hops for multi-hop routing
#define MAX_BUFFER_SIZE 10    // Max queued messages
#define MAX_RSSI_SAMPLES 10   // Max RSSI samples for interference check

enum MeshMessageType {
  MSG_DATA,           // Sensor data messages
  MSG_CONTROL,        // Switch control messages
  MSG_JOIN,           // Node join request
  MSG_REMOVE,         // Node removal
  MSG_CHANNEL,        // Channel switch broadcast
  MSG_CHANNEL_ACK,    // Channel switch acknowledgment
  MSG_KEY,            // Key rotation broadcast
  MSG_TOPOLOGY,       // Topology update request
  MSG_CHALLENGE       // Authentication challenge
};

enum MeshRole {
  ROLE_COORDINATOR,   // Manages network, bridges to HA
  ROLE_ROUTER,        // Routes messages, has switches/sensors
  ROLE_END_DEVICE     // Leaf node, battery-powered, sensors only
};

struct MeshMessagePriority {
  MeshMessage msg;
  bool operator<(const MeshMessagePriority &other) const {
    return msg.msg_type == MSG_CONTROL && other.msg.msg_type != MSG_CONTROL;
  }
};

struct MeshMessage {
  MeshMessageType msg_type; // Message type
  MeshRole role;            // Sender role
  uint8_t entity_endpoint;  // Endpoint ID for entity
  uint16_t network_id;      // Network identifier
  uint8_t src_node_id[3];   // Source node ID (MAC-derived)
  uint8_t dest_node_id[3];  // Destination node ID
  uint8_t hops_role;        // Upper 4 bits: hops, lower 4: role
  uint8_t data_len;         // Data length
  uint8_t data[MeshMessage::MAX_DATA_LEN]; // Payload
  static const size_t MAX_DATA_LEN = 32;   // Max payload size
};

struct RouteEntry {
  uint8_t next_hop[6];  // Next hop MAC address
  uint8_t cost;         // Routing cost
  uint32_t last_seen;   // Last update timestamp
  int8_t rssi;          // Signal strength
  bool is_router;       // Router flag
  uint8_t retry_count;  // Retry counter
  uint16_t short_addr;  // Short address
};

class ESPNowMesh : public Component {
 public:
  ESPNowMesh() : node_count_sensor_(new Sensor()), 
                 mesh_topology_sensor_(new TextSensor()), 
                 network_health_sensor_(new BinarySensor()) {}

  // Setup: Initialize hardware and HA integration
  void setup() override;
  // Loop: Main runtime logic
  void loop() override;

  // Send a message over the mesh
  void send_message(const MeshMessage &msg);
  // Get adaptive beacon interval based on node count
  uint32_t get_beacon_interval();
  // Register HA services
  void register_services();

  // HA service to remove a node
  void remove_node_service(const std::vector<uint8_t> &node_id);
  // HA service to control a switch
  void control_switch_service(const std::vector<uint8_t> &node_id, uint8_t endpoint, uint8_t command);
  // Send sensor data
  void send_data(uint8_t endpoint, const std::vector<float> &values, const std::array<uint8_t, 3> &target_id = {0, 0, 0});
  // Send control command
  void send_control(uint8_t endpoint, bool state, const std::array<uint8_t, 3> &target_id);
  // Custom action: Send 'on' command
  void send_on(uint8_t endpoint, const std::array<uint8_t, 3> &target_id);
  // Custom action: Send 'off' command
  void send_off(uint8_t endpoint, const std::array<uint8_t, 3> &target_id);
  // Custom action: Send 'toggle' command
  void send_toggle(uint8_t endpoint, const std::array<uint8_t, 3> &target_id);
  // Send join request
  void send_join_request();
  // Send channel switch ACK
  void send_channel_ack(uint8_t channel);
  // Broadcast channel switch
  void send_channel_switch(uint8_t channel);
  // Send removal ACK
  void send_remove_ack(const std::array<uint8_t, 3> &target_id);
  // Request topology update
  void request_topology();

 protected:
  // Process join request with authentication
  void process_join_request_(const MeshMessage &msg, const uint8_t *mac_addr);
  // Process node removal
  void process_remove_message_(const MeshMessage &msg);
  // Process channel switch
  void process_channel_message_(const MeshMessage &msg);
  // Process channel ACK
  void process_channel_ack_(const MeshMessage &msg);
  // Process key rotation
  void process_key_message_(const MeshMessage &msg);
  // Process topology update
  void process_topology_message_(const MeshMessage &msg);
  // Process control command
  void process_control_message_(const MeshMessage &msg);

  // Update HA network map
  void update_mesh_info();
  // Callback for promiscuous RX
  void promiscuous_rx_callback(void *buf, wifi_promiscuous_pkt_type_t type);
  // Calculate average RSSI
  int8_t get_average_rssi();
  // Scan for least interfered channel
  uint8_t scan_channels();
  // Generate authentication challenge
  void generate_challenge(uint8_t *challenge);
  // Verify authentication response
  bool verify_response(const uint8_t *node_id, const uint8_t *response, const uint8_t *challenge);
  // Encrypt network key
  void encrypt_network_key(const uint8_t *key, uint8_t *output);
  // Compress data for transmission
  void compress_data(const uint8_t *input, uint8_t input_len, uint8_t *output);
  // Decompress received data
  void decompress_data(const uint8_t *input, uint8_t input_len, std::vector<float> &output);
  // Detect network conflicts (placeholder)
  void detect_network_conflict();
  // Load keys from NVS
  void load_network_keys();
  // Save keys to NVS
  void save_network_keys();
  // Get Wi-Fi channel from AP
  uint8_t get_wifi_channel();
  // Encrypt message data
  void encrypt_data_(uint8_t *data, size_t len, uint8_t *nonce);
  // Verify encrypted message
  bool verify_key(const uint8_t *data, size_t len, const uint8_t *nonce);

  MeshRole role_ = ROLE_END_DEVICE;           // Device role
  bool use_wifi_channel_ = true;              // Use Wi-Fi channel for Coordinator
  uint8_t current_channel_ = 1;               // Current ESP-NOW channel
  bool channel_saved_ = false;                // Saved channel flag
  uint8_t saved_channel_ = 1;                 // Saved channel
  uint8_t node_id_[6];                        // Full MAC address
  std::array<uint8_t, 3> coordinator_id_ = {0, 0, 0}; // Coordinator short ID
  std::vector<float> last_sensor_values_;     // Last sensor values
  bool node_joined_ = false;                  // Node joined status
  uint16_t network_id_ = 0xFFFF;              // Network ID (default random)
  std::string network_key_;                   // Current AES-128 key
  std::string network_key_previous_;          // Previous key for grace period
  uint32_t key_rotation_grace_start_ = 0;     // Grace period start time
  mbedtls_ccm_context ccm_ctx_;               // Current key context
  mbedtls_ccm_context ccm_ctx_previous_;      // Previous key context
  uint32_t nonce_counter_ = 0;                // Nonce counter
  bool permit_join_ = true;                   // Allow new nodes
  uint32_t last_channel_beacon_time_ = 0;     // Last beacon time
  uint32_t last_channel_check_ = 0;           // Last RSSI check
  uint32_t last_key_rotation_time_ = 0;       // Last key rotation
  uint32_t last_topology_update_ = 0;         // Last topology update
  uint32_t channel_switch_start_time_ = 0;    // Channel switch start
  std::set<std::array<uint8_t, 3>> pending_channel_acks_; // Pending channel ACKs
  std::set<std::array<uint8_t, 3>> pending_topology_acks_; // Pending topology ACKs
  std::map<std::array<uint8_t, 3>, RouteEntry> routing_table_; // Routing table
  std::set<std::array<uint8_t, 3>> banned_nodes_;             // Banned nodes
  std::set<uint16_t> detected_network_ids_;                   // Detected network IDs
  std::priority_queue<MeshMessagePriority> local_buffer_;     // Message queue
  bool awaiting_ack_ = false;                                 // Awaiting ACK flag
  uint8_t join_attempts_ = 0;                                 // Join attempts
  uint8_t max_join_attempts_ = 10;                            // Max join attempts
  uint32_t last_scan_retry_ = 0;                              // Last scan retry
  bool channel_found_ = false;                                // Channel found flag
  uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast MAC
  std::queue<int8_t> rssi_samples_;                           // RSSI sample queue

  Sensor *node_count_sensor_;         // HA node count sensor
  TextSensor *mesh_topology_sensor_;  // HA topology sensor
  BinarySensor *network_health_sensor_; // HA health sensor
};

#endif  // ESPNOW_MESH_H
