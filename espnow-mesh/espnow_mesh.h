#ifndef ESPNOW_MESH_H  // Prevent multiple inclusions of this header file
#define ESPNOW_MESH_H

#include <esphome.h>         // ESPHome core library for component integration
#include <queue>             // Standard library for priority queue (traffic queuing)
#include <set>               // Standard library for sets (banned nodes, acks)
#include <nvs_flash.h>       // Non-volatile storage library for key persistence
#include <ArduinoJson.h>     // JSON library for topology data serialization

class ESPNowMesh : public esphome::Component {  // Define the ESPNowMesh class, inheriting from ESPHome Component
 public:
  // Enum for device roles in the mesh network
  enum Role { 
    ROLE_COORDINATOR = 0,  // Coordinator: manages network, connects to HA via Wi-Fi
    ROLE_ROUTER = 1,       // Router: relays messages, extends network range
    ROLE_END_DEVICE = 2    // End Device: battery-powered, low-power nodes
  };

  // Enum for message types used in the mesh protocol
  enum MessageType {
    MSG_JOIN = 0,         // Join request from new nodes
    MSG_DATA = 1,         // Sensor data or control updates
    MSG_CONTROL = 2,      // Switch control messages
    MSG_RREQ = 3,         // Route request (AODV-like routing)
    MSG_RREP = 4,         // Route reply (AODV-like routing)
    MSG_KEY = 5,          // Key rotation message
    MSG_CHANNEL = 6,      // Channel announcement beacon
    MSG_CHANNEL_ACK = 7,  // Acknowledgment for channel switch
    MSG_CHALLENGE = 8,    // Authentication challenge
    MSG_RESPONSE = 9,     // Authentication response
    MSG_REMOVE = 10,      // Node removal/ban message
    MSG_TOPOLOGY = 11     // Topology request/response for network map
  };

  // Constructor: initialize with default role as End Device
  ESPNowMesh() : Component(), role_(ROLE_END_DEVICE) {}

  // Setter for device role (Coordinator, Router, End Device)
  void set_role(Role role) { role_ = role; }

  // Setter for network ID (optional, defaults to random if unset)
  void set_network_id(uint16_t id) { network_id_ = id; }

  // Setter for install code (16-byte AES-128 key for initial authentication)
  void set_install_code(const std::string &code) { 
    install_code_ = code; 
    if (code.length() < 16) install_code_.resize(16, 0);  // Pad to 16 bytes if short
  }

  // Setup function: initialize hardware, network, and HA entities
  void setup() override;

  // Loop function: main runtime logic for mesh operations
  void loop() override;

  // Register HA services (e.g., node removal)
  void register_services() override;

  // HA service to remove a node from the network
  void remove_node_service(const std::vector<uint8_t> &node_id);

  // HA sensors for mesh information (defined in C++, auto-registered)
  sensor::Sensor *node_count_sensor_ = new sensor::Sensor();        // Tracks number of nodes
  sensor::Sensor *mesh_topology_sensor_ = new sensor::Sensor();     // Provides JSON topology data
  binary_sensor::BinarySensor *network_health_sensor_ = new binary_sensor::BinarySensor();  // Indicates network health

 protected:
  // Structure for ESP-NOW messages (20 bytes total)
  struct MeshMessage {
    uint16_t network_id;        // 2 bytes: Network identifier to isolate meshes
    uint8_t src_node_id[3];     // 3 bytes: Source node ID (MAC-based)
    uint8_t dest_node_id[3];    // 3 bytes: Destination node ID (MAC-based or broadcast)
    uint8_t msg_type;           // 1 byte: Type of message (see MessageType enum)
    uint8_t hops_role;          // 1 byte: 4 bits hops (0-15), 4 bits role (0-2)
    uint8_t entity_endpoint;    // 1 byte: Endpoint for data/control (e.g., sensor ID)
    uint8_t data_len;           // 1 byte: Length of data payload (0-9)
    uint8_t data[10];           // 10 bytes: Payload data (max 9 bytes used)
    static const uint8_t MAX_DATA_LEN = 9;  // Maximum payload length
  };

  // Structure for prioritized message queuing
  struct MeshMessagePriority {
    MeshMessage msg;           // Message to be queued
    // Comparison operator: prioritize non-data messages (switches/auth) over data
    bool operator<(const MeshMessagePriority &other) const {
      if (msg.msg_type == MSG_DATA && other.msg_type != MSG_DATA) return true;
      if (msg.msg_type != MSG_DATA && other.msg_type == MSG_DATA) return false;
      return false;
    }
  };

  // Structure for routing table entries
  struct RouteEntry {
    uint8_t next_hop[6];       // 6 bytes: MAC address of next hop
    uint32_t cost;             // 4 bytes: Cost metric (hops * 100 - RSSI + retries * 50)
    uint32_t last_seen;        // 4 bytes: Timestamp of last activity (ms)
    int8_t rssi;               // 1 byte: Received Signal Strength Indicator (-127 to 0 dBm)
    bool is_router;            // 1 byte: True if node is a Router
    uint8_t retry_count;       // 1 byte: Number of retries for this route
    uint16_t short_addr;       // 2 bytes: Short address assigned by Coordinator
  };

  // Constants for timing and limits
  static const uint8_t MAX_RETRIES = 5;                // Max retries for message delivery
  static const uint32_t BACKOFF_BASE_MS = 10;          // Base backoff time for retries (ms)
  static const uint32_t BACKOFF_MAX_MS = 500;          // Max backoff time for retries (ms)
  static const uint32_t MAINS_TIMEOUT_MS = 60000;      // Timeout for mains-powered nodes (60s)
  static const uint32_t BATTERY_TIMEOUT_MS = 86400000; // Timeout for battery nodes (24h)
  static const uint32_t ROUTE_EXPIRY_MS = 60000;       // Route expiration time (60s)
  static const uint32_t PERMIT_JOIN_TIMEOUT_MS = 120000; // Join window duration (120s)
  static const uint32_t CHANNEL_SWITCH_TIMEOUT_MS = 12000; // Channel switch timeout (12s)
  static const uint32_t INTERFERENCE_CHECK_INTERVAL_MS = 2000; // Interference check (2s)
  static const uint32_t TOPOLOGY_UPDATE_INTERVAL_MS = 60000;   // Topology update (60s)
  static const uint32_t KEY_ROTATION_INTERVAL_MS = 86400000;   // Key rotation (24h)
  static const uint32_t KEY_GRACE_PERIOD_MS = 10000;           // Key transition grace (10s)
  static const uint32_t AWAKE_DURATION_MS = 1000;              // Awake time for End Devices (1s)
  static const uint32_t SLEEP_DURATION_MS = 61000;             // Sleep time for End Devices (61s)
  static const uint8_t MAX_HOPS = 3;                           // Max hops for multi-hop messages
  static const size_t MAX_BUFFER_SIZE = 10;                    // Max messages in priority queue
  static const size_t MAX_RSSI_SAMPLES = 10;                   // Max RSSI samples for averaging
  static const int8_t RSSI_INTERFERENCE_THRESHOLD = -85;       // RSSI threshold for interference (-85 dBm)
  static const float CHANNEL_ACK_THRESHOLD = 0.8;              // Ack ratio for channel switch (80%)

  // Member variables
  Role role_;                                  // Current device role (Coordinator, Router, End Device)
  uint16_t network_id_{0xFFFF};               // Network ID (default: random if unset)
  std::string install_code_;                   // Install code for initial authentication (16 bytes)
  std::string network_key_;                    // Current AES-128 network key (16 bytes)
  std::string network_key_previous_;           // Previous network key for grace period (16 bytes)
  uint32_t last_key_rotation_time_{0};         // Timestamp of last key rotation (ms)
  uint8_t node_id_[3];                         // 3-byte node ID (MAC-based)
  std::array<uint8_t, 3> coordinator_id_{0, 0, 0}; // Coordinator’s node ID
  bool use_wifi_channel_{false};               // Use Wi-Fi channel for ESP-NOW (Coordinator only)
  uint8_t current_channel_{1};                 // Current ESP-NOW channel (1-13)
  uint8_t saved_channel_{0};                   // Saved channel for persistence
  bool channel_saved_{false};                  // Flag for saved channel validity
  bool permit_join_{false};                    // Allow new nodes to join
  bool node_joined_{false};                    // Node has joined the network
  bool awaiting_ack_{false};                   // Waiting for message acknowledgment
  uint64_t nonce_counter_{0};                  // Nonce counter for encryption
  std::map<std::array<uint8_t, 3>, RouteEntry> routing_table_; // Routing table (node ID -> route info)
  std::set<std::array<uint8_t, 3>> banned_nodes_; // Set of banned node IDs
  std::set<std::array<uint8_t, 3>> pending_channel_acks_; // Nodes pending channel switch ack
  std::set<std::array<uint8_t, 3>> pending_topology_acks_; // Nodes pending topology response
  std::priority_queue<MeshMessagePriority> local_buffer_; // Priority queue for message retries
  std::queue<int8_t> rssi_samples_;            // Queue of recent RSSI samples
  std::vector<float> last_sensor_values_;      // Last sensor values sent (for End Devices)
  uint32_t permit_join_timeout_{0};            // Timestamp for join window expiration
  uint32_t last_channel_beacon_time_{0};       // Timestamp of last beacon
  uint32_t last_channel_check_{0};             // Timestamp of last interference check
  uint32_t channel_switch_start_time_{0};      // Timestamp of channel switch start
  uint32_t last_topology_update_{0};           // Timestamp of last topology update
  uint32_t join_attempts_{0};                  // Number of join attempts (End Devices)
  uint32_t max_join_attempts_{720};            // Max join attempts before deep sleep (720 = 24h)
  uint32_t last_scan_retry_{0};                // Timestamp of last channel scan retry
  uint32_t channel_activity_count_{0};         // Count of channel activity (interference detection)
  int8_t parent_rssi_{-127};                   // RSSI of parent node (-127 = no parent)
  mbedtls_ccm_context ccm_ctx_;                // Current AES-128 CCM context
  mbedtls_ccm_context ccm_ctx_previous_;       // Previous AES-128 CCM context (grace period)

  // Protected methods (placeholders for brevity)
  void send_message(const MeshMessage &msg);   // Send an ESP-NOW message
  uint8_t get_wifi_channel();                  // Get current Wi-Fi channel
  void encrypt_data_(uint8_t *data, size_t len, uint8_t *nonce); // Encrypt message data
  bool verify_key(const uint8_t *data, size_t len, const uint8_t *nonce); // Verify message key
  void process_join_request_(const MeshMessage &msg, const uint8_t *mac_addr); // Handle join requests
  void process_remove_message_(const MeshMessage &msg); // Handle node removal
  void process_channel_message_(const MeshMessage &msg); // Handle channel beacons
  void process_channel_ack_(const MeshMessage &msg);    // Handle channel switch acks
  void process_key_message_(const MeshMessage &msg);    // Handle key rotation
  void process_topology_message_(const MeshMessage &msg); // Handle topology updates
  void send_data(uint8_t endpoint, const std::vector<float> &values); // Send sensor data
  void send_join_request();                     // Send join request (End Devices)
  void send_channel_ack(uint8_t channel);       // Send channel switch ack
  void send_remove_ack(const std::array<uint8_t, 3> &target_id); // Send removal ack
  void send_channel_switch(uint8_t channel);    // Send channel switch command
  void rotate_network_key();                    // Rotate network key (24h)
  void request_topology();                      // Request topology data
  void update_mesh_info();                      // Update HA mesh info entities
  void generate_challenge(uint8_t *challenge);  // Generate authentication challenge
  bool verify_response(const uint8_t *node_id, const uint8_t *response, const uint8_t *challenge); // Verify auth response
  void encrypt_network_key(const uint8_t *key, uint8_t *output); // Encrypt new network key
  void compress_data(const uint8_t *input, uint8_t input_len, uint8_t *output); // Compress data payload
  void decompress_data(const uint8_t *input, uint8_t input_len, std::vector<float> &output); // Decompress data
  int8_t get_average_rssi();                    // Calculate average RSSI
  uint8_t scan_channels();                      // Scan for clear channels
  void detect_network_conflict();               // Detect and resolve network ID conflicts
  void load_network_keys();                     // Load keys from NVS
  void save_network_keys();                     // Save keys to NVS
  static void promiscuous_rx_callback(void *buf, wifi_promiscuous_pkt_type_t type); // Callback for RSSI monitoring
};

#endif
