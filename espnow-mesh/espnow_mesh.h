#ifndef ESPNOW_MESH_H
#define ESPNOW_MESH_H

#include "esphome.h"
#include <array>
#include <vector>
#include <queue>
#include <set>
#include <mbedtls/ccm.h>
#include <random>

#define MAX_HOPS 5
#define MAX_BUFFER_SIZE 10
#define MAX_RSSI_SAMPLES 10

enum MeshMessageType {
  MSG_DATA,
  MSG_CONTROL,
  MSG_JOIN,
  MSG_REMOVE,
  MSG_CHANNEL,
  MSG_CHANNEL_ACK,
  MSG_KEY,
  MSG_TOPOLOGY,
  MSG_CHALLENGE  // Added for authentication
};

enum MeshRole {
  ROLE_COORDINATOR,
  ROLE_ROUTER,
  ROLE_END_DEVICE
};

struct MeshMessagePriority {
  MeshMessage msg;
  bool operator<(const MeshMessagePriority &other) const {
    return msg.msg_type == MSG_CONTROL && other.msg.msg_type != MSG_CONTROL;
  }
};

struct MeshMessage {
  MeshMessageType msg_type;
  MeshRole role;
  uint8_t entity_endpoint;
  uint16_t network_id;
  uint8_t src_node_id[3];
  uint8_t dest_node_id[3];
  uint8_t hops_role;  // Upper 4 bits: hops, lower 4 bits: role
  uint8_t data_len;
  uint8_t data[MeshMessage::MAX_DATA_LEN];
  static const size_t MAX_DATA_LEN = 32;
};

struct RouteEntry {
  uint8_t next_hop[6];
  uint8_t cost;
  uint32_t last_seen;
  int8_t rssi;
  bool is_router;
  uint8_t retry_count;
  uint16_t short_addr;
};

class ESPNowMesh : public Component {
 public:
  ESPNowMesh() : node_count_sensor_(new Sensor()), 
                 mesh_topology_sensor_(new TextSensor()), 
                 network_health_sensor_(new BinarySensor()) {}

  void setup() override;
  void loop() override;

  void send_message(const MeshMessage &msg);
  uint32_t get_beacon_interval();
  void register_services();

  void remove_node_service(const std::vector<uint8_t> &node_id);
  void control_switch_service(const std::vector<uint8_t> &node_id, uint8_t endpoint, uint8_t command);
  void send_data(uint8_t endpoint, const std::vector<float> &values, const std::array<uint8_t, 3> &target_id = {0, 0, 0});
  void send_control(uint8_t endpoint, bool state, const std::array<uint8_t, 3> &target_id);
  void send_on(uint8_t endpoint, const std::array<uint8_t, 3> &target_id);
  void send_off(uint8_t endpoint, const std::array<uint8_t, 3> &target_id);
  void send_toggle(uint8_t endpoint, const std::array<uint8_t, 3> &target_id);
  void send_join_request();
  void send_channel_ack(uint8_t channel);
  void send_channel_switch(uint8_t channel);
  void send_remove_ack(const std::array<uint8_t, 3> &target_id);
  void request_topology();

 protected:
  void process_join_request_(const MeshMessage &msg, const uint8_t *mac_addr);
  void process_remove_message_(const MeshMessage &msg);
  void process_channel_message_(const MeshMessage &msg);
  void process_channel_ack_(const MeshMessage &msg);
  void process_key_message_(const MeshMessage &msg);
  void process_topology_message_(const MeshMessage &msg);
  void process_control_message_(const MeshMessage &msg);

  void update_mesh_info();
  void promiscuous_rx_callback(void *buf, wifi_promiscuous_pkt_type_t type);
  int8_t get_average_rssi();
  uint8_t scan_channels();
  void generate_challenge(uint8_t *challenge);
  bool verify_response(const uint8_t *node_id, const uint8_t *response, const uint8_t *challenge);
  void encrypt_network_key(const uint8_t *key, uint8_t *output);
  void compress_data(const uint8_t *input, uint8_t input_len, uint8_t *output);
  void decompress_data(const uint8_t *input, uint8_t input_len, std::vector<float> &output);
  void detect_network_conflict();
  void load_network_keys();
  void save_network_keys();
  uint8_t get_wifi_channel();
  void encrypt_data_(uint8_t *data, size_t len, uint8_t *nonce);
  bool verify_key(const uint8_t *data, size_t len, const uint8_t *nonce);

  MeshRole role_ = ROLE_END_DEVICE;
  bool use_wifi_channel_ = true;
  uint8_t current_channel_ = 1;
  bool channel_saved_ = false;
  uint8_t saved_channel_ = 1;
  uint8_t node_id_[6];
  std::array<uint8_t, 3> coordinator_id_ = {0, 0, 0};
  std::vector<float> last_sensor_values_;
  bool node_joined_ = false;
  uint16_t network_id_ = 0xFFFF;
  std::string network_key_;
  std::string network_key_previous_;
  uint32_t key_rotation_grace_start_ = 0;  // New for grace period
  mbedtls_ccm_context ccm_ctx_;
  mbedtls_ccm_context ccm_ctx_previous_;
  uint32_t nonce_counter_ = 0;
  bool permit_join_ = true;
  uint32_t last_channel_beacon_time_ = 0;
  uint32_t last_channel_check_ = 0;
  uint32_t last_key_rotation_time_ = 0;
  uint32_t last_topology_update_ = 0;
  uint32_t channel_switch_start_time_ = 0;
  std::set<std::array<uint8_t, 3>> pending_channel_acks_;
  std::set<std::array<uint8_t, 3>> pending_topology_acks_;
  std::map<std::array<uint8_t, 3>, RouteEntry> routing_table_;
  std::set<std::array<uint8_t, 3>> banned_nodes_;
  std::set<uint16_t> detected_network_ids_;
  std::priority_queue<MeshMessagePriority> local_buffer_;
  bool awaiting_ack_ = false;
  uint8_t join_attempts_ = 0;
  uint8_t max_join_attempts_ = 10;
  uint32_t last_scan_retry_ = 0;
  bool channel_found_ = false;
  uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  std::queue<int8_t> rssi_samples_;

  Sensor *node_count_sensor_;
  TextSensor *mesh_topology_sensor_;
  BinarySensor *network_health_sensor_;
};

#endif  // ESPNOW_MESH_H
