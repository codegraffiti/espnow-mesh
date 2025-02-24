#include "espnow_mesh.h"
#include <mbedtls/md.h>

#define MAX_HOPS 5
#define BACKOFF_BASE_MS 10
#define MAX_BUFFER_SIZE 10
#define KEY_ROTATION_INTERVAL_MS 86400000  // 24 hours
#define KEY_GRACE_PERIOD_MS 3600000        // 1 hour
#define INTERFERENCE_CHECK_INTERVAL_MS 2000
#define TOPOLOGY_UPDATE_INTERVAL_MS 60000
#define AWAKE_DURATION_MS 1000
#define SLEEP_DURATION_MS 61000
#define JOIN_LISTEN_DURATION_MS 5000
#define CHANNEL_SWITCH_TIMEOUT_MS 5000
#define CHANNEL_ACK_THRESHOLD 0.9
#define RSSI_INTERFERENCE_THRESHOLD -85
#define MAX_RETRIES 3
#define MAX_RSSI_SAMPLES 10
#define PERMIT_JOIN_TIMEOUT_MS 120000  // 2 minutes

// Setup: Initialize hardware and HA integration
void ESPNowMesh::setup() {
  ESP_LOGI(TAG, "Setting up ESPNowMesh component for role: %d", role_);
  if (role_ == ROLE_COORDINATOR && use_wifi_channel_) {
    WiFi.mode(WIFI_STA);
    uint32_t start = millis();
    while (!WiFi.isConnected() && millis() - start < 5000) { delay(100); }
    current_channel_ = WiFi.isConnected() ? get_wifi_channel() : (channel_saved_ ? saved_channel_ : 1);
    if (WiFi.isConnected()) saved_channel_ = current_channel_, channel_saved_ = true;
    ESP_LOGI(TAG, "Coordinator Wi-Fi channel set to %d", current_channel_);
  } else {
    WiFi.mode(WIFI_OFF);
    ESP_LOGI(TAG, "Wi-Fi disabled for role %d", role_);
  }
  esp_wifi_set_channel(current_channel_, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_callback);
  uint32_t start = millis();
  while (esp_now_init() != ESP_OK && millis() - start < 5000) { delay(100); }
  last_sensor_values_.resize(5, 0.0);
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }
  load_network_keys();
  if (role_ == ROLE_COORDINATOR) {
    if (network_id_ == 0xFFFF) {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<> dis(0, 65535);
      network_id_ = dis(gen);
      while (detected_network_ids_.find(network_id_) != detected_network_ids_.end()) network_id_ = dis(gen);
      ESP_LOGI(TAG, "Generated network ID: %04X", network_id_);
    }
    if (network_key_.empty()) {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<> dis(0, 255);
      network_key_.resize(16);
      for (int i = 0; i < 16; i++) network_key_[i] = dis(gen);
      save_network_keys();
      ESP_LOGI(TAG, "Generated initial network key");
    }
    node_count_sensor_->set_name("ESPNow Mesh Node Count");
    node_count_sensor_->set_unit_of_measurement("devices");
    node_count_sensor_->set_icon("mdi:counter");
    App.register_sensor(node_count_sensor_);
    mesh_topology_sensor_->set_name("ESPNow Mesh Topology");
    mesh_topology_sensor_->set_icon("mdi:network");
    App.register_sensor(mesh_topology_sensor_);
    network_health_sensor_->set_name("ESPNow Mesh Network Health");
    network_health_sensor_->set_device_class("connectivity");
    App.register_binary_sensor(network_health_sensor_);
    update_mesh_info();
  }
  if (!network_key_.empty()) {
    mbedtls_ccm_init(&ccm_ctx_);
    mbedtls_ccm_setkey(&ccm_ctx_, MBEDTLS_CCM_ENCRYPT, (const unsigned char *)network_key_.c_str(), 128);
    ESP_LOGI(TAG, "AES-128 CCM initialized with current key");
  }
  if (!network_key_previous_.empty()) {
    mbedtls_ccm_init(&ccm_ctx_previous_);
    mbedtls_ccm_setkey(&ccm_ctx_previous_, MBEDTLS_CCM_ENCRYPT, (const unsigned char *)network_key_previous_.c_str(), 128);
    ESP_LOGI(TAG, "AES-128 CCM initialized with previous key");
  }
  esp_read_mac(node_id_, ESP_MAC_WIFI_STA);
  register_services();
}

// Loop: Main runtime logic
void ESPNowMesh::loop() {
  if (role_ == ROLE_COORDINATOR) {
    // Key rotation every 24 hours
    if (millis() - last_key_rotation_time_ >= KEY_ROTATION_INTERVAL_MS) {
      rotate_network_key();
      last_key_rotation_time_ = millis();
      key_rotation_grace_start_ = millis();
      ESP_LOGI(TAG, "Network key rotated");
    }
    // Clear previous key after grace period
    if (!network_key_previous_.empty() && millis() - key_rotation_grace_start_ >= KEY_GRACE_PERIOD_MS) {
      network_key_previous_.clear();
      ESP_LOGI(TAG, "Previous network key cleared after grace period");
    }
    // Interference detection every 2 seconds
    if (millis() - last_channel_check_ >= INTERFERENCE_CHECK_INTERVAL_MS) {
      update_mesh_info();
      int8_t avg_rssi = get_average_rssi();
      if (avg_rssi < RSSI_INTERFERENCE_THRESHOLD) {
        uint8_t new_channel = scan_channels();
        if (new_channel != current_channel_) {
          current_channel_ = new_channel;
          esp_wifi_set_channel(current_channel_, WIFI_SECOND_CHAN_NONE);
          send_channel_switch(current_channel_);
          pending_channel_acks_ = std::set<std::array<uint8_t, 3>>();
          for (const auto &entry : routing_table_) pending_channel_acks_.insert(entry.first);
          channel_switch_start_time_ = millis();
          ESP_LOGI(TAG, "Switching to channel %d due to interference", new_channel);
        }
      }
      last_channel_check_ = millis();
    }
    // Handle channel switch ACKs
    if (channel_switch_start_time_ != 0) {
      static uint8_t total_retries = 0;
      size_t total_nodes = routing_table_.size();
      size_t acks_received = total_nodes - pending_channel_acks_.size();
      float ack_ratio = total_nodes > 0 ? static_cast<float>(acks_received) / total_nodes : 1.0;
      if (pending_channel_acks_.empty() || ack_ratio >= CHANNEL_ACK_THRESHOLD || 
          millis() - channel_switch_start_time_ >= CHANNEL_SWITCH_TIMEOUT_MS * (total_retries + 1)) {
        esp_wifi_set_channel(current_channel_, WIFI_SECOND_CHAN_NONE);
        channel_switch_start_time_ = 0;
        total_retries = 0;
        ESP_LOGI(TAG, "Channel switch completed, %d/%d ACKs", acks_received, total_nodes);
      } else if (millis() - channel_switch_start_time_ >= CHANNEL_SWITCH_TIMEOUT_MS) {
        if (total_retries < 1) {
          send_channel_switch(current_channel_);
          channel_switch_start_time_ = millis();
          total_retries++;
          ESP_LOGW(TAG, "Retrying channel switch, %d/%d ACKs", acks_received, total_nodes);
        }
      }
    }
    // Permit join timeout
    if (permit_join_ && millis() - permit_join_start_time_ >= PERMIT_JOIN_TIMEOUT_MS) {
      permit_join_ = false;
      ESP_LOGI(TAG, "Permit join timeout, new nodes blocked");
    }
  }
  // Beacon broadcasts
  if ((role_ == ROLE_COORDINATOR || role_ == ROLE_ROUTER) && 
      millis() - last_channel_beacon_time_ >= get_beacon_interval()) {
    MeshMessage beacon_msg;
    beacon_msg.msg_type = MSG_CHANNEL;
    beacon_msg.role = role_;
    beacon_msg.data_len = 2;
    beacon_msg.network_id = network_id_;
    memset(beacon_msg.dest_node_id, 0xFF, 3);
    beacon_msg.data[0] = current_channel_;
    beacon_msg.data[1] = permit_join_ ? 1 : 0;
    send_message(beacon_msg);
    last_channel_beacon_time_ = millis();
    ESP_LOGD(TAG, "Beacon sent on channel %d", current_channel_);
  }
  // End Device sleep/wake cycle
  if (role_ == ROLE_END_DEVICE && node_joined_) {
    static uint32_t last_wake = 0;
    if (millis() - last_wake >= AWAKE_DURATION_MS + SLEEP_DURATION_MS) {
      std::vector<float> values = {25.5, 60.2, 500.0, 1.0, 0.0}; // Example sensor values
      send_data(1, values);
      last_wake = millis();
      esp_light_sleep_start();
      ESP_LOGI(TAG, "End Device woke up, sent data, sleeping");
    }
  } else if (role_ == ROLE_END_DEVICE && !node_joined_) {
    join_attempts_++;
    if (join_attempts_ >= max_join_attempts_) {
      App.enter_deep_sleep();
      ESP_LOGE(TAG, "Max join attempts reached, entering deep sleep");
      return;
    }
    current_channel_ = saved_channel_ ? saved_channel_ : 1;
    esp_wifi_set_channel(current_channel_, WIFI_SECOND_CHAN_NONE);
    send_join_request();
    uint32_t start = millis();
    while (millis() - start < JOIN_LISTEN_DURATION_MS && !channel_found_) { yield(); delay(1); }
    last_scan_retry_ = millis();
    if (!channel_found_) App.enter_deep_sleep(JOIN_LISTEN_DURATION_MS);
    ESP_LOGI(TAG, "Join attempt %d, listening for %dms", join_attempts_, JOIN_LISTEN_DURATION_MS);
  }
  // Process queued messages
  if (!local_buffer_.empty()) {
    MeshMessage msg = local_buffer_.top().msg;
    local_buffer_.pop();
    send_message(msg);
    ESP_LOGD(TAG, "Sending queued message type %d", msg.msg_type);
  }
  // Topology update
  if (role_ == ROLE_COORDINATOR && millis() - last_topology_update_ >= TOPOLOGY_UPDATE_INTERVAL_MS) {
    request_topology();
    update_mesh_info();
    last_topology_update_ = millis();
    ESP_LOGI(TAG, "Requested topology update");
  }
}

// Send message with collision avoidance
void ESPNowMesh::send_message(const MeshMessage &msg) {
  if (!node_joined_ && msg.msg_type != MSG_JOIN) return;
  MeshMessage enhanced_msg = msg;
  enhanced_msg.network_id = network_id_;
  uint8_t current_hops = enhanced_msg.hops_role >> 4;
  if (current_hops < MAX_HOPS) enhanced_msg.hops_role += (1 << 4);
  memcpy(enhanced_msg.src_node_id, node_id_, 3);
  uint8_t retries = 0;
  do {
    uint8_t target_mac[6];
    bool direct = false;
    if (memcmp(enhanced_msg.dest_node_id, broadcast_mac, 3) == 0) {
      memcpy(target_mac, broadcast_mac, 6);
    } else {
      auto it = routing_table_.find({enhanced_msg.dest_node_id[0], enhanced_msg.dest_node_id[1], enhanced_msg.dest_node_id[2]});
      if (it != routing_table_.end()) {
        if (it->second.rssi > -85) {
          memcpy(target_mac, it->second.next_hop, 6);
          direct = true;
        } else {
          memcpy(target_mac, it->second.next_hop, 6);
        }
      } else {
        ESP_LOGW(TAG, "No route to %02X:%02X:%02X", enhanced_msg.dest_node_id[0], enhanced_msg.dest_node_id[1], enhanced_msg.dest_node_id[2]);
        return;
      }
    }

    if (enhanced_msg.msg_type == MSG_DATA || enhanced_msg.msg_type == MSG_CONTROL) {
      uint8_t compressed_data[10];
      compress_data(enhanced_msg.data, enhanced_msg.data_len, compressed_data);
      memcpy(enhanced_msg.data, compressed_data, 8);
      enhanced_msg.data_len = 8;
      enhanced_msg.data[7] = static_cast<uint8_t>(parent_rssi_);
    }

    uint8_t buffer[sizeof(MeshMessage)];
    memcpy(buffer, &enhanced_msg, sizeof(MeshMessage));
    uint8_t nonce[13] = {0};
    memcpy(nonce, node_id_, 3);
    uint32_t timestamp = millis();
    memcpy(nonce + 3, &timestamp, 4);
    memcpy(nonce + 7, &nonce_counter_++, 4);
    encrypt_data_(buffer, sizeof(MeshMessage), nonce);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 10);
    delay(dis(gen));

    if (role_ == ROLE_COORDINATOR) {
      App.get_light("mesh_activity_light")->turn_on();
      delay(50);
      App.get_light("mesh_activity_light")->turn_off();
    }

    esp_now_send(target_mac, buffer, sizeof(MeshMessage));
    uint32_t start = millis();
    while (awaiting_ack_ && millis() - start < 200) yield();
    if (!awaiting_ack_) return;
    retries++;
    if (retries == MAX_RETRIES) {
      if (!direct && memcmp(target_mac, broadcast_mac, 6) != 0) {
        auto it = routing_table_.find({enhanced_msg.dest_node_id[0], enhanced_msg.dest_node_id[1], enhanced_msg.dest_node_id[2]});
        if (it != routing_table_.end()) it->second.retry_count++;
      }
      if (local_buffer_.size() < MAX_BUFFER_SIZE) {
        local_buffer_.push({enhanced_msg});
        ESP_LOGW(TAG, "Message queued, type %d", enhanced_msg.msg_type);
      } else {
        ESP_LOGE(TAG, "Message dropped, queue full, type %d", enhanced_msg.msg_type);
      }
      scan_for_coordinator_channel();
      return;
    }
    delay(BACKOFF_BASE_MS * (1 << retries));
  } while (retries < MAX_RETRIES && millis() - start < 5000);
}

// Get adaptive beacon interval
uint32_t ESPNowMesh::get_beacon_interval() {
  size_t node_count = routing_table_.size();
  if (node_count <= 100) return 10000;
  else if (node_count <= 500) return 20000;
  else return 30000;
}

// Register HA services
void ESPNowMesh::register_services() {
  if (role_ == ROLE_COORDINATOR) {
    register_service(&ESPNowMesh::remove_node_service, "remove_node", {"node_id"});
    register_service(&ESPNowMesh::control_switch_service, "control_switch", {"node_id", "endpoint", "command"});
  }
}

// Remove node via HA service
void ESPNowMesh::remove_node_service(const std::vector<uint8_t> &node_id) {
  if (node_id.size() != 3) {
    ESP_LOGE(TAG, "Invalid node_id size: %u", node_id.size());
    return;
  }
  std::array<uint8_t, 3> target_id;
  memcpy(target_id.data(), node_id.data(), 3);
  if (routing_table_.erase(target_id)) {
    banned_nodes_.insert(target_id);
    MeshMessage remove_msg;
    remove_msg.msg_type = MSG_REMOVE;
    remove_msg.role = role_;
    remove_msg.data_len = 3;
    remove_msg.network_id = network_id_;
    remove_msg.hops_role = (0 << 4) | role_;
    memcpy(remove_msg.dest_node_id, target_id.data(), 3);
    send_message(remove_msg);
    ESP_LOGI(TAG, "Node %02X:%02X:%02X removed", target_id[0], target_id[1], target_id[2]);
  }
}

// Control switch via HA service
void ESPNowMesh::control_switch_service(const std::vector<uint8_t> &node_id, uint8_t endpoint, uint8_t command) {
  if (node_id.size() != 3) {
    ESP_LOGE(TAG, "Invalid node_id size: %u", node_id.size());
    return;
  }
  std::array<uint8_t, 3> target_id;
  memcpy(target_id.data(), node_id.data(), 3);
  MeshMessage control_msg;
  control_msg.msg_type = MSG_CONTROL;
  control_msg.role = role_;
  control_msg.entity_endpoint = endpoint;
  control_msg.network_id = network_id_;
  memcpy(control_msg.dest_node_id, target_id.data(), 3);
  control_msg.data_len = 1;
  control_msg.data[0] = command;  // 1 = on, 0 = off, 2 = toggle
  send_message(control_msg);
  ESP_LOGI(TAG, "Control command %d sent to %02X:%02X:%02X, endpoint %d", command, target_id[0], target_id[1], target_id[2], endpoint);
}

// Send sensor data
void ESPNowMesh::send_data(uint8_t endpoint, const std::vector<float> &values, const std::array<uint8_t, 3> &target_id) {
  MeshMessage data_msg;
  data_msg.msg_type = MSG_DATA;
  data_msg.role = role_;
  data_msg.entity_endpoint = endpoint;
  data_msg.network_id = network_id_;
  if (target_id[0] == 0 && target_id[1] == 0 && target_id[2] == 0) {
    memcpy(data_msg.dest_node_id, coordinator_id_.data(), 3);
  } else {
    memcpy(data_msg.dest_node_id, target_id.data(), 3);
  }
  data_msg.data_len = std::min(values.size() * 2, (size_t)MeshMessage::MAX_DATA_LEN);
  for (size_t i = 0; i < values.size() && i * 2 + 1 < MeshMessage::MAX_DATA_LEN; i++) {
    uint16_t val = static_cast<uint16_t>(values[i] * 10);
    data_msg.data[i * 2] = val >> 8;
    data_msg.data[i * 2 + 1] = val & 0xFF;
  }
  send_message(data_msg);
  ESP_LOGD(TAG, "Sent data to endpoint %d", endpoint);
}

// Send control command
void ESPNowMesh::send_control(uint8_t endpoint, bool state, const std::array<uint8_t, 3> &target_id) {
  MeshMessage control_msg;
  control_msg.msg_type = MSG_CONTROL;
  control_msg.role = role_;
  control_msg.entity_endpoint = endpoint;
  control_msg.network_id = network_id_;
  memcpy(control_msg.dest_node_id, target_id.data(), 3);
  control_msg.data_len = 1;
  control_msg.data[0] = state ? 1 : 0;
  send_message(control_msg);
}

// Send 'on' command
void ESPNowMesh::send_on(uint8_t endpoint, const std::array<uint8_t, 3> &target_id) {
  send_control(endpoint, true, target_id);
  ESP_LOGI(TAG, "Sending ON to %02X:%02X:%02X, endpoint %d", target_id[0], target_id[1], target_id[2], endpoint);
}

// Send 'off' command
void ESPNowMesh::send_off(uint8_t endpoint, const std::array<uint8_t, 3> &target_id) {
  send_control(endpoint, false, target_id);
  ESP_LOGI(TAG, "Sending OFF to %02X:%02X:%02X, endpoint %d", target_id[0], target_id[1], target_id[2], endpoint);
}

// Send 'toggle' command
void ESPNowMesh::send_toggle(uint8_t endpoint, const std::array<uint8_t, 3> &target_id) {
  MeshMessage control_msg;
  control_msg.msg_type = MSG_CONTROL;
  control_msg.role = role_;
  control_msg.entity_endpoint = endpoint;
  control_msg.network_id = network_id_;
  memcpy(control_msg.dest_node_id, target_id.data(), 3);
  control_msg.data_len = 1;
  control_msg.data[0] = 2;
  send_message(control_msg);
  ESP_LOGI(TAG, "Sending TOGGLE to %02X:%02X:%02X, endpoint %d", target_id[0], target_id[1], target_id[2], endpoint);
}

// Send join request
void ESPNowMesh::send_join_request() {
  MeshMessage join_msg;
  join_msg.msg_type = MSG_JOIN;
  join_msg.role = role_;
  join_msg.network_id = network_id_;
  memset(join_msg.dest_node_id, 0xFF, 3);
  join_msg.data_len = 3;
  memcpy(join_msg.data, node_id_, 3);
  send_message(join_msg);
  ESP_LOGI(TAG, "Join request from %02X:%02X:%02X", node_id_[0], node_id_[1], node_id_[2]);
}

// Process join request with authentication
void ESPNowMesh::process_join_request_(const MeshMessage &msg, const uint8_t *mac_addr) {
  if (!permit_join_ || msg.network_id != network_id_) return;
  std::array<uint8_t, 3> new_node_id;
  memcpy(new_node_id.data(), msg.data, 3);
  if (banned_nodes_.find(new_node_id) != banned_nodes_.end()) {
    ESP_LOGW(TAG, "Join request from banned node %02X:%02X:%02X", new_node_id[0], new_node_id[1], new_node_id[2]);
    return;
  }

  uint8_t challenge[8];
  generate_challenge(challenge);
  MeshMessage challenge_msg;
  challenge_msg.msg_type = MSG_CHALLENGE;
  challenge_msg.role = role_;
  challenge_msg.network_id = network_id_;
  memcpy(challenge_msg.dest_node_id, new_node_id.data(), 3);
  challenge_msg.data_len = 8;
  memcpy(challenge_msg.data, challenge, 8);
  send_message(challenge_msg);

  RouteEntry entry;
  memcpy(entry.next_hop, mac_addr, 6);
  entry.cost = 100;
  entry.last_seen = millis();
  entry.rssi = -70;
  entry.is_router = (msg.role == ROLE_ROUTER);
  entry.retry_count = 0;
  entry.short_addr = routing_table_.size() + 1;
  routing_table_[new_node_id] = entry;
  if (role_ == ROLE_COORDINATOR) coordinator_id_ = {node_id_[0], node_id_[1], node_id_[2]};
  node_joined_ = true;
  ESP_LOGI(TAG, "Node %02X:%02X:%02X joined (pending auth)", new_node_id[0], new_node_id[1], new_node_id[2]);
}

// Update HA network map
void ESPNowMesh::update_mesh_info() {
  if (role_ != ROLE_COORDINATOR) return;
  node_count_sensor_->publish_state(routing_table_.size());
  StaticJsonDocument<4096> doc;
  JsonArray nodes = doc.createNestedArray("nodes");
  for (const auto &entry : routing_table_) {
    JsonObject node = nodes.createNestedObject();
    node["id"] = String(entry.first[0], HEX) + ":" + String(entry.first[1], HEX) + ":" + String(entry.first[2], HEX);
    node["role"] = entry.second.is_router ? "Router" : "End Device";
    node["rssi"] = entry.second.rssi;
    node["parent"] = String(entry.second.next_hop[0], HEX) + ":" + 
                     String(entry.second.next_hop[1], HEX) + ":" + 
                     String(entry.second.next_hop[2], HEX);
  }
  String topology_json;
  serializeJson(doc, topology_json);
  mesh_topology_sensor_->publish_state(topology_json);
  network_health_sensor_->publish_state(get_average_rssi() >= RSSI_INTERFERENCE_THRESHOLD);
  ESP_LOGI(TAG, "Updated HA topology, %d nodes", routing_table_.size());
}

// Send channel ACK
void ESPNowMesh::send_channel_ack(uint8_t channel) {
  MeshMessage ack_msg;
  ack_msg.msg_type = MSG_CHANNEL_ACK;
  ack_msg.role = role_;
  ack_msg.data_len = 1;
  ack_msg.network_id = network_id_;
  memcpy(ack_msg.dest_node_id, coordinator_id_.data(), 3);
  ack_msg.data[0] = channel;
  send_message(ack_msg);
}

// Broadcast channel switch
void ESPNowMesh::send_channel_switch(uint8_t channel) {
  MeshMessage switch_msg;
  switch_msg.msg_type = MSG_CHANNEL;
  switch_msg.role = role_;
  switch_msg.data_len = 2;
  switch_msg.network_id = network_id_;
  memset(switch_msg.dest_node_id, 0xFF, 3);
  switch_msg.data[0] = channel;
  switch_msg.data[1] = permit_join_ ? 1 : 0;
  send_message(switch_msg);
}

// Send removal ACK
void ESPNowMesh::send_remove_ack(const std::array<uint8_t, 3> &target_id) {
  MeshMessage ack_msg;
  ack_msg.msg_type = MSG_REMOVE;
  ack_msg.role = role_;
  ack_msg.data_len = 3;
  ack_msg.network_id = network_id_;
  memcpy(ack_msg.dest_node_id, coordinator_id_.data(), 3);
  memcpy(ack_msg.data, target_id.data(), 3);
  send_message(ack_msg);
}

// Request topology update
void ESPNowMesh::request_topology() {
  MeshMessage topo_msg;
  topo_msg.msg_type = MSG_TOPOLOGY;
  topo_msg.role = role_;
  topo_msg.data_len = 0;
  topo_msg.network_id = network_id_;
  memset(topo_msg.dest_node_id, 0xFF, 3);
  send_message(topo_msg);
  pending_topology_acks_ = std::set<std::array<uint8_t, 3>>();
  for (const auto &entry : routing_table_) pending_topology_acks_.insert(entry.first);
}

// Process channel switch message
void ESPNowMesh::process_channel_message_(const MeshMessage &msg) {
  if (msg.network_id != network_id_) return;
  uint8_t nonce[13] = {0};
  memcpy(nonce, msg.src_node_id, 3);
  uint32_t timestamp = millis();
  memcpy(nonce + 3, &timestamp, 4);
  memcpy(nonce + 7, &nonce_counter_, 4);
  if (!verify_key(msg.data, msg.data_len, nonce)) {
    ESP_LOGE(TAG, "Channel message verification failed");
    return;
  }
  current_channel_ = msg.data[0];
  esp_wifi_set_channel(current_channel_, WIFI_SECOND_CHAN_NONE);
  permit_join_ = msg.data[1] == 1;
  send_channel_ack(current_channel_);
  ESP_LOGI(TAG, "Channel switched to %d", current_channel_);
}

// Process channel ACK
void ESPNowMesh::process_channel_ack_(const MeshMessage &msg) {
  if (channel_switch_start_time_ == 0 || msg.network_id != network_id_) return;
  uint8_t nonce[13] = {0};
  memcpy(nonce, msg.src_node_id, 3);
  uint32_t timestamp = millis();
  memcpy(nonce + 3, &timestamp, 4);
  memcpy(nonce + 7, &nonce_counter_, 4);
  if (!verify_key(msg.data, msg.data_len, nonce)) {
    ESP_LOGE(TAG, "Channel ACK verification failed");
    return;
  }
  std::array<uint8_t, 3> src_id;
  memcpy(src_id.data(), msg.src_node_id, 3);
  pending_channel_acks_.erase(src_id);
  ESP_LOGI(TAG, "Channel ack from %02X:%02X:%02X", src_id[0], src_id[1], src_id[2]);
}

// Process removal message
void ESPNowMesh::process_remove_message_(const MeshMessage &msg) {
  if (msg.network_id != network_id_) return;
  uint8_t nonce[13] = {0};
  memcpy(nonce, msg.src_node_id, 3);
  uint32_t timestamp = millis();
  memcpy(nonce + 3, &timestamp, 4);
  memcpy(nonce + 7, &nonce_counter_, 4);
  if (!verify_key(msg.data, msg.data_len, nonce)) {
    ESP_LOGE(TAG, "Remove message verification failed");
    return;
  }
  std::array<uint8_t, 3> target_id;
  memcpy(target_id.data(), msg.data, 3);
  if (memcmp(target_id.data(), node_id_, 3) == 0) {
    node_joined_ = false;
    ESP_LOGI(TAG, "Node %02X:%02X:%02X removed self", node_id_[0], node_id_[1], node_id_[2]);
  } else {
    routing_table_.erase(target_id);
    send_remove_ack(target_id);
  }
}

// Process key rotation message
void ESPNowMesh::process_key_message_(const MeshMessage &msg) {
  if (msg.network_id != network_id_) return;
  uint8_t nonce[13] = {0};
  memcpy(nonce, msg.src_node_id, 3);
  uint32_t timestamp = millis();
  memcpy(nonce + 3, &timestamp, 4);
  memcpy(nonce + 7, &nonce_counter_, 4);
  if (!verify_key(msg.data, msg.data_len, nonce)) {
    ESP_LOGE(TAG, "Key message verification failed");
    return;
  }
  network_key_previous_ = network_key_;
  network_key_.resize(16);
  memcpy(network_key_.data(), msg.data, 16);
  save_network_keys();
  mbedtls_ccm_init(&ccm_ctx_previous_);
  mbedtls_ccm_setkey(&ccm_ctx_previous_, MBEDTLS_CCM_ENCRYPT, (const unsigned char *)network_key_previous_.c_str(), 128);
  mbedtls_ccm_init(&ccm_ctx_);
  mbedtls_ccm_setkey(&ccm_ctx_, MBEDTLS_CCM_ENCRYPT, (const unsigned char *)network_key_.c_str(), 128);
  ESP_LOGI(TAG, "Key rotated");
}

// Process topology update
void ESPNowMesh::process_topology_message_(const MeshMessage &msg) {
  if (msg.network_id != network_id_) return;
  uint8_t nonce[13] = {0};
  memcpy(nonce, msg.src_node_id, 3);
  uint32_t timestamp = millis();
  memcpy(nonce + 3, &timestamp, 4);
  memcpy(nonce + 7, &nonce_counter_, 4);
  if (!verify_key(msg.data, msg.data_len, nonce)) {
    ESP_LOGE(TAG, "Topology message verification failed");
    return;
  }
  std::array<uint8_t, 3> src_id;
  memcpy(src_id.data(), msg.src_node_id, 3);
  pending_topology_acks_.erase(src_id);
  auto it = routing_table_.find(src_id);
  if (it != routing_table_.end()) {
    it->second.last_seen = millis();
    it->second.rssi = msg.data[7];
    ESP_LOGI(TAG, "Topology update from %02X:%02X:%02X", src_id[0], src_id[1], src_id[2]);
  }
}

// Process control message
void ESPNowMesh::process_control_message_(const MeshMessage &msg) {
  if (msg.network_id != network_id_) return;
  uint8_t nonce[13] = {0};
  memcpy(nonce, msg.src_node_id, 3);
  uint32_t timestamp = millis();
  memcpy(nonce + 3, &timestamp, 4);
  memcpy(nonce + 7, &nonce_counter_, 4);
  if (!verify_key(msg.data, msg.data_len, nonce)) {
    ESP_LOGE(TAG, "Control message verification failed");
    return;
  }
  if (memcmp(msg.dest_node_id, node_id_, 3) == 0) {
    uint8_t endpoint = msg.entity_endpoint;
    bool state = msg.data[0] == 1;
    ESP_LOGI(TAG, "Control received: endpoint %d, state %d", endpoint, state);
  }
}

// Promiscuous RX callback for activity LED and RSSI
void ESPNowMesh::promiscuous_rx_callback(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_DATA) return;
  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
  if (pkt->rx_ctrl.rssi < -127 || pkt->rx_ctrl.rssi > 0) return;
  if (role_ == ROLE_COORDINATOR) {
    App.get_light("mesh_activity_light")->turn_on();
    delay(50);
    App.get_light("mesh_activity_light")->turn_off();
  }
  rssi_samples_.push(pkt->rx_ctrl.rssi);
  if (rssi_samples_.size() > MAX_RSSI_SAMPLES) rssi_samples_.pop();
}

// Calculate average RSSI
int8_t ESPNowMesh::get_average_rssi() {
  if (rssi_samples_.empty()) return -70;
  int32_t sum = 0;
  size_t count = rssi_samples_.size();
  std::queue<int8_t> temp = rssi_samples_;
  while (!temp.empty()) {
    sum += temp.front();
    temp.pop();
  }
  return static_cast<int8_t>(sum / count);
}

// Scan channels for interference
uint8_t ESPNowMesh::scan_channels() {
  int8_t best_rssi = -127;
  uint8_t best_channel = current_channel_;
  for (uint8_t ch = 1; ch <= 13; ch++) {
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    delay(100);
    int8_t rssi = get_average_rssi();
    if (rssi > best_rssi) {
      best_rssi = rssi;
      best_channel = ch;
    }
  }
  return best_channel;
}

// Generate authentication challenge
void ESPNowMesh::generate_challenge(uint8_t *challenge) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 255);
  for (int i = 0; i < 8; i++) challenge[i] = dis(gen);
}

// Verify authentication response
bool ESPNowMesh::verify_response(const uint8_t *node_id, const uint8_t *response, const uint8_t *challenge) {
  uint8_t expected[16];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, (const unsigned char *)network_key_.c_str(), network_key_.size());
  mbedtls_md_hmac_update(&ctx, challenge, 8);
  mbedtls_md_hmac_update(&ctx, node_id, 3);
  mbedtls_md_hmac_finish(&ctx, expected);
  mbedtls_md_free(&ctx);
  return memcmp(expected, response, 16) == 0;
}

// Encrypt network key
void ESPNowMesh::encrypt_network_key(const uint8_t *key, uint8_t *output) {
  uint8_t nonce[13] = {0};
  memcpy(nonce, node_id_, 3);
  uint32_t timestamp = millis();
  memcpy(nonce + 3, &timestamp, 4);
  memcpy(nonce + 7, &nonce_counter_++, 4);
  mbedtls_ccm_encrypt_and_tag(&ccm_ctx_, 16, nonce, 12, nullptr, 0, key, output, output + 16, 8);
}

// Compress data for transmission
void ESPNowMesh::compress_data(const uint8_t *input, uint8_t input_len, uint8_t *output) {
  for (uint8_t i = 0; i < input_len && i < 8; i++) output[i] = input[i];
}

// Decompress received data
void ESPNowMesh::decompress_data(const uint8_t *input, uint8_t input_len, std::vector<float> &output) {
  output.resize(input_len / 2);
  for (uint8_t i = 0; i < input_len / 2; i++) {
    uint16_t val = (input[i * 2] << 8) | input[i * 2 + 1];
    output[i] = val / 10.0;
  }
}

// Detect network conflicts (placeholder)
void ESPNowMesh::detect_network_conflict() {
}

// Load keys from NVS
void ESPNowMesh::load_network_keys() {
  nvs_handle_t handle;
  esp_err_t err = nvs_open("espnow_mesh", NVS_READONLY, &handle);
  if (err != ESP_OK) return;
  size_t key_len = 16;
  char key[16];
  err = nvs_get_blob(handle, "network_key", key, &key_len);
  if (err == ESP_OK) network_key_ = std::string(key, key_len);
  err = nvs_get_blob(handle, "prev_key", key, &key_len);
  if (err == ESP_OK) network_key_previous_ = std::string(key, key_len);
  nvs_close(handle);
}

// Save keys to NVS
void ESPNowMesh::save_network_keys() {
  nvs_handle_t handle;
  esp_err_t err = nvs_open("espnow_mesh", NVS_READWRITE, &handle);
  if (err != ESP_OK) return;
  nvs_set_blob(handle, "network_key", network_key_.data(), 16);
  if (!network_key_previous_.empty()) nvs_set_blob(handle, "prev_key", network_key_previous_.data(), 16);
  nvs_commit(handle);
  nvs_close(handle);
}

// Get Wi-Fi channel from AP
uint8_t ESPNowMesh::get_wifi_channel() { 
  return WiFi.channel(); 
}

// Encrypt message data
void ESPNowMesh::encrypt_data_(uint8_t *data, size_t len, uint8_t *nonce) {
  if (network_key_.empty()) return;
  unsigned char output[sizeof(MeshMessage)];
  size_t safe_len = std::min(len, sizeof(MeshMessage));
  mbedtls_ccm_encrypt_and_tag(&ccm_ctx_, safe_len, nonce, 12, nullptr, 0, data, output, data + safe_len, 8);
  memcpy(data, output, safe_len);
}

// Verify encrypted message
bool ESPNowMesh::verify_key(const uint8_t *data, size_t len, const uint8_t *nonce) {
  unsigned char output[sizeof(MeshMessage)];
  size_t safe_len = std::min(len, sizeof(MeshMessage));
  int ret_current = mbedtls_ccm_auth_decrypt(&ccm_ctx_, safe_len, nonce, 12, nullptr, 0, data, output, data + safe_len, 8);
  if (ret_current == 0) return true;
  if (!network_key_previous_.empty()) {
    int ret_previous = mbedtls_ccm_auth_decrypt(&ccm_ctx_previous_, safe_len, nonce, 12, nullptr, 0, data, output, data + safe_len, 8);
    return ret_previous == 0;
  }
  return false;
}
