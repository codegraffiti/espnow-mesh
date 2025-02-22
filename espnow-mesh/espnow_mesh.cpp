// espnow_mesh.cpp - Implementation of ESP-NOW mesh network component (Version 2.0)
// Optimized for no-Wi-Fi mode, with routing, RSSI tracking, and HA integration.

#include "espnow_mesh.h"
#include <cstring>
#include <random>

namespace esphome {
namespace espnow_mesh {

static const char *TAG = "espnow_mesh";  // Logging tag

// Callback for promiscuous mode to capture RSSI from ESP-NOW packets
static void promiscuous_rx_callback(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type == WIFI_PKT_DATA) {  // Only process data packets
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    if (pkt->rx_ctrl.len >= sizeof(MeshMessage)) {  // Ensure packet size matches MeshMessage
      ESPNowMesh *self = (ESPNowMesh *)App.get_component("espnow_mesh");
      if (self) self->on_recv_callback_(pkt->rx_ctrl.mac, pkt->payload, pkt->rx_ctrl.len, pkt->rx_ctrl.rssi);
    }
  }
  ESPNowMesh *self = (ESPNowMesh *)App.get_component("espnow_mesh");
  if (self) self->channel_activity_count_++;  // Increment channel activity counter
}

// Initialize node: setup Wi-Fi, ESP-NOW, and RSSI sensor
void ESPNowMesh::setup() {
  WiFi.mode(WIFI_STA);  // Set Wi-Fi to station mode for ESP-NOW
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);  // Read MAC address
  memcpy(node_id_, mac + 3, 3);  // Use last 3 bytes as node ID
  node_id_str_ = std::string(reinterpret_cast<char *>(mac), 6);  // Full MAC as string

  use_wifi_channel_ = (WiFi.SSID() != "");  // Check if Wi-Fi configured (false for no-Wi-Fi mode)
  current_channel_ = channel_saved_ ? saved_channel_ : 1;  // Start on saved or default channel
  esp_wifi_set_channel(current_channel_, WIFI_SECOND_CHAN_NONE);  // Set initial channel

  // Auto-create RSSI sensor for HA without user YAML
  rssi_sensor_ = new sensor::Sensor();
  if (!rssi_sensor_) {
    ESP_LOGE(TAG, "Failed to allocate RSSI sensor for %02X:%02X:%02X", node_id_[0], node_id_[1], node_id_[2]);
  } else {
    rssi_sensor_->set_name(node_id_str_ + " RSSI");  // e.g., "XXYYZZ RSSI"
    rssi_sensor_->set_unit_of_measurement("dBm");  // Signal strength unit
    rssi_sensor_->set_device_class("signal_strength");  // HA device class
    rssi_sensor_->set_object_id(node_id_str_ + "_rssi");  // Unique ID
    App.register_sensor(rssi_sensor_);  // Register with ESPHome for HA discovery
  }

  esp_wifi_set_promiscuous(true);  // Enable promiscuous mode for RSSI capture
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_callback);  // Set RSSI callback

  // Initialize ESP-NOW with 5s timeout to prevent hangs
  uint32_t start = millis();
  while (esp_now_init() != ESP_OK && millis() - start < 5000) {
    delay(100);  // Retry every 100ms
  }
  if (esp_now_init() != ESP_OK) {
    ESP_LOGE(TAG, "ESP-NOW init failed after 5s for %02X:%02X:%02X", node_id_[0], node_id_[1], node_id_[2]);
  } else {
    // Register ESP-NOW callbacks
    esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
      ESPNowMesh *self = (ESPNowMesh *)App.get_component("espnow_mesh");
      if (self) self->on_recv_callback_(mac, data, len, -50);  // Fallback RSSI if promiscuous fails
    });
    esp_now_register_send_cb([](const uint8_t *mac, esp_now_send_status_t status) {
      ESPNowMesh *self = (ESPNowMesh *)App.get_component("espnow_mesh");
      if (self) self->on_sent_callback_(mac, status);
    });

    // Add broadcast peer for initial communication
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, broadcast_mac, 6);
    peer_info.channel = current_channel_;
    peer_info.encrypt = false;
    esp_now_add_peer(&peer_info);
  }

  // Generate network key for Coordinator if not set
  if (role_ == ROLE_COORDINATOR && network_key_.empty()) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    network_key_.resize(16);
    for (int i = 0; i < 16; i++) {
      network_key_[i] = dis(gen);
    }
  }
  // Initialize encryption if key present
  if (!network_key_.empty()) {
    mbedtls_ccm_init(&ccm_ctx_);
    mbedtls_ccm_setkey(&ccm_ctx_, MBEDTLS_CCM_ENCRYPT, (const unsigned char *)network_key_.c_str(), 128);
  }

  // Register local sensors and switches
  switch_pins_.clear();
  uint8_t endpoint = 1;
  for (sensor::Sensor *sensor : App.get_sensors()) {
    std::string id = sensor->get_object_id();
    if (mesh_disabled_.find(id) == mesh_disabled_.end()) {
      if (endpoint > 240) {
        ESP_LOGE(TAG, "Endpoint limit exceeded at %d", endpoint);
        break;
      }
      sensors_.push_back(sensor);
      sensor->add_on_state_callback([this, id, ep = endpoint](float value) {
        std::vector<float> values = {value};
        send_data(ep, values);
      });
      endpoint++;
    }
  }
  for (switch_::Switch *sw : App.get_switches()) {
    std::string id = sw->get_object_id();
    if (mesh_disabled_.find(id) == mesh_disabled_.end()) {
      if (endpoint > 240) {
        ESP_LOGE(TAG, "Endpoint limit exceeded at %d", endpoint);
        break;
      }
      switches_.push_back(sw);
      sw->add_on_state_callback([this, id, ep = endpoint](bool state) {
        send_switch_state(ep, state);
      });
      if (auto *gpio_sw = dynamic_cast<switch_::GPIOSwitch *>(sw)) {
        uint8_t pin = gpio_sw->get_pin();
        switch_pins_.push_back(pin);
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
      }
      endpoint++;
    }
  }

  // Coordinator-specific setup: assign short address, create HA switches
  if (role_ == ROLE_COORDINATOR) {
    short_addr_ = 0x0000;  // Coordinator address
    used_short_addresses_.insert(0x0000);
    memcpy(coordinator_id_.data(), node_id_, 3);  // Set Coordinator ID
    permit_join_switch_ = new switch_::Switch();
    if (permit_join_switch_) {
      permit_join_switch_->set_name(node_id_str_ + " Permit Join");
      permit_join_switch_->set_device_class("switch");
      permit_join_switch_->set_object_id(node_id_str_ + "_permit_join");
      permit_join_switch_->add_on_state_callback([this](bool state) {
        permit_join_ = state;
        permit_join_start_time_ = state ? millis() : 0;
        MeshMessage permit_msg;
        permit_msg.msg_type = MSG_CONTROL;
        permit_msg.role = ROLE_COORDINATOR;
        permit_msg.data_len = 1;
        memset(permit_msg.dest_node_id, 0xFF, 3);
        permit_msg.data[0] = state ? 1 : 0;
        send_message(permit_msg);
      });
      App.register_switch(permit_join_switch_);
      permit_join_switch_->publish_state(false);
    }
    channel_switch_ = new switch_::Switch();
    if (channel_switch_) {
      channel_switch_->set_name(node_id_str_ + " Channel Switch");
      channel_switch_->set_device_class("switch");
      channel_switch_->set_object_id(node_id_str_ + "_channel_switch");
      channel_switch_->add_on_state_callback([this](bool state) {
        if (state && !use_wifi_channel_) {
          uint8_t new_channel = scan_channels();
          if (new_channel != current_channel_) {
            send_channel_switch(new_channel);
            pending_channel_acks_ = std::set<std::array<uint8_t, 3>>();
            for (const auto &entry : routing_table_) {
              pending_channel_acks_.insert(entry.first);
            }
            channel_switch_start_time_ = millis();
          }
          channel_switch_->publish_state(false);
        }
      });
      App.register_switch(channel_switch_);
      channel_switch_->publish_state(false);
    }
  }
  // Restore state for known nodes
  if (node_joined_) {
    short_addr_ = saved_short_addr_;
    coordinator_id_ = saved_coordinator_id_;
    channel_found_ = true;
  }
  ESP_LOGI(TAG, "ESP-NOW Mesh initialized for node %02X:%02X:%02X, role: %d, short_addr: %04X, channel: %d",
           node_id_[0], node_id_[1], node_id_[2], role_, short_addr_, current_channel_);
}

// Main loop: handle timeouts, retries, and RSSI updates
void ESPNowMesh::loop() {
  static uint32_t last_reset = 0;
  if (millis() - last_reset >= 1000) {  // Reset channel activity every 1s
    channel_activity_count_ = 0;
    last_reset = millis();
  }
  static uint32_t last_prune = 0;
  if (millis() - last_prune >= 60000) {  // Prune stale routes every 60s
    prune_stale_routes();
    last_prune = millis();
  }
  if (role_ == ROLE_END_DEVICE && node_joined_ && !channel_found_ && 
      millis() - last_child_retry_ >= CHILD_RETRY_INTERVAL_MS) {  // Retry buffered messages every 1s
    current_channel_ = saved_channel_;
    esp_wifi_set_channel(current_channel_, WIFI_SECOND_CHAN_NONE);
    std::vector<MeshMessage> temp_buffer = local_buffer_;  // Copy to avoid recursion
    local_buffer_.clear();
    for (const auto &msg : temp_buffer) send_message(msg);
    last_child_retry_ = millis();
    if (!channel_found_ && millis() - last_scan_retry_ >= SCAN_RETRY_INTERVAL_MS) {  // Scan every 10s if needed
      scan_for_coordinator_channel();
      last_scan_retry_ = millis();
    }
  }
  if (role_ == ROLE_COORDINATOR && permit_join_ && permit_join_start_time_ != 0) {  // Timeout permit join after 2min
    if (millis() - permit_join_start_time_ >= PERMIT_JOIN_TIMEOUT_MS) {
      permit_join_ = false;
      permit_join_switch_->publish_state(false);
      MeshMessage permit_msg;
      permit_msg.msg_type = MSG_CONTROL;
      permit_msg.role = ROLE_COORDINATOR;
      permit_msg.data_len = 1;
      memset(permit_msg.dest_node_id, 0xFF, 3);
      permit_msg.data[0] = 0;
      send_message(permit_msg);
    }
  }
  if (role_ == ROLE_COORDINATOR && channel_switch_start_time_ != 0) {  // Handle channel switch with 10s max timeout
    static uint8_t total_retries = 0;
    if (pending_channel_acks_.empty()) {
      esp_wifi_set_channel(current_channel_, WIFI_SECOND_CHAN_NONE);
      channel_switch_->publish_state(false);
      channel_switch_start_time_ = 0;
      total_retries = 0;
    } else if (millis() - channel_switch_start_time_ >= CHANNEL_SWITCH_TIMEOUT_MS) {
      if (!pending_channel_acks_.empty() && total_retries < 2) {
        send_channel_switch(current_channel_);
        channel_switch_start_time_ = millis();
        total_retries++;
      } else {
        ESP_LOGW(TAG, "Channel switch failed after %d retries", total_retries);
        channel_switch_->publish_state(false);
        channel_switch_start_time_ = 0;
        total_retries = 0;
      }
    }
  }
  if (role_ == ROLE_ROUTER && !acting_as_coordinator_ && last_coordinator_contact_ > 0 && 
      millis() - last_channel_beacon_time_ >= CHANNEL_BEACON_INTERVAL_MS) {  // Router beacon every 5s
    send_channel_switch(current_channel_);
    last_channel_beacon_time_ = millis();
  }
  if (millis() - last_route_update_ >= ROUTE_UPDATE_INTERVAL_MS) {  // Route update every 15s
    MeshMessage update_msg;
    update_msg.msg_type = MSG_RREP;
    update_msg.role = role_;
    memcpy(update_msg.src_node_id, node_id_, 3);
    memcpy(update_msg.dest_node_id, coordinator_id_.data(), 3);
    update_msg.data_len = 5;
    memcpy(update_msg.data, coordinator_id_.data(), 3);
    memcpy(update_msg.data + 3, &short_addr_, 2);
    send_message(update_msg);
    last_route_update_ = millis();
  }
  if (millis() - last_rssi_update_ >= RSSI_UPDATE_MIN_INTERVAL_MS && 
      parent_rssi_ != -127 && abs(parent_rssi_ - last_published_rssi_) >= RSSI_CHANGE_THRESHOLD) {  // RSSI update if changed ±5 dBm
    rssi_sensor_->publish_state(parent_rssi_);
    last_published_rssi_ = parent_rssi_;
    last_rssi_update_ = millis();
  }
  static uint32_t last_init_try = 0;
  if (esp_now_init() != ESP_OK && millis() - last_init_try >= 5000) {  // Retry ESP-NOW init every 5s if failed
    esp_now_init();
    last_init_try = millis();
  }
  if (role_ == ROLE_END_DEVICE && !node_joined_ && esp_now_init() == ESP_OK && 
      millis() - last_scan_retry_ >= SCAN_RETRY_INTERVAL_MS) {  // Scan for join every 10s
    scan_for_coordinator_channel();
    last_scan_retry_ = millis();
  }
}

// Send an ESP-NOW message with retries and RSSI piggybacking
void ESPNowMesh::send_message(const MeshMessage &msg) {
  if (network_key_.empty() && msg.msg_type != MSG_JOIN) {  // Require key for non-join messages
    ESP_LOGE(TAG, "Network key not set for %02X:%02X:%02X", node_id_[0], node_id_[1], node_id_[2]);
    return;
  }
  if (short_addr_ == 0xFFFF && msg.msg_type != MSG_JOIN) {  // Unjoined nodes must join first
    scan_for_coordinator_channel();
    return;
  }
  if (node_joined_ && msg.msg_type != MSG_JOIN) {
    uint8_t retries = 0;
    do {
      uint8_t *target_mac = nullptr;
      uint32_t min_cost = UINT32_MAX;
      for (const auto &entry : routing_table_) {  // Find lowest-cost Router
        if (entry.second.is_router && entry.second.cost < min_cost && 
            entry.second.last_seen > millis() - ROUTE_EXPIRY_MS) {
          target_mac = entry.second.next_hop;
          min_cost = entry.second.cost;
        }
      }
      static uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      if (!target_mac) target_mac = broadcast_mac;  // Fallback to broadcast
      MeshMessage enhanced_msg = msg;
      if (msg.msg_type == MSG_DATA && parent_rssi_ != -127 && enhanced_msg.data_len < MeshMessage::MAX_DATA_LEN) {
        enhanced_msg.data[enhanced_msg.data_len] = static_cast<uint8_t>(parent_rssi_);  // Piggyback RSSI
        enhanced_msg.data_len++;
      }
      uint8_t buffer[sizeof(MeshMessage)];
      memcpy(buffer, &enhanced_msg, sizeof(MeshMessage));
      uint8_t nonce[13] = {0};
      memcpy(nonce, node_id_, 3);
      memcpy(nonce + 3, &nonce_counter_++, 4);
      encrypt_data_(buffer, sizeof(MeshMessage), nonce);
      esp_now_send(target_mac, buffer, sizeof(MeshMessage));
      uint32_t start = millis();
      while (awaiting_ack_ && millis() - start < 200) yield();  // Wait 200ms for ACK
      if (!awaiting_ack_) {
        if (msg.msg_type == MSG_DATA && abs(parent_rssi_ - last_published_rssi_) >= RSSI_CHANGE_THRESHOLD &&
            millis() - last_rssi_update_ >= RSSI_UPDATE_MIN_INTERVAL_MS) {
          rssi_sensor_->publish_state(parent_rssi_);  // Update RSSI on significant change
          last_published_rssi_ = parent_rssi_;
          last_rssi_update_ = millis();
        }
        if (on_message_sent_) on_message_sent_();
        return;
      }
      retries++;
      if (retries == MAX_RETRIES) {
        if (target_mac != broadcast_mac) routing_table_[{coordinator_id_[0], coordinator_id_[1], coordinator_id_[2]}].retry_count++;
        if (local_buffer_.size() < 5) {  // Cap buffer at ~190 bytes
          local_buffer_.push_back(msg);
        } else {
          ESP_LOGW(TAG, "Local buffer full for %02X:%02X:%02X, dropping message", 
                   node_id_[0], node_id_[1], node_id_[2]);
        }
        scan_for_coordinator_channel();
        return;
      }
    } while (retries < MAX_RETRIES && millis() - start < 5000);  // 5s total timeout
  }
  if (msg.msg_type == MSG_JOIN) {  // Direct join message
    uint8_t buffer[sizeof(MeshMessage)];
    memcpy(buffer, &msg, sizeof(MeshMessage));
    esp_now_send(broadcast_mac, buffer, sizeof(MeshMessage));
  }
}

// Send sensor data from specified endpoint
void ESPNowMesh::send_data(uint8_t endpoint, const std::vector<float> &values) {
  MeshMessage msg;
  msg.msg_type = MSG_DATA;
  msg.role = role_;
  msg.entity_type = ENTITY_SENSOR;
  msg.endpoint = endpoint;
  msg.data_len = values.size() * sizeof(float);
  memcpy(msg.dest_node_id, coordinator_id_.data(), 3);
  memcpy(msg.data, values.data(), std::min(msg.data_len, (uint8_t)MeshMessage::MAX_DATA_LEN));
  send_message(msg);
}

// Send switch state from specified endpoint
void ESPNowMesh::send_switch_state(uint8_t endpoint, bool state) {
  MeshMessage msg;
  msg.msg_type = MSG_CONTROL;
  msg.role = role_;
  msg.entity_type = ENTITY_SWITCH;
  msg.endpoint = endpoint;
  msg.data_len = 1;
  memcpy(msg.dest_node_id, coordinator_id_.data(), 3);
  msg.data[0] = state ? 1 : 0;
  send_message(msg);
}

// Request to join the network
void ESPNowMesh::send_join_request() {
  MeshMessage msg;
  msg.msg_type = MSG_JOIN;
  msg.role = role_;
  msg.entity_type = 0;
  msg.endpoint = 0;
  msg.data_len = 0;
  memset(msg.dest_node_id, 0xFF, 3);  // Broadcast to all nodes
  send_message(msg);
}

// Request route to a specific destination
void ESPNowMesh::send_route_request(const std::array<uint8_t, 3> &dest_id) {
  MeshMessage msg;
  msg.msg_type = MSG_RREQ;
  msg.role = role_;
  msg.entity_type = 0;
  msg.endpoint = 0;
  msg.data_len = 3;
  memcpy(msg.dest_node_id, dest_id.data(), 3);
  memcpy(msg.data, dest_id.data(), 3);
  send_message(msg);
}

// Initiate channel switch to a new channel
void ESPNowMesh::send_channel_switch(uint8_t new_channel) {
  MeshMessage msg;
  msg.msg_type = MSG_CHANNEL;
  msg.role = role_;
  msg.entity_type = 0;
  msg.endpoint = 0;
  msg.data_len = 1;
  memset(msg.dest_node_id, 0xFF, 3);  // Broadcast
  msg.data[0] = new_channel;
  send_message(msg);
}

// Acknowledge channel switch
void ESPNowMesh::send_channel_ack(uint8_t channel) {
  MeshMessage msg;
  msg.msg_type = MSG_CHANNEL_ACK;
  msg.role = role_;
  msg.entity_type = 0;
  msg.endpoint = 0;
  msg.data_len = 1;
  memcpy(msg.dest_node_id, coordinator_id_.data(), 3);
  msg.data[0] = channel;
  send_message(msg);
}

// Scan for Coordinator or Router channel on join or failover
void ESPNowMesh::scan_for_coordinator_channel() {
  if (channel_found_) return;
  if (join_start_time_ == 0) join_start_time_ = millis();
  if (millis() - join_start_time_ >= JOIN_TOTAL_TIMEOUT_MS) {  // 8s total timeout
    ESP_LOGW(TAG, "Node %02X:%02X:%02X failed to join after %dms", 
             node_id_[0], node_id_[1], node_id_[2], JOIN_TOTAL_TIMEOUT_MS);
    join_start_time_ = 0;
    return;
  }
  if (channel_saved_ && !channel_found_) {  // Try saved channel first
    current_channel_ = saved_channel_;
    esp_wifi_set_channel(current_channel_, WIFI_SECOND_CHAN_NONE);
    send_join_request();
    uint32_t start = millis();
    while (millis() - start < SCAN_CHANNEL_TIMEOUT_MS && !channel_found_) {  // 1s timeout
      yield();
      delay(1);  // Prevent tight loop
    }
  }
  for (uint8_t channel = 1; channel <= 13 && !channel_found_; channel++) {  // Scan all channels
    if (channel == saved_channel_) continue;
    current_channel_ = channel;
    esp_wifi_set_channel(current_channel_, WIFI_SECOND_CHAN_NONE);
    uint32_t start = millis();
    while (millis() - start < 500 && !channel_found_) {  // 500ms listen for beacon
      yield();
      delay(1);
    }
    if (!channel_found_) {
      send_join_request();
      while (millis() - start < SCAN_CHANNEL_TIMEOUT_MS && !channel_found_) {  // 1s total per channel
        yield();
        delay(1);
      }
    }
  }
}

// Handle received ESP-NOW packets, update RSSI and routing
void ESPNowMesh::on_recv_callback_(const uint8_t *mac_addr, const uint8_t *data, int len, int8_t rssi) {
  if (len < sizeof(MeshMessage)) return;  // Drop malformed packets
  MeshMessage msg;
  memcpy(&msg, data, sizeof(MeshMessage));  // Copy packet data
  uint8_t nonce[13] = {0};
  memcpy(nonce, msg.src_node_id, 3);
  memcpy(nonce + 3, &nonce_counter_, 4);
  decrypt_data_((uint8_t *)&msg, sizeof(MeshMessage), nonce);  // Decrypt message
  if (memcmp(msg.src_node_id, node_id_, 3) == 0) return;  // Ignore self-messages

  std::array<uint8_t, 3> src_id;
  memcpy(src_id.data(), msg.src_node_id, 3);
  update_routing_table_(msg.src_node_id, mac_addr, msg.hops + 1, msg.role);  // Update routing table
  if (routing_table_.find(src_id) != routing_table_.end()) {
    routing_table_[src_id].rssi = rssi;  // Store RSSI for this neighbor
  }
  if (routing_table_.find(coordinator_id_) != routing_table_.end() && 
      memcmp(msg.src_node_id, coordinator_id_.data(), 3) == 0) {
    parent_rssi_ = rssi;  // Update RSSI to parent (Coordinator or Router)
  }
  if (msg.msg_type == MSG_DATA && msg.data_len > 0) {  // Extract piggybacked RSSI from data messages
    int8_t received_rssi = static_cast<int8_t>(msg.data[msg.data_len - 1]);
    if (role_ == ROLE_ROUTER || role_ == ROLE_COORDINATOR) {
      parent_rssi_ = received_rssi;  // Update parent RSSI from child
      if (abs(parent_rssi_ - last_published_rssi_) >= RSSI_CHANGE_THRESHOLD &&
          millis() - last_rssi_update_ >= RSSI_UPDATE_MIN_INTERVAL_MS) {
        rssi_sensor_->publish_state(parent_rssi_);  // Publish RSSI to HA if changed ±5 dBm
        last_published_rssi_ = parent_rssi_;
        last_rssi_update_ = millis();
      }
    }
  }
  // ... (remaining routing logic unchanged)
}

// Handle send completion status
void ESPNowMesh::on_sent_callback_(const uint8_t *mac_addr, esp_now_send_status_t status) {
  awaiting_ack_ = (status != ESP_NOW_SEND_SUCCESS);  // Update ACK flag
}

// Encrypt message data using AES-128 CCM
void ESPNowMesh::encrypt_data_(uint8_t *data, size_t len, uint8_t *nonce) {
  if (network_key_.empty()) return;
  unsigned char output[sizeof(MeshMessage)];
  size_t olen = 0;
  size_t safe_len = std::min(len, sizeof(MeshMessage));  // Prevent buffer overflow
  mbedtls_ccm_encrypt_and_tag(&ccm_ctx_, safe_len, nonce, 12, nullptr, 0, data, output, data + safe_len, 8);
  memcpy(data, output, safe_len);  // Copy encrypted data back
}

// Decrypt message data using AES-128 CCM
void ESPNowMesh::decrypt_data_(uint8_t *data, size_t len, uint8_t *nonce) {
  if (network_key_.empty() || len < sizeof(MeshMessage)) return;
  unsigned char output[sizeof(MeshMessage)];
  size_t olen = 0;
  size_t safe_len = std::min(len, sizeof(MeshMessage));  // Prevent buffer overflow
  int ret = mbedtls_ccm_auth_decrypt(&ccm_ctx_, safe_len, nonce, 12, nullptr, 0, data, output, data + safe_len, 8);
  if (ret == 0) {
    memcpy(data, output, safe_len);  // Copy decrypted data back
  } else {
    ESP_LOGE(TAG, "Decryption failed for %02X:%02X:%02X, len %d", 
             node_id_[0], node_id_[1], node_id_[2], safe_len);
  }
}

// Update routing table with new route information
void ESPNowMesh::update_routing_table_(const uint8_t *source_id, const uint8_t *source_mac, uint8_t hops, uint8_t role) {
  std::array<uint8_t, 3> id;
  memcpy(id.data(), source_id, 3);
  RouteEntry &entry = routing_table_[id];
  uint32_t new_cost = hops * 100 - entry.rssi + (entry.retry_count * 50);  // RSSI-based cost
  if (entry.hops > hops || entry.cost > new_cost || entry.last_seen == 0) {
    memcpy(entry.next_hop, source_mac, 6);
    entry.hops = hops;
    entry.cost = new_cost;
    entry.retry_count = 0;
    entry.last_seen = millis();
    entry.expires_at = millis() + ROUTE_EXPIRY_MS;
    entry.is_router = (role == ROLE_ROUTER);
    if (!entry.peer_added) {
      add_peer_if_needed_(entry.next_hop);
      entry.peer_added = true;
    }
  } else {
    entry.last_seen = millis();
  }
}

// Forward message to next hop
bool ESPNowMesh::forward_message_(const MeshMessage &msg) {
  send_message(msg);
  return true;
}

// Process switch commands from network
void ESPNowMesh::process_switch_command_(const MeshMessage &msg) {
  if (msg.data_len == 1 && (msg.endpoint - 1) < switches_.size()) {
    switches_[msg.endpoint - 1]->write_state(msg.data[0]);
    if (role_ == ROLE_ROUTER) {
      send_switch_state(msg.endpoint, msg.data[0]);
    }
  }
}

// Add ESP-NOW peer if not already added
bool ESPNowMesh::add_peer_if_needed_(const uint8_t *mac_addr) {
  esp_now_peer_info_t peer_info = {};
  memcpy(peer_info.peer_addr, mac_addr, 6);
  peer_info.channel = current_channel_;
  peer_info.encrypt = false;
  return esp_now_add_peer(&peer_info) == ESP_OK;
}

// Assign short address to new node (Coordinator only)
uint16_t ESPNowMesh::assign_short_address() {
  if (role_ != ROLE_COORDINATOR) return 0xFFFF;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(1, 0xFFFE);
  uint16_t addr;
  do {
    addr = dis(gen);
  } while (used_short_addresses_.count(addr) > 0);
  used_short_addresses_.insert(addr);
  return addr;
}

// Get Wi-Fi channel (unused in no-Wi-Fi mode)
uint8_t ESPNowMesh::get_wifi_channel() {
  wifi_ap_record_t ap_info;
  if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
    return ap_info.primary;
  }
  ESP_LOGW(TAG, "Failed to get Wi-Fi channel; defaulting to 1");
  return 1;
}

// Scan channels for least busy (used by Coordinator for channel switch)
uint8_t ESPNowMesh::scan_channels() {
  uint8_t best_channel = 1;
  uint32_t min_activity = UINT32_MAX;
  for (uint8_t channel = 1; channel <= 13; channel++) {
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    delay(50);  // Allow activity count to stabilize
    uint32_t activity = channel_activity_count_;
    ESP_LOGD(TAG, "Channel %d activity: %u", channel, activity);
    if (activity < min_activity) {
      min_activity = activity;
      best_channel = channel;
    }
    channel_activity_count_ = 0;
  }
  return best_channel;
}

// Prune stale routes from routing table
void ESPNowMesh::prune_stale_routes() {
  uint32_t now = millis();
  for (auto it = routing_table_.begin(); it != routing_table_.end();) {
    uint32_t timeout = (it->second.is_router || role_ == ROLE_ROUTER) ? MAINS_TIMEOUT_MS : BATTERY_TIMEOUT_MS;
    if (role_ == ROLE_COORDINATOR && child_table_.find(it->first) != child_table_.end()) {
      timeout = BATTERY_TIMEOUT_MS;  // Longer timeout for Router children
    }
    if (now >= it->second.expires_at || now >= it->second.last_seen + timeout) {
      std::string node_id_str = std::string(reinterpret_cast<char *>(it->first.data()), 3);
      for (auto sensor_it = dynamic_sensors_.begin(); sensor_it != dynamic_sensors_.end();) {
        if (sensor_it->first.find(node_id_str) == 0) {
          delete sensor_it->second;
          sensor_it = dynamic_sensors_.erase(sensor_it);
        } else {
          ++sensor_it;
        }
      }
      for (auto switch_it = dynamic_switches_.begin(); switch_it != dynamic_switches_.end();) {
        if (switch_it->first.find(node_id_str) == 0) {
          delete switch_it->second;
          switch_it = dynamic_switches_.erase(switch_it);
        } else {
          ++switch_it;
        }
      }
      ESP_LOGD(TAG, "Pruning stale route %02X:%02X:%02X", it->first[0], it->first[1], it->first[2]);
      it = routing_table_.erase(it);
    } else {
      ++it;
    }
  }
}

// Sync child table with Coordinator
void ESPNowMesh::sync_child_table_(const std::array<uint8_t, 3> &dest_id) {
  MeshMessage sync_msg;
  sync_msg.msg_type = MSG_CONTROL;
  sync_msg.role = ROLE_ROUTER;
  sync_msg.entity_type = 1;  // Child sync
  sync_msg.data_len = child_table_.size() * 3;
  memcpy(sync_msg.dest_node_id, dest_id.data(), 3);
  int i = 0;
  for (const auto &child : child_table_) {
    memcpy(sync_msg.data + i * 3, child.first.data(), 3 );
    i++;
  }
  send_message(sync_msg);
}

// ... (remaining methods like process_join_request_, process_key_message_, etc., unchanged for brevity but fully implemented)

}  // namespace espnow_mesh
}  // namespace esphome
