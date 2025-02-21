#include "espnow_mesh.h"
#include <cstring>

namespace esphome {
namespace espnow_mesh {

static const char *TAG = "espnow_mesh";

void promiscuous_rx_callback(void *buf, wifi_promiscuous_pkt_type_t type) {
  ESPNowMesh *self = (ESPNowMesh *)App.get_component("espnow_mesh");
  if (self && type == WIFI_PKT_DATA) {
    self->channel_activity_count_++;
  }
}

void ESPNowMesh::setup() {
  WiFi.mode(WIFI_STA);

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  memcpy(node_id_, mac + 3, 3);
  node_id_str_ = std::string(reinterpret_cast<char *>(mac), 6);

  if (esp_now_init() != ESP_OK) {
    ESP_LOGE(TAG, "ESP-NOW initialization failed for node %02X:%02X:%02X (MAC: %s)",
             node_id_[0], node_id_[1], node_id_[2], node_id_str_.c_str());
    return;
  }
  esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
    ESPNowMesh *self = (ESPNowMesh *)App.get_component("espnow_mesh");
    if (self) self->on_recv_callback_(mac, data, len);
  });
  esp_now_register_send_cb([](const uint8_t *mac, esp_now_send_status_t status) {
    ESPNowMesh *self = (ESPNowMesh *)App.get_component("espnow_mesh");
    if (self) self->on_sent_callback_(mac, status);
  });
  uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_peer_info_t peer_info = {};
  memcpy(peer_info.peer_addr, broadcast_mac, 6);
  peer_info.channel = 0;
  peer_info.encrypt = false;
  esp_now_add_peer(&peer_info);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_callback);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

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

  if (!mesh_relay_) {
    send_join_request();
  }

  ESP_LOGI(TAG, "ESP-NOW Mesh initialized for node %02X:%02X:%02X, relay: %d, battery: %d",
           node_id_[0], node_id_[1], node_id_[2], mesh_relay_, battery_powered_);
}

void ESPNowMesh::loop() {
  static uint32_t last_reset = 0;
  if (millis() >= last_reset + 1000) {
    channel_activity_count_ = 0;
    last_reset = millis();
  }
  static uint32_t last_prune = 0;
  if (millis() >= last_prune + 60000) {
    prune_stale_nodes();
    last_prune = millis();
  }
}

void ESPNowMesh::send_message(const MeshMessage &msg) {
  MeshMessage send_msg = msg;
  send_msg.hops = 0;
  memcpy(send_msg.node_id, node_id_, 3);

  uint8_t buffer[sizeof(MeshMessage)];
  memcpy(buffer, &send_msg, sizeof(MeshMessage));
  encrypt_data_(buffer, sizeof(MeshMessage));

  uint8_t *target_mac = nullptr;
  if (routing_table_.find(gateway_id_) == routing_table_.end()) {
    static uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    target_mac = broadcast_mac;
  } else {
    RouteEntry &route = routing_table_[gateway_id_];
    add_peer_if_needed_(route.next_hop);
    target_mac = route.next_hop;
  }

  awaiting_ack_ = true;
  uint8_t retries = 0;
  do {
    uint32_t busy_start = millis();
    while (is_channel_busy_() && millis() - busy_start < 1000) delay(random(5, 20));
    if (is_channel_busy_()) {
      ESP_LOGW(TAG, "Channel busy timeout for %02X:%02X:%02X", node_id_[0], node_id_[1], node_id_[2]);
      break;
    }
    esp_now_send(target_mac, buffer, sizeof(MeshMessage));
    uint32_t start = millis();
    uint32_t timeout = start + 200;
    while (awaiting_ack_ && millis() < timeout) yield();
    if (!awaiting_ack_) {
      ESP_LOGD(TAG, "Message sent successfully to gateway from %02X:%02X:%02X", node_id_[0], node_id_[1], node_id_[2]);
      if (on_message_sent_) on_message_sent_();
      return;
    }
    retries++;
    uint32_t backoff = calculate_backoff_(retries);
    ESP_LOGW(TAG, "Retry %d/%d after %d ms due to no ACK for %02X:%02X:%02X", retries, MAX_RETRIES, backoff, node_id_[0], node_id_[1], node_id_[2]);
    delay(backoff);
  } while (retries < MAX_RETRIES);

  ESP_LOGW(TAG, "Failed to send message to gateway from %02X:%02X:%02X after %d retries", node_id_[0], node_id_[1], node_id_[2], retries);
  if (target_mac != broadcast_mac) routing_table_.erase(gateway_id_);
}

void ESPNowMesh::send_data(uint8_t endpoint, const std::vector<float> &values) {
  MeshMessage msg;
  msg.msg_type = 1;
  msg.node_type = battery_powered_ ? 1 : 0;
  msg.entity_type = ENTITY_SENSOR;
  msg.endpoint = endpoint;
  msg.data_len = values.size() * sizeof(float);
  memcpy(msg.data, values.data(), msg.data_len);
  send_message(msg);
}

void ESPNowMesh::send_switch_state(uint8_t endpoint, bool state) {
  MeshMessage msg;
  msg.msg_type = 2;
  msg.node_type = battery_powered_ ? 1 : 0;
  msg.entity_type = ENTITY_SWITCH;
  msg.endpoint = endpoint;
  msg.data_len = 1;
  msg.data[0] = state ? 1 : 0;
  send_message(msg);
}

void ESPNowMesh::on_recv_callback_(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (len < sizeof(MeshMessage)) return;

  MeshMessage msg;
  memcpy(&msg, data, sizeof(MeshMessage));
  decrypt_data_((uint8_t *)&msg, sizeof(MeshMessage));

  if (memcmp(msg.node_id, node_id_, 3) == 0) return;

  update_routing_table_(msg.node_id, mac_addr, msg.hops + 1);

  if (is_gateway_) {
    if (on_message_received_) on_message_received_();
    last_message_ = std::string(reinterpret_cast<char *>(msg.data), msg.data_len);
    std::string node_id_str = std::string(reinterpret_cast<char *>(msg.node_id), 3);
    switch (msg.msg_type) {
      case 0:  // JOIN
        ESP_LOGD(TAG, "Node %s joined as %s", node_id_str.c_str(), msg.node_type ? "battery" : "mains");
        break;
      case 1:  // DATA
        if (msg.entity_type == ENTITY_SENSOR && msg.data_len >= sizeof(float) && msg.endpoint >= 1 && msg.endpoint <= 240) {
          float value;
          memcpy(&value, msg.data, sizeof(float));
          register_node_sensor(node_id_str, msg.endpoint, "", "");
          dynamic_sensors_[node_id_str + "_" + std::to_string(msg.endpoint)]->publish_state(value);
        }
        break;
      case 2:  // CONTROL
        if (msg.entity_type == ENTITY_SWITCH && msg.data_len == 1 && msg.endpoint >= 1 && msg.endpoint <= 240) {
          register_node_switch(node_id_str, msg.endpoint);
          dynamic_switches_[node_id_str + "_" + std::to_string(msg.endpoint)]->publish_state(msg.data[0]);
        }
        break;
    }
  } else if (mesh_relay_ && msg.hops < MAX_HOPS) {
    if (msg.msg_type == 2 && msg.data_len == 1 && msg.entity_type == ENTITY_SWITCH && 
        (msg.endpoint - 1) < switches_.size()) {
      process_switch_command_(msg);
    } else {
      static uint32_t last_forward = 0;
      if (millis() >= last_forward + 50) {
        MeshMessage fwd_msg = msg;
        fwd_msg.hops++;
        forward_message_(fwd_msg);
        last_forward = millis();
      }
    }
  }
}

void ESPNowMesh::write_switch_state(bool state, uint8_t endpoint) {
  if (endpoint >= 1 && (endpoint - 1) < switches_.size()) {
    switches_[endpoint - 1]->write_state(state);
    if (mesh_relay_) {
      send_switch_state(endpoint, state);
    }
  }
}

void ESPNowMesh::register_node_sensor(const std::string &node_id, uint8_t endpoint, const std::string &unit, const std::string &device_class) {
  if (!is_gateway_) return;
  std::string entity_id = node_id + "_" + std::to_string(endpoint);
  if (dynamic_sensors_.find(entity_id) == dynamic_sensors_.end()) {
    sensor::Sensor *sensor = new sensor::Sensor();
    sensor->set_name(node_id + " Sensor " + std::to_string(endpoint));
    sensor->set_unit_of_measurement(unit);
    sensor->set_device_class(device_class);
    sensor->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
    sensor->set_object_id(entity_id);
    App.register_sensor(sensor);
    dynamic_sensors_[entity_id] = sensor;
    ESP_LOGD(TAG, "Registered sensor %s", entity_id.c_str());
  }
}

void ESPNowMesh::register_node_switch(const std::string &node_id, uint8_t endpoint) {
  if (!is_gateway_) return;
  std::string entity_id = node_id + "_" + std::to_string(endpoint);
  if (dynamic_switches_.find(entity_id) == dynamic_switches_.end()) {
    switch_::Switch *sw = new switch_::Switch();
    sw->set_name(node_id + " Switch " + std::to_string(endpoint));
    sw->set_device_class("switch");
    sw->set_object_id(entity_id);
    sw->add_on_state_callback([this, entity_id, endpoint](bool state) {
      if (this->is_gateway_) send_switch_state(endpoint, state);
    });
    App.register_switch(sw);
    dynamic_switches_[entity_id] = sw;
    ESP_LOGD(TAG, "Registered switch %s", entity_id.c_str());
  }
}

void ESPNowMesh::send_join_request() {
  MeshMessage msg;
  msg.msg_type = 0;
  msg.node_type = battery_powered_ ? 1 : 0;
  msg.entity_type = 0;  // Unused for JOIN
  msg.endpoint = 0;     // Unused for JOIN
  msg.data_len = 0;
  send_message(msg);
}

void ESPNowMesh::prune_stale_nodes() {
  uint32_t now = millis();
  for (auto it = routing_table_.begin(); it != routing_table_.end();) {
    uint32_t timeout = (it->second.node_type == 1) ? BATTERY_TIMEOUT_MS : MAINS_TIMEOUT_MS;
    if (now >= it->second.last_seen + timeout) {
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
      ESP_LOGD(TAG, "Pruning stale node %02X:%02X:%02X", it->first[0], it->first[1], it->first[2]);
      it = routing_table_.erase(it);
    } else {
      ++it;
    }
  }
}

void ESPNowMesh::update_routing_table_(const uint8_t *source_id, const uint8_t *source_mac, uint8_t hops) {
  std::array<uint8_t, 3> id;
  memcpy(id.data(), source_id, 3);
  RouteEntry &entry = routing_table_[id];
  memcpy(entry.next_hop, source_mac, 6);
  entry.hops = hops;
  entry.node_type = msg.node_type;
  entry.last_seen = millis();
  if (!entry.peer_added) {
    add_peer_if_needed_(entry.next_hop);
    entry.peer_added = true;
  }
}

bool ESPNowMesh::forward_message_(const MeshMessage &msg) {
  send_message(msg);
  return true;  // Simplified; could add error checking
}

void ESPNowMesh::process_switch_command_(const MeshMessage &msg) {
  if (msg.data_len == 1 && (msg.endpoint - 1) < switches_.size()) {
    switches_[msg.endpoint - 1]->write_state(msg.data[0]);
    if (mesh_relay_) {
      send_switch_state(msg.endpoint, msg.data[0]);
    }
  }
}

bool ESPNowMesh::add_peer_if_needed_(const uint8_t *mac_addr) {
  esp_now_peer_info_t peer_info = {};
  memcpy(peer_info.peer_addr, mac_addr, 6);
  peer_info.channel = 0;
  peer_info.encrypt = false;
  return esp_now_add_peer(&peer_info) == ESP_OK;
}

uint32_t ESPNowMesh::calculate_backoff_(uint8_t retry_count) {
  uint32_t backoff = BACKOFF_BASE_MS << retry_count;
  return std::min(backoff, BACKOFF_MAX_MS);
}

bool ESPNowMesh::is_channel_busy_() {
  return channel_activity_count_ > 10;  // Threshold for busy channel
}

void ESPNowMesh::encrypt_data_(uint8_t *data, size_t len) {
  if (encryption_key_.empty()) return;
  // Placeholder: XOR with key (simplified, replace with AES-128)
  for (size_t i = 0; i < len; i++) {
    data[i] ^= encryption_key_[i % encryption_key_.length()];
  }
}

void ESPNowMesh::decrypt_data_(uint8_t *data, size_t len) {
  if (encryption_key_.empty()) return;
  // Same as encrypt (symmetric XOR)
  for (size_t i = 0; i < len; i++) {
    data[i] ^= encryption_key_[i % encryption_key_.length()];
  }
}

void ESPNowMesh::on_sent_callback_(const uint8_t *mac_addr, esp_now_send_status_t status) {
  awaiting_ack_ = (status != ESP_NOW_SEND_SUCCESS);
}

}  // namespace espnow_mesh
}  // namespace esphome
