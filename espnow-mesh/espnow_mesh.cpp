#include "espnow_mesh.h"

namespace esphome {
namespace espnow_mesh {

static const char *TAG = "espnow_mesh";  // Logging tag

void ESPNowMesh::setup() {
  WiFi.mode(WIFI_STA);  // Set to station mode for ESP-NOW
  if (esp_now_init() != ESP_OK) {
    ESP_LOGE(TAG, "ESP-NOW initialization failed for node %s", node_id_.c_str());
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
  if (switch_pin_ != 255) {
    pinMode(switch_pin_, OUTPUT);
    digitalWrite(switch_pin_, LOW);
  }
  ESP_LOGI(TAG, "ESP-NOW Mesh initialized for node %s", node_id_.c_str());
}

void ESPNowMesh::loop() {}

void ESPNowMesh::send_message(const std::string &data) {
  std::string message = node_id_ + ":" + std::to_string(0) + ":" + data;
  encrypt_data_(message);
  uint32_t jitter = random(0, 100);  // Jitter to avoid collisions
  delay(jitter);
  const std::string dest_id = "gateway";
  if (routing_table_.find(dest_id) == routing_table_.end()) {
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_send(broadcast_mac, (uint8_t *)message.c_str(), message.length());
    ESP_LOGD(TAG, "Broadcasting due to unknown route: %s", message.c_str());
  } else {
    RouteEntry &route = routing_table_[dest_id];
    add_peer_if_needed_(route.next_hop);
    awaiting_ack_ = true;
    uint8_t retries = 0;
    do {
      esp_now_send(route.next_hop, (uint8_t *)message.c_str(), message.length());
      uint32_t timeout = millis() + 200;
      while (awaiting_ack_ && millis() < timeout) {
        yield();
      }
      if (!awaiting_ack_) {
        ESP_LOGD(TAG, "Sent message to %s via %02X:%02X:%02X:%02X:%02X:%02X",
                 dest_id.c_str(), route.next_hop[0], route.next_hop[1], route.next_hop[2],
                 route.next_hop[3], route.next_hop[4], route.next_hop[5]);
        return;
      }
      retries++;
      delay(calculate_backoff_(retries));
    } while (retries < MAX_RETRIES);
    ESP_LOGW(TAG, "Failed to send to %s after %d retries", dest_id.c_str(), retries);
    routing_table_.erase(dest_id);
  }
}

void ESPNowMesh::on_recv_callback_(const uint8_t *mac_addr, const uint8_t *data, int len) {
  std::string message((char *)data, len);
  decrypt_data_(message);
  std::vector<std::string> parts;
  size_t pos = 0;
  while ((pos = message.find(":")) != std::string::npos) {
    parts.push_back(message.substr(0, pos));
    message.erase(0, pos + 1);
  }
  parts.push_back(message);
  if (parts.size() < 3) {
    ESP_LOGW(TAG, "Invalid message format received");
    return;
  }
  std::string source_id = parts[0];
  uint8_t hops = atoi(parts[1].c_str());
  std::string payload = parts[2];
  if (source_id == node_id_) return;
  update_routing_table_(source_id, mac_addr, hops + 1);
  if (is_gateway_) {
    if (payload.find("SWITCH") == 0) {
      process_switch_command_(payload);
    } else {
      last_message_ = payload;
      ESP_LOGD(TAG, "Gateway received: %s", payload.c_str());
    }
  } else if (is_relay_ && hops < MAX_HOPS) {
    if (payload.find("SWITCH") == 0 && parts.size() == 4 && parts[3] == node_id_) {
      process_switch_command_(payload);
    } else {
      static uint32_t last_forward = 0;
      if (millis() - last_forward > 50) {
        forward_message_(source_id + ":" + std::to_string(hops + 1) + ":" + payload, "gateway");
        last_forward = millis();
      }
    }
  }
}

void ESPNowMesh::on_sent_callback_(const uint8_t *mac_addr, esp_now_send_status_t status) {
  awaiting_ack_ = (status != ESP_NOW_SEND_SUCCESS);
  if (status != ESP_NOW_SEND_SUCCESS) {
    ESP_LOGW(TAG, "Send failed to %02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1],
             mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  }
}

void ESPNowMesh::encrypt_data_(std::string &data) {
  for (size_t i = 0; i < data.length(); i++) {
    data[i] ^= encryption_key_[i % encryption_key_.length()];
  }
}

void ESPNowMesh::decrypt_data_(std::string &data) {
  encrypt_data_(data);
}

void ESPNowMesh::update_routing_table_(const std::string &source_id, const uint8_t *source_mac, uint8_t hops) {
  if (routing_table_.find(source_id) == routing_table_.end() || routing_table_[source_id].hops > hops) {
    RouteEntry entry;
    memcpy(entry.next_hop, source_mac, 6);
    entry.hops = hops;
    entry.peer_added = false;
    routing_table_[source_id] = entry;
    ESP_LOGD(TAG, "Updated route to %s via %02X:%02X:%02X:%02X:%02X:%02X, hops=%d",
             source_id.c_str(), source_mac[0], source_mac[1], source_mac[2],
             source_mac[3], source_mac[4], source_mac[5], hops);
  }
}

bool ESPNowMesh::forward_message_(const std::string &message, const std::string &dest_id) {
  if (routing_table_.find(dest_id) == routing_table_.end()) {
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_send(broadcast_mac, (uint8_t *)message.c_str(), message.length());
    ESP_LOGD(TAG, "Broadcasting due to unknown route: %s", message.c_str());
    return false;
  }
  RouteEntry &route = routing_table_[dest_id];
  add_peer_if_needed_(route.next_hop);
  awaiting_ack_ = true;
  uint8_t retries = 0;
  do {
    esp_now_send(route.next_hop, (uint8_t *)message.c_str(), message.length());
    uint32_t timeout = millis() + 200;
    while (awaiting_ack_ && millis() < timeout) {
      yield();
    }
    if (!awaiting_ack_) {
      ESP_LOGD(TAG, "Forwarded to %s via %02X:%02X:%02X:%02X:%02X:%02X",
               dest_id.c_str(), route.next_hop[0], route.next_hop[1], route.next_hop[2],
               route.next_hop[3], route.next_hop[4], route.next_hop[5]);
      return true;
    }
    retries++;
    delay(calculate_backoff_(retries));
  } while (retries < MAX_RETRIES);
  ESP_LOGW(TAG, "Failed to forward to %s after %d retries", dest_id.c_str(), retries);
  routing_table_.erase(dest_id);
  uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_send(broadcast_mac, (uint8_t *)message.c_str(), message.length());
  return false;
}

bool ESPNowMesh::add_peer_if_needed_(const uint8_t *mac_addr) {
  for (auto &entry : routing_table_) {
    if (memcmp(entry.second.next_hop, mac_addr, 6) == 0 && entry.second.peer_added) {
      return true;
    }
  }
  esp_now_peer_info_t peer_info = {};
  memcpy(peer_info.peer_addr, mac_addr, 6);
  peer_info.channel = 0;
  peer_info.encrypt = false;
  if (esp_now_add_peer(&peer_info) == ESP_OK) {
    for (auto &entry : routing_table_) {
      if (memcmp(entry.second.next_hop, mac_addr, 6) == 0) {
        entry.second.peer_added = true;
      }
    }
    ESP_LOGD(TAG, "Added peer %02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1],
             mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    return true;
  }
  ESP_LOGW(TAG, "Failed to add peer");
  return false;
}

uint32_t ESPNowMesh::calculate_backoff_(uint8_t retry_count) {
  return BACKOFF_BASE_MS * (1 << retry_count);
}

void ESPNowMesh::process_switch_command_(const std::string &payload) {
  if (payload == "SWITCH:ON" && switch_) {
    write_switch_state(true);
  } else if (payload == "SWITCH:OFF" && switch_) {
    write_switch_state(false);
  }
}

void ESPNowMesh::write_switch_state(bool state) {
  if (switch_pin_ != 255) {
    digitalWrite(switch_pin_, state ? HIGH : LOW);
  }
  if (switch_) {
    switch_->publish_state(state);
  }
  if (is_relay_) {
    std::string message = "SWITCH:" + std::string(state ? "ON" : "OFF");
    forward_message_(node_id_ + ":0:" + message, "gateway");
  }
}
}  // namespace espnow_mesh
}  // namespace esphome
