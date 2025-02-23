#include "espnow_mesh.h"
#include <mbedtls/md.h>

#define MAX_HOPS 5          // Maximum hops between nodes (v1.0)
#define BACKOFF_BASE_MS 10  // Base backoff time in ms for collision avoidance (v1.0)
#define MAX_BUFFER_SIZE 10  // Maximum queue size for prioritized traffic (v1.0)
#define KEY_ROTATION_INTERVAL_MS 86400000  // 24 hours in milliseconds (v1.0)
#define INTERFERENCE_CHECK_INTERVAL_MS 2000  // 2 seconds (v1.0)
#define TOPOLOGY_UPDATE_INTERVAL_MS 60000    // 60 seconds (v1.0)
#define AWAKE_DURATION_MS 1000   // 1 second awake (v1.0)
#define SLEEP_DURATION_MS 61000  // 61 seconds sleep (v1.0)
#define JOIN_LISTEN_DURATION_MS 5000  // 5 seconds (v1.0)
#define CHANNEL_SWITCH_TIMEOUT_MS 5000  // 5 seconds (v1.0)
#define CHANNEL_ACK_THRESHOLD 0.9  // 90% ACKs for channel switch (v1.0)
#define RSSI_INTERFERENCE_THRESHOLD -85  // RSSI threshold for interference (v1.0)
#define MAX_RETRIES 3  // Maximum retries for message sending (v1.0)
#define MAX_RSSI_SAMPLES 10  // Maximum RSSI samples for averaging (v1.0)

// Setup: initialize hardware and HA
void ESPNowMesh::setup() {
  if (role_ == ROLE_COORDINATOR && use_wifi_channel_) {
    WiFi.mode(WIFI_STA);
    uint32_t start = millis();
    while (!WiFi.isConnected() && millis() - start < 5000) { delay(100); }
    current_channel_ = WiFi.isConnected() ? get_wifi_channel() : (channel_saved_ ? saved_channel_ : 1);
    if (WiFi.isConnected()) saved_channel_ = current_channel_, channel_saved_ = true;
  } else {
    WiFi.mode(WIFI_OFF);
    current_channel_ = channel_saved_ ? saved_channel_ : 1;
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
    }
    if (network_key_.empty()) {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<> dis(0, 255);
      network_key_.resize(16);
      for (int i = 0; i < 16; i++) network_key_[i] = dis(gen);
      save_network_keys();
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
  }
  if (!network_key_previous_.empty()) {
    mbedtls_ccm_init(&ccm_ctx_previous_);
    mbedtls_ccm_setkey(&ccm_ctx_previous_, MBEDTLS_CCM_ENCRYPT, (const unsigned char *)network_key_previous_.c_str(), 128);
  }
  esp_read_mac(node_id_, ESP_MAC_WIFI_STA);
  register_services();
}

// Loop: runtime logic
void ESPNowMesh::loop() {
  if (role_ == ROLE_COORDINATOR && millis() - last_key_rotation_time_ >= KEY_ROTATION_INTERVAL_MS) {
    rotate_network_key();
    last_key_rotation_time_ = millis();
  }
  if (role_ == ROLE_COORDINATOR && millis() - last_channel_check_ >= INTERFERENCE_CHECK_INTERVAL_MS) {
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
      }
    }
    last_channel_check_ = millis();
  }
  if (role_ == ROLE_COORDINATOR && channel_switch_start_time_ != 0) {
    static uint8_t total_retries = 0;
    size_t total_nodes = routing_table_.size();
    size_t acks_received = total_nodes - pending_channel_acks_.size();
    float ack_ratio = total_nodes > 0 ? static_cast<float>(acks_received) / total_nodes : 1
