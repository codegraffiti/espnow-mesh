#include "espnow_mesh.h"

// Setup function: initialize ESP32 hardware, ESP-NOW, and HA integration
void ESPNowMesh::setup() {
  // Configure Wi-Fi for Coordinator if using Wi-Fi channel
  if (role_ == ROLE_COORDINATOR && use_wifi_channel_) {
    WiFi.mode(WIFI_STA);  // Set Wi-Fi to Station mode for HA uplink
    uint32_t start = millis();  // Record start time for timeout
    while (!WiFi.isConnected() && millis() - start < 5000) { delay(100); }  // Wait up to 5s for connection
    current_channel_ = WiFi.isConnected() ? get_wifi_channel() : (channel_saved_ ? saved_channel_ : 1);  // Set ESP-NOW channel
    if (WiFi.isConnected()) saved_channel_ = current_channel_, channel_saved_ = true;  // Save channel if connected
  } else {
    WiFi.mode(WIFI_OFF);  // Disable Wi-Fi for Routers/End Devices
    current_channel_ = channel_saved_ ? saved_channel_ : 1;  // Default to channel 1 if no saved channel
  }

  // Set ESP-NOW channel
  esp_wifi_set_channel(current_channel_, WIFI_SECOND_CHAN_NONE);  // Configure ESP-NOW channel (no secondary channel)

  // Enable promiscuous mode for RSSI monitoring
  esp_wifi_set_promiscuous(true);  // Turn on promiscuous mode to capture all packets
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_callback);  // Register callback for RSSI data

  // Initialize ESP-NOW
  uint32_t start = millis();  // Record start time for timeout
  while (esp_now_init() != ESP_OK && millis() - start < 5000) { delay(100); }  // Wait up to 5s for ESP-NOW init

  // Initialize sensor values vector
  last_sensor_values_.resize(5, 0.0);  // Reserve space for 5 sensor values (default 0.0)

  // Initialize Non-Volatile Storage (NVS) for key persistence
  esp_err_t err = nvs_flash_init();  // Attempt to initialize NVS
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {  // Check for errors
    nvs_flash_erase();  // Erase NVS if no free pages or version mismatch
    nvs_flash_init();   // Retry initialization
  }

  // Load existing network keys from NVS
  load_network_keys();  // Retrieve saved keys (if any)

  // Coordinator-specific setup
  if (role_ == ROLE_COORDINATOR) {
    // Generate random network ID if not set
    if (network_id_ == 0xFFFF) {  // Check if network_id is unset (default)
      std::random_device rd;  // Random device for seeding
      std::mt19937 gen(rd()); // Mersenne Twister generator
      std::uniform_int_distribution<> dis(0, 65535);  // Range for 16-bit ID
      network_id_ = dis(gen);  // Assign random ID
      while (detected_network_ids_.find(network_id_) != detected_network_ids_.end()) network_id_ = dis(gen);  // Ensure uniqueness
    }

    // Generate network key if not set
    if (network_key_.empty()) {  // Check if network key is unset
      std::random_device rd;  // Random device for seeding
      std::mt19937 gen(rd()); // Mersenne Twister generator
      std::uniform_int_distribution<> dis(0, 255);  // Range for 8-bit bytes
      network_key_.resize(16);  // Resize to 16 bytes for AES-128
      for (int i = 0; i < 16; i++) network_key_[i] = dis(gen);  // Fill with random bytes
      save_network_keys();  // Persist new key to NVS
    }

    // Register HA sensors for mesh information
    node_count_sensor_->set_name("ESPNow Mesh Node Count");  // Set friendly name for HA
    node_count_sensor_->set_unit_of_measurement("devices");  // Define unit as "devices"
    node_count_sensor_->set_icon("mdi:counter");  // Set HA icon for UI
    App.register_sensor(node_count_sensor_);  // Register with ESPHome

    mesh_topology_sensor_->set_name("ESPNow Mesh Topology");  // Set friendly name for HA
    mesh_topology_sensor_->set_icon("mdi:network");  // Set HA icon for UI
    App.register_sensor(mesh_topology_sensor_);  // Register with ESPHome

    network_health_sensor_->set_name("ESPNow Mesh Network Health");  // Set friendly name for HA
    network_health_sensor_->set_device_class("connectivity");  // Set HA device class (on/off)
    App.register_binary_sensor(network_health_sensor_);  // Register with ESPHome

    update_mesh_info();  // Perform initial update of mesh info
  }

  // Initialize AES-128 CCM context with current key
  if (!network_key_.empty()) {  // Check if key exists
    mbedtls_ccm_init(&ccm_ctx_);  // Initialize CCM context
    mbedtls_ccm_setkey(&ccm_ctx_, MBEDTLS_CCM_ENCRYPT, (const unsigned char *)network_key_.c_str(), 128);  // Set 128-bit key
  }

  // Initialize AES-128 CCM context with previous key (if any)
  if (!network_key_previous_.empty()) {  // Check if previous key exists
    mbedtls_ccm_init(&ccm_ctx_previous_);  // Initialize CCM context
    mbedtls_ccm_setkey(&ccm_ctx_previous_, MBEDTLS_CCM_ENCRYPT, (const unsigned char *)network_key_previous_.c_str(), 128);  // Set 128-bit key
  }

  // Read MAC address as node ID
  esp_read_mac(node_id_, ESP_MAC_WIFI_STA);  // Get 3-byte node ID from MAC

  // Register HA services (e.g., remove_node)
  register_services();  // Call service registration function
}

// Main loop: handle ongoing mesh operations
void ESPNowMesh::loop() {
  // Rotate network key every 24 hours (Coordinator only)
  if (role_ == ROLE_COORDINATOR && millis() - last_key_rotation_time_ >= KEY_ROTATION_INTERVAL_MS) {
    rotate_network_key();  // Generate and broadcast new key
    last_key_rotation_time_ = millis();  // Update timestamp
  }

  // Check for interference every 2 seconds (Coordinator only)
  if (role_ == ROLE_COORDINATOR && millis() - last_channel_check_ >= INTERFERENCE_CHECK_INTERVAL_MS) {
    update_mesh_info();  // Update HA mesh info entities
    int8_t avg_rssi = get_average_rssi();  // Calculate average RSSI
    if (avg_rssi < RSSI_INTERFERENCE_THRESHOLD) {  // Check if below threshold (-85 dBm)
      uint8_t new_channel = scan_channels();  // Find a clear channel
      if (new_channel != current_channel_) {  // If channel change needed
        current_channel_ = new_channel;  // Update current channel
        esp_wifi_set_channel(current_channel_, WIFI_SECOND_CHAN_NONE);  // Set new channel
        send_channel_switch(current_channel_);  // Broadcast channel switch
        pending_channel_acks_ = std::set<std::array<uint8_t, 3>>();  // Reset pending acks
        for (const auto &entry : routing_table_) pending_channel_acks_.insert(entry.first);  // Add all nodes
        channel_switch_start_time_ = millis();  // Record start time
      }
    }
    last_channel_check_ = millis();  // Update timestamp
  }

  // Handle ongoing channel switch (Coordinator only)
  if (role_ == ROLE_COORDINATOR && channel_switch_start_time_ != 0) {
    static uint8_t total_retries = 0;  // Retry counter for channel switch
    size_t total_nodes = routing_table_.size();  // Total nodes in network
    size_t acks_received = total_nodes - pending_channel_acks_.size();  // Nodes that acknowledged
    float ack_ratio = total_nodes > 0 ? static_cast<float>(acks_received) / total_nodes : 1.0;  // Ack percentage
    if (pending_channel_acks_.empty() || ack_ratio >= CHANNEL_ACK_THRESHOLD || 
        millis() - channel_switch_start_time_ >= CHANNEL_SWITCH_TIMEOUT_MS * (total_retries + 1)) {
      esp_wifi_set_channel(current_channel_, WIFI_SECOND_CHAN_NONE);  // Finalize channel switch
      channel_switch_start_time_ = 0;  // Reset switch process
      total_retries = 0;  // Reset retries
    } else if (millis() - channel_switch_start_time_ >= CHANNEL_SWITCH_TIMEOUT_MS) {
      if (total_retries < 1) {  // Retry once if not enough acks
        send_channel_switch(current_channel_);  // Resend switch command
        channel_switch_start_time_ = millis();  // Update timestamp
        total_retries++;  // Increment retries
      }
    }
  }

  // Send periodic beacons (Coordinator or Router)
  if ((role_ == ROLE_COORDINATOR || (role_ == ROLE_ROUTER && !acting_as_coordinator_)) && 
      millis() - last_channel_beacon_time_ >= get_beacon_interval()) {
    MeshMessage beacon_msg;  // Create beacon message
    beacon_msg.msg_type = MSG_CHANNEL;  // Set type to channel beacon
    beacon_msg.role = role_;  // Set sender role
    beacon_msg.data_len = 2;  // 2 bytes of data
    beacon_msg.network_id = network_id_;  // Set network ID
    memset(beacon_msg.dest_node_id, 0xFF, 3);  // Broadcast to all nodes
    beacon_msg.data[0] = current_channel_;  // Current channel
    beacon_msg.data[1] = permit_join_ ? 1 : 0;  // Join permission flag
    send_message(beacon_msg);  // Send beacon
    last_channel_beacon_time_ = millis();  // Update timestamp
  }

  // Handle End Device sleep/wake cycle
  if (role_ == ROLE_END_DEVICE && node_joined_) {
    static uint32_t last_wake = 0;  // Track last wake time
    if (millis() - last_wake >= AWAKE_DURATION_MS + SLEEP_DURATION_MS) {  // Check if cycle complete (62s)
      std::vector<float> values = {25.5, 60.2, 500.0, 1.0, 0.0};  // Example sensor data
      send_data(1, values);  // Send data to Coordinator
      last_wake = millis();  // Update wake time
      esp_light_sleep_start();  // Enter light sleep (61s)
    }
  } else if (role_ == ROLE_END_DEVICE && !node_joined_) {  // Handle joining process
    join_attempts_++;  // Increment join attempts
    if (join_attempts_ >= max_join_attempts_) {  // Check if max attempts reached (720 = 24h)
      App.enter_deep_sleep();  // Enter deep sleep if failed
      return;
    }
    current_channel_ = saved_channel_ ? saved_channel_ : 1;  // Set channel (saved or default)
    esp_wifi_set_channel(current_channel_, WIFI_SECOND_CHAN_NONE);  // Apply channel
    send_join_request();  // Send join request
    uint32_t start = millis();  // Record start time
    while (millis() - start < JOIN_LISTEN_DURATION_MS && !channel_found_) { yield(); delay(1); }  // Wait for response
    last_scan_retry_ = millis();  // Update retry timestamp
    if (!channel_found_) App.enter_deep_sleep(JOIN_LISTEN_INTERVAL_MS);  // Sleep if no channel found
  }

  // Process queued messages
  if (!local_buffer_.empty()) {  // Check if queue has messages
    MeshMessage msg = local_buffer_.top().msg;  // Get highest priority message
    local_buffer_.pop();  // Remove from queue
    send_message(msg);  // Retry sending
  }

  // Request topology update every 60s (Coordinator only)
  if (role_ == ROLE_COORDINATOR && millis() - last_topology_update_ >= TOPOLOGY_UPDATE_INTERVAL_MS) {
    request_topology();  // Send topology request
    update_mesh_info();  // Update HA topology data
    last_topology_update_ = millis();  // Update timestamp
  }
}

// Send an ESP-NOW message with retries and queuing
void ESPNowMesh::send_message(const MeshMessage &msg) {
  if (node_joined_ && msg.msg_type != MSG_JOIN) {  // Ensure node is joined (except for join requests)
    MeshMessage enhanced_msg = msg;  // Copy message for modification
    enhanced_msg.network_id = network_id_;  // Set network ID
    uint8_t retries = 0;  // Retry counter
    do {
      uint8_t *target_mac = nullptr;  // Placeholder for target MAC (to be implemented)
      if (msg.msg_type == MSG_DATA && parent_rssi_ != -127 && enhanced_msg.data_len < MeshMessage::MAX_DATA_LEN) {
        uint8_t compressed_data[10];  // Buffer for compressed data
        compress_data(enhanced_msg.data, enhanced_msg.data_len, compressed_data);  // Compress payload
        memcpy(enhanced_msg.data, compressed_data, 8);  // Copy compressed data
        enhanced_msg.data_len = 8;  // Set new length
        enhanced_msg.data[7] = static_cast<uint8_t>(parent_rssi_);  // Append RSSI
      }
      uint8_t buffer[sizeof(MeshMessage)];  // Buffer for encrypted message
      memcpy(buffer, &enhanced_msg, sizeof(MeshMessage));  // Copy message to buffer
      uint8_t nonce[13] = {0};  // 13-byte nonce for encryption
      memcpy(nonce, node_id_, 3);  // First 3 bytes: node ID
      memcpy(nonce + 3, &nonce_counter_++, 4);  // Next 4 bytes: nonce counter
      encrypt_data_(buffer, sizeof(MeshMessage), nonce);  // Encrypt the message
      esp_now_send(target_mac, buffer, sizeof(MeshMessage));  // Send via ESP-NOW
      uint32_t start = millis();  // Record start time
      while (awaiting_ack_ && millis() - start < 200) yield();  // Wait for ack (200ms timeout)
      if (!awaiting_ack_) return;  // Exit if acknowledged
      retries++;  // Increment retries
      if (retries == MAX_RETRIES) {  // Check if max retries reached
        if (target_mac != broadcast_mac) routing_table_[{coordinator_id_[0], coordinator_id_[1], coordinator_id_[2]}].retry_count++;  // Update retry count
        if (local_buffer_.size() < MAX_BUFFER_SIZE) local_buffer_.push({enhanced_msg});  // Queue if space available
        scan_for_coordinator_channel();  // Scan for Coordinator if failed
        return;
      }
    } while (retries < MAX_RETRIES && millis() - start < 5000);  // Retry up to 5s
  }
}

// Register HA services (e.g., node removal)
void ESPNowMesh::register_services() {
  if (role_ == ROLE_COORDINATOR) {  // Only for Coordinator
    register_service(&ESPNowMesh::remove_node_service, "remove_node", {"node_id"});  // Register remove_node service
  }
}

// HA service to remove a node from the network
void ESPNowMesh::remove_node_service(const std::vector<uint8_t> &node_id) {
  if (node_id.size() != 3) {  // Validate node_id length (3 bytes)
    ESP_LOGE(TAG, "Invalid node_id size: %u", node_id.size());  // Log error
    return;
  }
  std::array<uint8_t, 3> target_id;  // Target node ID
  memcpy(target_id.data(), node_id.data(), 3);  // Copy node_id to array
  if (routing_table_.erase(target_id)) {  // Remove from routing table
    banned_nodes_.insert(target_id);  // Add to banned list
    MeshMessage remove_msg;  // Create removal message
    remove_msg.msg_type = MSG_REMOVE;  // Set type
    remove_msg.role = ROLE_COORDINATOR;  // Set sender role
    remove_msg.data_len = 3;  // 3 bytes of data (node ID)
    remove_msg.network_id = network_id_;  // Set network ID
    remove_msg.hops_role = (0 << 4) | role_;  // Initialize hops to 0
    memset(remove_msg.dest_node_id, 0xFF, 3);  // Broadcast to all
    memcpy(remove_msg.data, target_id.data(), 3);  // Set target ID
    send_message(remove_msg);  // Send removal message
    ESP_LOGI(TAG, "Node %02X:%02X:%02X removed and banned", target_id[0], target_id[1], target_id[2]);  // Log success
  }
}

// Update HA mesh information entities
void ESPNowMesh::update_mesh_info() {
  if (role_ != ROLE_COORDINATOR) return;  // Only for Coordinator
  node_count_sensor_->publish_state(routing_table_.size());  // Publish node count
  StaticJsonDocument<4096> doc;  // 4 KB JSON document for topology (500 nodes)
  JsonArray nodes = doc.createNestedArray("nodes");  // Array for node data
  for (const auto &entry : routing_table_) {  // Iterate over routing table
    JsonObject node = nodes.createNestedObject();  // Create node object
    node["id"] = String(entry.first[0], HEX) + ":" + String(entry.first[1], HEX) + ":" + String(entry.first[2], HEX);  // Node ID
    node["role"] = entry.second.is_router ? "Router" : "End Device";  // Node role
    node["rssi"] = entry.second.rssi;  // RSSI value
    node["parent"] = String(entry.second.next_hop[0], HEX) + ":" + 
                     String(entry.second.next_hop[1], HEX) + ":" + 
                     String(entry.second.next_hop[2], HEX);  // Parent ID
  }
  String topology_json;  // String to hold JSON
  serializeJson(doc, topology_json);  // Serialize to string
  mesh_topology_sensor_->publish_state(topology_json);  // Publish topology
  network_health_sensor_->publish_state(get_average_rssi() >= RSSI_INTERFERENCE_THRESHOLD);  // Publish health status
}

// Placeholder implementations for remaining methods (to be fully implemented)
uint8_t ESPNowMesh::get_wifi_channel() { return WiFi.channel(); }  // Get current Wi-Fi channel
void ESPNowMesh::encrypt_data_(uint8_t *data, size_t len, uint8_t *nonce) {
  if (network_key_.empty()) return;  // Skip if no key
  unsigned char output[sizeof(MeshMessage)];  // Output buffer
  size_t safe_len = std::min(len, sizeof(MeshMessage));  // Ensure safe length
  mbedtls_ccm_encrypt_and_tag(&ccm_ctx_, safe_len, nonce, 12, nullptr, 0, data, output, data + safe_len, 8);  // Encrypt
  memcpy(data, output, safe_len);  // Copy encrypted data back
}
bool ESPNowMesh::verify_key(const uint8_t *data, size_t len, const uint8_t *nonce) {
  unsigned char output[sizeof(MeshMessage)];  // Output buffer
  size_t safe_len = std::min(len, sizeof(MeshMessage));  // Ensure safe length
  int ret_current = mbedtls_ccm_auth_decrypt(&ccm_ctx_, safe_len, nonce, 12, nullptr, 0, data, output, data + safe_len, 8);  // Try current key
  if (ret_current == 0) return true;  // Success with current key
  if (!network_key_previous_.empty()) {  // Check previous key if exists
    int ret_previous = mbedtls_ccm_auth_decrypt(&ccm_ctx_previous_, safe_len, nonce, 12, nullptr, 0, data, output, data + safe_len, 8);  // Try previous key
    return ret_previous == 0;  // Return success/failure
  }
  return false;  // No valid key
}
void ESPNowMesh::process_join_request_(const MeshMessage &msg, const uint8_t *mac_addr) {}  // Handle join requests
void ESPNowMesh::process_remove_message_(const MeshMessage &msg) {}  // Handle node removal
void ESPNowMesh::process_channel_message_(const MeshMessage &msg) {}  // Handle channel beacons
void ESPNowMesh::process_channel_ack_(const MeshMessage &msg) {}  // Handle channel switch acks
void ESPNowMesh::process_key_message_(const MeshMessage &msg) {}  // Handle key rotation
void ESPNowMesh::process_topology_message_(const MeshMessage &msg) {}  // Handle topology updates
void ESPNowMesh::send_data(uint8_t endpoint, const std::vector<float> &values) {}  // Send sensor data
void ESPNowMesh::send_join_request() {}  // Send join request
void ESPNowMesh::send_channel_ack(uint8_t channel) {}  // Send channel switch ack
void ESPNowMesh::send_remove_ack(const std::array<uint8_t, 3> &target_id) {}  // Send removal ack
void ESPNowMesh::send_channel_switch(uint8_t channel) {}  // Send channel switch command
void ESPNowMesh::rotate_network_key() {}  // Rotate network key
void ESPNowMesh::request_topology() {}  // Request topology data
void ESPNowMesh::generate_challenge(uint8_t *challenge) {}  // Generate auth challenge
bool ESPNowMesh::verify_response(const uint8_t *node_id, const uint8_t *response, const uint8_t *challenge) { return true; }  // Verify auth response
void ESPNowMesh::encrypt_network_key(const uint8_t *key, uint8_t *output) {}  // Encrypt new network key
void ESPNowMesh::compress_data(const uint8_t *input, uint8_t input_len, uint8_t *output) {}  // Compress data payload
void ESPNowMesh::decompress_data(const uint8_t *input, uint8_t input_len, std::vector<float> &output) {}  // Decompress data
int8_t ESPNowMesh::get_average_rssi() { return -70; }  // Placeholder for RSSI averaging
uint8_t ESPNowMesh::scan_channels() { return 1; }  // Placeholder for channel scanning
void ESPNowMesh::detect_network_conflict() {}  // Detect and resolve network ID conflicts
void ESPNowMesh::load_network_keys() {}  // Load keys from NVS
void ESPNowMesh::save_network_keys() {}  // Save keys to NVS
void ESPNowMesh::promiscuous_rx_callback(void *buf, wifi_promiscuous_pkt_type_t type) {}  // Callback for RSSI monitoring
