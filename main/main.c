/* ESP-IDF Example: LoRa Gateway
 *
 * This code demonstrates an ESP32 acting as a LoRa gateway.
 * It supports broadcasting to allow other nodes to join the network,
 * sending acknowledgments, and managing nodes in a database.
 */

#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "lora.h"

#define BROADCAST_INTERVAL_MS 100 //time per node
#define TIMEOUT_MS 6000 //=3*max node*time per node
#define MAX_NODES 20
#define JOIN_REQUEST_BUF "Open"

typedef struct {
    uint8_t id;
    float latitude;
    float longitude;
    TickType_t last_seen;
} node_info_t;

static node_info_t nodes[MAX_NODES];
static int node_count = 0;
static node_info_t old_nodes[MAX_NODES];
static int old_node_count = 0;
static node_info_t new_nodes[MAX_NODES];
static int new_node_count = 0;
static const char *TAG = "LoRa_Gateway";

static void reset_nodes() {
    // Copy current nodes to old_nodes
    memcpy(old_nodes, nodes, sizeof(nodes));
    old_node_count = node_count;

    // Clear nodes and reset counters
    memset(nodes, 0, sizeof(nodes));
    node_count = 0;

    // Clear new_nodes and reset counter
    memset(new_nodes, 0, sizeof(new_nodes));
    new_node_count = 0;
}

static void add_node(uint8_t id, float latitude, float longitude) {
    // Check if node is already in the current cycle
    for (int i = 0; i < node_count; i++) {
        if (nodes[i].id == id) {
            nodes[i].latitude = latitude;
            nodes[i].longitude = longitude;
            nodes[i].last_seen = xTaskGetTickCount();
            ESP_LOGI(TAG, "Node %d updated. Lat: %.2f, Lon: %.2f", id, latitude, longitude);
            return;
        }
    }

    // Add node to the new list if not already added
    if (new_node_count < MAX_NODES) {
        new_nodes[new_node_count].id = id;
        new_nodes[new_node_count].latitude = latitude;
        new_nodes[new_node_count].longitude = longitude;
        new_nodes[new_node_count].last_seen = xTaskGetTickCount();
        new_node_count++;
    }

    // Add node to the current cycle
    if (node_count < MAX_NODES) {
        nodes[node_count].id = id;
        nodes[node_count].latitude = latitude;
        nodes[node_count].longitude = longitude;
        nodes[node_count].last_seen = xTaskGetTickCount();
        node_count++;
        ESP_LOGI(TAG, "Node %d added to the network. Lat: %.2f, Lon: %.2f", id, latitude, longitude);
    } else {
        ESP_LOGW(TAG, "Node list full, cannot add node %d.", id);
    }
}

void task_lora_gateway(void *pvParameters) {
    ESP_LOGI(TAG, "Gateway task started.");
    uint8_t buf[256];
    TickType_t start_time = xTaskGetTickCount();

    while (1) {
        TickType_t elapsed_time = xTaskGetTickCount() - start_time;

        if (elapsed_time < pdMS_TO_TICKS(TIMEOUT_MS)) {
            // Broadcast message
            int send_len = sprintf((char *)buf, JOIN_REQUEST_BUF);
            lora_send_packet(buf, send_len);
            ESP_LOGI(TAG, "Broadcasted JOIN_REQUEST.");

            // Listen for responses
            lora_receive();
            if (lora_received()) {
                int rxLen = lora_receive_packet(buf, sizeof(buf));
                buf[rxLen] = '\0'; // Null-terminate for safe string handling
                ESP_LOGI(TAG, "Received: %s", buf);

                if (strncmp((char *)buf, "id", 2) == 0) {
                    uint8_t node_id;
                    float latitude, longitude;
                    if (sscanf((char *)buf, "id %hhu %f %f", &node_id, &latitude, &longitude) == 3) {
                        // Send ACK
                        send_len = sprintf((char *)buf, "%d ACK", node_id);
                        lora_send_packet(buf, send_len);
                        ESP_LOGI(TAG, "Sent ACK to node %d.", node_id);

                        // Add node to the database
                        add_node(node_id, latitude, longitude);

                        // Send accept packet
                        float T_min = 15.0, T_max = 30.0, H_min = 40.0, H_max = 60.0;
                        send_len = sprintf((char *)buf, "%d STT %.2f %.2f %.2f %.2f", node_id, T_min, T_max, H_min, H_max);
                        lora_send_packet(buf, send_len);
                        ESP_LOGI(TAG, "Sent accept packet to node %d.", node_id);
                    } else {
                        ESP_LOGW(TAG, "Invalid assign packet format.");
                    }
                }
            }
        } else {
            ESP_LOGI(TAG, "Timeout reached. Resetting node lists.");
            reset_nodes();
            start_time = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(BROADCAST_INTERVAL_MS));
    }
}

void app_main() {
    if (lora_init() == 0) {
        ESP_LOGE(TAG, "LoRa module not recognized.");
        while (1) {
            vTaskDelay(1);
        }
    }

    ESP_LOGI(TAG, "LoRa initialized. Setting parameters...");
    lora_set_frequency(433e6); // 433MHz
    lora_enable_crc();
    lora_set_coding_rate(1);
    lora_set_bandwidth(7);
    lora_set_spreading_factor(7);

    xTaskCreate(&task_lora_gateway, "LoRa_Gateway", 1024 * 4, NULL, 5, NULL);
}
