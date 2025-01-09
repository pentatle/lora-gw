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

#define BROADCAST_INTERVAL_MS 100
#define TIMEOUT_MS 6000
#define MAX_NODES 20
#define JOIN_REQUEST_BUF "Open"
#define MAX_RETRIES 3

typedef struct {
    uint8_t id;
    float latitude;
    float longitude;
    float t; // Temperature
    float d; // Humidity
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
    // Identify nodes that are no longer active
    old_node_count = 0;
    for (int i = 0; i < node_count; i++) {
        int active = 0;
        for (int j = 0; j < node_count; j++) {
            if (nodes[i].id == nodes[j].id) {
                active = 1;
                break;
            }
        }
        if (!active && old_node_count < MAX_NODES) {
            old_nodes[old_node_count++] = nodes[i];
        }
    }

    // Clear nodes and reset counters
    memset(nodes, 0, sizeof(nodes));
    node_count = 0;

    // Clear new_nodes and reset counter
    memset(new_nodes, 0, sizeof(new_nodes));
    new_node_count = 0;
}

static void add_node(uint8_t id, float latitude, float longitude, float t, float d) {
    // Check if node is already in the current cycle
    for (int i = 0; i < node_count; i++) {
        if (nodes[i].id == id) {
            nodes[i].latitude = latitude;
            nodes[i].longitude = longitude;
            nodes[i].t = t;
            nodes[i].d = d;
            nodes[i].last_seen = xTaskGetTickCount();
            ESP_LOGI(TAG, "Node %d updated. Lat: %.2f, Lon: %.2f, Temp: %.2f, Humidity: %.2f", id, latitude, longitude, t, d);
            return;
        }
    }

    // Check if node is new
    int is_new = 1;
    for (int i = 0; i < old_node_count; i++) {
        if (old_nodes[i].id == id) {
            is_new = 0;
            break;
        }
    }

    // Add node to the new list if it is new
    if (is_new && new_node_count < MAX_NODES) {
        new_nodes[new_node_count].id = id;
        new_nodes[new_node_count].latitude = latitude;
        new_nodes[new_node_count].longitude = longitude;
        new_nodes[new_node_count].t = t;
        new_nodes[new_node_count].d = d;
        new_nodes[new_node_count].last_seen = xTaskGetTickCount();
        new_node_count++;
    }

    // Add node to the current cycle
    if (node_count < MAX_NODES) {
        nodes[node_count].id = id;
        nodes[node_count].latitude = latitude;
        nodes[node_count].longitude = longitude;
        nodes[node_count].t = t;
        nodes[node_count].d = d;
        nodes[node_count].last_seen = xTaskGetTickCount();
        node_count++;
        ESP_LOGI(TAG, "Node %d added to the network. Lat: %.2f, Lon: %.2f, Temp: %.2f, Humidity: %.2f", id, latitude, longitude, t, d);
    } else {
        ESP_LOGW(TAG, "Node list full, cannot add node %d.", id);
    }
}

static int send_with_ack(const char *message, uint8_t expected_ack_id) {
    uint8_t buf[256];
    int retries = 0;

    while (retries <= MAX_RETRIES) {
        lora_send_packet((uint8_t *)message, strlen(message));
        ESP_LOGI(TAG, "Sent: %s", message);

        TickType_t start_wait = xTaskGetTickCount();
        while ((xTaskGetTickCount() - start_wait) < pdMS_TO_TICKS(1000)) { // 1 second timeout
            lora_receive();
            if (lora_received()) {
                int rxLen = lora_receive_packet(buf, sizeof(buf));
                buf[rxLen] = '\0';
                ESP_LOGI(TAG, "Received: %s", buf);
                if (sscanf((char *)buf, "%hhu ACK", &expected_ack_id) == 1) {
                    return 1; // ACK received
                }
            }
        }

        retries++;
        ESP_LOGW(TAG, "Retry %d for message: %s", retries, message);
    }

    ESP_LOGE(TAG, "Failed to receive ACK after %d retries: %s", MAX_RETRIES, message);
    return 0;
}

static void send_ack(uint8_t id) {
    uint8_t buf[256];
    int send_len = sprintf((char *)buf, "%d ACK", id);
    lora_send_packet(buf, send_len);
    ESP_LOGI(TAG, "Sent ACK to node %d.", id);
}

static void send_accept_packet(uint8_t id) {
    uint8_t buf[256];
    float T_min = 15.0, T_max = 30.0, H_min = 40.0, H_max = 60.0;
    int send_len = sprintf((char *)buf, "%d STT %.2f %.2f %.2f %.2f", id, T_min, T_max, H_min, H_max);
    send_with_ack((char *)buf, id);
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
            lora_send_packet((uint8_t *)buf, send_len);
            ESP_LOGI(TAG, "Broadcasted: %s", buf);

            // Listen for responses
            lora_receive();
            if (lora_received()) {
                int rxLen = lora_receive_packet(buf, sizeof(buf));
                buf[rxLen] = '\0'; // Null-terminate for safe string handling
                ESP_LOGI(TAG, "Received: %s", buf);

                if (strncmp((char *)buf, "id", 2) == 0) {
                    uint8_t node_id;
                    float latitude, longitude, t = -1, d = -1;
                    if (sscanf((char *)buf, "id %hhu %f %f %f %f", &node_id, &latitude, &longitude, &t, &d) >= 3) {
                        send_ack(node_id);
                        add_node(node_id, latitude, longitude, t, d);
                        send_accept_packet(node_id);
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
