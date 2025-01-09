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

#define BROADCAST_LISTEN_INTERVAL_MS 100  // Short delay to avoid overloading
#define ONE_DATA_PACKET_SEND_INTERVAL_MS 100  
#define TIMEOUT_ASSIGN_TASK_MS 6000
#define TIMEOUT_REQUEST_DATA_TASK_MS 6000
#define ACK_LISTEN_TIMEOUT_MS 100
#define MAX_NODES 20
#define JOIN_REQUEST_BUF "Open"
#define MAX_RETRIES 3

typedef struct {
    uint8_t id;
    uint8_t count;
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
            ESP_LOGI(TAG, "Node %d updated. Lat: %.1f, Lon: %.1f, Temp: %.1f, Humidity: %.1f", id, latitude, longitude, t, d);
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
        nodes[node_count].id = node_count+1;
        nodes[node_count].latitude = latitude;
        nodes[node_count].longitude = longitude;
        nodes[node_count].t = t;
        nodes[node_count].d = d;
        nodes[node_count].last_seen = xTaskGetTickCount();
        node_count++;
        ESP_LOGI(TAG, "Node %d added to the network. Lat: %.1f, Lon: %.1f, Temp: %.1f, Humidity: %.1f", id, latitude, longitude, t, d);
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
        while ((xTaskGetTickCount() - start_wait) < pdMS_TO_TICKS(ACK_LISTEN_TIMEOUT_MS)) {
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
    int send_len = sprintf((char *)buf, "%d %d %.1f %.1f %.1f %.1f", id, node_count, T_min, T_max, H_min, H_max);
    send_with_ack((char *)buf, id);
}

void task_lora_gateway(void *pvParameters) {
    ESP_LOGI(TAG, "Gateway task started.");


    // -------------------------------------------------------Assign phase-------------------------------------------------------
    uint8_t buf[256];
    TickType_t start_time = xTaskGetTickCount();
    while (1) {
        TickType_t elapsed_time = xTaskGetTickCount() - start_time;

        if (elapsed_time < pdMS_TO_TICKS(TIMEOUT_ASSIGN_TASK_MS)) {

            // Broadcast message
            int send_len = sprintf((char *)buf, JOIN_REQUEST_BUF);
            lora_send_packet((uint8_t *)buf, send_len);
            ESP_LOGI(TAG, "Broadcasted: %s (length: %d bytes)", buf, send_len);

            // Listen for responses until timeout
            TickType_t listen_start_time = xTaskGetTickCount();
            while ((xTaskGetTickCount() - listen_start_time) < pdMS_TO_TICKS(BROADCAST_LISTEN_INTERVAL_MS)) {
                lora_receive();
                if (lora_received()) {
                    int rxLen = lora_receive_packet(buf, sizeof(buf));
                    buf[rxLen] = '\0'; // Null-terminate for safe string handling
                    ESP_LOGI(TAG, "Received: %s", buf);

                    if (strncmp((char *)buf, "1", 1) == 0) {
                        uint8_t node_id;
                        float latitude, longitude, t = -1, d = -1;
                        if (sscanf((char *)buf, "%hhd %f %f", &node_id, &latitude, &longitude) >= 3) {
                            send_ack(node_id);
                            add_node(node_id, latitude, longitude, t, d);
                            send_accept_packet(node_id);
                            break;
                        } else {
                            ESP_LOGW(TAG, "Invalid assign packet format.");
                        }
                    }
                }
                
                vTaskDelay(pdMS_TO_TICKS(BROADCAST_LISTEN_INTERVAL_MS)); // Short delay to avoid overloading
            }
        } 
        // else {
        //     ESP_LOGI(TAG, "Timeout reached. Resetting node lists.");
        //     reset_nodes();
        //     start_time = xTaskGetTickCount();
        // }
    }
    // -------------------------------------------------------ENd assign phase-------------------------------------------------------


    // -------------------------------------------------------Request data phase-------------------------------------------------------
    TickType_t start_time = xTaskGetTickCount();
    while (1) {
        TickType_t elapsed_time = xTaskGetTickCount() - start_time;

        if (elapsed_time < pdMS_TO_TICKS(TIMEOUT_REQUEST_DATA_TASK_MS)) {
            for (int i = 0; i < node_count; i++) {
                uint8_t buf[256];
                int retries = 0;
                uint8_t node_id = nodes[i].id;

                // Send "id R" request to node
                int send_len = sprintf((char *)buf, "%d R", node_id);
                lora_send_packet((uint8_t *)buf, send_len);
                ESP_LOGI(TAG, "Sent request to node %d: %s", node_id, buf);

                // Listen for ACK
                TickType_t ack_start_time = xTaskGetTickCount();
                int ack_received = 0;

                while ((xTaskGetTickCount() - ack_start_time) < pdMS_TO_TICKS(ACK_LISTEN_TIMEOUT_MS)) {
                    lora_receive();
                    if (lora_received()) {
                        int rxLen = lora_receive_packet(buf, sizeof(buf));
                        buf[rxLen] = '\0'; // Null-terminate for safe string handling
                        if (sscanf((char *)buf, "%hhu ACK", &node_id) == 1) {
                            ESP_LOGI(TAG, "ACK received from node %d.", node_id);
                            ack_received = 1;
                            break;
                        }
                    }
                }

                if (!ack_received) {
                    ESP_LOGW(TAG, "No ACK received from node %d. Retrying...", node_id);
                    if (++retries > MAX_RETRIES) {
                        ESP_LOGE(TAG, "Node %d did not respond after %d retries. Skipping.", node_id, MAX_RETRIES);
                        continue;
                    }
                    i--; // Retry the same node
                    continue;
                }

                // Listen for data packet
                TickType_t data_start_time = xTaskGetTickCount();
                int data_received = 0;
                float t = -1, d = -1;

                while ((xTaskGetTickCount() - data_start_time) < pdMS_TO_TICKS(ONE_DATA_PACKET_SEND_INTERVAL_MS)) {
                    lora_receive();
                    if (lora_received()) {
                        int rxLen = lora_receive_packet(buf, sizeof(buf));
                        buf[rxLen] = '\0'; // Null-terminate for safe string handling
                        if (sscanf((char *)buf, "%hhu %f %f", &node_id, &t, &d) == 3) {
                            ESP_LOGI(TAG, "Data received from node %d: Temp=%.1f, Humidity=%.1f", node_id, t, d);
                            if (node_id == nodes[i].id) {
                                nodes[i].t = t;
                                nodes[i].d = d;
                                data_received = 1;
                                break;
                            }
                        }
                    }
                }

                if (!data_received) {
                    ESP_LOGW(TAG, "No data received from node %d within timeout.", node_id);
                    continue;
                }

                // Send "Ok" packet
                send_len = sprintf((char *)buf, "%d Ok", node_id);
                lora_send_packet((uint8_t *)buf, send_len);
                ESP_LOGI(TAG, "Sent 'Ok' to node %d.", node_id);

                // Listen for ACK to "Ok"
                ack_start_time = xTaskGetTickCount();
                ack_received = 0;

                while ((xTaskGetTickCount() - ack_start_time) < pdMS_TO_TICKS(ACK_LISTEN_TIMEOUT_MS)) {
                    lora_receive();
                    if (lora_received()) {
                        int rxLen = lora_receive_packet(buf, sizeof(buf));
                        buf[rxLen] = '\0';
                        if (sscanf((char *)buf, "%hhu ACK", &node_id) == 1) {
                            ESP_LOGI(TAG, "ACK received for 'Ok' from node %d.", node_id);
                            ack_received = 1;
                            break;
                        }
                    }
                }

                if (!ack_received) {
                    ESP_LOGW(TAG, "No ACK received for 'Ok' from node %d. Retrying...", node_id);
                    if (++retries > MAX_RETRIES) {
                        ESP_LOGE(TAG, "Node %d did not respond to 'Ok' after %d retries. Skipping.", node_id, MAX_RETRIES);
                    } else {
                        i--; // Retry the same node
                    }
                }
            }
        }
    }
    // -------------------------------------------------------End request data phase-------------------------------------------------------

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
