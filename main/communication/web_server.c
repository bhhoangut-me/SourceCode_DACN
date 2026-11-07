#include "web_server.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include <string.h>

#include "tasks/control_task.h"
#include "tasks/state_task.h"
#include "drivers/tctr5000.h"

static const char *TAG = "WEB_SERVER";

// Nhúng file dashboard.html vào firmware binary
extern const uint8_t dashboard_html_start[] asm("_binary_dashboard_html_start");
extern const uint8_t dashboard_html_end[]   asm("_binary_dashboard_html_end");

static httpd_handle_t s_http_server = NULL;
static int s_ws_client_fd = -1;

/* =========================================================================
 * 1. HTTP GET / : Trả về trang Web Dashboard HTML
 * ========================================================================= */
static esp_err_t root_get_handler(httpd_req_t *req) {
    const size_t html_len = dashboard_html_end - dashboard_html_start;
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    // Chống trình duyệt lưu cache trang web, luôn tải bản mới nhất từ ESP32
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Expires", "0");
    return httpd_resp_send(req, (const char *)dashboard_html_start, html_len);
}

/* =========================================================================
 * 2. HTTP POST /ota : Nhận file .bin firmware và ghi vào phân vùng OTA
 * ========================================================================= */
static esp_err_t ota_post_handler(httpd_req_t *req) {
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    if (update_partition == NULL) {
        ESP_LOGE(TAG, "Khong tim thay phan vung OTA hop le");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Bat dau ghi OTA vao phan vung: %s (Offset: 0x%08" PRIx32 ", Size: %" PRIu32 " bytes)",
             update_partition->label, update_partition->address, update_partition->size);

    esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin that bai: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char buf[1024];
    int remaining = req->content_len;
    int total_received = 0;

    while (remaining > 0) {
        int recv_len = httpd_req_recv(req, buf, sizeof(buf) < (size_t)remaining ? sizeof(buf) : (size_t)remaining);
        if (recv_len <= 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
                continue; // Thu lai neu timeout nhe
            }
            ESP_LOGE(TAG, "Loi nhan file OTA");
            esp_ota_abort(ota_handle);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        err = esp_ota_write(ota_handle, buf, recv_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write that bai: %s", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        remaining -= recv_len;
        total_received += recv_len;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end that bai: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition that bai: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "=== NAP FIRMWARE OTA THANH CONG! Tong dung luong: %d bytes ===", total_received);
    httpd_resp_sendstr(req, "OTA Success. Rebooting...");

    // Doi 1 giay roi khoi dong lai vao firmware moi
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK;
}

/* =========================================================================
 * 3. WebSocket /ws : Gui telemetry va nhan lenh dieu khien tu Dashboard
 * ========================================================================= */
static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        // Client WebSocket vua ket noi
        s_ws_client_fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "WebSocket client ket noi, fd = %d", s_ws_client_fd);
        return ESP_OK;
    }

    // Nhan frame tu Dashboard
    httpd_ws_frame_t ws_pkt;
    uint8_t buf[128];
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = buf;
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, sizeof(buf) - 1);
    if (ret != ESP_OK) return ret;

    buf[ws_pkt.len] = '\0';
    ESP_LOGI(TAG, "Nhan lenh WebSocket: %s", (char *)buf);

    // Parse lenh JSON tu Dashboard
    cJSON *root = cJSON_Parse((const char *)buf);
    if (root) {
        cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
        if (cmd && cmd->valuestring) {
            if (strcmp(cmd->valuestring, "start") == 0) {
                ESP_LOGI(TAG, "Lenh START tu Dashboard!");
                state_set(ROBOT_STATE_DETECT_OBSTACLE);
            } else if (strcmp(cmd->valuestring, "stop") == 0) {
                ESP_LOGI(TAG, "Lenh STOP tu Dashboard!");
                state_set(ROBOT_STATE_STOPPED);
            } else if (strcmp(cmd->valuestring, "reset") == 0) {
                ESP_LOGI(TAG, "Lenh RESET tu Dashboard!");
                state_set(ROBOT_STATE_WAIT_START);
            } else if (strcmp(cmd->valuestring, "speed") == 0) {
                cJSON *val = cJSON_GetObjectItem(root, "val");
                if (val) {
                    control_set_target_speed((float)val->valuedouble);
                    ESP_LOGI(TAG, "Dat toc do muc tieu moi: %.2f m/s", (float)val->valuedouble);
                }
            } else if (strcmp(cmd->valuestring, "pwm") == 0) {
                cJSON *val = cJSON_GetObjectItem(root, "val");
                if (val) {
                    control_set_manual_pwm((float)val->valuedouble);
                }
            } else if (strcmp(cmd->valuestring, "mode") == 0) {
                cJSON *val = cJSON_GetObjectItem(root, "val");
                if (val && val->valuestring) {
                    bool is_manual = (strcmp(val->valuestring, "manual") == 0);
                    control_set_mode(is_manual);
                }
            }
        }
        cJSON_Delete(root);
    }

    return ESP_OK;
}

/* =========================================================================
 * 4. Ham truyen Telemetry dinh ky qua WebSocket xuong Dashboard
 *    Tu dong phat cho tat ca cac Client WebSocket dang ket noi (Broadcast)
 * ========================================================================= */
void web_server_send_telemetry(const line_sensor_data_t *line_data) {
    if (!s_http_server) return;

    size_t num_clients = 7;
    int client_fds[7];
    if (httpd_get_client_list(s_http_server, &num_clients, client_fds) != ESP_OK || num_clients == 0) {
        return;
    }

    char json_str[320];
    robot_state_t st = state_get_current();
    float v = control_get_current_speed();
    float steer = control_get_steer_angle_deg();
    float servo = control_get_servo_angle_deg();
    float e2 = control_get_lateral_error_mm();
    float rpm = control_get_current_rpm();
    float pwm = control_get_current_pwm();

    snprintf(json_str, sizeof(json_str),
             "{\"v\":%.2f,\"rpm\":%.0f,\"steer\":%.1f,\"servo\":%.1f,\"e2\":%.1f,\"pwm\":%.0f,\"state\":%d,"
             "\"mode\":\"%s\",\"mpwm\":%.0f,"
             "\"adc\":[%d,%d,%d,%d,%d,%d,%d,%d],\"lost\":%s,\"inter\":%s}",
             v, rpm, steer, servo, e2, pwm, (int)st,
             control_is_manual_mode() ? "manual" : "auto",
             control_get_manual_pwm(),
             line_data ? line_data->raw_adc[0] : 0,
             line_data ? line_data->raw_adc[1] : 0,
             line_data ? line_data->raw_adc[2] : 0,
             line_data ? line_data->raw_adc[3] : 0,
             line_data ? line_data->raw_adc[4] : 0,
             line_data ? line_data->raw_adc[5] : 0,
             line_data ? line_data->raw_adc[6] : 0,
             line_data ? line_data->raw_adc[7] : 0,
             (line_data && line_data->line_lost) ? "true" : "false",
             (line_data && line_data->intersection_detected) ? "true" : "false");

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t *)json_str;
    ws_pkt.len = strlen(json_str);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    for (size_t i = 0; i < num_clients; i++) {
        if (httpd_ws_get_fd_info(s_http_server, client_fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
            httpd_ws_send_frame_async(s_http_server, client_fds[i], &ws_pkt);
        }
    }
}

/* =========================================================================
 * 5. Khoi tao SoftAP Wi-Fi va khoi chay HTTP Server
 * ========================================================================= */
static esp_err_t start_web_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 7;
    config.lru_purge_enable = true;

    esp_err_t ret = httpd_start(&s_http_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Khoi dong HTTP server that bai: %d", ret);
        return ret;
    }

    // 1. Endpoint / (Dashboard HTML)
    httpd_uri_t uri_root = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = root_get_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(s_http_server, &uri_root);

    // 2. Endpoint /ws (WebSocket Telemetry)
    httpd_uri_t uri_ws = {
        .uri        = "/ws",
        .method     = HTTP_GET,
        .handler    = ws_handler,
        .user_ctx   = NULL,
        .is_websocket = true
    };
    httpd_register_uri_handler(s_http_server, &uri_ws);

    // 3. Endpoint /ota (Upload Firmware .bin)
    httpd_uri_t uri_ota = {
        .uri       = "/ota",
        .method    = HTTP_POST,
        .handler   = ota_post_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(s_http_server, &uri_ota);

    ESP_LOGI(TAG, "HTTP Server & WebSocket & OTA da san sang tai http://192.168.4.1");
    return ESP_OK;
}

esp_err_t web_server_init(void) {
    // 1. Khoi tao TCP/IP stack va Event Loop
    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    // 2. Tao default Wi-Fi AP netif
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    (void)ap_netif;

    // 3. Khoi tao Wi-Fi Driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 4. Cau hinh che do SoftAP (Robot tu phat Wi-Fi)
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASS,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "=== WI-FI ACCESS POINT DA PHAT ===");
    ESP_LOGI(TAG, "Ten Wi-Fi (SSID): %s", WIFI_AP_SSID);
    ESP_LOGI(TAG, "Mat khau (PASS) : %s", WIFI_AP_PASS);
    ESP_LOGI(TAG, "Dia chi IP      : http://192.168.4.1");

    // 5. Khoi dong Web Server
    return start_web_server();
}

