#include "notify.h"

#include <cstdio>
#include <cstring>

#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/dns.h"
#include "lwip/tcpip.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace {

constexpr const char *kConfigPartition = "wificfg";  // See partitions.csv.
constexpr const char *kConfigNamespace = "wifi";
constexpr const char *kDefaultServer = "https://ntfy.sh";
constexpr int kQueueLength = 4;
// ntfy.sh resolves to several servers in rotation and one of them may refuse
// connections, so each retry looks the name up again (see sender_task).
constexpr int kSendAttempts = 6;
constexpr TickType_t kRetryDelay = pdMS_TO_TICKS(2000);
constexpr TickType_t kWifiWait = pdMS_TO_TICKS(120000);  // Drop a message after this.
constexpr EventBits_t kConnectedBit = BIT0;

struct Config {
  char ssid[33]{};
  char password[65]{};
  char topic[65]{};
  char server[97]{};
};

struct Message {
  char title[64];
  char body[256];
};

Config g_config;
bool g_enabled = false;
QueueHandle_t g_queue = nullptr;
EventGroupHandle_t g_events = nullptr;
SemaphoreHandle_t g_status_lock = nullptr;
esp_ip4_addr_t g_ip{};
char g_last_result[48] = "NONE";

void set_last_result(const char *result) {
  xSemaphoreTake(g_status_lock, portMAX_DELAY);
  std::snprintf(g_last_result, sizeof(g_last_result), "%s", result);
  xSemaphoreGive(g_status_lock);
}

bool read_string(nvs_handle_t handle, const char *key, char *out, size_t size) {
  size_t length = size;
  return nvs_get_str(handle, key, out, &length) == ESP_OK;
}

// SSID and topic are required; the password may be empty (open network) and
// the server defaults to ntfy.sh.
bool load_config() {
  if (nvs_flash_init_partition(kConfigPartition) != ESP_OK) {
    return false;
  }
  nvs_handle_t handle;
  if (nvs_open_from_partition(kConfigPartition, kConfigNamespace, NVS_READONLY, &handle) !=
      ESP_OK) {
    return false;
  }
  const bool ok = read_string(handle, "ssid", g_config.ssid, sizeof(g_config.ssid)) &&
                  read_string(handle, "ntfy_topic", g_config.topic, sizeof(g_config.topic)) &&
                  g_config.ssid[0] != '\0' && g_config.topic[0] != '\0';
  read_string(handle, "password", g_config.password, sizeof(g_config.password));
  if (!read_string(handle, "ntfy_server", g_config.server, sizeof(g_config.server)) ||
      g_config.server[0] == '\0') {
    std::snprintf(g_config.server, sizeof(g_config.server), "%s", kDefaultServer);
  }
  nvs_close(handle);
  return ok;
}

void on_wifi_event(void *, esp_event_base_t base, int32_t id, void *data) {
  if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED)) {
    // Keep trying: the hotspot may come and go during a demo.
    xEventGroupClearBits(g_events, kConnectedBit);
    esp_wifi_connect();
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    g_ip = static_cast<ip_event_got_ip_t *>(data)->ip_info.ip;
    xEventGroupSetBits(g_events, kConnectedBit);
  }
}

void start_wifi() {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&init));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                      on_wifi_event, nullptr, nullptr));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                      on_wifi_event, nullptr, nullptr));
  wifi_config_t wifi = {};
  std::memcpy(wifi.sta.ssid, g_config.ssid, std::strlen(g_config.ssid));
  std::memcpy(wifi.sta.password, g_config.password, std::strlen(g_config.password));
  wifi.sta.threshold.authmode = g_config.password[0] != '\0' ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi));
  ESP_ERROR_CHECK(esp_wifi_start());
}

// POST the message to <server>/<topic>. Sets the HTTP status on success, and
// the underlying TLS/socket error on failure.
esp_err_t post(const Message &message, int *status, esp_err_t *tls_error, int *tls_code) {
  char url[sizeof(Config::server) + sizeof(Config::topic) + 2];
  std::snprintf(url, sizeof(url), "%s/%s", g_config.server, g_config.topic);
  esp_http_client_config_t config = {};
  config.url = url;
  config.method = HTTP_METHOD_POST;
  // ntfy.sh sometimes answers slowly after accepting a message; a short
  // timeout makes the retry send it twice.
  config.timeout_ms = 20000;
  config.crt_bundle_attach = esp_crt_bundle_attach;
  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == nullptr) {
    return ESP_ERR_NO_MEM;
  }
  esp_http_client_set_header(client, "Title", message.title);
  esp_http_client_set_header(client, "Priority", "high");
  esp_http_client_set_post_field(client, message.body, static_cast<int>(std::strlen(message.body)));
  const esp_err_t err = esp_http_client_perform(client);
  *status = esp_http_client_get_status_code(client);
  int tls_flags = 0;
  *tls_error = esp_http_client_get_and_clear_last_tls_error(client, tls_code, &tls_flags);
  esp_http_client_cleanup(client);
  return err;
}

void sender_task(void *) {
  Message message;
  while (true) {
    xQueueReceive(g_queue, &message, portMAX_DELAY);
    if ((xEventGroupWaitBits(g_events, kConnectedBit, pdFALSE, pdTRUE, kWifiWait) &
         kConnectedBit) == 0) {
      set_last_result("NO_WIFI");
      continue;
    }
    char result[48];
    for (int attempt = 1; attempt <= kSendAttempts; ++attempt) {
      std::snprintf(result, sizeof(result), "SENDING_%d", attempt);
      set_last_result(result);
      int status = 0;
      esp_err_t tls_error = ESP_OK;
      int tls_code = 0;
      const esp_err_t err = post(message, &status, &tls_error, &tls_code);
      if (err == ESP_OK) {
        std::snprintf(result, sizeof(result), "HTTP_%d", status);
      } else {
        // e.g. "ESP_ERR_HTTP_CONNECT/0x8006/-0x7280": HTTP, esp-tls and mbedTLS errors.
        std::snprintf(result, sizeof(result), "%s/0x%x/-0x%x", esp_err_to_name(err),
                      static_cast<unsigned>(tls_error), static_cast<unsigned>(-tls_code));
      }
      if (err == ESP_OK && status >= 200 && status < 300) {
        break;
      }
      if (attempt < kSendAttempts) {
        // lwIP caches the address for the record's TTL (2 min for ntfy.sh);
        // forget it so the next attempt can reach a different server.
        tcpip_callback([](void *) { dns_clear_cache(); }, nullptr);
        vTaskDelay(kRetryDelay);
      }
    }
    set_last_result(result);
  }
}

}  // namespace

void notify_start() {
  g_status_lock = xSemaphoreCreateMutex();
  // WiFi keeps its calibration data in the default NVS partition.
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    err = nvs_flash_init();
  }
  if (err != ESP_OK || !load_config()) {
    return;
  }
  g_events = xEventGroupCreate();
  g_queue = xQueueCreate(kQueueLength, sizeof(Message));
  start_wifi();
  // TLS needs a larger stack. Core 0, below the control task, which runs on core 1.
  xTaskCreatePinnedToCore(sender_task, "notify", 8192, nullptr, 2, nullptr, 0);
  g_enabled = true;
}

bool notify_send(const char *title, const char *message) {
  if (!g_enabled) {
    return false;
  }
  Message queued;
  std::snprintf(queued.title, sizeof(queued.title), "%s", title);
  std::snprintf(queued.body, sizeof(queued.body), "%s", message);
  return xQueueSend(g_queue, &queued, 0) == pdTRUE;
}

void notify_status(char *out, size_t size) {
  if (!g_enabled) {
    std::snprintf(out, size, "OFF,-,NONE");
    return;
  }
  const bool connected = (xEventGroupGetBits(g_events) & kConnectedBit) != 0;
  char ip[16] = "-";
  if (connected) {
    std::snprintf(ip, sizeof(ip), IPSTR, IP2STR(&g_ip));
  }
  xSemaphoreTake(g_status_lock, portMAX_DELAY);
  std::snprintf(out, size, "%s,%s,%s,heap=%lu", connected ? "CONNECTED" : "CONNECTING", ip,
                g_last_result, static_cast<unsigned long>(esp_get_free_heap_size()));
  xSemaphoreGive(g_status_lock);
}
