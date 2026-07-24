#include "wifi.h"
#include "config.h"

#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <cstring>

static const char *TAG = "action_cat";

void InitWiFi()
{
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_ap();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  wifi_config_t wcfg = {};
  strcpy((char *)wcfg.ap.ssid, WIFI_SSID);
  wcfg.ap.ssid_len = strlen(WIFI_SSID);
  wcfg.ap.authmode = WIFI_AUTH_OPEN;
  wcfg.ap.max_connection = WIFI_MAX_CONN;

  esp_wifi_set_mode(WIFI_MODE_AP);
  esp_wifi_set_config(WIFI_IF_AP, &wcfg);
  esp_wifi_start();
  ESP_LOGI(TAG, "WiFi AP: %s (open)", WIFI_SSID);
}
