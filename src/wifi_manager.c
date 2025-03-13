#include "wifi_manager.h"
#include "esp_netif.h"
#include <string.h>

static int retry_count = 0;
EventGroupHandle_t wifi_event_group;

// Event Handler for Wi-Fi events
static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_count < MAX_RETRIES) {
            esp_wifi_connect();
            retry_count++;
            ESP_LOGI("WiFi", "Retrying connection (%d/%d)", retry_count, MAX_RETRIES);
        } else {
            ESP_LOGI("WiFi", "Failed to connect to Wi-Fi");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("WiFi", "Connected! IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0; // Reset retry count
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Initialize Wi-Fi in Station Mode
void wifi_init_sta() {
    // Initialize TCP/IP stack
    esp_netif_init();
    wifi_event_group = xEventGroupCreate();

    // Initialize the event loop
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    // Initialize Wi-Fi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // Register event handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, 
                                        &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, 
                                        &wifi_event_handler, NULL, &instance_got_ip);

    // Configure Wi-Fi settings
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    // Start Wi-Fi in station mode
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI("WiFi", "Wi-Fi initialization completed.");
}

void set_dns() {
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif) {
        esp_netif_dns_info_t dns;
        dns.ip.u_addr.ip4.addr = esp_ip4addr_aton("8.8.8.8");  // Google DNS
        dns.ip.type = ESP_NETIF_DNS_MAIN;
        esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns);
        ESP_LOGI("DNS", "Set DNS to 8.8.8.8");
    } else {
        ESP_LOGE("DNS", "Failed to get netif handle");
    }
}
