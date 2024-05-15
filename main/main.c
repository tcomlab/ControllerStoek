/* Captive Portal Example

    This example code is in the Public Domain (or CC0 licensed, at your option.)

    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
*/

#include <sys/param.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "lwip/inet.h"

#include "esp_http_server.h"
#include "dns_server.h"

#include "led_strip.h"
#include "esp_log.h"
#include "esp_err.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

// GPIO assignment
#define LED_STRIP1_BLINK_GPIO  6
#define LED_STRIP2_BLINK_GPIO  7

#define KEY1 GPIO_NUM_2
#define KEY2 GPIO_NUM_3

// Numbers of the LED in the strip
#define LED_STRIP_LED_NUMBERS 56
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

led_strip_handle_t led_strip1;
led_strip_handle_t led_strip2;

#define EXAMPLE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_MAX_STA_CONN CONFIG_ESP_MAX_STA_CONN

extern const char root_start[] asm("_binary_root_html_start");
extern const char root_end[] asm("_binary_root_html_end");

static const char *TAG = "MyApp:";

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

static void wifi_init_softap(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);

    char ip_addr[16];
    inet_ntoa_r(ip_info.ip.addr, ip_addr, 16);
    ESP_LOGI(TAG, "Set up softAP with IP: %s", ip_addr);

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:'%s' password:'%s'",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

// HTTP GET Handler
static esp_err_t root_get_handler(httpd_req_t *req)
{
    const uint32_t root_len = root_end - root_start;

    ESP_LOGI(TAG, "Serve root");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, root_start, root_len);

    return ESP_OK;
}

static const httpd_uri_t root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler
};

// HTTP Error (404) Handler - Redirects all requests to the root page
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    // Set status
    httpd_resp_set_status(req, "302 Temporary Redirect");
    // Redirect to the "/" root directory
    httpd_resp_set_hdr(req, "Location", "/");
    // iOS requires content in the response to detect a captive portal, simply redirecting is not sufficient.
    httpd_resp_send(req, "Redirect to the captive portal", HTTPD_RESP_USE_STRLEN);

    ESP_LOGI(TAG, "Redirecting to root");
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 13;
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &root);
        httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);
    }
    return server;
}

void strip_init()
{
  // LED strip general initialization, according to your led board design
    led_strip_config_t strip1_config = {
        .strip_gpio_num = LED_STRIP1_BLINK_GPIO,   // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_NUMBERS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

    led_strip_config_t strip2_config = {
        .strip_gpio_num = LED_STRIP2_BLINK_GPIO,   // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_NUMBERS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config1 = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
    };

    led_strip_rmt_config_t rmt_config2 = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
    };

    // LED Strip object handle
    
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip1_config, &rmt_config1, &led_strip1));
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip2_config, &rmt_config2, &led_strip2));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    //return led_strip;


}


bool key1_action = false;
bool key2_action = false;

static void IRAM_ATTR isrButtonPress(void* arg)
{
    switch ((uint32_t) arg)
    {
    case KEY1:
        key1_action = true;
        
        break;
    case KEY2:
        key2_action = true;
        break;
    
    default:
        break;
    }
    gpio_intr_disable(KEY1);
    gpio_intr_disable(KEY2);
}

void isr_init()
{
     esp_err_t err = gpio_install_isr_service(0);
  if (err == ESP_ERR_INVALID_STATE) {
    ESP_LOGW("ISR", "GPIO isr service already installed");
  };

  // Настраиваем вывод для кнопки
  gpio_reset_pin(KEY1);
  gpio_set_direction(KEY1, GPIO_MODE_INPUT);
  gpio_set_pull_mode(KEY1, GPIO_FLOATING);
 

  gpio_reset_pin(KEY2);
  gpio_set_direction(KEY2, GPIO_MODE_INPUT);
  gpio_set_pull_mode(KEY2, GPIO_FLOATING);

  // Регистрируем обработчик прерывания на нажатие кнопки
  gpio_isr_handler_add(KEY1, isrButtonPress, (void*) KEY1);
  gpio_isr_handler_add(KEY2, isrButtonPress, (void*) KEY2);
  
  // Устанавливаем тип события для генерации прерывания - по низкому уровню
  gpio_set_intr_type(KEY1, GPIO_INTR_NEGEDGE);
  gpio_set_intr_type(KEY2, GPIO_INTR_NEGEDGE);
  
  // Разрешаем использование прерываний
  gpio_intr_enable(KEY1);
  gpio_intr_enable(KEY2);
}


void welcome_strip()
{
    for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) { 
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip1, i, 0, 255, 0));
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip2, i, 0, 255, 0));
            ESP_ERROR_CHECK(led_strip_refresh(led_strip1));
            ESP_ERROR_CHECK(led_strip_refresh(led_strip2));

            vTaskDelay(pdMS_TO_TICKS(100));
        }  
}


void blink_strip(uint32_t nled, uint32_t count, uint32_t period)
{
    bool led_on_off = false;
    while (count--) {
        if (led_on_off) {
            for (int i = 0; i < LED_STRIP_LED_NUMBERS; i++) { //                 R  G   B
                if(nled == 1) ESP_ERROR_CHECK(led_strip_set_pixel(led_strip1, i, 0, 0, 255));
                if(nled == 2) ESP_ERROR_CHECK(led_strip_set_pixel(led_strip2, i, 240, 252, 3));
            }  
            if(nled == 1)ESP_ERROR_CHECK(led_strip_refresh(led_strip1));
            if(nled == 2)ESP_ERROR_CHECK(led_strip_refresh(led_strip2));
        } else {
            if(nled == 1) ESP_ERROR_CHECK(led_strip_clear(led_strip1));
            if(nled == 2) ESP_ERROR_CHECK(led_strip_clear(led_strip2));
        }

        led_on_off = !led_on_off;

        vTaskDelay(pdMS_TO_TICKS(period));
    }

    ESP_ERROR_CHECK(led_strip_clear(led_strip1));
    ESP_ERROR_CHECK(led_strip_clear(led_strip2));
}

void app_main(void)
{
    /*
        Turn of warnings from HTTP server as redirecting traffic will yield
        lots of invalid requests
    */
    //esp_log_level_set("httpd_uri", ESP_LOG_ERROR);
    //esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);
    //esp_log_level_set("httpd_parse", ESP_LOG_ERROR);

    strip_init();
    isr_init();
    // Initialize networking stack
    //ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop needed by the  main app
    //ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize NVS needed by Wi-Fi
   // ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize Wi-Fi including netif with default config
    //esp_netif_create_default_wifi_ap();

    // Initialise ESP32 in SoftAP mode
    //wifi_init_softap();

    // Start the server for the first time
    //start_webserver();

    // Start the DNS server that will redirect all queries to the softAP IP
    //dns_server_config_t config = DNS_SERVER_CONFIG_SINGLE("*" /* all A queries */, "WIFI_AP_DEF" /* softAP netif ID */);
    //start_dns_server(&config);

    welcome_strip();

    ESP_LOGI(TAG, "Information messages which describe normal flow of events");
    while (1) {
       
        if (key1_action){
            ESP_LOGI(TAG, "KEY1 Action");
            key1_action = false;
            blink_strip(1,10,100);
            gpio_intr_enable(KEY1);
            gpio_intr_enable(KEY2);
            
        }

        if (key2_action){
            ESP_LOGI(TAG, "KEY2 Action");
            key2_action = false;
            blink_strip(2,10,100);
            gpio_intr_enable(KEY1);
            gpio_intr_enable(KEY2);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
