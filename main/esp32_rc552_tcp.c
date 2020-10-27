/* Scan Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
    This example shows how to use the All Channel Scan or Fast Scan to connect
    to a Wi-Fi network.

    In the Fast Scan mode, the scan will stop as soon as the first network matching
    the SSID is found. In this mode, an application can set threshold for the
    authentication mode and the Signal strength. Networks that do not meet the
    threshold requirements will be ignored.

    In the All Channel Scan mode, the scan will end only after all the channels
    are scanned, and connection will start with the best network. The networks
    can be sorted based on Authentication Mode or Signal Strength. The priority
    for the Authentication mode is:  WPA2 > WPA > WEP > Open
*/

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "driver/gpio.h"

#include "rc522_tcp.h"


/* Set the SSID and Password via project configuration, or can set directly here */
#define DEFAULT_SSID CONFIG_WIFI_SSID
#define DEFAULT_PWD CONFIG_WIFI_PASSWORD

#if CONFIG_WIFI_ALL_CHANNEL_SCAN
#define DEFAULT_SCAN_METHOD WIFI_ALL_CHANNEL_SCAN
#elif CONFIG_WIFI_FAST_SCAN
#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
#else
#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
#endif /*CONFIG_SCAN_METHOD*/

#if CONFIG_WIFI_CONNECT_AP_BY_SIGNAL
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#elif CONFIG_WIFI_CONNECT_AP_BY_SECURITY
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SECURITY
#else
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#endif /*CONFIG_SORT_METHOD*/

#if CONFIG_FAST_SCAN_THRESHOLD
#define DEFAULT_RSSI CONFIG_FAST_SCAN_MINIMUM_SIGNAL
#if CONFIG_FAST_SCAN_WEAKEST_AUTHMODE_OPEN
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#elif CONFIG_FAST_SCAN_WEAKEST_AUTHMODE_WEP
#define DEFAULT_AUTHMODE WIFI_AUTH_WEP
#elif CONFIG_FAST_SCAN_WEAKEST_AUTHMODE_WPA
#define DEFAULT_AUTHMODE WIFI_AUTH_WPA_PSK
#elif CONFIG_FAST_SCAN_WEAKEST_AUTHMODE_WPA2
#define DEFAULT_AUTHMODE WIFI_AUTH_WPA2_PSK
#else
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#endif
#else
#define DEFAULT_RSSI -127
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#endif /*CONFIG_FAST_SCAN_THRESHOLD*/


void rst_gpio_get(int socket_fd) {

    esp_err_t err = gpio_set_direction(CONFIG_RST_GPIO_NUM, GPIO_MODE_INPUT);
    uint8_t* response = malloc(2);

    response[0] = err == ESP_OK ? OPERATION_OK : OPERATION_FAIL;              // Operation result
    response[1] = err == ESP_OK ? gpio_get_level(CONFIG_RST_GPIO_NUM) : 0xFF; // GPIO state or value should be ignored on fail

    send(socket_fd, response, 2, 0);
    free(response);

    if (err != ESP_OK)
        printf("RST_GPIO_GET failed: %s\n", esp_err_to_name(err));
}

void rst_gpio_set(int socket_fd, rc522_tcp_gpio_state_t gpio_state) {
    esp_err_t err;

    err = gpio_set_direction(CONFIG_RST_GPIO_NUM, GPIO_MODE_OUTPUT);
    if (err == ESP_OK)
        err = gpio_set_level(CONFIG_RST_GPIO_NUM, gpio_state);

    // Operation result
    uint8_t response = err == ESP_OK ? OPERATION_OK : OPERATION_FAIL;
    send(socket_fd, &response, 1, 0);

    if (err != ESP_OK)
        printf("RST_GPIO_SET failed: %s\n", esp_err_to_name(err));
}


spi_device_handle_t spi_device_handle;

void spi_transceive(int socket_fd, uint8_t length) {

    uint8_t* data = malloc(length);

    if (recv(socket_fd, data, length, 0) < 0)
        printf("recv failed: errno %d\n", errno);

    spi_transaction_t spi_transaction = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = length*8,
        .tx_buffer = data,
        .rx_buffer = data
    };

    esp_err_t err = spi_device_transmit(spi_device_handle, &spi_transaction);
    
    uint8_t* response = malloc(1 + length);
    response[0] = err == ESP_OK ? OPERATION_OK : OPERATION_FAIL;
    memcpy(response + 1, data, length);

    send(socket_fd, &response, 1 + length, 0);

    if (err != ESP_OK)
        printf("SPI_TRANSCEIVE failed: %s\n", esp_err_to_name(err));
    free(data);
    free(response);
}


void tcp_client_task(void *pvParameters) {
    while (true) {
        
        int socket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (socket_fd < 0) {
            printf("Unable to create socket: errno %d\n", errno); break;
        }
        printf("Socket created, connecting to %s:%d\n", CONFIG_DESTINATION_ADDRESS, CONFIG_DESTINATION_PORT);


        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(CONFIG_DESTINATION_ADDRESS);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(CONFIG_DESTINATION_PORT);

        int err = connect(socket_fd, &dest_addr, sizeof(struct sockaddr_in));
        if (err) {
            printf("Socket unable to connect: errno %d\n", errno); break;
        }
        printf("Socket connected.\n");


        uint8_t* operation = malloc(2);

        while (true) {
            
            int recv_length = recv(socket_fd, operation, 2, 0);  // Receive operation code and argument
            if (recv_length < 0) {
                printf("recv failed: errno %d\n", errno);
                break;
            }

            switch (operation[0]) {

                case RST_GPIO_GET:
                    rst_gpio_get(socket_fd);
                break;

                case RST_GPIO_SET:
                    rst_gpio_set(socket_fd, operation[1]);
                break;
                
                case SPI_TRANSCEIVE:
                    spi_transceive(socket_fd, operation[1]);
                break;

                default:
                break;
            }
        }
        
        // Clean socket
        free(operation);
        shutdown(socket_fd, 0);
        close(socket_fd);
    }
}


/* Wi-Fi event handler */
void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf("Got ip:" IPSTR "\n", IP2STR(&event->ip_info.ip));

    }
}

/* Initialize Wi-Fi as sta and set scan method */
void fast_scan() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    // Initialize default station as network interface instance (esp-netif)
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    // Initialize and start WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = DEFAULT_SSID,
            .password = DEFAULT_PWD,
            .scan_method = DEFAULT_SCAN_METHOD,
            .sort_method = DEFAULT_SORT_METHOD,
            .threshold.rssi = DEFAULT_RSSI,
            .threshold.authmode = DEFAULT_AUTHMODE,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}


void spi_init() {

    spi_bus_config_t spi_bus_config = {
        .sclk_io_num = CONFIG_SPI_CLK_GPIO_NUM,
        .mosi_io_num = CONFIG_SPI_MOSI_GPIO_NUM,
        .miso_io_num = CONFIG_SPI_MISO_GPIO_NUM,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        .flags = SPICOMMON_BUSFLAG_MASTER
    };

    spi_device_interface_config_t spi_device_interface_config = {
        .address_bits = 0,
        .command_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .clock_speed_hz = SPI_MASTER_FREQ_8M / 2,  // 4MHz
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bus_config, 1));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &spi_device_interface_config, &spi_device_handle));
    printf("[RC522_TCP] - SPI initialized.\n");
}


void app_main(void) {

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    spi_init();
    fast_scan();
}
