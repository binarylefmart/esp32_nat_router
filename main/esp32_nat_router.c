/* Console example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "driver/usb_serial_jtag.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_eap_client.h"

#include "lwip/opt.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "dhcpserver/dhcpserver.h"
#include "dhcpserver/dhcpserver_options.h"

#include "cmd_decl.h"
#include <esp_http_server.h>

//Rajout Deepsleep
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

//Rajout MQTT
#include "mqtt_client.h"

//Rajout INA219
#include "ina219.h"

#if !IP_NAPT
#error "IP_NAPT must be defined"
#endif
#include "lwip/lwip_napt.h"

#include "router_globals.h"

//GPIO DEEPSLEEP
#define GPIO_SENSOR_PIN 26

//Configuration INA219
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_FREQ_HZ 1000000

static const char *INA219_TAG = "INA219";

ina219_t dev;


// On board LED
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define BLINK_GPIO 44
#else
#define BLINK_GPIO 2
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about one event
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

#define DEFAULT_AP_IP "192.168.4.1"
#define DEFAULT_DNS "8.8.8.8"

/* Global vars */
uint16_t connect_count = 0;
bool ap_connect = false;
bool has_static_ip = false;

uint32_t my_ip;
uint32_t my_ap_ip;

struct portmap_table_entry {
  u32_t daddr;
  u16_t mport;
  u16_t dport;
  u8_t proto;
  u8_t valid;
};
struct portmap_table_entry portmap_tab[IP_PORTMAP_MAX];

esp_netif_t* wifiAP;
esp_netif_t* wifiSTA;

httpd_handle_t start_webserver(void);

static const char *TAG = "ESP32 NAT router";

/* Console command history can be stored to and loaded from a file.
 * The easiest way to do this is to use FATFS filesystem on top of
 * wear_levelling library.
 */
#if CONFIG_STORE_HISTORY

#define MOUNT_PATH "/data"
#define HISTORY_PATH MOUNT_PATH "/history.txt"

static void initialize_filesystem(void)
{
    static wl_handle_t wl_handle;
    const esp_vfs_fat_mount_config_t mount_config = {
            .max_files = 4,
            .format_if_mount_failed = true
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(MOUNT_PATH, "storage", &mount_config, &wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
}
#endif // CONFIG_STORE_HISTORY

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

esp_err_t apply_portmap_tab() {
    for (int i = 0; i<IP_PORTMAP_MAX; i++) {
        if (portmap_tab[i].valid) {
            ip_portmap_add(portmap_tab[i].proto, my_ip, portmap_tab[i].mport, portmap_tab[i].daddr, portmap_tab[i].dport);
        }
    }
    return ESP_OK;
}

esp_err_t delete_portmap_tab() {
    for (int i = 0; i<IP_PORTMAP_MAX; i++) {
        if (portmap_tab[i].valid) {
            ip_portmap_remove(portmap_tab[i].proto, portmap_tab[i].mport);
        }
    }
    return ESP_OK;
}

void print_portmap_tab() {
    for (int i = 0; i<IP_PORTMAP_MAX; i++) {
        if (portmap_tab[i].valid) {
            printf ("%s", portmap_tab[i].proto == PROTO_TCP?"TCP ":"UDP ");
            ip4_addr_t addr;
            addr.addr = my_ip;
            printf (IPSTR":%d -> ", IP2STR(&addr), portmap_tab[i].mport);
            addr.addr = portmap_tab[i].daddr;
            printf (IPSTR":%d\n", IP2STR(&addr), portmap_tab[i].dport);
        }
    }
}

esp_err_t get_portmap_tab() {
    esp_err_t err;
    nvs_handle_t nvs;
    size_t len;

    err = nvs_open(PARAM_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_get_blob(nvs, "portmap_tab", NULL, &len);
    if (err == ESP_OK) {
        if (len != sizeof(portmap_tab)) {
            err = ESP_ERR_NVS_INVALID_LENGTH;
        } else {
            err = nvs_get_blob(nvs, "portmap_tab", portmap_tab, &len);
            if (err != ESP_OK) {
                memset(portmap_tab, 0, sizeof(portmap_tab));
            }
        }
    }
    nvs_close(nvs);

    return err;
}

esp_err_t add_portmap(u8_t proto, u16_t mport, u32_t daddr, u16_t dport) {
    esp_err_t err;
    nvs_handle_t nvs;

    for (int i = 0; i<IP_PORTMAP_MAX; i++) {
        if (!portmap_tab[i].valid) {
            portmap_tab[i].proto = proto;
            portmap_tab[i].mport = mport;
            portmap_tab[i].daddr = daddr;
            portmap_tab[i].dport = dport;
            portmap_tab[i].valid = 1;

            err = nvs_open(PARAM_NAMESPACE, NVS_READWRITE, &nvs);
            if (err != ESP_OK) {
                return err;
            }
            err = nvs_set_blob(nvs, "portmap_tab", portmap_tab, sizeof(portmap_tab));
            if (err == ESP_OK) {
                err = nvs_commit(nvs);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "New portmap table stored.");
                }
            }
            nvs_close(nvs);

            ip_portmap_add(proto, my_ip, mport, daddr, dport);

            return ESP_OK;
        }
    }
    return ESP_ERR_NO_MEM;
}

esp_err_t del_portmap(u8_t proto, u16_t mport) {
    esp_err_t err;
    nvs_handle_t nvs;

    for (int i = 0; i<IP_PORTMAP_MAX; i++) {
        if (portmap_tab[i].valid && portmap_tab[i].mport == mport && portmap_tab[i].proto == proto) {
            portmap_tab[i].valid = 0;

            err = nvs_open(PARAM_NAMESPACE, NVS_READWRITE, &nvs);
            if (err != ESP_OK) {
                return err;
            }
            err = nvs_set_blob(nvs, "portmap_tab", portmap_tab, sizeof(portmap_tab));
            if (err == ESP_OK) {
                err = nvs_commit(nvs);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "New portmap table stored.");
                }
            }
            nvs_close(nvs);

            ip_portmap_remove(proto, mport);
            return ESP_OK;
        }
    }
    return ESP_OK;
}

static void initialize_console(void)
{
    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

#if CONFIG_ESP_CONSOLE_UART_DEFAULT || CONFIG_ESP_CONSOLE_UART_CUSTOM
    /* Drain stdout before reconfiguring it */
    fflush(stdout);
    fsync(fileno(stdout));
    
    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_port_set_rx_line_endings(0, ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_port_set_tx_line_endings(0, ESP_LINE_ENDINGS_CRLF);

    /* Configure UART. Note that REF_TICK is used so that the baud rate remains
     * correct while APB frequency is changing in light sleep mode.
     */
    const uart_config_t uart_config = {
            .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            #if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2)
                .source_clk = UART_SCLK_REF_TICK,
            #else
                .source_clk = UART_SCLK_XTAL,
            #endif
    };
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM,
            256, 0, 0, NULL, 0) );
    ESP_ERROR_CHECK( uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
#endif

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    /* Enable non-blocking mode on stdin and stdout */
    fcntl(fileno(stdout), F_SETFL, O_NONBLOCK);
    fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_usb_serial_jtag_set_rx_line_endings(ESP_LINE_ENDINGS_CR);

    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_usb_serial_jtag_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .tx_buffer_size = 256,
        .rx_buffer_size = 256,
    };

    /* Install USB-SERIAL-JTAG driver for interrupt-driven reads and writes */
    usb_serial_jtag_driver_install(&usb_serial_jtag_config);

    /* Tell vfs to use usb-serial-jtag driver */
    esp_vfs_usb_serial_jtag_use_driver();
#endif

    /* Initialize the console */
    esp_console_config_t console_config = {
            .max_cmdline_args = 8,
            .max_cmdline_length = 256,
#if CONFIG_LOG_COLORS
            .hint_color = atoi(LOG_COLOR_CYAN)
#endif
    };
    ESP_ERROR_CHECK( esp_console_init(&console_config) );

    /* Configure linenoise line completion library */
    /* Enable multiline editing. If not set, long commands will scroll within
     * single line.
     */
    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

    /* Set command history size */
    linenoiseHistorySetMaxLen(100);

#if CONFIG_STORE_HISTORY
    /* Load command history from filesystem */
    linenoiseHistoryLoad(HISTORY_PATH);
#endif
}

void * led_status_thread(void * p)
{
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (true)
    {
        gpio_set_level(BLINK_GPIO, ap_connect);

        for (int i = 0; i < connect_count; i++)
        {
            gpio_set_level(BLINK_GPIO, 1 - ap_connect);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            gpio_set_level(BLINK_GPIO, ap_connect);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    esp_netif_dns_info_t dns;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGI(TAG,"disconnected - retry to connect to the AP");
        ap_connect = false;
        esp_wifi_connect();
        ESP_LOGI(TAG, "retry to connect to the AP");
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        ap_connect = true;
        my_ip = event->ip_info.ip.addr;
        delete_portmap_tab();
        apply_portmap_tab();
        if (esp_netif_get_dns_info(wifiSTA, ESP_NETIF_DNS_MAIN, &dns) == ESP_OK)
        {
            esp_netif_set_dns_info(wifiAP, ESP_NETIF_DNS_MAIN, &dns);
            ESP_LOGI(TAG, "set dns to:" IPSTR, IP2STR(&(dns.ip.u_addr.ip4)));
        }
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        connect_count++;
        ESP_LOGI(TAG,"%d. station connected", connect_count);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        connect_count--;
        ESP_LOGI(TAG,"station disconnected - %d remain", connect_count);
    }
}

const int CONNECTED_BIT = BIT0;
#define JOIN_TIMEOUT_MS (2000)


void wifi_init(const char* ssid, const char* ent_username, const char* ent_identity, const char* passwd, const char* static_ip, const char* subnet_mask, const char* gateway_addr, const char* ap_ssid, const char* ap_passwd, const char* ap_ip)
{
    esp_netif_dns_info_t dnsserver;
    // esp_netif_dns_info_t dnsinfo;

    wifi_event_group = xEventGroupCreate();
  
    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifiAP = esp_netif_create_default_wifi_ap();
    wifiSTA = esp_netif_create_default_wifi_sta();

    esp_netif_ip_info_t ipInfo_sta;
    if ((strlen(ssid) > 0) && (strlen(static_ip) > 0) && (strlen(subnet_mask) > 0) && (strlen(gateway_addr) > 0)) {
        has_static_ip = true;
        ipInfo_sta.ip.addr = esp_ip4addr_aton(static_ip);
        ipInfo_sta.gw.addr = esp_ip4addr_aton(gateway_addr);
        ipInfo_sta.netmask.addr = esp_ip4addr_aton(subnet_mask);
        esp_netif_dhcpc_stop(ESP_IF_WIFI_STA); // Don't run a DHCP client
        esp_netif_set_ip_info(ESP_IF_WIFI_STA, &ipInfo_sta);
        apply_portmap_tab();
    }

    my_ap_ip = esp_ip4addr_aton(ap_ip);

    esp_netif_ip_info_t ipInfo_ap;
    ipInfo_ap.ip.addr = my_ap_ip;
    ipInfo_ap.gw.addr = my_ap_ip;
    esp_netif_set_ip4_addr(&ipInfo_ap.netmask, 255,255,255,0);
    esp_netif_dhcps_stop(wifiAP); // stop before setting ip WifiAP
    esp_netif_set_ip_info(wifiAP, &ipInfo_ap);
    esp_netif_dhcps_start(wifiAP);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* ESP WIFI CONFIG */
    wifi_config_t wifi_config = { 0 };
        wifi_config_t ap_config = {
        .ap = {
            .channel = 0,
            .authmode = WIFI_AUTH_WPA2_WPA3_PSK,
            .ssid_hidden = 0,
            .max_connection = 8,
            .beacon_interval = 100,
        }
    };

    strlcpy((char*)ap_config.sta.ssid, ap_ssid, sizeof(ap_config.sta.ssid));
    if (strlen(ap_passwd) < 8) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    } else {
	    strlcpy((char*)ap_config.sta.password, ap_passwd, sizeof(ap_config.sta.password));
    }

    if (strlen(ssid) > 0) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA) );

        //Set SSID
        strlcpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
        //Set passwprd
        if(strlen(ent_username) == 0) {
            ESP_LOGI(TAG, "STA regular connection");
            strlcpy((char*)wifi_config.sta.password, passwd, sizeof(wifi_config.sta.password));
        }
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
        if(strlen(ent_username) != 0 && strlen(ent_identity) != 0) {
            ESP_LOGI(TAG, "STA enterprise connection");
            if(strlen(ent_username) != 0 && strlen(ent_identity) != 0) {
                esp_eap_client_set_identity((uint8_t *)ent_identity, strlen(ent_identity)); //provide identity
            } else {
                esp_eap_client_set_identity((uint8_t *)ent_username, strlen(ent_username));
            }
            esp_eap_client_set_username((uint8_t *)ent_username, strlen(ent_username)); //provide username
            esp_eap_client_set_password((uint8_t *)passwd, strlen(passwd)); //provide password
            esp_wifi_sta_enterprise_enable();
        }

        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config) );
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP) );
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config) );        
    }



    // Enable DNS (offer) for dhcp server
    dhcps_offer_t dhcps_dns_value = OFFER_DNS;
    esp_netif_dhcps_option(wifiAP,ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &dhcps_dns_value, sizeof(dhcps_dns_value));

    // // Set custom dns server address for dhcp server
    dnsserver.ip.u_addr.ip4.addr = esp_ip4addr_aton(DEFAULT_DNS);
    dnsserver.ip.type = ESP_IPADDR_TYPE_V4;
    esp_netif_set_dns_info(wifiAP, ESP_NETIF_DNS_MAIN, &dnsserver);

    // esp_netif_get_dns_info(ESP_IF_WIFI_AP, ESP_NETIF_DNS_MAIN, &dnsinfo);
    // ESP_LOGI(TAG, "DNS IP:" IPSTR, IP2STR(&dnsinfo.ip.u_addr.ip4));

    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
        pdFALSE, pdTRUE, JOIN_TIMEOUT_MS / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(esp_wifi_start());

    if (strlen(ssid) > 0) {
        ESP_LOGI(TAG, "wifi_init_apsta finished.");
        ESP_LOGI(TAG, "connect to ap SSID: %s ", ssid);
    } else {
        ESP_LOGI(TAG, "wifi_init_ap with default finished.");      
    }
}

char* ssid = NULL;
char* ent_username = NULL;
char* ent_identity = NULL;
char* passwd = NULL;
char* static_ip = NULL;
char* subnet_mask = NULL;
char* gateway_addr = NULL;
char* ap_ssid = NULL;
char* ap_passwd = NULL;
char* ap_ip = NULL;

char* param_set_default(const char* def_val) {
    char * retval = malloc(strlen(def_val)+1);
    strcpy(retval, def_val);
    return retval;
}

static esp_err_t mqtt(esp_mqtt_event_handle_t event)
{
    int rssi; // Déclaration de la variable rssi
    int gpio_state;

    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            if (esp_wifi_sta_get_rssi(&rssi) == ESP_OK) { // Appel de esp_wifi_sta_get_rssi
                ESP_LOGI(TAG, "RSSI: %d", rssi);
            } else {
                ESP_LOGE(TAG, "Could not get RSSI");
            }
            gpio_state = gpio_get_level(GPIO_NUM_26);
            ESP_LOGI(TAG, "GPIO 26 state: %d", gpio_state);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        // Ajoutez des cas pour d'autres événements que vous souhaitez gérer...

        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;    
}

void init_ina219()
{
    i2cdev_init();

    memset(&dev, 0, sizeof(ina219_t));

    ESP_ERROR_CHECK(ina219_init_desc(&dev, INA219_ADDR_GND_GND, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    ESP_ERROR_CHECK(ina219_init(&dev));
}

void read_ina219(esp_mqtt_client_handle_t mqtt_client)
{
    float bus_voltage;
    float shunt_voltage;
    float current;

    if (ina219_get_bus_voltage(&dev, &bus_voltage) == ESP_OK &&
        ina219_get_shunt_voltage(&dev, &shunt_voltage) == ESP_OK &&
        ina219_get_current(&dev, &current) == ESP_OK)
    {
        // Convertir le courant en milliampères
        float current_mA = current * 1000;
        // Convertir la tension en pourcentage de la batterie
        float voltage_max = 4.2; // tension maximale de la batterie
        float voltage_min = 3.0; // tension minimale de la batterie
        int battery_percentage = 0;
        if (bus_voltage <= voltage_min) {
            battery_percentage = 0;
        } else if (bus_voltage >= voltage_max) {
            battery_percentage = 100;
        } else {
            battery_percentage = (bus_voltage - voltage_min) * 100 / (voltage_max - voltage_min);
        }
        ESP_LOGI(INA219_TAG, "Battery percentage: %d%%", battery_percentage);
        ESP_LOGI(INA219_TAG, "Bus voltage: %.2fV", bus_voltage);
        ESP_LOGI(INA219_TAG, "Current: %.2fmA", current_mA);
        // Publier le pourcentage de la batterie sur MQTT
        char battery_percentage_str[10];
        sprintf(battery_percentage_str, "%d", battery_percentage);
        esp_mqtt_client_publish(mqtt_client, "/esp32/battery_percentage", battery_percentage_str, 0, 1, 0);
        // Publier la tension de la batterie sur MQTT
        char bus_voltage_str[10];
        sprintf(bus_voltage_str, "%.2f", bus_voltage);
        esp_mqtt_client_publish(mqtt_client, "/esp32/bus_voltage", bus_voltage_str, 0, 1, 0);
        // Publier le courant sur MQTT
        char current_str[10];
        sprintf(current_str, "%.2f", current_mA);
        esp_mqtt_client_publish(mqtt_client, "/esp32/current_mA", current_str, 0, 1, 0);
        // Obtenir le temps actuel
        time_t now;
        time(&now);
        struct tm *timeinfo = localtime(&now);
        // Convertir le temps en chaîne de caractères
        char date_str[20];
        char time_str[10];
        // Formater la date et l'heure
        sprintf(date_str, sizeof(date_str), "%02d-%02d-%04d", timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900);
        sprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);    
        // Publier l'heure de la dernière notification sur MQTT
        esp_mqtt_client_publish(mqtt_client, "/esp32/last_notification_date", date_str, 0, 1, 0);
        esp_mqtt_client_publish(mqtt_client, "/esp32/last_notification_time", time_str, 0, 1, 0);
    }
    else
    {
        ESP_LOGE(TAG, "Could not read data from INA219");
    }
}

void read_wifi(esp_mqtt_client_handle_t mqtt_client)
{
    // Récupérer le RSSI
    wifi_ap_record_t wifidata;
    if (esp_wifi_sta_get_ap_info(&wifidata) == ESP_OK) {
        printf("RSSI: %d\n", wifidata.rssi);
        char rssi_str[10];
        sprintf(rssi_str, "%d", wifidata.rssi);
        esp_mqtt_client_publish(mqtt_client, "/esp32/rssi", rssi_str, 0, 1, 0);
        // Convertir le RSSI en pourcentage
        int rssi = wifidata.rssi;
        int rssi_max = -30;
        int rssi_min = -100;
        int rssi_percentage = 0;
        if (rssi <= rssi_min) {
            rssi_percentage = 0;
        } else if (rssi >= rssi_max) {
            rssi_percentage = 100;
        } else {
            rssi_percentage = (rssi - rssi_min) * 100 / (rssi_max - rssi_min);
        }
        printf("Force Signal: %d%%\n", rssi_percentage);
        char rssi_percentage_str[10];
        sprintf(rssi_percentage_str, "%d", rssi_percentage);
        esp_mqtt_client_publish(mqtt_client, "/esp32/rssi_percentage", rssi_percentage_str, 0, 1, 0);
        // Obtenir le temps actuel
        time_t now;
        time(&now);
        struct tm *timeinfo = localtime(&now);
        // Convertir le temps en chaîne de caractères
        char date_str[20];
        char time_str[10];
        // Formater la date et l'heure
        sprintf(date_str, sizeof(date_str), "%02d-%02d-%04d", timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900);
        sprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);    
        // Publier l'heure de la dernière notification sur MQTT
        esp_mqtt_client_publish(mqtt_client, "/esp32/last_notification_date", date_str, 0, 1, 0);
        esp_mqtt_client_publish(mqtt_client, "/esp32/last_notification_time", time_str, 0, 1, 0);
    } else {
        ESP_LOGE(TAG, "Could not get RSSI");
    }
}

void code_main(void)
{
    initialize_nvs();

#if CONFIG_STORE_HISTORY
    initialize_filesystem();
    ESP_LOGI(TAG, "Command history enabled");
#else
    ESP_LOGI(TAG, "Command history disabled");
#endif

    get_config_param_str("ssid", &ssid);
    if (ssid == NULL) {
        ssid = param_set_default("");
    }
    get_config_param_str("ent_username", &ent_username);
    if (ent_username == NULL) {
        ent_username = param_set_default("");
    }
    get_config_param_str("ent_identity", &ent_identity);
    if (ent_identity == NULL) {
        ent_identity = param_set_default("");
    }
    get_config_param_str("passwd", &passwd);
    if (passwd == NULL) {
        passwd = param_set_default("");
    }
    get_config_param_str("static_ip", &static_ip);
    if (static_ip == NULL) {
        static_ip = param_set_default("");
    }
    get_config_param_str("subnet_mask", &subnet_mask);
    if (subnet_mask == NULL) {
        subnet_mask = param_set_default("");
    }
    get_config_param_str("gateway_addr", &gateway_addr);
    if (gateway_addr == NULL) {
        gateway_addr = param_set_default("");
    }
    get_config_param_str("ap_ssid", &ap_ssid);
    if (ap_ssid == NULL) {
        ap_ssid = param_set_default("ESP32_NAT_Router");
    }   
    get_config_param_str("ap_passwd", &ap_passwd);
    if (ap_passwd == NULL) {
        ap_passwd = param_set_default("");
    }
    get_config_param_str("ap_ip", &ap_ip);
    if (ap_ip == NULL) {
        ap_ip = param_set_default(DEFAULT_AP_IP);
    }

    get_portmap_tab();

    // Setup WIFI
    wifi_init(ssid, ent_username, ent_identity, passwd, static_ip, subnet_mask, gateway_addr, ap_ssid, ap_passwd, ap_ip);

    pthread_t t1;
    pthread_create(&t1, NULL, led_status_thread, NULL);

    ip_napt_enable(my_ap_ip, 1);
    ESP_LOGI(TAG, "NAT is enabled");

    char* lock = NULL;
    get_config_param_str("lock", &lock);
    if (lock == NULL) {
        lock = param_set_default("0");
    }
    if (strcmp(lock, "0") ==0) {
        ESP_LOGI(TAG,"Starting config web server");
        start_webserver();
    }
    free(lock);

    initialize_console();

    /* Register commands */
    esp_console_register_help_command();
    register_system();
    register_nvs();
    register_router();

    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    const char* prompt = LOG_COLOR_I "esp32> " LOG_RESET_COLOR;

    printf("\n"
           "ESP32 NAT ROUTER\n"
           "Type 'help' to get the list of commands.\n"
           "Use UP/DOWN arrows to navigate through command history.\n"
           "Press TAB when typing command name to auto-complete.\n");

    if (strlen(ssid) == 0) {
         printf("\n"
               "Unconfigured WiFi\n"
               "Configure using 'set_sta' and 'set_ap' and restart.\n");       
    }

    /* Figure out if the terminal supports escape sequences */
    int probe_status = linenoiseProbe();
    if (probe_status) { /* zero indicates success */
        printf("\n"
               "Your terminal application does not support escape sequences.\n"
               "Line editing and history features are disabled.\n"
               "On Windows, try using Putty instead.\n");
        linenoiseSetDumbMode(1);
#if CONFIG_LOG_COLORS
        /* Since the terminal doesn't support escape sequences,
         * don't use color codes in the prompt.
         */
        prompt = "esp32> ";
#endif //CONFIG_LOG_COLORS
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
    // Déclarez et initialisez le client MQTT
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = "mqtt://MQTT_ID:MQTT_PWD@IP_SERVER:PORT",
            },
        },
        // initialiser les autres membres...
    };
    esp_mqtt_client_handle_t mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(mqtt_client);

    //INA219 Setup
    init_ina219();
    /* Main loop */
    while (true) {
        /* Lire l'état du capteur sur le GPIO 26 */
        int sensor_state = gpio_get_level(GPIO_SENSOR_PIN);

        /* Vérifier l'état du capteur */
        if (sensor_state == 1) {
            printf("Il fait nuit, GPIO à l'état High; Mise en deep sleep...\n");
            /* Publier l'état du capteur sur MQTT */
            esp_mqtt_client_publish(mqtt_client, "/sensor/state", "Nuit", 0, 1, 0);
            // Surveiller et publier le wif
            read_wifi(mqtt_client);
            // Surveiller et publier le pourcentage de la batterie
            read_ina219(mqtt_client);
            vTaskDelay(pdMS_TO_TICKS(5000));
            break; // Sortir de la boucle avant le deep sleep
             
        } else {
            printf("Il fait jour, GPIO à l'état Low; En attente ...\n");
            /* Publier l'état du capteur sur MQTT */
            esp_mqtt_client_publish(mqtt_client, "/sensor/state", "Jour", 0, 1, 0);
            // Surveiller et publier le wifi
            read_wifi(mqtt_client);
            // Surveiller et publier le pourcentage de la batterie
            read_ina219(mqtt_client);
            /* Attendre un court laps de temps avant de vérifier à nouveau */
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
    /* Mettre l'ESP32 en deep sleep */
    esp_deep_sleep_start();
}

void app_main() {

    esp_rom_gpio_pad_select_gpio(GPIO_SENSOR_PIN);
    gpio_set_direction(GPIO_SENSOR_PIN, GPIO_MODE_INPUT);

    // Configurer le GPIO 26 comme déclencheur de réveil sur front montant (statut 0)
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_sleep_enable_ext1_wakeup(1ULL << GPIO_SENSOR_PIN, ESP_EXT1_WAKEUP_ALL_LOW);

    // Lire l'état du capteur sur le GPIO 26
    int sensor_state = gpio_get_level(GPIO_SENSOR_PIN);

    // Vérifier l'état du capteur
    if (sensor_state == 0) {
        printf("Il fait jour, GPIO à l'état Low; En attente ...\n");
        // Exécuter le code
        code_main();
    } else if (sensor_state == 1) {
        printf("Il fait nuit, GPIO à l'état High; Mise en deep sleep...\n");
        // Mettre l'ESP32 en deep sleep
        esp_deep_sleep_start();
    }   

    // Attendre un court laps de temps avant de vérifier à nouveau
    vTaskDelay(pdMS_TO_TICKS(3000));
}
