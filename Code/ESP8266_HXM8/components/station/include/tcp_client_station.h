#ifndef ESP8266_TCP_CLIENT_STATION
#define ESP8266_TCP_CLIENT_STATION

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>

/* Configuration Definitions  */
/*  WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define ESP_WIFI_SSID "mywifissid"
*/
#define ESP_WIFI_SSID               CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS               CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY           CONFIG_ESP_MAXIMUM_RETRY

/*TCPIP Configuration Parameters, can be set via project configuration menu

    If you'd rather not, just change the below entries to strings with
    the config you want - ie #define HOST_IP_ADD "IPADDRESS"
*/
#define HOST_IP_ADDR                CONFIG_IP_ADDR
#define PORT                        CONFIG_PORT
/* End Configuration Definitions */


/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT          BIT0
#define WIFI_FAIL_BIT               BIT1

#define TCP_CLIENT_PRIORITY         15          //priority of TCP tasks
#define TCP_TRANSMISSION_PRIORITY   1           //priority of the transmission, whether you deem transmission or receiving higher priority adjust this value(negative values will assume receiver priority)
#define TCP_TRANSMIT_TIME_DELAY     50          //delay time in ms for transmitting data
#define TCP_QUEUE_SIZE              10
#define TCP_RECV_BUFFER             1024

#define TCP_QUEUE_DATA_SIZE         45


char rx_buffer[TCP_RECV_BUFFER];                //used to store the incoming data from the socket

typedef enum {
    MSG_AVAIL,
    MSG_NOT_AVAIL,        
} message_avail_t;

typedef struct {
    uint32_t data_len;
    void* data;
} tcp_tx_data_t;

QueueHandle_t s_tcp_rx_queue;                   //queue handle to be used in functions throughout project


/* Defined Functions*/
void wifi_init();                           //initialize WiFi and TCPIP Stack

void tcp_stack_start();                     //starts the TCP client stack 

BaseType_t tcp_tx_data(tcp_tx_data_t data_info);         //places items in queue to be called upon to transmit data


#ifdef __cplusplus
}
#endif

#endif