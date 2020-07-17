/* WiFi station

   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "tcp_client_station.h"



/* Function Definitions */
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);  //Handle events on wifi initialization
static void wifi_init_sta(void);                                                                        //Initialize wifi and TCPIP stack
static void tcp_client_tx_task(void* pvParameters);                                                     //Start TCP client and listen for incoming messages
static void tcp_client_task(void* pvParameters);                                                        //TCP transmission queue handler, will transmit when queue has available items

static message_avail_t tcp_recv_fun();

/* FreeRTOS event group to signal when connected */
static EventGroupHandle_t s_wifi_event_group;

/* Task handle(s) for created tasks */
static TaskHandle_t s_tcp_socket = NULL;
static TaskHandle_t s_tcp_tx_stream = NULL;

/* Queue handle(s) for created queues */
static QueueHandle_t s_tcp_queue;


/* Global variables used in file */
static int s_retry_num = 0;                                                                             //number of retries counter for when trying to connect to wifi
static const char *TAG = "wifi station";                                                                //tag used to define logging
static int sock;                                                                                        //used as reference to the socket opened, so it may be used in multiple tasks
static char addr_str[128];                                                                                     //used to store the sockets IP Address



static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}



static void tcp_client_task(void* pvParameters)
{
    int addr_family;
    int ip_protocol;

    while (1) {
#ifdef CONFIG_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 destAddr;
        inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        sock = socket(addr_family, SOCK_STREAM, ip_protocol);

        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            continue;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = connect(sock, (struct sockaddr*) & destAddr, sizeof(destAddr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            shutdown(sock, 0);
            close(sock);
            continue;
        }
        ESP_LOGI(TAG, "Successfully connected");
        xTaskCreate(tcp_client_tx_task, "tcp_client", 4096, NULL, (TCP_CLIENT_PRIORITY), &s_tcp_tx_stream);    //start a task to transmit data at a priority the same as tcp_client_task

        while (1) {

            if(tcp_recv_fun() == MSG_NOT_AVAIL) {
                break;
            }

        }
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
            vTaskDelete(s_tcp_tx_stream);
        }
    }
    vTaskDelete(s_tcp_socket);
    vTaskDelete(s_tcp_tx_stream);
}


static void tcp_client_tx_task(void* pvParameters)
{
    BaseType_t xStatus;
    tcp_tx_data_t payload;            //payload that will be delivered over TCP socket

    while (1) {
        xStatus = xQueueReceive(s_tcp_queue, &payload, portMAX_DELAY);  //wait for items to be available in the queue
        if(xStatus == pdPASS) {
            int err = send(sock, payload.data, payload.data_len, 0);
            
            //If transmission fails continue to attempt to send packets until socket is available once again
            if (err < 0) {
                ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                continue;
            }
        }
        else {
            continue;
        }
    }
}

static message_avail_t tcp_recv_fun()
{
    int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);


    // Error occured during receiving
    if (len < 0) {
        ESP_LOGE(TAG, "recv failed: errno %d", errno);
        //Message is not available
        return MSG_NOT_AVAIL;
    }
    // Data received
    else {
        rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
        ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
        ESP_LOGI(TAG, "%s", rx_buffer);
        xQueueOverwrite(s_tcp_rx_queue, rx_buffer);  //add the received message to the back of the queue
    }
    //Message is available
    return MSG_AVAIL;
}



void wifi_init()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

}


void tcp_stack_start()
{
    ESP_LOGI(TAG, "TCP_CLIENT_TASK_START");
    s_tcp_queue = xQueueCreate(TCP_QUEUE_SIZE, TCP_QUEUE_DATA_SIZE * sizeof( uint8_t ) );       //create queue for transitting data over tcp connection
    s_tcp_rx_queue = xQueueCreate(TCP_QUEUE_SIZE, TCP_RECV_BUFFER );                            //create queue for receiving data over tcp connection

    if(s_tcp_queue != NULL) {
        xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, TCP_CLIENT_PRIORITY, &s_tcp_socket);
    }
}

BaseType_t tcp_tx_data(tcp_tx_data_t data_info)
{
    BaseType_t xStatus;
    xStatus = xQueueSendToBack(s_tcp_queue, &data_info, portMAX_DELAY);
    return xStatus;
}