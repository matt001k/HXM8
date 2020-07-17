#include "HXM8_MCP2515_COMMUNICATION.h"
#include "MCP2515_can_controller.h"


/***begin FreeRTOS libraries***/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
/***end FreeRTOS libraries***/

/***begin user configured libraries***/
#include "driver/gpio.h"
#include "esp_log.h"
#include "spi_master_HAL.h"
#include "tcp_client_station.h"
#include <stdlib.h>
/***end user configured libraries***/

/***begin function definitions***/
static void rx_0_isr_handler(void *arg);
static void rx_1_isr_handler(void *arg);
static void can_comm_task_rx(void* pvParameters);
static void wifi_recv_task_rx(void* pvParameters);
/***end function definitions***/

/***begin global variables***/
static const char* TAG = "HXM8_MCP2515_COMM";

static uint32_t* rx_0_identifier;
static uint32_t* rx_0_data;
static uint32_t* rx_0_id_data;

static uint32_t* rx_1_identifier;
static uint32_t* rx_1_data;
static uint32_t* rx_1_id_data;

static QueueHandle_t rx_buff_filled;
static TaskHandle_t can_comm_task_1 = NULL;
static TaskHandle_t wifi_comm_task_1 = NULL;
/***end global variables***/


/***begin ISR handler***/
static void rx_0_isr_handler(void *arg) {
    mcp2515_rx_buffer_read(0, rx_0_identifier, rx_0_data);
    *rx_0_id_data = *rx_0_identifier;
    *(rx_0_id_data + 1) = *(rx_0_data);
    *(rx_0_id_data + 2) = *(rx_0_data + 1);
    
    xQueueSendFromISR(rx_buff_filled, &rx_0_id_data, NULL);
}
static void rx_1_isr_handler(void *arg) {
    mcp2515_rx_buffer_read(1, rx_1_identifier, rx_1_data);
    *rx_1_id_data = *rx_1_identifier;
    *(rx_1_id_data + 1) = *(rx_1_data);
    *(rx_1_id_data + 2) = *(rx_1_data + 1);
    
    xQueueSendFromISR(rx_buff_filled, &rx_1_id_data, NULL);
}
/***end ISR handler***/

/***begin CAN communication task***/
static void can_comm_task_rx(void* pvParameters) {
    BaseType_t xStatus;
    tcp_tx_data_t data_info;    //tcp tx data to send over tcp stream along with the data length, defined in tcp_client_station.h
    data_info.data = (uint32_t*)malloc(3 * sizeof(uint32_t));
    data_info.data_len = 3 * sizeof(uint32_t);

    ESP_LOGI(TAG, "CAN communication online");

    while(1) {
        xStatus = xQueueReceive(rx_buff_filled, &data_info.data, portMAX_DELAY);  //wait for items to be available in the queue
        if(xStatus == pdPASS) {
            /***begin functions for queue handling***/
            ESP_LOGI(TAG, "RX BUFFER DATA: %d + %d RX BUFFER ID: %d", *((uint32_t*)data_info.data + 1), *((uint32_t*)data_info.data + 2), *((uint32_t*)data_info.data));
            tcp_tx_data(data_info);
            //vTaskDelay(15 / portTICK_PERIOD_MS);
            //tcp_tx_data(data + 1);
            //vTaskDelay(15 / portTICK_PERIOD_MS);
            //tcp_tx_data(data + 2);
            //vTaskDelay(15 / portTICK_PERIOD_MS);
            /***end functions for queue handling***/
        }
        else {
            continue;
        }
    }
    free(data_info.data);
}
/***end CAN communication task***/

/***begin custom task(s)***/
static void wifi_recv_task_rx(void* pvParameters) {
    BaseType_t xStatus;
    char payload[TCP_RECV_BUFFER];
    uint32_t dir_forward[2] = {FOR, WARD};
    uint32_t dir_back = BACK;
    uint32_t dir_right[2] = {R, IGHT};
    uint32_t dir_left = LEFT;

    while(1) {
        xStatus = xQueueReceive(s_tcp_rx_queue, payload, portMAX_DELAY);  //wait for items to be available in the received queue
        //if queue message properly received respond to CAN Bus with according message
        if(xStatus == pdPASS) {
            if(!(strcmp ( payload, "forward" ))) {
                mcp2515_fast_tx_message(ID_INFORMATION, dir_forward, 0, 64);
            }
            else if(!(strcmp ( payload, "back" ))) {
                mcp2515_fast_tx_message(ID_INFORMATION, &dir_back, 0, 32);
            }
            else if(!(strcmp ( payload, "left" ))) {
                mcp2515_fast_tx_message(ID_INFORMATION, &dir_left, 0, 32);
            }
            else if(!(strcmp ( payload, "right" ))) {
                mcp2515_fast_tx_message(ID_INFORMATION, dir_right, 0, 64);
            }
            else {
                continue;
            }
        }
    }
}
/***end custom task(s)***/


void hxm8_mcp2515_init(void) {
    rx_0_data = (uint32_t*)malloc(2 * sizeof(uint32_t));
    rx_0_identifier = (uint32_t*)malloc(sizeof(uint32_t));
    rx_0_id_data = (uint32_t*)malloc(3 * sizeof(uint32_t));

    rx_1_data = (uint32_t*)malloc(2 * sizeof(uint32_t));
    rx_1_identifier = (uint32_t*)malloc(sizeof(uint32_t));
    rx_1_id_data = (uint32_t*)malloc(3 * sizeof(uint32_t));

    /***begin SPI initialization***/
    spi_interface_init();
    /***end SPI initialization***/

    /***begin initialize mcp2515 driver***/
    mcp2515_init();
    /***end initialize mcp2515 driver***/
    
    /***begin GPIO configuration***/
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT; 
    io_conf.pull_up_en = 1;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    gpio_config(&io_conf);
    /***end GPIO configuration***/

    /***begin queue handle gpio event from ISR***/
    rx_buff_filled = xQueueCreate(5, sizeof(uint32_t));
    /***end queue handle gpio event from ISR***/

    /***begin accessory tasks***/
    wifi_init();
    tcp_stack_start();
    /***end accessory tasks***/

    /***begin CAN communication task***/
    xTaskCreate(can_comm_task_rx, "can_comm_task_rx", 2048, NULL, CAN_BUS_TASK_PRIORITY, &can_comm_task_1);
    xTaskCreate(wifi_recv_task_rx, "wifi_comm_task_rx", 2048, NULL, CAN_BUS_TASK_PRIORITY, &wifi_comm_task_1);
    /***end CAN communication task***/

    /***begin configure and initialize interrupts for the system***/
    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, rx_0_isr_handler, (void *) 0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, rx_1_isr_handler, (void *) 1);
    /***end configure and initialize interrupts for the system***/
}