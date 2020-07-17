#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "tcp_client_station.h"
#include "HXM8_MCP2515_COMMUNICATION.h"
#include "spi_master_HAL.h"
#include "MCP2515_can_controller.h"


void app_main()
{
    hxm8_mcp2515_init();
    
    //wifi_init();

    //tcp_stack_start();
    
    /*uint32_t *data;
    data = (uint32_t*)(malloc(2 * sizeof(uint32_t)));
    *(data + 1) = 0x54415254;
    *(data) = 0x53595353;

    uint32_t identifier = 0x04;
    uint32_t rtr_bit = 0x00;



    spi_interface_init();
    mcp2515_init();

    while (1) {
    mcp2515_fast_tx_message(identifier, data, rtr_bit, 64);
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    }*/

}

