#include "HXM8_MCP2515_COMMUNICATION.h"
#include "MCP2515_can_controller.h"


/***begin function definitions***/
void EXTI0_1_IRQHandler(void);
static void can_comm_task_rx(void* pvParameters);
/***end function definitions***/

/***begin global variables***/
static uint32_t* rx_0_identifier;
static uint32_t* rx_0_data;
static uint32_t* rx_0_id_data;

static uint32_t* rx_1_identifier;
static uint32_t* rx_1_data;
static uint32_t* rx_1_id_data;

static QueueHandle_t rx_buff_filled;
static TaskHandle_t can_comm_task_1 = NULL;
/***end global variables***/


/***begin ISR handler***/
void EXTI0_1_IRQHandler(void) {
    mcp2515_rx_buffer_read(0, rx_0_identifier, rx_0_data);

    *rx_0_id_data = *rx_0_identifier;
    *(rx_0_id_data + 1) = *(rx_0_data);
    *(rx_0_id_data + 2) = *(rx_0_data + 1);

    xQueueSendFromISR(rx_buff_filled, &rx_0_id_data, NULL);

    if (EXTI->PR & (1 << 1)) {
      EXTI->PR |= (1 << 1);
    }
}
/***end ISR handler***/

/***begin CAN communication task***/
static void can_comm_task_rx(void* pvParameters) {
    BaseType_t xStatus;
    uint32_t *data;
    data = (uint32_t*)malloc(3 * sizeof(uint32_t));

    mcp2515_fast_tx_message(ID_INFORMATION, (uint32_t*)MSG_START, 0, 64);


    while(1) {
        xStatus = xQueueReceive(rx_buff_filled, &data, portMAX_DELAY);  //wait for items to be available in the queue

        if(xStatus == pdPASS) {
            /***begin functions for queue handling***/
	    if(*(data + 1) == FOR) {
		if(*(data + 2) == WARD) {
		    hxm8_direction = 1;
		}
	    }
	    else if(*(data + 1) == BACK) {
		hxm8_direction = 0;
	    }
	    else if(*(data + 1) == R) {
		if(*(data + 2) == IGHT) {
		    hxm8_direction = 1;
		}
	    }
	    else {
		hxm8_direction = 0;
	    }
            /***end functions for queue handling***/
        }
        else {
            continue;
        }
    }
    free(data);
}
/***end CAN communication task***/


void hxm8_mcp2515_init(void) {
    rx_0_data = (uint32_t*)malloc(2 * sizeof(uint32_t));
    rx_0_identifier = (uint32_t*)malloc(sizeof(uint32_t));
    rx_0_id_data = (uint32_t*)malloc(3 * sizeof(uint32_t));

    rx_1_data = (uint32_t*)malloc(2 * sizeof(uint32_t));
    rx_1_identifier = (uint32_t*)malloc(sizeof(uint32_t));
    rx_1_id_data = (uint32_t*)malloc(3 * sizeof(uint32_t));

    /***begin SPI initialization***/
    spi_master_init();
    /***end SPI initialization***/

    /***begin initialize mcp2515 driver***/
    mcp2515_init();
    /***end initialize mcp2515 driver***/

    /***begin GPIO configuration***/
    input_handle_t rx_interrupt_0;
    rx_interrupt_0.port = GPIOA;
    rx_interrupt_0.pin = 1;
    rx_interrupt_0.pull = GPIO_PULLUP;

    gpio_input_init(rx_interrupt_0);

    /***end GPIO configuration***/

    /***begin queue handle gpio event from ISR***/
    rx_buff_filled = xQueueCreate(5, sizeof(uint32_t));
    /***end queue handle gpio event from ISR***/

    /***begin accessory tasks***/

    /***end accessory tasks***/

    /***begin CAN communication task***/
    xTaskCreate(can_comm_task_rx, "can_comm_task_rx", 128, NULL, CAN_BUS_TASK_PRIORITY, &can_comm_task_1);
    /***end CAN communication task***/

    /***begin configure and initialize interrupts for the system***/
    gpio_interrupt_init(rx_interrupt_0, INTERRUPT_FALLING_EDGE, 0);
    /***end configure and initialize interrupts for the system***/
}
