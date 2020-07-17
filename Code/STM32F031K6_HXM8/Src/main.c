/*
 ******************************************************************************
 * @file           : main.c
 * @author         : Matt Krause
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include"main.h"


static TaskHandle_t xTaskMovement = NULL;
static TaskHandle_t xTaskCAN = NULL;

static void vMovementTask(void* pvParameters);
static void vCANTask(void* pvParameters);

static void direction_forward(void);


static void direction_forward(void) {
  PCA9685_SETANGLE(1, 60);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  PCA9685_SETANGLE(0, 135);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  PCA9685_SETANGLE(2, 170);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  PCA9685_SETANGLE(1, 0);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  PCA9685_SETANGLE(0, 45);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  PCA9685_SETANGLE(2, 140);
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

static void vMovementTask(void* pvParameters) {
  i2c_master_init();
  PCA9685_Init();


  while(1) {
      if(hxm8_direction == 1) {
	  direction_forward();
      }
      else {
	  continue;
      }

  }
}

static void vCANTask(void* pvParameters) {
    spi_master_init();
    mcp2515_init();

    /*uint32_t *data;
    data = (uint32_t*)(malloc(2 * sizeof(uint32_t)));
    *(data + 1) = 0x54415254;
    *(data) = 0x53595353;*/

    uint32_t identifier = 0x403;
    uint32_t rtr_bit = 0x00;
    char* data = MSG_START;

    while(1) {
	mcp2515_fast_tx_message(identifier, (uint32_t*)data, rtr_bit, 64);
	vTaskDelay(10000);

     }
     free(data);
}


int main() {
	/*
	 * RUNNING TOTAL FOR MEMORY ALLOCATION FOR TASKS
	 * WALK = 512B
	 * CAN = 512B
	 * IDLE = 128B
	 * TOTAL = 1152B
	 *
	 * OUT OF: 2000B
	 */

	xTaskCreate(vMovementTask, "Walk", 128, NULL, configMAX_PRIORITIES, &xTaskMovement);
	//xTaskCreate(vCANTask, "CAN", 128, NULL, configMAX_PRIORITIES, &xTaskCAN);

	hxm8_mcp2515_init();


	vTaskStartScheduler();


	return 0;
}
