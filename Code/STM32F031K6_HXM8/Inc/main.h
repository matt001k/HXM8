#ifndef MAIN_H
#define MAIN_H

#include "system_stm32f0xx.h"
#include "STM32F031x6_SPI_DRIVER.h"
#include "STM32F031x6_I2C_DRIVER.h"
#include "STM32F031x6_GPIO_DRIVER.h"
#include "MCP2515_can_controller.h"
#include "HXM8_MCP2515_COMMUNICATION.h"
#include "PCA9685.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>

#ifdef FREERTOS_CONFIG_H
#define malloc(size) pvPortMalloc(size)
#define free(ptr) vPortFree(ptr)
#endif


#endif
