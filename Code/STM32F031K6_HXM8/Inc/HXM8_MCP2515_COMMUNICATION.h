#ifndef HXM8_MCP2515_COMMUNICATION
#define HXM8_MCP2515_COMMUNICATION

/***begin FreeRTOS libraries***/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/***end FreeRTOS libraries***/


/***begin user configured libraries***/
#include "stm32f031x6.h"
#include "STM32F031x6_GPIO_DRIVER.h"
#include "STM32F031x6_SPI_DRIVER.h"
#include <stdlib.h>
/***end user configured libraries***/


/*BEGIN USER DEFINED MESSAGES OVER CAN*/

//systart
#define MSG_START				"SYSSTART"

//forward direction definition
#define FOR                                     0x00664F52
#define WARD                                    0x57415244

//back direction definition
#define BACK                                    0x6261636B

//right direction definition
#define R                                       0x00000072
#define IGHT                                    0x69676874

//left direction definition
#define LEFT                                    0x6C656674

/*END USER DEFINED MESSAGES OVER CAN*/


/*BEGIN IDENTIFIER DEFINITIONS*/
#define ID_INFORMATION                          4
#define ID_WARNING                              5
#define ID_ERROR                                6
#define ID_CRITICAL                             7
/*END IDENTIFIER DEFINITIONS*/


/*BEGIN USER CONFIGURATION PARAMETERS*/
#define CAN_BUS_TASK_PRIORITY                   5  //PRIORITY OF THE CAN BUS TASK
/*END USER CONFIGURATION PARAMETERS*/

/*USER DEFINED PREPROCESSOR*/
#define GPIO_INPUT_IO_0                         4
#define GPIO_INPUT_IO_1                         5
#define GPIO_INPUT_PIN_SEL                      ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
/*END USER DEFINED PREPROCESSOR*/

volatile uint8_t hxm8_direction;

void hxm8_mcp2515_init(void);

#endif
