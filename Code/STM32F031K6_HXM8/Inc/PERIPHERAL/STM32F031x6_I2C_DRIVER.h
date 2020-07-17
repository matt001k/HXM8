#ifndef STM32F031X6_I2C_DRIVER
#define	STM32F031X6_I2C_DRIVER

#include "stm32f031x6.h"

/***BEGIN USER INCLUDED LIBRARIES***/
#include "main.h"
/***END USER INCLUDED LIBRARIES***/

/***USER DEFINED PREPROCESSOR DEFINITIONS***/

#define ADDRESSING_MODE				0		//set if addressing mode is 10 bit
#define TIMING_CONF				0x00101D2D	//configuration for timing registers for I2C speed register, values found from excel cheatsheet

/***END USER DEFINED PREPROCESSOR DEFINITIONS***/


/***SYSTEM VARIABLES***/

typedef enum {
  I2C_SEND = 0,
  I2C_RECV
}i2c_mode_t;

typedef struct {
  i2c_mode_t i2c_mode;
  uint8_t device;
  uint8_t address;
  uint8_t* data;
  uint8_t data_length_bytes;
}i2c_config_t;

/***END SYSTEM VARIABLES***/


/***SPI SYSTEM FUNCTIONS***/

void i2c_master_init();

void i2c_master_transmit(i2c_config_t i2c_config);

/***END SPI SYSTEM FUNCTIONS***/

#endif
