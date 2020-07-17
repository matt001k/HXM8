/*	SPI DRIVER HEADER FILE FOR STM32F031X6 CHIP
*
*	DRIVER TAKES IN INFORMATION FROM A STRUCTURE, spi_config_t,
*	AND USES THIS TO SEND AND RECEIVE INFORMATION BACK AND FORTH
*	BETWEEN DEVICES. THIS DEVICE COMMUNICATES WITH ONE SS PIN.
*
*
*
*
*	CREATED BY:
*			MATTHEW KRAUSE(2020)
*
*/

#ifndef STM32F031X6_SPI_H
#define STM32F031X6_SPI_H

#include "stm32f031x6.h"

/***BEGIN USER INCLUDED LIBRARIES***/
#include "main.h"
/***END USER INCLUDED LIBRARIES***/

/***USER DEFINED PREPROCESSOR DEFINITIONS***/

#define BAUD_RATE_CTRL					SPI_CR1_BR_1		//DEFINE THE BAUD RATE PRESCALE TO BE USED FOR SPI
#define FRAME_FORMAT					0			//DEFINE ORDER OF DATA BEING SENT, 0 FOR MSB FIRST, 1 FOR LSB FIRST

#define CPHA_ENABLE					0			//DEFINE CPHA AS 1 OR 0
#define CPOL_ENABLE					0			//DEFINE CPOL AS 1 OR 0

#define SPI_READ_MSG					0U			//WHAT WILL BE SENT ON MOSI WHEN READING FROM SLAVE DEVICE

/***END USER DEFINED PREPROCESSOR DEFINITIONS***/

/***SYSTEM VARIABLES***/

typedef enum {
	SPI_SEND = 0,
	SPI_RECV
}spi_mode_t;

typedef struct {
	uint8_t cmd;
	uint8_t addr;
	uint8_t* data;
	uint8_t cmd_len;
	uint16_t data_len;
	spi_mode_t spi_mode;
}spi_config_t;

/***END SYSTEM VARIABLES***/


/***SPI SYSTEM FUNCTIONS***/

void spi_master_init(void);					//initialize spi communications

void spi_master_transmit(spi_config_t spi_trans);		//transmit data through data type spi_config_t

/***END SPI SYSTEM FUNCTIONS***/


#endif
