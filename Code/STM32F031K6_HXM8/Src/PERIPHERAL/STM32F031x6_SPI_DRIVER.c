/*	SPI DRIVER HEADER FILE FOR STM32F031x6 CHIP
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

#include "stm32f031x6.h"
#include <stdlib.h>
#include "STM32F031x6_SPI_DRIVER.h"

static void spi_master_send(uint8_t* data, uint32_t data_len_bytes);								//will send data to slave
static void spi_master_send_recv(uint8_t* cmd_addr, uint8_t* data, uint32_t cmd_len_bytes, uint32_t data_len_bytes);		//will send command and address to slave and receive data back after

static void spi_master_send(uint8_t* data, uint32_t data_len_bytes) {
  while( (SPI1->SR & SPI_SR_BSY) );

  GPIOB->BSRR |= GPIO_BSRR_BR_1;
  for(int i = 0; i < data_len_bytes; i++) {
	  while( !(SPI1->SR & SPI_SR_TXE) );
	  *(uint8_t*)&(SPI1->DR) = *(data + i);
  }
  while( (SPI1->SR & SPI_SR_BSY) );
  GPIOB->BSRR |= GPIO_BSRR_BS_1;
}

static void spi_master_send_recv(uint8_t* cmd_addr, uint8_t* data, uint32_t cmd_len_bytes , uint32_t data_len_bytes) {
  if( ((SPI1->SR & SPI_SR_FRLVL) >> SPI_SR_FRLVL_Pos) > 0 ) {		//if anything is stored in rx buffer
      while(SPI1->SR & SPI_SR_RXNE) {
	  uint8_t temp = *(uint8_t*)&(SPI1->DR);			//if anything is stored in rx buffer, throw it away
      }
  }

  while( (SPI1->SR & SPI_SR_BSY) );
  GPIOB->BSRR |= GPIO_BSRR_BR_1;
  for(int i = 0; i < data_len_bytes; i++) {
    while( !(SPI1->SR & SPI_SR_TXE) );
    *(uint8_t*)&(SPI1->DR) = *(cmd_addr + i); 				//send command and address over buffer MOSI
    while( !(SPI1->SR & SPI_SR_RXNE) );
    *(data + i) = *(uint8_t*)&(SPI1->DR);				//receive data over MISO
  }
  while( (SPI1->SR & SPI_SR_BSY) );
  GPIOB->BSRR |= GPIO_BSRR_BS_1;

  for(int i = 0; i < data_len_bytes; i++) {				//eliminate the received data from the transferred commands
      *(data + i) = *(data + i + cmd_len_bytes);
  }
}


void spi_master_transmit(spi_config_t spi_trans) {
  uint32_t total_len = spi_trans.cmd_len + spi_trans.data_len;
  uint8_t *data;
  data = (uint8_t*)( malloc( (total_len/8) * sizeof(uint8_t)) );
  int j = 1;

  *data = spi_trans.cmd;
  if(spi_trans.cmd_len > 8) {
	  *(data + j) = spi_trans.addr;
	  j++;
  }

  if(spi_trans.spi_mode == SPI_SEND) {
	  for(int i = 0; i < (spi_trans.data_len / 8); i++) {
		  *(data + i + j) = *(spi_trans.data + i);
	  }
	  spi_master_send(data, (total_len / 8));
  }
  else {
	  for(int i = 0; i < (spi_trans.data_len / 8); i++) {
		  *(data + i + j) = SPI_READ_MSG;
	  }
	  spi_master_send_recv(data, spi_trans.data, (spi_trans.cmd_len / 8), (total_len / 8));
  }
  free(data);
}

void spi_master_init(void) {
  //clock configuration for GPIO
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //turn on clock for SPI interface
  RCC->AHBENR   |= RCC_AHBENR_GPIOBEN;

  //set up the GPIO for SS
  GPIOB->MODER &= ~( GPIO_MODER_MODER1 ); //clear GPIOB pin 1
  GPIOB->MODER |= (GPIO_MODER_MODER1_0); 	//set GPIOB pin 1 as output mode
  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_1); 	//output push-pull
  GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR1;//configure speed of pin as fast
  GPIOB->PUPDR |= GPIO_PUPDR_PUPDR1_0; //GPIOB pin 1 will be pulled-up
  GPIOB->BSRR |= GPIO_BSRR_BS_1;


  //set up the GPIO for SPI
  GPIOB->MODER &= ~( GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5); //clear GPIO registers
  GPIOB->AFR[0] &= ~( GPIO_AFRL_AFSEL3 | GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5); //set modes of MOSI, MISO and SCK for SPI as alternate modes
  GPIOB->OSPEEDR |= ( GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5 ); //configure speed of pins
  GPIOB->MODER |= ( GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1); //set modes of MOSI, MISO and SCK for SPI as alternate modes

  //configure the SPI
  SPI1->CR1 &= ~(SPI_CR1_SPE); //make sure that the peripheral is off
  RCC->APB2RSTR |=  (RCC_APB2RSTR_SPI1RST);
  RCC->APB2RSTR &= ~(RCC_APB2RSTR_SPI1RST);
  SPI1->CR1 = ( SPI_CR1_SSM | SPI_CR1_SSI | (FRAME_FORMAT << SPI_CR1_LSBFIRST_Pos) | BAUD_RATE_CTRL | SPI_CR1_MSTR | (SPI_CR1_CPHA & CPHA_ENABLE) | (SPI_CR1_CPOL & CPOL_ENABLE) );
  SPI1->CR2 |= ( (0x07UL << SPI_CR2_DS_Pos) | SPI_CR2_SSOE | SPI_CR2_FRXTH);
  SPI1->CR1 |= SPI_CR1_SPE;
}
