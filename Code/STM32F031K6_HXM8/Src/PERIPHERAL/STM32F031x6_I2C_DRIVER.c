#include "stm32f031x6.h"
#include "STM32F031x6_I2C_DRIVER.h"

static void i2c_master_send(uint8_t address, uint8_t* data, uint8_t data_length_bytes);
static void i2c_master_recv(uint8_t* data, uint8_t data_length_bytes);


static void i2c_master_send(uint8_t address, uint8_t* data, uint8_t data_length_bytes) {
  //send the address to be accessed/send data
  I2C1->CR2 |= ( (data_length_bytes + 1) << I2C_CR2_NBYTES_Pos ); //set number of bytes to send from slave
  I2C1->CR2 &= ~(I2C_CR2_RD_WRN);
  I2C1->CR2 |= I2C_CR2_START;
  while( !(I2C1->ISR & I2C_ISR_TXE) );
  I2C1->TXDR = address;

  //send the data to the device
  for(int i = 0; i < data_length_bytes; i++) {
	while( !(I2C1->ISR & I2C_ISR_TXE) );
	I2C1->TXDR = *(data + i);
  }

  //wait for transfer to complete and end
  while(!(I2C1->ISR & I2C_ISR_TC));
  I2C1->CR2 |= I2C_CR2_STOP;
}

  static void i2c_master_recv(uint8_t* data, uint8_t data_length_bytes) {
    //start transmission in read mode
    I2C1->CR2 |= ( data_length_bytes << I2C_CR2_NBYTES_Pos ); //set number of bytes to recv from slave
    I2C1->CR2 |= I2C_CR2_RD_WRN;
    I2C1->CR2 |= I2C_CR2_START;

    //recv the data to the device
    for(int i = 0; i < data_length_bytes; i++) {
	while( !(I2C1->ISR & I2C_ISR_RXNE) );
	*(data + i) = I2C1->RXDR;
    }

    //wait for transfer to complete and end
    while(!(I2C1->ISR & I2C_ISR_TC));
    I2C1->CR2 |= I2C_CR2_STOP;
  }



void i2c_master_transmit(i2c_config_t i2c_config) {
  //wait while the bus is busy
  while( I2C1->ISR & I2C_ISR_BUSY );

  //configure the device address to send to
  I2C1->ICR &= ~(I2C_ICR_NACKCF); //clear the nack in ISR
  I2C1->CR2 &= ~(I2C_CR2_SADD);	//clear address register
  I2C1->CR2 |= i2c_config.device;

  //set number of bytes to send to device
  I2C1->CR2 &= ~(I2C_CR2_NBYTES); //clear nbytes

  if(i2c_config.i2c_mode == I2C_SEND) {
      i2c_master_send(i2c_config.address, i2c_config.data, i2c_config.data_length_bytes);
  }
  else {
      i2c_master_recv(i2c_config.data, i2c_config.data_length_bytes);
  }

}

void i2c_master_init() {
  //clock configuration for GPIO
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  //set up the GPIO for I2C
  GPIOA->MODER &= ~( GPIO_MODER_MODER9 | GPIO_MODER_MODER10 );				//disable GPIO in case it is enabled
  GPIOA->MODER |= ( GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1 );			//set up GPIO as alternate mode
  GPIOA->OTYPER |=  ( 1 << 9 | 1 << 10 );
  GPIOA->OSPEEDR |= ( GPIO_OSPEEDR_OSPEEDR9 | GPIO_OSPEEDR_OSPEEDR10 );			//set GPIO speed as high
  GPIOA->PUPDR |= ( GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_0 );			//set GPIO as pull-up
  GPIOA->AFR[1] |= ( (0x04 << GPIO_AFRH_AFSEL9_Pos) | (0x04 << GPIO_AFRH_AFSEL10_Pos) );//set up GPIO as alternate AF4

  //configure the I2C
  I2C1->CR1 &= ~( I2C_CR1_PE );
  I2C1->TIMINGR |= (TIMING_CONF );	//setup the timing register to configure timing
  I2C1->CR1 |= ( I2C_CR1_ANFOFF ); //disable analog filter
  I2C1->CR2 |= ( (ADDRESSING_MODE << I2C_CR2_ADD10_Pos) );
  I2C1->CR1 |= I2C_CR1_PE; //enable peripheral
}
