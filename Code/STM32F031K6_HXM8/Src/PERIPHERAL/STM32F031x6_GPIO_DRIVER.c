#include "STM32F031x6_GPIO_DRIVER.h"

/***begin global variables***/

/*
 * PORT_RCC_DEF is used in order to determine if the clock is already on for the specified GPIO port
 * How it works:
 *	GPIOA = 0x01 or 00000001
 *	GPIOB = 0x02 or 00000010
 *	GPIOC = 0x04 or 00000100
 *	GPIOF = 0x08 or 00001000
 *
 */
volatile uint8_t port_rcc_def = 0;

/*
 * SYSCFGEN_RCC_DEF can only be zero or one to determine if RCC has been used or not for interrupt
 */
volatile uint8_t syscfgen_rcc_def = 0;

/***end global variables***/

static void rcc_config(GPIO_TypeDef* port);					//used to turn on the clocks for the GPIO port

/***begin function definitions***/
static void rcc_config(GPIO_TypeDef* port) {
  if(port == GPIOA) {
      if(port_rcc_def & 0x01) {

      }
      else {
	  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	  port_rcc_def |= 0x01;
      }
  }
  else if(port == GPIOB) {
      if(port_rcc_def & 0x02) {

      }
      else {
	  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	  port_rcc_def |= 0x02;
      }
  }
  else if(port == GPIOC) {
      if(port_rcc_def & 0x04) {

      }
      else {
	  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	  port_rcc_def |= 0x04;
      }
  }
  else if(port == GPIOF) {
      if(port_rcc_def & 0x08) {

      }
      else {
	  RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
	  port_rcc_def |= 0x08;
      }
  }
}


void gpio_input_init(input_handle_t gpio_handle) {
  rcc_config(gpio_handle.port);

  gpio_handle.port->MODER |= ( 0x00 << (gpio_handle.pin * 2) );
  gpio_handle.port->PUPDR |= ( gpio_handle.pull << (gpio_handle.pin * 2) );

}

void gpio_output_init(output_handle_t gpio_handle) {
  rcc_config(gpio_handle.port);

  gpio_handle.port->MODER &= ~( 0x03 << (gpio_handle.pin * 2) );
  gpio_handle.port->MODER |= ( 0x01 << (gpio_handle.pin * 2) );
  gpio_handle.port->OTYPER |= ( gpio_handle.type << gpio_handle.pin );
  gpio_handle.port->PUPDR |= ( gpio_handle.pull << (gpio_handle.pin * 2) );
  gpio_handle.port->OSPEEDR |= ( gpio_handle.speed << (gpio_handle.pin * 2) );

}

void gpio_set_output_level(output_handle_t gpio_handle, uint8_t level) {
  if (level == 0) {
      gpio_handle.port->BSRR |= ( 1 << (16 + gpio_handle.pin) );
  }
  else {
      gpio_handle.port->BSRR |= ( 1 << (gpio_handle.pin) );
  }


}

uint8_t gpio_get_input_level(input_handle_t gpio_handle) {
  if( gpio_handle.port->IDR & (1 << gpio_handle.pin) ) {
      return 1;
  }
  else {
      return 0;
  }

}

void gpio_interrupt_init(input_handle_t gpio_handle, interrupt_trigger_type_t trigger, uint8_t priority) {
  uint8_t port_selection;
  uint8_t interrupt_handler_selection;

  if( port_rcc_def == 0) {
      RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  }

  if(gpio_handle.port == GPIOA) {
      port_selection = 0x00;
  }
  else if(gpio_handle.port == GPIOB) {
      port_selection = 0x01;
  }
  else if(gpio_handle.port == GPIOC) {
      port_selection = 0x02;
  }
  else {
      port_selection = 0x05;
  }


  SYSCFG->EXTICR[gpio_handle.pin / 4] &= ~( 0x0F << ((gpio_handle.pin % 2) * 4) );
  SYSCFG->EXTICR[gpio_handle.pin / 4] |= (port_selection << ((gpio_handle.pin % 2) * 4) );

  EXTI->IMR |= (1 << gpio_handle.pin);

  if(trigger == INTERRUPT_RISING_EDGE) {
      EXTI->RTSR |= (1 << gpio_handle.pin);
      EXTI->FTSR &=  ~(1 <<  gpio_handle.pin);
  }
  else {
      EXTI->RTSR &= ~(1 << gpio_handle.pin);
      EXTI->FTSR |=  (1 <<  gpio_handle.pin);
  }

  //determine the interrupts selection
  if(gpio_handle.pin < 2) {
      interrupt_handler_selection = EXTI0_1_IRQn;
  }
  else if(gpio_handle.pin < 4) {
      interrupt_handler_selection = EXTI2_3_IRQn;
  }
  else {
      interrupt_handler_selection = EXTI4_15_IRQn;
  }

  //set priority of interrupt and enable the IRQ
  NVIC_SetPriority(interrupt_handler_selection, priority);
  NVIC_EnableIRQ(interrupt_handler_selection);

  //ISR must now be configured in the NVIC files
}
/***end function definitions***/
