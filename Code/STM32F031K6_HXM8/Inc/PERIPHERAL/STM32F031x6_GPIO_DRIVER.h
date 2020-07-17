#ifndef STM32F031X6_GPIO_DRIVER
#define STM32F031X6_GPIO_DRIVER

#include "stm32f031x6.h"

/***BEGIN USER INCLUDED LIBRARIES***/
#include "main.h"
/***END USER INCLUDED LIBRARIES***/

/***BEGIN SYSTEM VARIABLES***/
typedef enum {
  GPIO_PUSH_PULL = 0,
  GPIO_OPEN_DRAIN
}output_type_t;

typedef enum {
  GPIO_LOW_SPEED = 0,
  GPIO_MED_SPEED = 1,
  GPIO_HIGH_SPEED = 3
}speed_type_t;

typedef enum {
  GPIO_NO_PULL = 0,
  GPIO_PULLUP,
  GPIO_PULLDOWN
}pull_type_t;

typedef enum {
  GPIO_RESET = 0,
  GPIO_SET
}output_state_t;

typedef struct {
  GPIO_TypeDef* port;		//define as: GPIOA, GPIOB, GPIOC, GPIOF
  uint8_t pin;
  pull_type_t pull;
}input_handle_t;

typedef struct {
  GPIO_TypeDef* port;		//define as: GPIOA, GPIOB, GPIOC, GPIOF
  uint8_t pin;
  pull_type_t pull;
  output_type_t type;
  speed_type_t speed;
}output_handle_t;

typedef enum {
  INTERRUPT_RISING_EDGE = 0,
  INTERRUPT_FALLING_EDGE
}interrupt_trigger_type_t;
/***END SYSTEM VARIABLES***/

void gpio_input_init(input_handle_t gpio_handle);
void gpio_output_init(output_handle_t gpio_handle);
void gpio_set_output_level(output_handle_t gpio_handle, uint8_t level);
uint8_t gpio_get_input_level(input_handle_t gpio_handle);

void gpio_interrupt_init(input_handle_t gpio_handle, interrupt_trigger_type_t trigger, uint8_t priority);

#endif
