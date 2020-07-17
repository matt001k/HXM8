/*
* Code to drive the PCA9685 for PWM signals/LED/servo motor control utilizing the
* STM32
*
*
*
*
*/

#include <stdint.h>

#ifndef PCA9685_H
#define PCA9685_H

/*ALLOWS FOR THE USE OF SERVO MOTORS TO BE USED WITHIN THE MODULE*/
#define PCA9685_SERVO_ON
#ifdef PCA9685_SERVO_ON
#define MIN_ANGLE                   0.0
#define MAX_ANGLE                   180.0
#define SERVO_PWM_PERIOD            50
#define SERVO_MIN					100
#define SERVO_MAX					500
#endif /*PCA9685_SERVO_ON*/

#define PCA9685_ADDRESS             0x80

#define PCA9685_SWRST_ADD           0x00


#define PCA9685_LED0_ON_L           0x06

#define PCA9685_ALL_LED_ON_L        0xFA
#define PCA9685_ALL_LED_ON_H        0xFB
#define PCA9685_ALL_LED_OFF_L       0xFC
#define PCA9685_ALL_LED_OFF_H       0xFD


#define PCA9685_MODE1               0x00

#define PCA9685_MODE1_ALLCALL_BIT   	0
#define PCA9685_MODE1_SUB3_BIT      	1
#define PCA9685_MODE1_SUB2_BIT      	2
#define PCA9685_MODE1_SUB1_BIT      	3
#define PCA9685_MODE1_SLEEP_BIT		4
#define PCA9685_MODE1_AI_BIT		5
#define PCA9685_MODE1_EXTCLK_BIT	6
#define PCA9685_MODE1_RESTART_BIT	7


#define PCA9685_PRE_SCALE           0xFE

#define PCA9685_INT_CLK             25000000

typedef enum
{
    PCA9685_SEND = 0,
    PCA9685_RECV
}pca9685_transmission_mode_t;

typedef enum
{
    PCA9685_STATUS_OK = 0,
    PCA9685_STATUS_ERROR = 1
}PCA9685_STATUS;


/*USED TO RESET THE PCA9685 MODULE POWER-UP STATE*/
PCA9685_STATUS PCA9685_SWRST(void);


/*USED TO SET AND RESET REGISTER BITS OF A CHANNEL ON THE PCA9685*/
PCA9685_STATUS PCA9685_SET_BIT(uint8_t channel_Name, uint8_t bit, uint8_t value);


/*MODE1 FUNCTIONS THAT CHANGE THE REGISTER BITS OF REGISTER 0x00*/
PCA9685_STATUS PCA9685_RESTART(uint8_t enable);
PCA9685_STATUS PCA9685_EXTCLK_SET(uint8_t enable, uint32_t clk_freq, uint16_t update_rate);
PCA9685_STATUS PCA9685_INCREMENT(uint8_t enable);
PCA9685_STATUS PCA9685_SLEEP(uint8_t enable);
PCA9685_STATUS PCA9685_SUB1_RESP(uint8_t enable);
PCA9685_STATUS PCA9685_SUB2_RESP(uint8_t enable);
PCA9685_STATUS PCA9685_SUB3_RESP(uint8_t enable);
PCA9685_STATUS PCA9685_ALL_CALL(uint8_t enable);


/*USED TO SET THE CLOCK UPDATE RATE FOR THE PWM */
PCA9685_STATUS PCA9685_PWM_CLK_CYCLE(uint32_t clk_freq, uint16_t update_rate);


/*FUNCTIONS USED TO SET THE PWM VALUES OF THE DIRECTED PIN */
PCA9685_STATUS PCA9685_SETPWM(uint8_t channel_Name, uint16_t on, uint16_t off);
PCA9685_STATUS PCA9685_SETPIN(uint8_t channel_Name, uint16_t value, uint8_t invert);

/*FUNCTION USED TO SET THE ANGLE ON THE SERVO MOTOR*/
#ifdef PCA9685_SERVO_ON
PCA9685_STATUS PCA9685_SETANGLE(uint8_t channel_Name, float angle);
#endif

/*INITIALIZATION FUNCTION FOR THE PCA9685*/
PCA9685_STATUS PCA9685_Init();

#endif /*PCA9685_H*/
