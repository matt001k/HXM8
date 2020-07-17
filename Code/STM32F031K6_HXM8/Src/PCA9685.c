/*
 * PCA9685.c
 *
 *  Created on: Apr 25, 2020
 *      Author: Matthew kRAUSE
 */

/*
* Code to drive the PCA9685 for servo motor control
*
*
*
*
*
*/

#include "PCA9685.h"
#include "math.h"

/***begin user defined libraries***/
#include "STM32F031x6_I2C_DRIVER.h"
/***end user defined libraries***/

static void pca9685_i2c_interface(pca9685_transmission_mode_t trans_mode, uint8_t device, uint8_t address, uint8_t* data, uint8_t data_len_bytes);

static void pca9685_i2c_interface(pca9685_transmission_mode_t trans_mode, uint8_t device, uint8_t address, uint8_t* data, uint8_t data_len_bytes) {
    i2c_config_t i2c1;
    i2c1.device = device;
    i2c1.address = address;
    i2c1.i2c_mode = I2C_SEND;

    if(trans_mode == PCA9685_SEND) {
	i2c1.data = data;
	i2c1.data_length_bytes = data_len_bytes;

	i2c_master_transmit(i2c1);
    }
    else {
	i2c_config_t i2c2;
	i2c1.data = 0x00;
	i2c1.data_length_bytes = 0x00;
	i2c2.device = device;
	i2c2.address = address;
	i2c2.data = data;
	i2c2.data_length_bytes = data_len_bytes;
	i2c2.i2c_mode = I2C_RECV;

	i2c_master_transmit(i2c1);
	i2c_master_transmit(i2c2);
    }
}


/*USED TO RESET THE PCA9685 MODULE INTO PRE-INITIALIZATION*/
PCA9685_STATUS PCA9685_SWRST(void)
{
    uint8_t cmd = 0x6;

    pca9685_i2c_interface(PCA9685_SEND, PCA9685_SWRST_ADD, cmd, 0x00, 0);

    return PCA9685_STATUS_OK;
}


/*USED TO SET THE BIT OF A MODE1 REGISTER TO THE VALUE INDICATED BY THE USER*/
PCA9685_STATUS PCA9685_SET_BIT(uint8_t channel_Name, uint8_t bit, uint8_t value)
{
    uint8_t tmp;
    if(value) value = 1;

    pca9685_i2c_interface(PCA9685_RECV, PCA9685_ADDRESS, channel_Name, &tmp, 1);

    tmp &= ~((1<<PCA9685_MODE1_RESTART_BIT)|(1<<bit));
    tmp |= (value&1)<<bit;

    pca9685_i2c_interface(PCA9685_SEND, PCA9685_ADDRESS, channel_Name, &tmp, 1);

    return PCA9685_STATUS_OK;
}


/*RESTARTS THE PCA9685, RESET MUST BE SET FOR ENABLE TO CLEAR THE LOGIC TO 0, AN ENABLE UNSET VALUE HAS NO USE*/
PCA9685_STATUS PCA9685_RESTART(uint8_t enable)
{
    return PCA9685_SET_BIT(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, enable);
}


/*SET TO USE THE EXTCLK PIN, THE SLEEP BIT MUST BE SET TO STOP THE OSCILLATOR, THIS BIT IS STICKY AND A SWRST OR
  POWER CYCLE MUST OCCUR IN ORDER TO RESET THE BIT*/
PCA9685_STATUS PCA9685_EXTCLK_SET(uint8_t enable, uint32_t clk_freq, uint16_t update_rate)
{
    PCA9685_SLEEP(1);
    if(PCA9685_SET_BIT(PCA9685_MODE1, PCA9685_MODE1_EXTCLK_BIT, enable) == PCA9685_STATUS_OK)
    {
        if(PCA9685_PWM_CLK_CYCLE(clk_freq, update_rate) == PCA9685_STATUS_OK)
        {
            PCA9685_SLEEP(0);
            return PCA9685_STATUS_OK;
        }
        PCA9685_SLEEP(0);

        return PCA9685_STATUS_ERROR;
    }
    PCA9685_SLEEP(0);

    return PCA9685_STATUS_ERROR;
}


/*SET TO ACTIVATE THE AUTOINCREMENT FUNCTIONALITY WHICH WILL ALLOW THE CONTROL REGISTER TO AUTOMATICALLY
  INCREMENT AFTER A READ OR WRITE TO A REGISTER,ALLOWS FOR MULTIPLE REGISTERS TO BE WRITTEN TO IN ONE TRANMISSION*/
PCA9685_STATUS PCA9685_INCREMENT(uint8_t enable)
{
    return PCA9685_SET_BIT(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, enable);
}


/*LOW POWER MODE. SET FOR OSCILLATOR OFF. MUST BE SET TO CHANGE PWM_CLK_CYCLE OR USE EXTCLK_SET*/
PCA9685_STATUS PCA9685_SLEEP(uint8_t enable)
{
    return PCA9685_SET_BIT(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, enable);
}


/*SET TO RESPOND TO THE I2C SUBADDRESS 1*/
PCA9685_STATUS PCA9685_SUB1_RESP(uint8_t enable)
{
    return PCA9685_SET_BIT(PCA9685_MODE1, PCA9685_MODE1_SUB1_BIT, enable);
}


/*SET TO RESPOND TO THE I2C SUBADDRESS 2*/
PCA9685_STATUS PCA9685_SUB2_RESP(uint8_t enable)
{
    return PCA9685_SET_BIT(PCA9685_MODE1, PCA9685_MODE1_SUB2_BIT, enable);
}


/*SET TO RESPOND TO THE I2C SUBADDRESS 3*/
PCA9685_STATUS PCA9685_SUB3_RESP(uint8_t enable)
{
    return PCA9685_SET_BIT(PCA9685_MODE1, PCA9685_MODE1_SUB3_BIT, enable);
}


/*SET FOR PCA9685 TO RESPOND TO LED ALL CALL I2C BUS ADDRESSES*/
PCA9685_STATUS PCA9685_ALL_CALL(uint8_t enable)
{
    return PCA9685_SET_BIT(PCA9685_MODE1, PCA9685_MODE1_ALLCALL_BIT, enable);
}


/*SETS THE PRESCALE VALUE IN ACCORDANCE TO TO THE UPDATE_RATE, FOR A SERVO MOST PWM FREQUENCIES OF 50Hz*/
PCA9685_STATUS PCA9685_PWM_CLK_CYCLE(uint32_t clk_freq, uint16_t update_rate)
{
    float temp;
    uint8_t prescale;

    if(update_rate >= 1526)
    {
        prescale = 0x03;
    }
    else if(update_rate <= 24)
    {
        prescale = 0xFF;
    }
    else
    {
        temp = ((clk_freq)/(4096.0 * (float)update_rate)) - 1; //EQUATION TO SET THE REGISTER VALUE OF PRESCALE
        prescale = floor(temp + .5);
    }

    PCA9685_SLEEP(1);  //MUST BE PUT TO SLEEP IN ORDER TO SET THE PRESCALE
    pca9685_i2c_interface(PCA9685_SEND, PCA9685_ADDRESS, PCA9685_PRE_SCALE, &prescale, 1);
    PCA9685_SLEEP(0);
    PCA9685_RESTART(1);

    return PCA9685_STATUS_OK;
}


/*WILL SET THE PWM VALUE OF THE CHANNEL*/
PCA9685_STATUS PCA9685_SETPWM(uint8_t channel_Name, uint16_t on, uint16_t off)
{
    uint8_t startRegister;
    uint8_t registerValue[4];

    startRegister = PCA9685_LED0_ON_L + (4*channel_Name); //THE REGISTER ADDRESS IS NOW THE VALUE OF LEDx_ON_L TO ALLOW FOR WRITING OF THE NEXT 4 REGISTERS TO COMPLETE PWM VALUE
    registerValue[0] = on & 0xFF;   //LEDx_ON_L
    registerValue[1] = on >> 8;     //LEDx_ON_H
    registerValue[2] = off & 0xFF;  //LEDx_OFF_L
    registerValue[3] = off >> 8;    //LEDx_OFF_H

    pca9685_i2c_interface(PCA9685_SEND, PCA9685_ADDRESS, startRegister, registerValue, 4);

    return PCA9685_STATUS_OK;
}


/*SETS THE VALUE OF WHAT THE CHANNELS PWM VALUE WILL BE IN DETERMINANCE OF WHEN THE CHANNEL WILL BE ON
  AND WHEN IT WILL TURN OFF, INVERT SET OPTION IS THE OPPOSITE*/
PCA9685_STATUS PCA9685_SETPIN(uint8_t channel_Name, uint16_t value, uint8_t invert)
{
    if(value > 4095) {value = 4095;}
    else if(value < 0) {value = 0;}


    if(invert == 1)
    {
        if(value == 4095)
        {
            return PCA9685_SETPWM(channel_Name, 0, 4095);
        }
        else if (value == 0)
        {
            return PCA9685_SETPWM(channel_Name, 4095, 0);
        }
        else
        {
            return PCA9685_SETPWM(channel_Name, 0, 4095-value);
        }
    }
    else
    {
        if(value == 4095)
        {
            return PCA9685_SETPWM(channel_Name, 4095, 0);
        }
        else if (value == 0)
        {
            return PCA9685_SETPWM(channel_Name, 0, 4095);
        }
        else
        {
            return PCA9685_SETPWM(channel_Name, 0, value);
        }
    }
}


/*SETS THE ANGLE OF THE SERVO MOTOR*/
#ifdef PCA9685_SERVO_ON
PCA9685_STATUS PCA9685_SETANGLE(uint8_t channel_Name, float angle)
{
	float value;
	if(angle < MIN_ANGLE) angle = MIN_ANGLE;
	if(angle > MAX_ANGLE) angle = MAX_ANGLE;

	/*EQUATION USED TO CONVERT ANGLE TO PWM, IN THIS CASE 0 DEGREES = 1MS PULSE AND 180 DEGREES = 2MS PULSE,
	  THEN THESE VALUES ARE CONVERTED TO VALUES FROM 0-4095 TO DETERMINE THE DUTY CYCLE OF THE PULSE*/
	value = (angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;

	return PCA9685_SETPIN(channel_Name, (uint16_t)value, 0);
}
#endif


/*INITIALIZATION FUNCTION FOR THE PCA9685*/
PCA9685_STATUS PCA9685_Init()
{

    PCA9685_SWRST();

#ifdef PCA9685_SERVO_ON
    PCA9685_PWM_CLK_CYCLE(PCA9685_INT_CLK, SERVO_PWM_PERIOD);
#else
    PCA9685_PWM_CLK_CYCLE(PCA9685_INT_CLK, 1000);
#endif
    PCA9685_INCREMENT(1);

    return PCA9685_STATUS_OK;
}
