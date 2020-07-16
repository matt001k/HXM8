# HXM8
A quadra-pod robot designed to autonomously roam around a living space. Elements of the project include PCB design, utilization of a CAN interface, IEEE 802.11 interface, servo motor control, machine vision, as well utilization of TOF (Time of Flight) sensors. Project will be updated throughout the process of the project.

## System Overview
The HXM8 consists of three MCUs, an ARM Cortex-M0 STMicroelectronic STM32F031K6, Espressif ESP8266 and Kendryte RISC-V K210. The system overview is displayed below.

<p align="center">
  <img width="" height="" src="https://github.com/matt001k/HXM8/blob/master/Photos/System%20Overview.png">
</p>

The STM32 utilizes I2C to communicate to a PCA9685 controller that is mainly used for driving LEDs, but can be used to drive any PWM signals from 16 outputs on the PCA9685. This is being used to control 12 servo motors on the legs in order to create movement for the robot. It also utilizes SPI to communicate to an MCP2515 CAN Bus controller in order to create a CAN Bus interface. This controller is able to transmit and receive messages over the CAN Bus, the baud rate is customizable, there are two receiving buffers and three transmit buffers. The two receiving buffers are configured to set off interrupts when messages are received in the buffer which are then run through an ISR in the controller. The CAN controller is connected to an MCP2562 that will create the differential voltage bus to the other transceivers and controllers on the bus.


The STM32 is responsible for movement of the robot. It will take commands from the other 2 MCUs over the CAN Bus and determine which movement pattern the servo motors will be responsible for.
