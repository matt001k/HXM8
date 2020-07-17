# HXM8
A quadra-pod robot designed to autonomously roam around a living space. Elements of the project include PCB design, utilization of a CAN interface, IEEE 802.11 interface, servo motor control, machine vision, as well utilization of TOF (Time of Flight) sensors. Project will be updated throughout the process of the project.

## System Overview
The HXM8 consists of three MCUs, an ARM Cortex-M0 STMicroelectronic STM32F031K6, Espressif ESP8266 and Kendryte RISC-V K210. The system overview is displayed below.

<p align="center">
  <img width="" height="" src="https://github.com/matt001k/HXM8/blob/master/Photos/System%20Overview.png">
</p>

#### STM32F031K6T7
The STM32 utilizes I2C to communicate to a PCA9685 controller that is mainly used for driving LEDs, but can be used to drive any PWM signals from 16 outputs on the PCA9685. This is being used to control 12 servo motors on the legs in order to create movement for the robot. It also utilizes SPI to communicate to an MCP2515 CAN Bus controller in order to create a CAN Bus interface. This controller is able to transmit and receive messages over the CAN Bus, the baud rate is customizable, there are two receiving buffers and three transmit buffers. The two receiving buffers are configured to set off interrupts when messages are received in the buffer which are then run through an ISR in the controller. The CAN controller is connected to an MCP2562 that will create the differential voltage bus to the other transceivers and controllers on the bus.

#### ESP8266
The ESP8266 is responsible for all communication to and from a server being run on an SBC. Commmunication will be done via a TCP socket that will close once communication is lost with the HXM8. It will send information/error logging information to the server as well as interface with the server for manual control of the HXM8. Also, the ESP8266 will communicate over the CAN Bus to the STM32 and K210 utilizing the MCP2515 and MCP2562.

#### K210
The K210, being able to run complex convulutional neural network operations, will be utilizing an interface with a camera, along with 2 TOF sensors used in order to determine if objects are at either side of HXM8 in order to run an algorithm to determine the movement pattern of the HXM8. The movement description will be transmitted to the HXM8 over the CAN Bus to the STM32 in order to tell which servos to fire and when. Any error information will be sent to the ESP8266 to be stored on the server.

## HXM8 Movement
The HXM8 will follow the accordingly:

<p align="center">
  <img width="" height="" src="https://github.com/matt001k/HXM8/blob/master/Photos/Robot%20Flow%20Chart.JPG">
</p>

The STM32 is responsible for movement of the robot. It will take commands from the other 2 MCUs over the CAN Bus and determine which movement pattern the servo motors will be responsible for. The user can determine what mode HXM8 will be in. There are 3 modes; auto, manual and sleep. In auto, HXM8 relies on the computer vision and TOF sensors to navigate around a space and make decisions on where to go. In manual mode, the user has full control over HXM8. Commands for directions are sent to the robot over the TCP interface to the ESP8266, and from over the ESP8266 to the STM32 by utilizing the CAN Bus.

## MCP2515 CAN Controller 

Further explanation and testing of the CAN Bus Controller and CAN Bus transceiver can be viewed at: 

<p align="center">
  <a href=https://github.com/matt001k/MCP2515_CAN_DRIVER>MCP2515_CAN_DRIVER</a>
</p>

The CAN driver proved to work well communicating between modules.

## ESP8266 Server Interface Testing

Further explanation and testing of the ESP8266 Wi-Fi communication, along with how information is logged can be viewed at:

<p align="center">
  <a href=https://github.com/matt001k/CAN_WIFI_MODULE>CAN_WIFI_COMMUNICATION</a>
</p>

Messages were able to be communicated to and from the server through the ESP8266 and over the CAN Bus. 

## STM32 and PCA 9685 Interface


## Circuit Design


