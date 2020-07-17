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

# Software Design

## MCP2515 CAN Controller 

Further explanation and testing of the CAN Bus Controller and CAN Bus transceiver can be viewed at: 

<p align="center">
  <a href=https://github.com/matt001k/MCP2515_CAN_DRIVER>MCP2515_CAN_DRIVER</a>
</p>

The CAN driver proved to work well communicating between modules. Custom drivers for SPI were created for the STM32 and ESP8266 in order to communicate to the MCP2515 controller.

## ESP8266 and Server Interface

Further explanation and testing of the ESP8266 Wi-Fi communication, along with how information is logged can be viewed at:

<p align="center">
  <a href=https://github.com/matt001k/CAN_WIFI_MODULE>CAN_WIFI_COMMUNICATION</a>
</p>

Messages were able to be communicated to and from the server through the ESP8266 and over the CAN Bus. 

## STM32 and PCA9685 Interface

The PCA9685 is an I2C bus controlled 16-channel PWM controller. It allows for a 12-bit resolution PWM signal that operates at a frequency from 24 Hz to 1526 Hz with a duty cycle that is adjustable from 0 - 100%.  A custom I2C driver was built for the STM32 in order to communicate with the PCA9685. The PCA9685 driver interface was also custom created and can be viewed within the STM32 code files.

An overview of the main functions:

 ```C
PCA9685_STATUS PCA9685_Init();
```
- Used in order to initialize the PCA9685 by setting the prescale value of the clock (user defined internal or external oscillator) to be used for PWM. 

 ```C
PCA9685_STATUS PCA9685_SETANGLE(uint8_t channel_Name, float angle);
```
- Sets the angle of the servo motor of *channel_Name*
- *channel_Name* values of 0 - 15

The following circuit was used in order to test the driver for both the I2C and the PCA9685:

<p align="center">
  <img width="" height="" src="https://github.com/matt001k/HXM8/blob/master/Photos/PCA9685_CIRCUIT.jpg">
</p>

A development board for the PCA9685 was utilized, created by Adafruit. 5V supply was used to power the servo motors and a 3.3V supply was used to power the PCA9685.

Walking motions for the robot were configured at first by simulation within Fusion360 by AutoDesk. Angle values for the legs were found by utilizing simulated movement of joints within the legs. Values were later tweaked when testing occured with the printed components for the legs.

## K210 and Camera Interface

Still needs completion...

## User Interface from Server

The user is able to interface with the HXM8 from the server being ran on the SBC. The user must only run, from the linux terminal:

 ```shell
HXM8_IA.py
```

This will then bring up the interface application as shown below (which is also the help screen):

<p align="center">
  <img width="" height="" src="https://github.com/matt001k/HXM8/blob/master/Photos/SERVER_UI-HELPHOME.PNG">
</p>

This then allows the user to type in any of the commands shown in the command console to:
- Access the different modes of the HXM8
- Display the current day's logging information
- Send the HXM8 back to its home base
- Bring up the help menu
- Exit the user interface

Shown below is the user interface for manual mode when selected:

<p align="center">
  <img width="" height="" src="https://github.com/matt001k/HXM8/blob/master/Photos/SERVER_UI-MANUAL.PNG">
</p>

As can be seen, when the keys are hit to send the robot in different directions, they are printed onto the terminal. When the user wants to exit manual mode, they must only hit n to send HXM8 back into automatic mode.

The next screen show in the logging screen which displays the logging information of that day:

<p align="center">
  <img width="" height="" src="https://github.com/matt001k/HXM8/blob/master/Photos/SERVER_UI-LOG.PNG">
</p>

Logging information is displayed on this screen. This test was show with just the very simple SYSSTART command shown in the ESP8266 and **Server Interface** section.

# Hardware Design

## Circuit Design

### Schematic Overview

All modules from the development kits, the PCA9685, STM32F031K6 and ESP8266, along with the custom created circuits for the MCP2515/MCP2562 were combined into the creation of a PCB. The development circuits were heavily referenced along with much of the material on the datasheets of each module for schematic design.
The schematics for the modules are shown below:

<p align="center">
  <img width="" height="" src="https://github.com/matt001k/HXM8/blob/master/Photos/HXM8_SCHEMATIC_1.PNG">
</p>

The first page of the schematic shows the STM32 and its modules along with the power distribution for the circuit. Incoming power is regulated from two Li-Ion batteries in series. The motors and MCP2562 need a 5V regulated output, and both MCUs, the MCP2515 and the PCA9685 all need a 3.3V regulated output. The power modules were carefully calculated to determine the needed output current. For the 5V regulators, the worse case scenario was observed, if all servo motors had stalled they would be pulling 800mA each, and there still should be enough power to ensure that the two MCP2562's still are able to run with current drawing at 70mA each per IC. The calculated current is as follows:

<p align="center">
  <img src="https://render.githubusercontent.com/render/math?math=(12 * .7A)%2B(2 * .07A) = 9.74A">
</p>

It was decided that 10A output would be needed. In order to do this, two LDO regulaters were used split the current between two sets of six servo motors. One regulator also fed the MCP2562 tranceivers as well. LDO regulators were used instead of a single switching regulator due to space constraints of the PCB and wanting to use just the top layer for component layouts. The two regulators used are the LM1084IT-5.0s by Texas Instruments.

For the 3.3V regulator, the ESP8266 draws 500mA, the STM32F031K6 draws 150mA, the two MCP2515 modules draw 17mA each and the PCA9685 will draw 10mA without any load and since each PWM output is connected to a 220Ohm  resistor, each output will draw 15mA of current. The maximum needed current output is calculated as follows:


<p align="center">
  <img src="https://render.githubusercontent.com/render/math?math=.5A%2B.15A%2B(2 * .017A)%2B(.01A%2B(12 * .015A)) = .874A">
</p>

In order to be safe, a 1A LDO was used, the LMS8117AMP-3.3, by Texas Instruments.

Along with the power supply, CAN interface and PCA9685, three external I2C connectors were used on the board to interface with the STM32, along with an external CAN connection, which will be used to connect to the Kendryte K210 MCU.

Lastly, on the MCP2562, there is a dip switch in place that can be used to terminate the end of the CAN Bus. This will only be used when the ESP8266 and the STM32 are the only controllers being used. When the K210 is introduced the switch can then be opened and terminated at the K210.

<p align="center">
  <img width="" height="" src="https://github.com/matt001k/HXM8/blob/master/Photos/HXM8_SCHEMATIC_2.PNG">
</p>

The second page shows the ESP8266 and its CAN Controller and Tranceiver. The ESP8266 was laid out according to its datasheet. An autoprogramming circuit was designed in order to avoid having to hit the RESET and GPIO0 switches to place the MCU into programming mode. Since there is no internal flash on the ESP8266, it communicates over SPI to a Flash Memory module to read the firmware flashed when programming. 

### PCB Layout

When designing the PCB, a small area was needed in order to keep the body of the HXM8 as small as possible. This was required in order to reduce weight and size of the overall project to ensure movement flowed smoothly. A 4 layer stackup was needed in order to obtain good RF qualities within the board. The 1st layer consisted of all traces for signaling and powering ICs, the second layer was a ground pour for, the third layer was the power plane for 5V and 3.3V and the bottom layer was used for traces and also consisted of a ground pour. The overall dimensions of the PCB ended up being about *70mmX70mm*. The following are the Gerber File layouts of the front and back of the PCB:

<p align="center">
  <img width="" height="" src="https://github.com/matt001k/HXM8/blob/master/Photos/PCB_1.PNG">
</p>

<p align="center">
  <img width="" height="" src="https://github.com/matt001k/HXM8/blob/master/Photos/PCB_2.PNG">
</p>

The ESP8266 was laid out at the top of the PCB in order to allow for the impedence matched circuit and traces to the antenna to be as short as possible to the edge of the board and connected to an SMA connector off the edge of the board. This MCP2515 modules are on the left hand of the board. The STM32 is in the middle and the PCA 9685 is just to the right of that. The connectors for the servo motors are on both sides of the boards. The power distribution is located at the bottom of the board. The two 5V regulators right next to one another and the 3.3V regulator off to the right. LEDs are used to determine when the board is powered up. 
