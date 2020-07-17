#ifndef MCP2515_CAN_CONTROLLER
#define MCP2515_CAN_CONTROLLER

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif



/* System Preprocessor Definitions */

/* System Constraints, configurable by user */
#define MCP2515_OSC_FREQ                        20000000    //Oscillator frequency for the CAN controller
#define CAN_BUS_BIT_RATE                          125000    //CAN line bit rate for TX and RX
#define TIME_QUANTA_AMOUNT                            16    //Number of time Quanta for timer configuration
#define PROP_SEGMENT_QUANTA_AMOUNT                     2    //Length of propagation segment amount
#define CLOCK_SAMPLE_MODE                              1    //Sample mode, number of samples to be taken at sample point pg 44

#define TX_RTS_PIN_0_ENABLE                            0    //Set to determine if TX_RTS_PIN_0 will be used for data transmission
#define TX_RTS_PIN_1_ENABLE                            0    //Set to determine if TX_RTS_PIN_0 will be used for data transmission
#define TX_RTS_PIN_2_ENABLE                            0    //Set to determine if TX_RTS_PIN_0 will be used for data transmission
#define TX_ONE_SHOT_ENABLE                             0    //Set to enable one shot mode, the controller will only send one message per transmit

#define RX_0_FILTER_MASK_ENABLE                        3    //Set to 3 to turn off filters and masks for RX 0 Buffer, 0 will turn on filters and masks
#define RX_1_FILTER_MASK_ENABLE                        3    //Set to 3 to turn off filters and masks for RX 1 Buffer, 0 will turn on filters and masks
#define RX_BUFFER_ROLLOVER                             0    //Determine if the received data will rollover from buffer 0 to buffer 1
#define RX_0_INTR_ENABLE                              10    //Set to 10 to activate interrupt on RX 0 pin
#define RX_1_INTR_ENABLE                               5    //Set to 5 to activate interrupt on RX 1 pin


#define RX_BUF_STA_0_MASK_CTR_DATA                 0x000    //Writes this standard mask to control rx buffer 0 see page 37
#define RX_BUF_STA_1_MASK_CTR_DATA                 0x000    //Writes this standard mask to control rx buffer 1 see page 37
#define RX_BUF_EXT_0_MASK_CTR_DATA               0x00000    //Writes this extended mask to control rx buffer 0 see page 38
#define RX_BUF_EXT_1_MASK_CTR_DATA               0x00000    //Writes this extended mask to control rx buffer 1 see page 38
#define RX_BUF_STA_0_FILTER_CTR_DATA               0x000    //Writes this standard filter to filter out data of rx buffer 0 see page 36
#define RX_BUF_STA_1_FILTER_CTR_DATA               0x000    //Writes this standard filter to filter out data of rx buffer 0 see page 36
#define RX_BUF_STA_2_FILTER_CTR_DATA               0x000    //Writes this standard filter to filter out data of rx buffer 1 see page 36
#define RX_BUF_STA_3_FILTER_CTR_DATA               0x000    //Writes this standard filter to filter out data of rx buffer 1 see page 36
#define RX_BUF_STA_4_FILTER_CTR_DATA               0x000    //Writes this standard filter to filter out data of rx buffer 1 see page 36
#define RX_BUF_STA_5_FILTER_CTR_DATA               0x000    //Writes this standard filter to filter out data of rx buffer 1 see page 36

#define RX_BUF_EXT_0_FILTER_CTR_DATA             0x00000    //Writes this standard filter to filter out data of rx buffer 0 see page 36
#define RX_BUF_EXT_1_FILTER_CTR_DATA             0x00000    //Writes this standard filter to filter out data of rx buffer 0 see page 36
#define RX_BUF_EXT_2_FILTER_CTR_DATA             0x00000    //Writes this standard filter to filter out data of rx buffer 1 see page 36
#define RX_BUF_EXT_3_FILTER_CTR_DATA             0x00000    //Writes this standard filter to filter out data of rx buffer 1 see page 36
#define RX_BUF_EXT_4_FILTER_CTR_DATA             0x00000    //Writes this standard filter to filter out data of rx buffer 1 see page 36
#define RX_BUF_EXT_5_FILTER_CTR_DATA             0x00000    //Writes this standard filter to filter out data of rx buffer 1 see page 36

#define INTR_ENABLE_DATA                            0x00    //What interrupts to be enabled on the CANINTE register see pg 53
/* End System Constraints */



/* Instruction Definition */
/* Command Definition of Instructions */
#define MCP2515_RESET                               0xC0    //Resets internal registers to the default state, sets Configuration mode

#define MCP2515_READ                                0x03    //Reads data from the register beginning at selected address              
#define MCP2515_READ_RX_BUF_0_RXB0SIDH              0x90    //Receive buffer 0 standard identifier range high
#define MCP2515_READ_RX_BUF_0_RXBOD0                0x92    //Receive buffer 0 data byte registers   
#define MCP2515_READ_RX_BUF_1_RXB1SIDH              0x94    //Receive buffer 1 standard identifier range high
#define MCP2515_READ_RX_BUF_1_RXBOD1                0x96    //Receive buffer 1 data byte registers   

#define MCP2515_WRITE                               0x02    //Writes data to the register beginning at the selected address
#define MCP2515_LOAD_TX_BUF_0_TXB0SIDH              0x40    //Transmit buffer 0 standard identifier register high          
#define MCP2515_LOAD_TX_BUF_0_TXB0D0                0x41    //Transmit buffer 0 data byte register
#define MCP2515_LOAD_TX_BUF_1_TXB1SIDH              0x42    //Transmit buffer 1 standard identifier register high
#define MCP2515_LOAD_TX_BUF_1_TXB1D0                0x43    //Transmit buffer 1 data byte register 
#define MCP2515_LOAD_TX_BUF_2_TXB2SIDH              0x44    //Transmit buffer 2 standard identifier register high
#define MCP2515_LOAD_TX_BUF_2_TXB2D0                0x45    //Transmit buffer 2 data byte register 

#define MCP2515_RTS_TXB0                            0x81    //Instructs controller to begin message transmission sequence for buffer 0                
#define MCP2515_RTS_TXB1                            0x82    //Instructs controller to begin message transmission sequence for buffer 1               
#define MCP2515_RTS_TXB2                            0x84    //Instructs controller to begin message transmission sequence for buffer 2                

#define MCP2515_READ_STATUS                         0xA0    //Quick polling command that reads several status bits for transmit and receive functions
#define MCP2515_RX_STATUS                           0xB0    //Quick polling command that indicates filter match and message type (standard, extended and/or remote) of received message
#define MCP2515_BIT_MODIFY                          0x05    //Allows the user to set or clear individual bits in a particular register
/* End Command Definition of Instructions */

/* Modes of Operation Registers */
#define MCP2515_CANCTRL                             0x0F    //CAN control register pg 60
#define MCP2515_CANSTAT                             0x0E    //CAN status register pg 61
/* Ends Modes of Operation Registers */
/* End Instruction Definition */

/* Register Definition */

/* Transmission Registers */
#define MCP2515_TRANSMIT_BUFFER_0_CONTROL           0x30    //Transmit buffer 0 control register pg 18
#define MCP2515_TRANSMIT_BUFFER_1_CONTROL           0x40    //Transmit buffer 1 control register pg 18
#define MCP2515_TRANSMIT_BUFFER_2_CONTROL           0x50    //Transmit buffer 2 control register pg 18

#define MCP2515_TX_RTS_CTRL                         0x0D    //Pin control and status register pg 19

#define MCP2515_TX_STANDARD_IDENTIFY_HIGH_0         0x31    //Transmit buffer 0 - standard identifier register high pg 20
#define MCP2515_TX_STANDARD_IDENTIFY_HIGH_1         0x41    //Transmit buffer 1 - standard identifier register high pg 20
#define MCP2515_TX_STANDARD_IDENTIFY_HIGH_2         0x51    //Transmit buffer 2 - standard identifier register high pg 20
#define MCP2515_TX_STANDARD_IDENTIFY_LOW_0          0x32    //Transmit buffer 0 - standard identifier register low pg 20
#define MCP2515_TX_STANDARD_IDENTIFY_LOW_1          0x42    //Transmit buffer 1 - standard identifier register low pg 20
#define MCP2515_TX_STANDARD_IDENTIFY_LOW_2          0x52    //Transmit buffer 2 - standard identifier register low pg 20

#define MCP2515_TX_EXTENDED_IDENTIFY_HIGH_0         0x33    //Transmit buffer 0 - extended identifier register high pg 21
#define MCP2515_TX_EXTENDED_IDENTIFY_HIGH_1         0x43    //Transmit buffer 1 - extended identifier register high pg 21
#define MCP2515_TX_EXTENDED_IDENTIFY_HIGH_2         0x53    //Transmit buffer 2 - extended identifier register high pg 21
#define MCP2515_TX_EXTENDED_IDENTIFY_LOW_0          0x34    //Transmit buffer 0 - extended identifier register low pg 21
#define MCP2515_TX_EXTENDED_IDENTIFY_LOW_1          0x44    //Transmit buffer 1 - extended identifier register low pg 21
#define MCP2515_TX_EXTENDED_IDENTIFY_LOW_2          0x54    //Transmit buffer 2 - extended identifier register low pg 21

#define MCP2515_TX_BUF_0_DATA_LEN                   0x35    //Transmit buffer 0 data length pg 22
#define MCP2515_TX_BUF_1_DATA_LEN                   0x45    //Transmit buffer 1 data length pg 22
#define MCP2515_TX_BUF_2_DATA_LEN                   0x55    //Transmit buffer 2 data length pg 22

#define MCP2515_TX_DATA_BUF_0_START                 0x36    //Transmit buffer 0 data start pg 22
#define MCP2515_TX_DATA_BUF_1_START                 0x46    //Transmit buffer 1 data start pg 22
#define MCP2515_TX_DATA_BUF_2_START                 0x56    //Transmit buffer 2 data start pg 22
/* End Transmission Registers */

/* Receive Registers */
#define MCP2515_RX_BUFFER_0_CONTROL                 0x60    //Receive buffer 0 control register pg 27
#define MCP2515_RX_BUFFER_1_CONTROL                 0x70    //Receive buffer 1 control register pg 28

#define MCP2515_RX_BFP_CTRL                         0x0C    //RX0BF & RX1BF pin control and status register pg 29

#define MCP2515_RX_STANDARD_IDENTIFY_HIGH_0         0x61    //Receive buffer 0 - standard identifier register high pg 30
#define MCP2515_RX_STANDARD_IDENTIFY_HIGH_1         0x71    //Receive buffer 1 - standard identifier register high pg 30
#define MCP2515_RX_STANDARD_IDENTIFY_LOW_0          0x62    //Receive buffer 0 - standard identifier register low pg 30
#define MCP2515_RX_STANDARD_IDENTIFY_LOW_1          0x72    //Receive buffer 1 - standard identifier register low pg 30

#define MCP2515_RX_EXTENDED_IDENTIFY_HIGH_0         0x63    //Receive buffer 0 - extended identifier register high pg 31
#define MCP2515_RX_EXTENDED_IDENTIFY_HIGH_1         0x73    //Receive buffer 1 - extended identifier register high pg 31
#define MCP2515_RX_EXTENDED_IDENTIFY_LOW_0          0x64    //Receive buffer 0 - extended identifier register low pg 31
#define MCP2515_RX_EXTENDED_IDENTIFY_LOW_1          0x74    //Receive buffer 1 - extended identifier register low pg 31

#define MCP2515_RX_BUF_0_DATA_LEN                   0x65    //Receive buffer 0 data length pg 32
#define MCP2515_RX_BUF_1_DATA_LEN                   0x75    //Receive buffer 1 data length pg 32

#define MCP2515_RX_DATA_BUF_0_START                 0x66    //Receive buffer 0 data start pg 32
#define MCP2515_RX_DATA_BUF_1_START                 0x76    //Receive buffer 0 data start pg 32

/* Receive Filter and Mask Registers */
/* MASKS AND FILTER REGISTERS MAY ONLY BE MODIFIED WHEN THE MCP2515 IS IN CONFIGURATION MODE*/
#define MCP2515_RX_FILTER_0_STANDARD_IDENTIFY_HIGH  0x00    //Filter 0 standard identifier register high pg 35
#define MCP2515_RX_FILTER_1_STANDARD_IDENTIFY_HIGH  0x04    //Filter 1 standard identifier register high pg 35
#define MCP2515_RX_FILTER_2_STANDARD_IDENTIFY_HIGH  0x08    //Filter 2 standard identifier register high pg 35
#define MCP2515_RX_FILTER_3_STANDARD_IDENTIFY_HIGH  0x10    //Filter 3 standard identifier register high pg 35
#define MCP2515_RX_FILTER_4_STANDARD_IDENTIFY_HIGH  0x14    //Filter 4 standard identifier register high pg 35
#define MCP2515_RX_FILTER_5_STANDARD_IDENTIFY_HIGH  0x18    //Filter 5 standard identifier register high pg 35

#define MCP2515_RX_FILTER_0_STANDARD_IDENTIFY_LOW   0x01    //Filter 0 standard identifier register low pg 35
#define MCP2515_RX_FILTER_1_STANDARD_IDENTIFY_LOW   0x05    //Filter 1 standard identifier register low pg 35
#define MCP2515_RX_FILTER_2_STANDARD_IDENTIFY_LOW   0x09    //Filter 2 standard identifier register low pg 35
#define MCP2515_RX_FILTER_3_STANDARD_IDENTIFY_LOW   0x11    //Filter 3 standard identifier register low pg 35
#define MCP2515_RX_FILTER_4_STANDARD_IDENTIFY_LOW   0x15    //Filter 4 standard identifier register low pg 35
#define MCP2515_RX_FILTER_5_STANDARD_IDENTIFY_LOW   0x19    //Filter 5 standard identifier register low pg 35

#define MCP2515_RX_FILTER_0_EXTENDED_IDENTIFY_HIGH  0x02    //Filter 0 extended identifier register high pg 36
#define MCP2515_RX_FILTER_1_EXTENDED_IDENTIFY_HIGH  0x06    //Filter 1 extended identifier register high pg 36
#define MCP2515_RX_FILTER_2_EXTENDED_IDENTIFY_HIGH  0x0A    //Filter 2 extended identifier register high pg 36
#define MCP2515_RX_FILTER_3_EXTENDED_IDENTIFY_HIGH  0x12    //Filter 3 extended identifier register high pg 36
#define MCP2515_RX_FILTER_4_EXTENDED_IDENTIFY_HIGH  0x16    //Filter 4 extended identifier register high pg 36
#define MCP2515_RX_FILTER_5_EXTENDED_IDENTIFY_HIGH  0x1A    //Filter 5 extended identifier register high pg 36

#define MCP2515_RX_FILTER_0_EXTENDED_IDENTIFY_LOW   0x03    //Filter 0 extended identifier register low pg 36
#define MCP2515_RX_FILTER_1_EXTENDED_IDENTIFY_LOW   0x07    //Filter 1 extended identifier register low pg 36
#define MCP2515_RX_FILTER_2_EXTENDED_IDENTIFY_LOW   0x0B    //Filter 2 extended identifier register low pg 36
#define MCP2515_RX_FILTER_3_EXTENDED_IDENTIFY_LOW   0x13    //Filter 3 extended identifier register low pg 36
#define MCP2515_RX_FILTER_4_EXTENDED_IDENTIFY_LOW   0x17    //Filter 4 extended identifier register low pg 36
#define MCP2515_RX_FILTER_5_EXTENDED_IDENTIFY_LOW   0x1B    //Filter 5 extended identifier register low pg 36


#define MCP2515_RX_MASK_0_STANDARD_IDENTIFY_HIGH    0x20    //Mask 0 standard identifier register high pg 37
#define MCP2515_RX_MASK_1_STANDARD_IDENTIFY_HIGH    0x24    //Mask 1 standard identifier register high pg 37

#define MCP2515_RX_MASK_0_STANDARD_IDENTIFY_LOW     0x21    //Mask 0 standard identifier register low pg 37
#define MCP2515_RX_MASK_1_STANDARD_IDENTIFY_LOW     0x25    //Mask 1 standard identifier register low pg 37

#define MCP2515_RX_MASK_0_EXTENDED_IDENTIFY_HIGH    0x22    //Mask 0 extended identifier register high pg 38
#define MCP2515_RX_MASK_1_EXTENDED_IDENTIFY_HIGH    0x26    //Mask 1 extended identifier register high pg 38

#define MCP2515_RX_MASK_0_EXTENDED_IDENTIFY_LOW     0x23    //Mask 0 extended identifier register low pg 38
#define MCP2515_RX_MASK_1_EXTENDED_IDENTIFY_LOW     0x27    //Mask 1 extended identifier register low pg 38
/* End Receive Filter and Mask Registers */
/* End Receive Registers */

/* Timing Registers */
/*SEE PAGE 43 FOR IMPORTANT INFORMATION REGARDING HOW TO PROGRAM TIME SEGMENTS WITHIN THE ASSOCIATED REGISTER*/
#define MCP2515_TIME_CNF_1                          0x2A    //Configuration register 1 pg 44
#define MCP2515_TIME_CNF_2                          0x29    //Configuration register 2 pg 44
#define MCP2515_TIME_CNF_3                          0x28    //Configuration register 3 pg 45
/* End Timing Register */

/* Error  Detection Registers */
#define MCP2515_TX_ERR_COUNTER                      0x1C    //Transmit error counter register pg 49
#define MCP2515_RX_ERR_COUNTER                      0x1D    //Receive error counter register pg 49
#define MCP2515_ERR_FLAG                            0x2D    //Error flag register pg 50 MAKE SURE BITS ARE CLEARED
/* End Error Detection Registers */

/* Interrupt Registers */
#define MCP2515_CANINTE                             0x2B    //CAN interrupt enable register pg 53
#define MCP2515_CANINTF                             0x2C    //CAN interrupt flag register pg 54
/* End Interrupt Registers */

/* End System Preprocessor Definitions */

/* System Variables */
typedef enum {
    MCP2515_STATUS_OK,
    MCP2515_STATUS_ERROR,
} mcp2515_status_t;

typedef enum {
    NORMAL_MODE = 0,
    SLEEP_MODE ,
    LOOPBACK_MODE,
    LISTEN_ONLY_MODE,
    CONFIGURATION_MODE,
} mcp2515_operation_mode_t;

typedef uint64_t    *mcp2515_data_rx_t;

typedef enum {
    MCP2515_SEND,
    MCP2515_RECV,
} mcp2515_transmission_mode_t;
/* End System Variables */

/* System Function Definitions */
mcp2515_status_t mcp2515_operation_mode(mcp2515_operation_mode_t op_mode); //Set the operation mode of the MCP2515 see page 59

mcp2515_status_t mcp2515_read_canintf_error();  //Reads what the current error flags are set within the CANINTF register

mcp2515_status_t mcp2515_fast_tx_message(uint32_t identifier, uint32_t *data, uint8_t rtr_bit, uint32_t data_len);     //Quickly transmits standard message through buffer 0

mcp2515_status_t mcp2515_tx_fill_buffer_0(uint32_t identifier, uint32_t *data, uint8_t rtr_bit, uint32_t data_len);        //Fills buffer zero with data
mcp2515_status_t mcp2515_tx_fill_buffer_1(uint32_t identifier, uint32_t *data, uint8_t rtr_bit, uint32_t data_len);        //Fills buffer one with data
mcp2515_status_t mcp2515_tx_fill_buffer_2(uint32_t identifier, uint32_t *data, uint8_t rtr_bit, uint32_t data_len);        //Fills buffer two with data
mcp2515_status_t mcp2515_tx_send_buffer(uint32_t tx_buf_num);                                          //Sends the buffer dignified to be transmitted along CAN bus

mcp2515_status_t mcp2515_rx_buffer_read(uint32_t rx_buf_num, uint32_t *identifier, uint32_t *data);   //Read data stored in buffer dignified

/*mcp2515_status_t mcp2515_eflg_read();*/

mcp2515_status_t mcp2515_init();    //Initialization of the MCP2515 CAN Controller THIS SHOULD BE ADJUSTED TO USER PREFERENCES


/* End System Function Definitions */

#ifdef __cplusplus
}
#endif

#endif