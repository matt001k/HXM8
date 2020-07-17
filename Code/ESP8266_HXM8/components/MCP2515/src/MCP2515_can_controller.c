#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "MCP2515_can_controller.h"

/* user defined include for their own spi interface functions */
#include "spi_master_HAL.h"
#include "esp_system.h"
#include "esp_log.h"
/* end user define include */

static mcp2515_status_t mcp2515_spi_interface(mcp2515_transmission_mode_t trans_mode, uint16_t cmd, uint8_t addr, uint32_t* data, uint32_t data_len);

static mcp2515_status_t mcp2515_clk_configuration(uint32_t cnf1, uint32_t cnf2, uint32_t cnf3);
static mcp2515_status_t mcp2515_tx_ctl(uint32_t* data, uint8_t reg_num);
static mcp2515_status_t mcp2515_tx_rts_ctl(uint32_t* data);

static mcp2515_status_t mcp2515_rx_ctl(uint32_t* data, uint8_t reg_num);
static mcp2515_status_t mcp2515_bfp_ctl(uint32_t* data);
static mcp2515_status_t mcp2515_rx_standard_mask_or_filter(uint32_t data, uint8_t reg_num);
static mcp2515_status_t mcp2515_rx_extended_mask_or_filter(uint32_t data, uint8_t reg_num);

static mcp2515_status_t mcp2515_caninte(uint32_t data);

static mcp2515_status_t mcp2515_tx_buf_identifier_set(uint32_t identifier, uint32_t instruct_num ,uint32_t reg_num);
static mcp2515_status_t mcp2515_tx_buf_data_set(uint32_t* data, uint32_t instruct_num, uint32_t reg_num, uint8_t rtr_bit, uint32_t data_len);

static mcp2515_status_t mcp2515_rx_buf_data_get(uint32_t* data, uint32_t* identifier, uint32_t instruct_num_1, uint32_t instruct_num_2, uint32_t reg_num, uint32_t rtr_bit);



static mcp2515_status_t mcp2515_spi_interface(mcp2515_transmission_mode_t trans_mode, uint16_t cmd, uint8_t addr, uint32_t* data, uint32_t data_len) {
    
    /* function used for the spi interface chosen by the user */
    spi_master_mode_t mode;
    uint32_t cmd_len;

    if(trans_mode == MCP2515_SEND) {
        mode = SPI_SEND;
    }
    else {
        mode = SPI_RECV;
    }
    
    if(addr != 255) {
        cmd_len = 16;
    }
    
    else {
        cmd_len = 8;
        addr = 0;
    }
    
    spi_master_transmit(mode, cmd, addr, cmd_len, data, data_len);    
    /* end spi interface */

    return MCP2515_STATUS_OK;
}

static mcp2515_status_t mcp2515_clk_configuration(uint32_t cnf1, uint32_t cnf2, uint32_t cnf3) {
    mcp2515_spi_interface(MCP2515_SEND, MCP2515_WRITE, MCP2515_TIME_CNF_1, &cnf1, 8);
    mcp2515_spi_interface(MCP2515_SEND, MCP2515_WRITE, MCP2515_TIME_CNF_2, &cnf2, 8);
    mcp2515_spi_interface(MCP2515_SEND, MCP2515_WRITE, MCP2515_TIME_CNF_3, &cnf3, 8);
    return MCP2515_STATUS_OK;
}

static mcp2515_status_t mcp2515_tx_ctl(uint32_t* data, uint8_t reg_num) {
    mcp2515_spi_interface(MCP2515_SEND, MCP2515_WRITE, reg_num, data, 8);
    return MCP2515_STATUS_OK;
}

static mcp2515_status_t mcp2515_tx_rts_ctl(uint32_t* data) {
    mcp2515_spi_interface(MCP2515_SEND, MCP2515_WRITE, MCP2515_TX_RTS_CTRL, data, 8);
    return MCP2515_STATUS_OK;
}

static mcp2515_status_t mcp2515_rx_ctl(uint32_t* data, uint8_t reg_num) {
    mcp2515_spi_interface(MCP2515_SEND, MCP2515_WRITE, reg_num, data, 8);
    return MCP2515_STATUS_OK;
}

static mcp2515_status_t mcp2515_bfp_ctl(uint32_t* data) {
    mcp2515_spi_interface(MCP2515_SEND, MCP2515_WRITE, MCP2515_RX_BFP_CTRL, data, 8);
    return MCP2515_STATUS_OK;
}

static mcp2515_status_t mcp2515_rx_standard_mask_or_filter(uint32_t data, uint8_t reg_num) {
    uint32_t upper_register = (data & 0x7F8) >> 3;
    uint32_t lower_register = (data & 0x007) << 5;

    mcp2515_spi_interface(MCP2515_SEND, MCP2515_WRITE, reg_num, &upper_register, 8);
    mcp2515_spi_interface(MCP2515_SEND, MCP2515_WRITE, (reg_num + 1), &lower_register, 8);
    return MCP2515_STATUS_OK;
}

static mcp2515_status_t mcp2515_rx_extended_mask_or_filter(uint32_t data, uint8_t reg_num) {
    uint32_t top_register = (data & 0x30000) >> 16 | 0x08;
    uint32_t top_register_mask = 0x03;
    uint32_t top_register_data = top_register | (top_register_mask << 8);
    uint32_t upper_register = (data & 0x0FF00) >> 8;
    uint32_t lower_register = (data & 0x000FF);


    mcp2515_spi_interface(MCP2515_SEND, MCP2515_BIT_MODIFY, (reg_num - 1), &top_register_data, 16);
    mcp2515_spi_interface(MCP2515_SEND, MCP2515_WRITE, reg_num, &upper_register, 8);
    mcp2515_spi_interface(MCP2515_SEND, MCP2515_WRITE, (reg_num + 1), &lower_register, 8);
    return MCP2515_STATUS_OK;
}

static mcp2515_status_t mcp2515_caninte(uint32_t data) {
    mcp2515_spi_interface(MCP2515_SEND, MCP2515_WRITE, MCP2515_CANINTE, &data, 8);
    return MCP2515_STATUS_OK;
}

static mcp2515_status_t mcp2515_tx_buf_identifier_set(uint32_t identifier, uint32_t instruct_num ,uint32_t reg_num) {
    uint32_t identifier_len = (log2((double)identifier + 1));
    
    if (identifier_len > 29) {
        return MCP2515_STATUS_ERROR;
    }

    if(identifier < 2048) {
        identifier = (identifier << 5);
        mcp2515_spi_interface(MCP2515_SEND, instruct_num, 255, &identifier, 16); //255 is used in order to declare there is no address to be written to due to the instruction
    }
    else {
        uint32_t sta_top_reg = (identifier & 0x1FE00000) << 3;
        uint32_t sta_bottom_reg = (identifier & 0x00E30000) | (0x08 << 16);
        uint32_t ext_top_reg = (identifier & 0x0000FF00);
        uint32_t ext_bottom_reg = (identifier & 0x000000FF);
        uint32_t data = sta_top_reg | sta_bottom_reg | ext_top_reg | ext_bottom_reg;

        mcp2515_spi_interface(MCP2515_SEND, instruct_num, 255, &data, 32); //255 is used in order to declare there is no address to be written to due to the instruction
    }

    return MCP2515_STATUS_OK;
}

static mcp2515_status_t mcp2515_tx_buf_data_set(uint32_t* data, uint32_t instruct_num, uint32_t reg_num, uint8_t rtr_bit, uint32_t data_len) { 
    uint32_t DLC_len = ((data_len + (8 - 1)) / 8);  //if rtr_bit, shift into appropriate register
    uint32_t DLC_data;

    if(rtr_bit == 1) {
        DLC_data = (rtr_bit << 6);
    }
    else {
        DLC_data = DLC_len;
    }

    if(DLC_len > 8) {
        return MCP2515_STATUS_ERROR;
    }

    mcp2515_spi_interface(MCP2515_SEND, MCP2515_WRITE, reg_num - 1, &DLC_data, 8); 

    if(rtr_bit == 1) {
        return MCP2515_STATUS_OK;
    }
    else {
        if(data_len < 33) {//if data is less than 2^(32) send one amount of data
            mcp2515_spi_interface(MCP2515_SEND, instruct_num, 255, data, DLC_len * 8); //255 is used in order to declare there is no address to be written to due to the instruction
        }
        else {
            mcp2515_spi_interface(MCP2515_SEND, instruct_num, 255, data, 32); //255 is used in order to declare there is no address to be written to due to the instruction
            mcp2515_spi_interface(MCP2515_SEND, MCP2515_WRITE, reg_num + 4, (data + 1), (DLC_len * 8) - 32); 
        }
    }
    return MCP2515_STATUS_OK;
}

static mcp2515_status_t mcp2515_rx_buf_data_get(uint32_t* data, uint32_t* identifier, uint32_t instruct_num_1, uint32_t instruct_num_2, uint32_t reg_num, uint32_t rtr_bit) {
    uint32_t data_len;
    
    mcp2515_spi_interface(MCP2515_RECV, instruct_num_1, 255, identifier, 32); //255 is used in order to declare there is no address to be written to due to the instruction   
    if(((*identifier & 0x80000) >> 19) == 1) { //Signifies extended frame enable bit
        *identifier = ((*identifier & 0xFFE00000) >> 3) |(*identifier & 0x0003FFFF); 
    }
    else {
        *identifier = (*identifier & 0xFFE00000) >> 21;
    }
    if(rtr_bit) {
        *data = 0x00;
        *(data + 1) = 0x00;
        return MCP2515_STATUS_OK;
    }   
    mcp2515_spi_interface(MCP2515_RECV, MCP2515_READ, reg_num, &data_len, 8);
    data_len = ((data_len & 0x0F000000) >> 24) * 8;
    if(data_len > 64)
    {
        data_len = 64;  //length of RX Recv Buff registers is 8 bytes
    }
    mcp2515_spi_interface(MCP2515_RECV, instruct_num_2, 255, data, data_len); //255 is used in order to declare there is no address to be written to due to the instruction
    if(data_len > 32) {
        *(data + 1) = (*(data + 1) >> (32 - (data_len - 32)));
    }
    else {
        *data = *data >> (32 - data_len);
    }

    return MCP2515_STATUS_OK;
}


mcp2515_status_t mcp2515_operation_mode(mcp2515_operation_mode_t op_mode) {
    uint32_t operation_mode = (uint32_t) op_mode;
    
    operation_mode = (operation_mode << 5) | (TX_ONE_SHOT_ENABLE << 3);
    mcp2515_spi_interface(MCP2515_SEND, MCP2515_WRITE, MCP2515_CANCTRL, &operation_mode, 8);
    return MCP2515_STATUS_OK;
}

mcp2515_status_t mcp2515_read_canintf_error() {
    return MCP2515_STATUS_OK;
}

mcp2515_status_t mcp2515_fast_tx_message(uint32_t identifier, uint32_t* data, uint8_t rtr_bit, uint32_t data_len) {
    mcp2515_tx_fill_buffer_0(identifier, data, rtr_bit, data_len);
    mcp2515_tx_send_buffer(0);
    return MCP2515_STATUS_OK;
}

mcp2515_status_t mcp2515_tx_fill_buffer_0(uint32_t identifier, uint32_t* data, uint8_t rtr_bit, uint32_t data_len) {
    if(mcp2515_tx_buf_identifier_set(identifier, MCP2515_LOAD_TX_BUF_0_TXB0SIDH, MCP2515_TX_EXTENDED_IDENTIFY_HIGH_0) == MCP2515_STATUS_ERROR) {
        return MCP2515_STATUS_ERROR;
    }
    mcp2515_tx_buf_data_set(data, MCP2515_LOAD_TX_BUF_0_TXB0D0, MCP2515_TX_DATA_BUF_0_START, rtr_bit, data_len);
    return MCP2515_STATUS_OK;
}

mcp2515_status_t mcp2515_tx_fill_buffer_1(uint32_t identifier, uint32_t* data, uint8_t rtr_bit, uint32_t data_len) {
    if(mcp2515_tx_buf_identifier_set(identifier, MCP2515_LOAD_TX_BUF_1_TXB1SIDH, MCP2515_TX_EXTENDED_IDENTIFY_HIGH_1) == MCP2515_STATUS_ERROR) {
        return MCP2515_STATUS_ERROR;
    }
    mcp2515_tx_buf_data_set(data, MCP2515_LOAD_TX_BUF_1_TXB1D0, MCP2515_TX_DATA_BUF_1_START, rtr_bit, data_len);
    return MCP2515_STATUS_OK;
}

mcp2515_status_t mcp2515_tx_fill_buffer_2(uint32_t identifier, uint32_t* data, uint8_t rtr_bit, uint32_t data_len) {
    if(mcp2515_tx_buf_identifier_set(identifier, MCP2515_LOAD_TX_BUF_2_TXB2SIDH, MCP2515_TX_EXTENDED_IDENTIFY_HIGH_2) == MCP2515_STATUS_ERROR) {
        return MCP2515_STATUS_ERROR;
    }
    mcp2515_tx_buf_data_set(data, MCP2515_LOAD_TX_BUF_2_TXB2D0, MCP2515_TX_DATA_BUF_2_START, rtr_bit, data_len);
    return MCP2515_STATUS_OK;
}

mcp2515_status_t mcp2515_tx_send_buffer(uint32_t tx_buf_num) {
    switch(tx_buf_num) {
        case 0:
            mcp2515_spi_interface(MCP2515_SEND, MCP2515_RTS_TXB0, 255, &tx_buf_num, 0); //255 is used in order to declare there is no address to be written to due to the instruction
            break;
        case 1:
            mcp2515_spi_interface(MCP2515_SEND, MCP2515_RTS_TXB1, 255, &tx_buf_num, 0); //255 is used in order to declare there is no address to be written to due to the instruction
            break;
        case 2:
            mcp2515_spi_interface(MCP2515_SEND, MCP2515_RTS_TXB2, 255, &tx_buf_num, 0); //255 is used in order to declare there is no address to be written to due to the instruction
            break;
    }

    return MCP2515_STATUS_OK;
}

mcp2515_status_t mcp2515_rx_buffer_read(uint32_t rx_buf_num, uint32_t *identifier, uint32_t *data) {
    uint32_t rtr_bit;

    switch(rx_buf_num) {
        case 0:
            mcp2515_spi_interface(MCP2515_RECV, MCP2515_READ, MCP2515_RX_BUFFER_0_CONTROL, &rtr_bit, 8);
            rtr_bit = (rtr_bit & 0x08000000) >> 27;
            mcp2515_rx_buf_data_get(data, identifier, MCP2515_READ_RX_BUF_0_RXB0SIDH, MCP2515_READ_RX_BUF_0_RXBOD0, MCP2515_RX_BUF_0_DATA_LEN, rtr_bit);
            break;
        case 1:
            mcp2515_spi_interface(MCP2515_RECV, MCP2515_READ, MCP2515_RX_BUFFER_1_CONTROL, &rtr_bit, 8);
            rtr_bit = (rtr_bit & 0x08000000) >> 27;
            mcp2515_rx_buf_data_get(data, identifier, MCP2515_READ_RX_BUF_1_RXB1SIDH, MCP2515_READ_RX_BUF_1_RXBOD1, MCP2515_RX_BUF_1_DATA_LEN, rtr_bit);
            break;
    }
    return MCP2515_STATUS_OK;
}

/*mcp2515_status_t mcp2515_eflg_read() {
    uint32_t data;
    return MCP2515_STATUS_OK;

}
*/

mcp2515_status_t mcp2515_init() {
    uint32_t NULL_PTR = NULL;
    uint32_t tx_0_ctl_data = 3;  //user define the data to be written to the transmission control register 0
    uint32_t tx_1_ctl_data = 2;  //user define the data to be written to the transmission control register 1 
    uint32_t tx_2_ctl_data = 1;  //user define the data to be written to the transmission control register 2
    uint32_t tx_rts_ctl_data = (TX_RTS_PIN_2_ENABLE  << 2) | (TX_RTS_PIN_1_ENABLE  << 1) | (TX_RTS_PIN_0_ENABLE);   //user define the data to be written to the transmission rts pin control register 

    uint32_t rx_0_ctl_data = (RX_0_FILTER_MASK_ENABLE << 5) | (RX_BUFFER_ROLLOVER << 2);    //user define the data to be written to the receive control register 0
    uint32_t rx_1_ctl_data = (RX_1_FILTER_MASK_ENABLE << 5);                                //user define the data to be written to the receive control register 1
    uint32_t bfp_ctl_data = (RX_1_INTR_ENABLE) | (RX_0_INTR_ENABLE);                        //user define the data to be written to the bfp control register 

    /*timer configuration variables for prescale, used to configure the clock in accordance to the CAN Bus baud rate, oscillator frequency and number of time quanta*/
    uint32_t cnf1, cnf2, cnf3;

    uint32_t SJW_bits = 0x00;
    double quanta_time;
    double BRP;
    uint32_t PS1;
    uint32_t PS2;

    quanta_time = (1.0/((double)CAN_BUS_BIT_RATE))/((double)TIME_QUANTA_AMOUNT);
    BRP = (quanta_time * (double)MCP2515_OSC_FREQ) / 2.0 - 1.0;

    PS1 = ( TIME_QUANTA_AMOUNT * 2 ) / 3 - PROP_SEGMENT_QUANTA_AMOUNT - 1;
    PS2 = TIME_QUANTA_AMOUNT - PS1 - PROP_SEGMENT_QUANTA_AMOUNT - 1;

    cnf1 = (SJW_bits << 6) | (uint32_t)BRP;
    cnf2 = (1 << 7) | (CLOCK_SAMPLE_MODE << 6) | ((PS1 - 1) << 3) | (PROP_SEGMENT_QUANTA_AMOUNT - 1);
    cnf3 = (0 << 7) | (0 << 6) | (PS2 - 1);
    /* end clock configuration */

    mcp2515_spi_interface(MCP2515_SEND, MCP2515_RESET, 255, &NULL_PTR, 0);

    mcp2515_operation_mode(CONFIGURATION_MODE);
    mcp2515_clk_configuration(cnf1, cnf2, cnf3);
    mcp2515_tx_ctl(&tx_0_ctl_data, MCP2515_TRANSMIT_BUFFER_0_CONTROL);
    mcp2515_tx_ctl(&tx_1_ctl_data, MCP2515_TRANSMIT_BUFFER_1_CONTROL);
    mcp2515_tx_ctl(&tx_2_ctl_data, MCP2515_TRANSMIT_BUFFER_2_CONTROL);
    mcp2515_tx_rts_ctl(&tx_rts_ctl_data);
    mcp2515_rx_ctl(&rx_0_ctl_data, MCP2515_RX_BUFFER_0_CONTROL);
    mcp2515_rx_ctl(&rx_1_ctl_data, MCP2515_RX_BUFFER_1_CONTROL);
    mcp2515_bfp_ctl(&bfp_ctl_data);

#if (RX_0_FILTER_MASK_ENABLE == 0)
    mcp2515_rx_standard_mask_or_filter(RX_BUF_STA_0_MASK_CTR_DATA, MCP2515_RX_MASK_0_STANDARD_IDENTIFY_HIGH);
    mcp2515_rx_extended_mask_or_filter(RX_BUF_EXT_0_MASK_CTR_DATA, MCP2515_RX_MASK_0_EXTENDED_IDENTIFY_HIGH);
    mcp2515_rx_standard_mask_or_filter(RX_BUF_STA_0_FILTER_CTR_DATA, MCP2515_RX_FILTER_0_STANDARD_IDENTIFY_HIGH);
    mcp2515_rx_extended_mask_or_filter(RX_BUF_EXT_0_FILTER_CTR_DATA, MCP2515_RX_FILTER_0_EXTENDED_IDENTIFY_HIGH);
    mcp2515_rx_extended_mask_or_filter(RX_BUF_EXT_1_FILTER_CTR_DATA, MCP2515_RX_FILTER_1_EXTENDED_IDENTIFY_HIGH);
    mcp2515_rx_standard_mask_or_filter(RX_BUF_STA_1_FILTER_CTR_DATA, MCP2515_RX_FILTER_1_STANDARD_IDENTIFY_HIGH);
#endif

#if (RX_1_FILTER_MASK_ENABLE == 0 & RX_BUFFER_ROLLOVER == 0)
    mcp2515_rx_standard_mask_or_filter(RX_BUF_STA_1_MASK_CTR_DATA, MCP2515_RX_MASK_1_STANDARD_IDENTIFY_HIGH);
    mcp2515_rx_extended_mask_or_filter(RX_BUF_EXT_1_MASK_CTR_DATA, MCP2515_RX_MASK_1_EXTENDED_IDENTIFY_HIGH);
    mcp2515_rx_extended_mask_or_filter(RX_BUF_EXT_2_FILTER_CTR_DATA, MCP2515_RX_FILTER_2_EXTENDED_IDENTIFY_HIGH);
    mcp2515_rx_standard_mask_or_filter(RX_BUF_STA_2_FILTER_CTR_DATA, MCP2515_RX_FILTER_2_STANDARD_IDENTIFY_HIGH);
    mcp2515_rx_extended_mask_or_filter(RX_BUF_EXT_3_FILTER_CTR_DATA, MCP2515_RX_FILTER_3_EXTENDED_IDENTIFY_HIGH);
    mcp2515_rx_standard_mask_or_filter(RX_BUF_STA_3_FILTER_CTR_DATA, MCP2515_RX_FILTER_3_STANDARD_IDENTIFY_HIGH);
    mcp2515_rx_extended_mask_or_filter(RX_BUF_EXT_4_FILTER_CTR_DATA, MCP2515_RX_FILTER_4_EXTENDED_IDENTIFY_HIGH);
    mcp2515_rx_standard_mask_or_filter(RX_BUF_STA_4_FILTER_CTR_DATA, MCP2515_RX_FILTER_4_STANDARD_IDENTIFY_HIGH);
    mcp2515_rx_extended_mask_or_filter(RX_BUF_EXT_5_FILTER_CTR_DATA, MCP2515_RX_FILTER_5_EXTENDED_IDENTIFY_HIGH);
    mcp2515_rx_standard_mask_or_filter(RX_BUF_STA_5_FILTER_CTR_DATA, MCP2515_RX_FILTER_5_STANDARD_IDENTIFY_HIGH);
#endif


    mcp2515_caninte(INTR_ENABLE_DATA);
    mcp2515_operation_mode(NORMAL_MODE);

    return MCP2515_STATUS_OK;
}