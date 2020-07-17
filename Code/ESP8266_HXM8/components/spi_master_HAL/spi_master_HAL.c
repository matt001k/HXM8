#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp8266/spi_struct.h"
#include "esp8266/gpio_struct.h"
#include "esp_system.h"
#include "esp_log.h"

#include "driver/spi.h"

#include "spi_master_HAL.h"

static const char *TAG = "spi_master";

void IRAM_ATTR spi_master_transmit(spi_master_mode_t trans_mode, uint8_t cmd, uint8_t addr, uint32_t cmd_len, uint32_t* data, uint32_t data_len)
{
    spi_trans_t trans;
    uint16_t cmd_trans;

    if (cmd_len > 16)
    {
        ESP_LOGE(TAG, "ESP8266 only support transmit 2bytes command");
        return;
    }

    if (data_len > 512) {
        ESP_LOGE(TAG, "ESP8266 only support transmit 64bytes at one time");
        return;
    }

    cmd_trans = (addr << 8 ) | (cmd);

    memset(&trans, 0x0, sizeof(trans));
    trans.bits.val = 0;            // clear all bit

    if (trans_mode == SPI_SEND) { 
        trans.bits.mosi = data_len;             // One time transmit only support 64bytes
        uint32_t temp;
        temp = *data << (24 - (data_len - 8));
        trans.mosi = &temp;
    } 
    else {
        trans.bits.miso = data_len;
        trans.miso = data;
    }

    trans.bits.cmd = cmd_len;
    trans.bits.addr = 0;   
    trans.cmd = &cmd_trans;
    trans.addr = NULL;
    
    spi_trans(HSPI_HOST, &trans);
}

void IRAM_ATTR spi_master_send_length(uint32_t len)
{
    spi_trans_t trans;
    uint16_t cmd = SPI_MASTER_WRITE_STATUS_TO_SLAVE_CMD;
    memset(&trans, 0x0, sizeof(trans));
    trans.bits.val = 0;
    trans.bits.cmd = 8 * 1;  
    trans.bits.addr = 0;          // transmit status do not use address bit
    trans.bits.mosi = 8 * 4;      // status length is 32bit
    trans.cmd = &cmd;
    trans.addr = NULL;
    trans.mosi = &len;
    spi_trans(HSPI_HOST, &trans);    
}


void spi_interface_init(void)
{
    ESP_LOGI(TAG, "init spi");
    spi_config_t spi_config;

    // Load default interface parameters
    // CS_EN:1, MISO_EN:1, MOSI_EN:1, BYTE_TX_ORDER:1, BYTE_TX_ORDER:1, BIT_RX_ORDER:0, BIT_TX_ORDER:0, CPHA:0, CPOL:0
    //spi_config.interface.val = SPI_DEFAULT_INTERFACE;
    spi_config.interface.cpol = 0;
    spi_config.interface.cpha = 0;
    spi_config.interface.bit_tx_order = 0;
    spi_config.interface.bit_rx_order = 0;
    spi_config.interface.byte_tx_order = 1;
    spi_config.interface.byte_rx_order = 1;
    spi_config.interface.mosi_en = 1;
    spi_config.interface.miso_en = 1;
    spi_config.interface.cs_en = 1;

    // Load default interrupt enable
    // TRANS_DONE: true, WRITE_STATUS: false, READ_STATUS: false, WRITE_BUFFER: false, READ_BUFFER: false
    spi_config.intr_enable.val = SPI_MASTER_DEFAULT_INTR_ENABLE;
    // Set SPI to master mode
    // ESP8266 Only support half-duplex
    spi_config.mode = SPI_MASTER_MODE;
    // Set the SPI clock frequency division factor
    spi_config.clk_div = SPI_CLOCK_SPEED;
    spi_config.event_cb = NULL;
    spi_init(HSPI_HOST, &spi_config);
}
