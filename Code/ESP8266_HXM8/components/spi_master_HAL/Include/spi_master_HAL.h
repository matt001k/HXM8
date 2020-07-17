#ifndef ESP8266_SPI_MASTER_HAL
#define ESP8266_SPI_MASTER_HAL

#ifdef __cplusplus
extern "C" {
#endif


/*Change this values to fit application*/
#define SPI_CLOCK_SPEED                                     SPI_2MHz_DIV 

/*Used to describe if a read command or a write is given to the SPI interface*/
typedef enum {
    SPI_SEND = 0,
    SPI_RECV
} spi_master_mode_t;

/* SPI transmit data, format: 8bit command + 8bit address + 64byte data */
void spi_master_transmit(spi_master_mode_t trans_mode, uint8_t cmd, uint8_t addr, uint32_t cmd_len, uint32_t* data, uint32_t data_len);

/* SPI master send length, format: 8bit command(value:1) + 32bit status length */
void spi_master_send_length(uint32_t len);

/*Initialization code for SPI*/
void spi_interface_init(void);


#ifdef __cplusplus
}
#endif

#endif