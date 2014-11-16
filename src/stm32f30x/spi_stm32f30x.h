#ifndef __SPI_STM32F30X_H__
#define __SPI_STM32F30X_H__

typedef enum spi_port_t
{
	SPI_PORT_1
} spi_port_t;

void *spiInit( spi_port_t port );
int spiSendByte(void *pv, uint8_t byte);

#endif /* __SPI_STM32F30X_H__ */

