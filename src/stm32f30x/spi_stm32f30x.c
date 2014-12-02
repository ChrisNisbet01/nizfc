#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_spi.h>
#include <spi.h>

#define SPI_FLAG_TIMEOUT             ((uint32_t)0x1000)

typedef enum spi_idx_t
{
	SPI1_IDX,
	MAX_SPI
} i2c_idx_t;

typedef struct spi_config_st
{
	SPI_TypeDef	*spi;

	spi_port_t port;

	void (*RCC_APBPeriphClockCmd)(uint32_t RCC_APB2Periph, FunctionalState NewState);
	void (*RCC_APBPeriphResetCmd)(uint32_t RCC_APB2Periph, FunctionalState NewState);
	uint32_t	RCC_APBPeriph;

	uint32_t		sckPin;
	uint8_t		sckPinSource;
	GPIO_TypeDef	*sckGpio;
	uint8_t		sckAF;
	uint32_t     RCC_AHBPeriphSck;

	uint32_t		mosiPin;
	uint8_t		mosiPinSource;
	GPIO_TypeDef	*mosiGpio;
	uint8_t		mosiAF;
	uint32_t     RCC_AHBPeriphMosi;

	uint32_t		misoPin;
	uint8_t		misoPinSource;
	GPIO_TypeDef	*misoGpio;
	uint8_t		misoAF;
	uint32_t     RCC_AHBPeriphMiso;

} spi_config_st;

static spi_config_st spi_configs[] =
{
	[SPI1_IDX] =
		{
			.spi = SPI1,
			.port = SPI_PORT_1,
			.RCC_APBPeriphClockCmd = RCC_APB2PeriphClockCmd,
			.RCC_APBPeriphResetCmd = RCC_APB2PeriphResetCmd,
			.RCC_APBPeriph = RCC_APB2Periph_SPI1,
			.sckPin = GPIO_Pin_5,
			.sckPinSource = GPIO_PinSource5,
			.sckGpio = GPIOA,
			.sckAF = GPIO_AF_5,
			.RCC_AHBPeriphSck = RCC_AHBPeriph_GPIOA,

			.mosiPin = GPIO_Pin_7,
			.mosiPinSource = GPIO_PinSource7,
			.mosiGpio = GPIOA,
			.mosiAF = GPIO_AF_5,
			.RCC_AHBPeriphMosi = RCC_AHBPeriph_GPIOA,

			.misoPin = GPIO_Pin_6,
			.misoPinSource = GPIO_PinSource6,
			.misoGpio = GPIOA,
			.misoAF = GPIO_AF_5,
			.RCC_AHBPeriphMiso = RCC_AHBPeriph_GPIOA
		}
};

#define NB_SPI_PORTS	(sizeof(spi_configs)/sizeof(spi_configs[0]))

static int spiTimeoutUserCallback( SPI_TypeDef *SPI )
{
	(void)SPI;
	// TODO: - SPI port based statistic

	return -1;
}

static spi_config_st const * spiPortLookup( spi_port_t const port )
{
	unsigned int i;

	for (i=0; i < NB_SPI_PORTS; i++)
	{
		if ( port == spi_configs[i].port )
			return &spi_configs[i];
	}

	return NULL;
}

void *spiInit( spi_port_t port )
{
	spi_config_st const * spi_config;
	SPI_TypeDef *spi;
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	if ((spi_config=spiPortLookup( port )) == NULL )
	{
		spi = NULL;
		goto done;
	}

	spi = spi_config->spi;

	/* Enable the SPI periph */
	spi_config->RCC_APBPeriphClockCmd(spi_config->RCC_APBPeriph, ENABLE);

	/* Enable SCK, MOSI and MISO GPIO clocks */
	RCC_AHBPeriphClockCmd(spi_config->RCC_AHBPeriphSck | spi_config->RCC_AHBPeriphMosi | spi_config->RCC_AHBPeriphMiso, ENABLE);

	GPIO_PinAFConfig(spi_config->sckGpio, spi_config->sckPinSource, spi_config->sckAF);
	GPIO_PinAFConfig(spi_config->misoGpio, spi_config->misoPinSource, spi_config->misoAF);
	GPIO_PinAFConfig(spi_config->mosiGpio, spi_config->mosiPinSource, spi_config->mosiAF);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = spi_config->sckPin;
	GPIO_Init(spi_config->sckGpio, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  spi_config->mosiPin;
	GPIO_Init(spi_config->mosiGpio, &GPIO_InitStructure);

	/* SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin = spi_config->misoPin;
	GPIO_Init(spi_config->mosiGpio, &GPIO_InitStructure);

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(spi);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(spi, &SPI_InitStructure);

	/* Configure the RX FIFO Threshold */
	SPI_RxFIFOThresholdConfig(spi, SPI_RxFIFOThreshold_QF);
	/* Enable SPI  */
	SPI_Cmd(spi, ENABLE);

#if 0
  /* Configure GPIO PIN for Lis Chip select */
  GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(L3GD20_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  GPIO_SetBits(L3GD20_SPI_CS_GPIO_PORT, L3GD20_SPI_CS_PIN);

  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_INT1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(L3GD20_SPI_INT1_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_INT2_PIN;
  GPIO_Init(L3GD20_SPI_INT2_GPIO_PORT, &GPIO_InitStructure);
}
#endif

done:

	return spi;

}

int spiSendByte(void *pv, uint8_t byte)
{
	SPI_TypeDef *spi = pv;
	uint_fast32_t timeout;

	/* Loop while DR register in not empty */
	timeout = SPI_FLAG_TIMEOUT;
	while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_TXE) == RESET)
	{
		if((timeout--) == 0)
			return spiTimeoutUserCallback( spi );
	}

	/* Send a Byte through the SPI peripheral */
	SPI_SendData8(spi, byte);

	/* Wait to receive a Byte */
	timeout = SPI_FLAG_TIMEOUT;
	while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_RXNE) == RESET)
	{
		if((timeout--) == 0)
			return spiTimeoutUserCallback( spi );
	}

	/* Return the Byte read from the SPI bus */
	return SPI_ReceiveData8(spi);
}


