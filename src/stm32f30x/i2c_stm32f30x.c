#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <stm32f30x_i2c.h>
#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_misc.h>

#include "i2c_stm32f30x.h"

#define I2C_SHORT_TIMEOUT             ((uint32_t)0x1000)
#define I2C_LONG_TIMEOUT             ((uint32_t)(10 * I2C_SHORT_TIMEOUT))

typedef enum i2c_idx_t
{
	I2C1_IDX,
	I2C2_IDX,
	MAX_I2C
} i2c_idx_t;

typedef struct i2c_config_st
{
	I2C_TypeDef     *i2c_port;
	uint_fast16_t	frequencyKHz;

	i2c_port_t		port;

	GPIO_TypeDef	*sclGpio;
	uint_fast16_t	sclPin;
	uint_fast8_t	sclPinSource;
	uint_fast32_t	sclRCC_AHBPeriph;
	uint_fast8_t	sclPinAF;

	GPIO_TypeDef	*sdaGpio;
	uint_fast16_t	sdaPin;
	uint_fast8_t	sdaPinSource;
	uint_fast32_t	sdaRCC_AHBPeriph;
	uint_fast8_t	sdaPinAF;
	uint_fast32_t	RCC_APB1Periph;
} i2c_config_st;

static const i2c_config_st i2c_configs[] =
{
	[I2C1_IDX] =
		{
			.i2c_port = I2C1,
			.frequencyKHz = 400,
			.port = I2C_PORT_1,
			.sclGpio = GPIOB,
			.sclPin = GPIO_Pin_6,
			.sclPinSource = GPIO_PinSource6,
			.sclRCC_AHBPeriph = RCC_AHBPeriph_GPIOB,
			.sclPinAF = GPIO_AF_4,
			.sdaGpio = GPIOB,
			.sdaPin = GPIO_Pin_7,
			.sdaPinSource = GPIO_PinSource7,
			.sdaRCC_AHBPeriph = RCC_AHBPeriph_GPIOB,
			.sdaPinAF = GPIO_AF_4,
			.RCC_APB1Periph = RCC_APB1Periph_I2C1
		},
	[I2C2_IDX] =
		{
			.i2c_port = I2C2,
			.frequencyKHz = 400,
			.port = I2C_PORT_2,
			.sclGpio = GPIOF,
			.sclPin = GPIO_Pin_6,
			.sclPinSource = GPIO_PinSource6,
			.sclRCC_AHBPeriph = RCC_AHBPeriph_GPIOF,
			.sclPinAF = GPIO_AF_4,
			.sdaGpio = GPIOA,
			.sdaPin = GPIO_Pin_10,
			.sdaPinSource = GPIO_PinSource10,
			.sdaRCC_AHBPeriph = RCC_AHBPeriph_GPIOA,
			.sdaPinAF = GPIO_AF_4,
			.RCC_APB1Periph = RCC_APB1Periph_I2C2
		}
};
#define NB_I2C_PORTS	(sizeof(i2c_configs)/sizeof(i2c_configs[0]))

static bool i2cTimeoutUserCallback( I2C_TypeDef *I2C )
{
	(void)I2C;
	// TODO: - port based statistic
	return false;
}

static i2c_config_st const * i2cPortLookup( i2c_port_t const port )
{
	unsigned int i;

	for (i=0; i < NB_I2C_PORTS; i++)
	{
		if ( port == i2c_configs[i].port )
			return &i2c_configs[i];
	}

	return NULL;
}

void *i2cInit(i2c_port_t port)
{
	i2c_config_st const * i2c_config;
	I2C_TypeDef *i2c;
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

	if ((i2c_config=i2cPortLookup(port)) == NULL)
	{
		i2c = NULL;
		goto done;
	}
	i2c = i2c_config->i2c_port;

	/* configure SYSCLK as source (72MHz) */
    RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);

    RCC_AHBPeriphClockCmd(i2c_config->sclRCC_AHBPeriph | i2c_config->sdaRCC_AHBPeriph, ENABLE);
	/* Both I2C using APB1 peripheral clock */
    RCC_APB1PeriphClockCmd(i2c_config->RCC_APB1Periph, ENABLE);

    GPIO_PinAFConfig(i2c_config->sclGpio, i2c_config->sclPinSource, i2c_config->sclPinAF);
    GPIO_PinAFConfig(i2c_config->sdaGpio, i2c_config->sdaPinSource, i2c_config->sdaPinAF);

    GPIO_StructInit(&GPIO_InitStructure);

    // Init pins
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStructure.GPIO_Pin = i2c_config->sclPin;
    GPIO_Init(i2c_config->sclGpio, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = i2c_config->sdaPin;
    GPIO_Init(i2c_config->sclGpio, &GPIO_InitStructure);


    I2C_StructInit(&I2C_InitStructure);

    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    I2C_InitStructure.I2C_DigitalFilter = 0x00;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	/* TODO: support user specified rate (e.g. 100kHz) */
	RCC_ClocksTypeDef clocks;

	RCC_GetClocksFreq(&clocks);
	/*
		set prescaler to 8MHz. Give 125ns resolution.
		XXX Assumes clock is running at a multiple of 8MHz
		Refer to page 755 of reference manual for examples on how to configure this register.
	*/
	I2C_InitStructure.I2C_Timing = ((((clocks.SYSCLK_Frequency/8000000)-1) & 0x0f) << 28)	/* PRESC */
									| (1 << 20)		/* SCLDEL 	- 2 x 125 = 250ns */
									| (2 << 16) 	/* SDADEL 	- 2 x 125 = 250ns */
									| (4 << 8)		/* SCLH   	- 5 x 125 = 650ns */
									| (10 << 0); 	/* SCLL		- 11 x 125 = 1375ns */
	/*
		gives total of 2525ns, which works out to just under 400kHz
		These values should be within the specs for the lsm303dlhc.
	*/

    I2C_Init(i2c, &I2C_InitStructure);

    I2C_Cmd(i2c, ENABLE);

done:

	return i2c;
}

/* bascially taken from STM32Discovery example code */
bool i2cWrite(void *pv, uint8_t addr_, uint8_t reg, uint8_t data)
{
    I2C_TypeDef *I2Cx = pv;
	uint_fast32_t i2cTimeout;

    /* Test on BUSY Flag */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

    /* Wait until TXIS flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Send Register address */
    I2C_SendData(I2Cx, (uint8_t) reg);

    /* Wait until TCR flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TCR) == RESET)
    {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

    /* Wait until TXIS flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Write data to TXDR */
    I2C_SendData(I2Cx, data);

    /* Wait until STOPF flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Clear STOPF flag */
    I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

    return true;
}

bool i2cRead(void *pv, uint8_t addr_, uint8_t reg, uint8_t* buf, uint_fast16_t len)
{
    I2C_TypeDef * I2Cx = pv;
	uint_fast32_t i2cTimeout;

    /* Test on BUSY Flag */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

    /* Wait until TXIS flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    if (len > 1) {
        reg |= 0x80;
    }

    /* Send Register address */
    I2C_SendData(I2Cx, (uint8_t) reg);

    /* Wait until TC flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TC) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, addr_, len, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

    /* Wait until all data are received */
    while (len) {
        /* Wait until RXNE flag is set */
        i2cTimeout = I2C_LONG_TIMEOUT;
        while (I2C_GetFlagStatus(I2Cx, I2C_ISR_RXNE) == RESET) {
            if ((i2cTimeout--) == 0) {
                return i2cTimeoutUserCallback(I2Cx);
            }
        }

        /* Read data from RXDR */
        *buf = I2C_ReceiveData(I2Cx);
        /* Point to the next location where the byte read will be saved */
        buf++;

        /* Decrement the read bytes counter */
        len--;
    }

    /* Wait until STOPF flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET) {
        if ((i2cTimeout--) == 0) {
            return i2cTimeoutUserCallback(I2Cx);
        }
    }

    /* Clear STOPF flag */
    I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

    /* If all operations OK */
    return true;
}


