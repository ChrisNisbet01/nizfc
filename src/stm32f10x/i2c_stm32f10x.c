/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stm32f10x_i2c.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <misc.h>
#include <utils.h>
#include <i2c.h>

typedef enum i2c_idx_t
{
	I2C1_IDX,
	I2C2_IDX,
	MAX_I2C
} i2c_idx_t;


typedef struct i2cDevice_t {
	i2c_port_t		port;
    I2C_TypeDef * dev;
    GPIO_TypeDef * gpio;
    uint16_t scl;
    uint16_t sda;
    uint8_t ev_irq;
    uint8_t er_irq;
    uint32_t peripheral;
} i2cDevice_t;

static const i2cDevice_t i2cHardwareMap[] = {
    [I2C1_IDX] =
    	{
    	.port = I2C_PORT_1,
    	.dev = I2C1,
    	.gpio = GPIOB,
    	.scl = GPIO_Pin_6,
    	.sda = GPIO_Pin_7,
    	.ev_irq = I2C1_EV_IRQn,
    	.er_irq = I2C1_ER_IRQn,
    	.peripheral = RCC_APB1Periph_I2C1
    	},
    [I2C2_IDX] =
    	{
    	.port = I2C_PORT_2,
    	.dev = I2C2,
    	.gpio = GPIOB,
    	.scl = GPIO_Pin_10,
    	.sda = GPIO_Pin_11,
    	.ev_irq = I2C2_EV_IRQn,
    	.er_irq = I2C2_ER_IRQn,
    	.peripheral = RCC_APB1Periph_I2C2
    	}
};
#define NB_I2C_PORTS	ARRAY_SIZE(i2cHardwareMap)

#define I2C_DEFAULT_TIMEOUT 30000	// TODO: use some real uints of time, not just a loop counter

static volatile uint16_t i2cErrorCount = 0;

typedef struct i2cContext_st
{
    I2C_TypeDef * i2c;

	volatile bool error;
	volatile bool busy;

	volatile uint8_t addr;
	volatile uint8_t reg;
	volatile uint8_t bytes;
	volatile uint8_t writing;
	volatile uint8_t reading;
	volatile uint8_t* write_p;
	volatile uint8_t* read_p;
} i2cContext_st;

static inline void digitalHi(GPIO_TypeDef *p, uint16_t i) { p->BSRR = i; }
static inline void digitalLo(GPIO_TypeDef *p, uint16_t i)     { p->BRR = i; }
static inline void digitalToggle(GPIO_TypeDef *p, uint16_t i) { p->ODR ^= i; }
static inline uint16_t digitalIn(GPIO_TypeDef *p, uint16_t i) {return p->IDR & i; }

static i2cContext_st i2cContexts[NB_I2C_PORTS];

static void gpioInit(uint32_t clk, GPIO_TypeDef *port, uint16_t pin, GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed  )
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  /* Enable the GPIO_LED Clock */
	RCC_APB2PeriphClockCmd(clk, ENABLE);

	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Speed = speed;
	GPIO_InitStructure.GPIO_Mode = mode;

	GPIO_Init(port, &GPIO_InitStructure);
}

static void i2cUnstick(i2cDevice_t const * i2c_config)
{
    GPIO_TypeDef *gpio;
    uint16_t scl, sda;
    int i;

    // prepare pins
    gpio = i2c_config->gpio;
    scl = i2c_config->scl;
    sda = i2c_config->sda;

    digitalHi(gpio, scl | sda);

    gpioInit(RCC_APB2Periph_GPIOB, gpio, scl|sda, GPIO_Mode_Out_OD, GPIO_Speed_2MHz);	// XXX clock stored in hardware map

    for (i = 0; i < 8; i++) {
        // Wait for any clock stretching to finish
        while (!digitalIn(gpio, scl))
            delayMicroseconds(10);

        // Pull low
        digitalLo(gpio, scl); // Set bus low
        delayMicroseconds(10);
        // Release high again
        digitalHi(gpio, scl); // Set bus high
        delayMicroseconds(10);
    }

    // Generate a start then stop condition
    // SCL  PB10
    // SDA  PB11
    digitalLo(gpio, sda); // Set bus data low
    delayMicroseconds(10);
    digitalLo(gpio, scl); // Set bus scl low
    delayMicroseconds(10);
    digitalHi(gpio, scl); // Set bus scl high
    delayMicroseconds(10);
    digitalHi(gpio, sda); // Set bus sda high

    // Init pins
    gpioInit(RCC_APB2Periph_GPIOB, gpio, scl|sda, GPIO_Mode_AF_OD, GPIO_Speed_2MHz);	// XXX clock stored in hardware map
}

static bool i2cHandleHardwareFailure(i2cContext_st * i2cContext)
{
	i2cDevice_t const * i2c_config = &i2cHardwareMap[i2cContext-&i2cContexts[0]];
    i2cErrorCount++;

    // reinit peripheral + clock out garbage
    i2cInit(i2c_config->port);

    return false;
}

static bool i2cWriteBuffer(void * pv, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
	i2cContext_st * i2cContext = pv;
	I2C_TypeDef *I2Cx = i2cContext->i2c;
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    i2cContext->addr = addr_ << 1;
    i2cContext->reg = reg_;
    i2cContext->writing = 1;
    i2cContext->reading = 0;
    i2cContext->write_p = data;
    i2cContext->read_p = data;
    i2cContext->bytes = len_;
    i2cContext->busy = 1;
    i2cContext->error = false;

    if (!(I2Cx->CR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(I2Cx->CR1 & 0x0100)) {                                    // ensure sending a start
            while (I2Cx->CR1 & 0x0200 && --timeout > 0) { ; }           // wait for any stop to finish sending
            if (timeout == 0) {
                return i2cHandleHardwareFailure(i2cContext);
            }
            I2C_GenerateSTART(I2Cx, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }

    timeout = I2C_DEFAULT_TIMEOUT;
    while (i2cContext->busy && --timeout > 0) { ; }
    if (timeout == 0)
    {
        return i2cHandleHardwareFailure(i2cContext);
    }

    return !i2cContext->error;
}

bool i2cWrite(void *pv, uint8_t addr_, uint8_t reg, uint8_t data)
{
    return i2cWriteBuffer(pv, addr_, reg, 1, &data);
}

bool i2cRead(void *pv, uint8_t addr_, uint8_t reg_, uint8_t* buf, uint_fast16_t len)
{
	i2cContext_st * i2cContext = pv;
	I2C_TypeDef *I2Cx = i2cContext->i2c;
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    i2cContext->addr = addr_ << 1;
    i2cContext->reg = reg_;
    i2cContext->writing = 0;
    i2cContext->reading = 1;
    i2cContext->read_p = buf;
    i2cContext->write_p = buf;
    i2cContext->bytes = len;
    i2cContext->busy = 1;
    i2cContext->error = false;

    if (!(I2Cx->CR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(I2Cx->CR1 & 0x0100)) {                                    // ensure sending a start
            while (I2Cx->CR1 & 0x0200 && --timeout > 0) { ; }           // wait for any stop to finish sending
            if (timeout == 0)
                return i2cHandleHardwareFailure(i2cContext);
            I2C_GenerateSTART(I2Cx, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }

    timeout = I2C_DEFAULT_TIMEOUT;
    while (i2cContext->busy && --timeout > 0) { ; }
    if (timeout == 0)
        return i2cHandleHardwareFailure(i2cContext);

    return !i2cContext->error;
}

static void i2c_er_handler(I2C_TypeDef *I2Cx, i2c_idx_t i2cIndex)
{
	i2cContext_st * i2cContext = &i2cContexts[i2cIndex];
    // Read the I2Cx status register
    volatile uint32_t SR1Register = I2Cx->SR1;

    if (SR1Register & 0x0F00) {                                         // an error
        i2cContext->error = true;
    }

    // If AF, BERR or ARLO, abandon the current job and commence new if there are jobs
    if (SR1Register & 0x0700) {
        (void)I2Cx->SR2;                                                // read second status register to clear ADDR if it is set (note that BTF will not be set after a NACK)
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                        // disable the RXNE/TXE interrupt - prevent the ISR tailchaining onto the ER (hopefully)
        if (!(SR1Register & 0x0200) && !(I2Cx->CR1 & 0x0200)) {         // if we dont have an ARLO error, ensure sending of a stop
            if (I2Cx->CR1 & 0x0100) {                                   // We are currently trying to send a start, this is very bad as start, stop will hang the peripheral
                // TODO - busy waiting in highest priority IRQ. Maybe only set flag and handle it from main loop
                while (I2Cx->CR1 & 0x0100) { ; }                        // wait for any start to finish sending
                I2C_GenerateSTOP(I2Cx, ENABLE);                         // send stop to finalise bus transaction
                while (I2Cx->CR1 & 0x0200) { ; }                        // wait for stop to finish sending
                i2cInit(i2cHardwareMap[i2cIndex].port);                 // reset and configure the hardware
            } else {
                I2C_GenerateSTOP(I2Cx, ENABLE);                         // stop to free up the bus
                I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);   // Disable EVT and ERR interrupts while bus inactive
            }
        }
    }
    I2Cx->SR1 &= ~0x0F00;                                               // reset all the error bits to clear the interrupt
    i2cContext->busy = 0;
}

void i2c_ev_handler(I2C_TypeDef *I2Cx, i2c_idx_t i2cIndex)
{
	i2cContext_st * i2cContext = &i2cContexts[i2cIndex];
    static uint8_t subaddress_sent, final_stop;                         // flag to indicate if subaddess sent, flag to indicate final bus condition
    static int8_t index;                                                // index is signed -1 == send the subaddress
    uint8_t SReg_1 = I2Cx->SR1;                                         // read the status register here

    if (SReg_1 & 0x0001) {                                              // we just sent a start - EV5 in ref manual
        I2Cx->CR1 &= ~0x0800;                                           // reset the POS bit so ACK/NACK applied to the current byte
        I2C_AcknowledgeConfig(I2Cx, ENABLE);                            // make sure ACK is on
        index = 0;                                                      // reset the index
        if (i2cContext->reading && (subaddress_sent || 0xFF == i2cContext->reg)) {              // we have sent the subaddr
            subaddress_sent = 1;                                        // make sure this is set in case of no subaddress, so following code runs correctly
            if (i2cContext->bytes == 2)
                I2Cx->CR1 |= 0x0800;                                    // set the POS bit so NACK applied to the final byte in the two byte read
            I2C_Send7bitAddress(I2Cx, i2cContext->addr, I2C_Direction_Receiver);    // send the address and set hardware mode
        } else {                                                        // direction is Tx, or we havent sent the sub and rep start
            I2C_Send7bitAddress(I2Cx, i2cContext->addr, I2C_Direction_Transmitter); // send the address and set hardware mode
            if (i2cContext->reg != 0xFF)                                            // 0xFF as subaddress means it will be ignored, in Tx or Rx mode
                index = -1;                                             // send a subaddress
        }
    } else if (SReg_1 & 0x0002) {                                       // we just sent the address - EV6 in ref manual
        // Read SR1,2 to clear ADDR
        __DMB();                                                        // memory fence to control hardware
        if (i2cContext->bytes == 1 && i2cContext->reading && subaddress_sent) {                 // we are receiving 1 byte - EV6_3
            I2C_AcknowledgeConfig(I2Cx, DISABLE);                       // turn off ACK
            __DMB();
            (void)I2Cx->SR2;                                            // clear ADDR after ACK is turned off
            I2C_GenerateSTOP(I2Cx, ENABLE);                             // program the stop
            final_stop = 1;
            I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                     // allow us to have an EV7
        } else {                                                        // EV6 and EV6_1
            (void)I2Cx->SR2;                                            // clear the ADDR here
            __DMB();
            if (i2cContext->bytes == 2 && i2cContext->reading && subaddress_sent) {             // rx 2 bytes - EV6_1
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                   // turn off ACK
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to fill
            } else if (i2cContext->bytes == 3 && i2cContext->reading && subaddress_sent)        // rx 3 bytes
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // make sure RXNE disabled so we get a BTF in two bytes time
            else                                                        // receiving greater than three bytes, sending subaddress, or transmitting
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
        }
    } else if (SReg_1 & 0x004) {                                        // Byte transfer finished - EV7_2, EV7_3 or EV8_2
        final_stop = 1;
        if (i2cContext->reading && subaddress_sent) {                               // EV7_2, EV7_3
            if (i2cContext->bytes > 2) {                                            // EV7_2
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                   // turn off ACK
                i2cContext->read_p[index++] = (uint8_t)I2Cx->DR;                    // read data N-2
                I2C_GenerateSTOP(I2Cx, ENABLE);                         // program the Stop
                final_stop = 1;                                         // required to fix hardware
                i2cContext->read_p[index++] = (uint8_t)I2Cx->DR;                    // read data N - 1
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                 // enable TXE to allow the final EV7
            } else {                                                    // EV7_3
                if (final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     // program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                    // program a rep start
                i2cContext->read_p[index++] = (uint8_t)I2Cx->DR;                    // read data N - 1
                i2cContext->read_p[index++] = (uint8_t)I2Cx->DR;                    // read data N
                index++;                                                // to show job completed
            }
        } else {                                                        // EV8_2, which may be due to a subaddress sent or a write completion
            if (subaddress_sent || (i2cContext->writing)) {
                if (final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     // program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                    // program a rep start
                index++;                                                // to show that the job is complete
            } else {                                                    // We need to send a subaddress
                I2C_GenerateSTART(I2Cx, ENABLE);                        // program the repeated Start
                subaddress_sent = 1;                                    // this is set back to zero upon completion of the current task
            }
        }
        // TODO - busy waiting in ISR
        // we must wait for the start to clear, otherwise we get constant BTF
        while (I2Cx->CR1 & 0x0100) { ; }
    } else if (SReg_1 & 0x0040) {                                       // Byte received - EV7
        i2cContext->read_p[index++] = (uint8_t)I2Cx->DR;
        if (i2cContext->bytes == (index + 3))
            I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                    // disable TXE to allow the buffer to flush so we can get an EV7_2
        if (i2cContext->bytes == index)                                             // We have completed a final EV7
            index++;                                                    // to show job is complete
    } else if (SReg_1 & 0x0080) {                                       // Byte transmitted EV8 / EV8_1
        if (index != -1) {                                              // we dont have a subaddress to send
            I2Cx->DR = i2cContext->write_p[index++];
            if (i2cContext->bytes == index)                                         // we have sent all the data
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush
        } else {
            index++;
            I2Cx->DR = i2cContext->reg;                                             // send the subaddress
            if (i2cContext->reading || !i2cContext->bytes)                                      // if receiving or sending 0 bytes, flush now
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush
        }
    }
    if (index == i2cContext->bytes + 1) {                                           // we have completed the current job
        subaddress_sent = 0;                                            // reset this here
        if (final_stop)                                                 // If there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTF
            I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);       // Disable EVT and ERR interrupts while bus inactive
        i2cContext->busy = 0;
    }
}

static i2cDevice_t const * i2cPortLookup( i2c_port_t const port )
{
	unsigned int i;

	for (i=0; i < NB_I2C_PORTS; i++)
	{
		if ( port == i2cHardwareMap[i].port )
			return &i2cHardwareMap[i];
	}

	return NULL;
}

void * i2cInit(i2c_port_t port)
{
	i2cDevice_t const * i2c_config;
	i2cContext_st * i2cContext = NULL;

    NVIC_InitTypeDef nvic;
    I2C_InitTypeDef i2c;

	if ((i2c_config=i2cPortLookup(port)) == NULL)
	{
		goto done;
	}

    // Turn on peripheral clock, save device and index
    RCC_APB1PeriphClockCmd(i2c_config->peripheral, ENABLE);

    // clock out stuff to make sure slaves arent stuck
    // This will also configure GPIO as AF_OD at the end
    i2cUnstick(i2c_config);

    // Init I2C peripheral
    I2C_DeInit(i2c_config->dev);
    I2C_StructInit(&i2c);

    I2C_ITConfig(i2c_config->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);               // Enable EVT and ERR interrupts - they are enabled by the first request
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed = 400000;
    I2C_Cmd(i2c_config->dev, ENABLE);
    I2C_Init(i2c_config->dev, &i2c);

#define I2C_ER_IRQ_PRIORITY                   0
#define I2C_ER_IRQ_SUBPRIORITY                0
#define I2C_EV_IRQ_PRIORITY                   0
#define I2C_EV_IRQ_SUBPRIORITY                0
    // I2C ER Interrupt
    nvic.NVIC_IRQChannel = i2c_config->er_irq;
    nvic.NVIC_IRQChannelPreemptionPriority = I2C_ER_IRQ_PRIORITY;
    nvic.NVIC_IRQChannelSubPriority = I2C_ER_IRQ_SUBPRIORITY;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    // I2C EV Interrupt
    nvic.NVIC_IRQChannel = i2c_config->ev_irq;
    nvic.NVIC_IRQChannelPreemptionPriority = I2C_EV_IRQ_PRIORITY;
    nvic.NVIC_IRQChannelSubPriority = I2C_EV_IRQ_SUBPRIORITY;
    NVIC_Init(&nvic);

	i2cContext = &i2cContexts[i2c_config - &i2cHardwareMap[0]];
	i2cContext->i2c = i2c_config->dev;
	i2cContext->busy = false;
	i2cContext->error = false;
done:
	return i2cContext;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

void I2C1_ER_IRQHandler(void)
{
    i2c_er_handler(I2C1, I2C1_IDX);
}

void I2C1_EV_IRQHandler(void)
{
    i2c_ev_handler(I2C1, I2C1_IDX);
}

void I2C2_ER_IRQHandler(void)
{
    i2c_er_handler(I2C2, I2C2_IDX);
}

void I2C2_EV_IRQHandler(void)
{
    i2c_ev_handler(I2C2, I2C2_IDX);
}

