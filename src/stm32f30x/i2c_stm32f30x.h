#ifndef __I2C_H__
#define __I2C_H__

typedef enum i2c_port_t
{
	I2C_PORT_1
} i2c_port_t;

void *i2cInit(i2c_port_t port);
bool i2cWrite(void *pv, uint8_t addr_, uint8_t reg, uint8_t *data);
bool i2cRead(void *pv, uint8_t addr_, uint8_t reg, uint8_t* buf, uint_fast16_t len);

#endif
