#ifndef __SERIAL_MSP__
#define __SERIAL_MSP__

typedef enum {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
} mspState_e;

typedef struct mspContext_st
{
    serial_port_st * serialPort;
    uint8_t offset;
    uint8_t dataSize;
    uint8_t checksum;
    uint8_t indRX;
    uint8_t * inBuf;
	unsigned int inBufSize;

    uint8_t * outBuf;
    unsigned int outBufSize;
    uint_fast8_t outIdx;

    mspState_e state;
    uint8_t cmdMSP;
} mspContext_st;

bool mspProcess(mspContext_st * ctx, uint8_t c);
void initMSPContext( mspContext_st * ctx, serial_port_st * port, uint8_t * inBuf, unsigned int inBufSize, uint8_t * outBuf, unsigned int outBufSize );

#endif /*  __SERIAL_MSP__ */
