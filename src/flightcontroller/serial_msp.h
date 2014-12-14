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
    void * ownerContext;
    uint8_t offset;
    uint8_t dataSize;
    uint8_t checksum;
    uint8_t indRX;
    uint8_t * inBuf;
	unsigned int inBufSize;

	void (*putChar)( void *pv, uint8_t ch );

    mspState_e state;
    uint8_t cmdMSP;
} mspContext_st;

bool mspProcess(mspContext_st * ctx, uint8_t c);
void initMSPContext( mspContext_st * ctx, void * ownerContext, uint8_t * inBuf, unsigned int inBufSize, void (*putChar)( void *pv, uint8_t ch ) );

#endif /*  __SERIAL_MSP__ */
