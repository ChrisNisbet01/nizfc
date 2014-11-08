#ifndef __RECEIVER_H__
#define __RECEIVER_H__

#define INVALID_CHANNEL_VALUE	0

uint_fast16_t readReceiverChannel(uint_fast8_t channel);
typedef void (*NewReceiverChannelDataCB)( uint32_t *channels, uint_fast8_t first_index, uint_fast8_t nb_channels );

void initReceiver( void );
void openReceiver( void );

#endif /*  __RECEIVER_H__ */
