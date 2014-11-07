#ifndef __RECEIVER_H__
#define __RECEIVER_H__

#define INVALID_CHANNEL_VALUE	0

typedef enum receiver_mode_type
{
	receiver_mode_pwm = 0,
	receiver_mode_ppm = 1
} receiver_mode_type;

uint_fast16_t readReceiverChannel(uint_fast8_t channel);
typedef void (*NewReceiverChannelDataCB)( uint32_t *channels, uint_fast8_t first_index, uint_fast8_t nb_channels );

void initReceiver( void );
void openReceiver( receiver_mode_type mode );

#endif /*  __RECEIVER_H__ */
