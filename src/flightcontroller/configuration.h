#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

bool initConfigurationSave( void );
bool saveConfigurationData( void const * const data, uint32_t length );
bool validateConfigurationData( void );
bool completeConfigurationSave( void );
int getConfigurationSize( void );
void const * getConfigurationData( unsigned int *data_size );

#endif /*  __CONFIGURATION_H__ */
