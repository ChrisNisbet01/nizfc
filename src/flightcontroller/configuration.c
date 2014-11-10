#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <utils.h>
#include <flash_stm32f30x.h>

#include <config_structure.h>

#define CONFIG_SIZE				(2048u)
#define NB_CONFIG_SECTORS		((CONFIG_SIZE+SECTOR_SIZE-1)/SECTOR_SIZE))
#define CONFIG_START_ADDRESS	(FLASH_START_ADDRESS + FLASH_SIZE - CONFIG_SIZE)

#define MAX_FLASH_WRITE_ATTEMPTS	3

typedef enum config_version_t
{
	config_version_1 = 1
} config_version_t;

/* this header is located at CONFIG_START_ADDRESS */

typedef struct config_header_st
{
	uint8_t 	check_value;	/* used to verify that configuration data is valid. Currently just a simple LRC */
	uint8_t 	version;		/* we may alter the configuration data structure over time */
	uint16_t 	length;			/* total length of configuration data (including this header */
} config_header_st;

typedef struct config_save_ctx_st
{
	bool			saving_config;
	uint8_t			check_value;
	uint_fast16_t 	word_offset;
	uint32_t		buffer;
	uint_fast8_t	bufferIndex;
	bool			eraseSector;
	uint_fast16_t	config_size;
} config_save_ctx_st;

static config_save_ctx_st config_save_ctx;


bool writeConfigWord( uint32_t value )
{
	bool wrote_word = false;

	wrote_word = writeWordToFlash( value,
									CONFIG_START_ADDRESS + (config_save_ctx.word_offset * sizeof(uint32_t)),
									MAX_FLASH_WRITE_ATTEMPTS,
									config_save_ctx.eraseSector );

	if ( wrote_word == true )
		config_save_ctx.word_offset++;

	return wrote_word;
}

bool writeConfigByte( uint8_t byte )
{
	bool wrote_byte = false;
	uint8_t *pdest = (uint8_t *)&config_save_ctx.buffer;

	if (config_save_ctx.word_offset * sizeof(uint32_t) >= CONFIG_SIZE)	/* can't write beyond our limits */
		goto done;

	pdest[config_save_ctx.bufferIndex++] = byte;

	if (config_save_ctx.bufferIndex == 4)	/* time to write the next word */
	{
		config_save_ctx.bufferIndex = 0;
		/* got a full word buffered up. Write it to FLASH. */
		if (writeConfigWord( config_save_ctx.buffer ) == false)
			goto done;
	}

	config_save_ctx.config_size++;
	config_save_ctx.check_value ^= byte;
	wrote_byte = true;

done:
	return wrote_byte;
}

bool initConfigurationSave( void )
{
	bool initOK = true;

	config_save_ctx.check_value = 0;
	/* first config data byte comes after the header */
	config_save_ctx.word_offset = (sizeof(config_header_st) + sizeof(uint32_t) - 1)/sizeof(uint32_t);
	config_save_ctx.config_size = 0;
	config_save_ctx.bufferIndex = 0;
	config_save_ctx.eraseSector = true;

	/* the first write isn't to the start of a sector, so we need to erase it specifically */
	initOK = eraseFlashPage( CONFIG_START_ADDRESS );

	config_save_ctx.saving_config = initOK;

	return initOK;
}

bool saveConfigurationData( void const * const data, uint32_t length )
{
	uint8_t const * pdata = data;
	bool wrote_data = false;

	if ( config_save_ctx.saving_config == true )
	{
		while ( length-- )
		{
			if ( writeConfigByte( *pdata++ ) == false )
				goto done;
		}
		wrote_data = true;
	}

done:

	return wrote_data;
}

bool validateConfigurationData( void )
{
	bool configurationValid = false;
	uint8_t check_value = 0;
	config_header_st *header = (config_header_st *)CONFIG_START_ADDRESS;
	uint8_t *pch = (uint8_t *)header;
	uint_fast16_t length;

	length = header->length;

	if ( length > CONFIG_SIZE )
		goto done;

	while (length--)
		check_value ^= *pch++;

	if (check_value != 0)
		goto done;

	configurationValid = true;
done:
	return configurationValid;
}

bool completeConfigurationSave( void )
{
	bool success = false;
	config_header_st header;
	uint8_t *pch;
	uint_fast8_t offset;

	/* pad out to the next word boundary */
	while ( config_save_ctx.bufferIndex != 0 )
	{
		/* pad out with a reserved value so that we can identify when the config ends */
		if (writeConfigByte( configuration_id_reserved ) == false)
			goto done;
	}

	/* include the header in the check value */
	header.length = config_save_ctx.config_size + sizeof header;	/* includes header and all config data and padding at the end */
	header.check_value = 0;
	header.version = config_version_1;

	/* update the check value with the header bytes */
	pch = (uint8_t *)&header;
	for	( offset = 0; offset < sizeof header; offset++ )
		config_save_ctx.check_value ^= *pch++;
	header.check_value = config_save_ctx.check_value;

	/* write out the header */
	config_save_ctx.eraseSector = false;	/* the first sector has already been erased */
	config_save_ctx.word_offset = 0;		/* header goes at the head */
	saveConfigurationData( &header, sizeof header );

	/* ensure that the final word has been written. It should have been if header size is a word size multiple */
	while ( config_save_ctx.bufferIndex != 0 )
	{
		if (writeConfigByte( 0 ) == false)
			goto done;
	}

	/* validate the data */
	success = validateConfigurationData();
done:

	config_save_ctx.saving_config = false;

	return success;
}

int getConfigurationSize( void )
{
	int configurationSize = -1;

	if (validateConfigurationData() == false)
		goto done;

	config_header_st *header = (config_header_st *)CONFIG_START_ADDRESS;
	configurationSize = header->length - sizeof *header;

done:

	return configurationSize;
}

void const * getConfigurationData( unsigned int *data_size )
{
	void const *configuration_data = NULL;
	config_header_st *header = (config_header_st *)CONFIG_START_ADDRESS;

	if (validateConfigurationData() == false)
		goto done;

	configuration_data = header + 1;
	*data_size = header->length - sizeof *header;

	/*
		Note that configuration data length may be 0.
		We differentiate between this and an error by returning a
		non-NULL pointer, but indicating length is 0.
	*/
done:

	return configuration_data;
}
