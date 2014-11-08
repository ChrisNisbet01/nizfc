#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <utils.h>
#include <stm32f30x_flash.h>

#include <config_structure.h>

/* provided by linker script */

#define FLASH_SIZE				(256u * 1024u)
#define SECTOR_SIZE				(2048u)
#define CONFIG_SIZE				(2048u)
#define NB_CONFIG_SECTORS		((CONFIG_SIZE+SECTOR_SIZE-1)/SECTOR_SIZE))
#define FLASH_START_ADDRESS		0x08000000
#define CONFIG_START_ADDRESS	(FLASH_START_ADDRESS + FLASH_SIZE - CONFIG_SIZE)

#define MAX_FLASH_WRITE_ATTEMPTS	3

typedef enum config_version_t
{
	config_version_1 = 0
} config_version_t;

typedef struct config_header_st
{
	uint8_t 	check_value;	/* used to verify that configuration data is valid */
	uint8_t 	version;		/* we may alter the configuration data structure over time */
	uint16_t 	length;			/* total length of configuration data (including this header */
} config_header_st;

typedef struct config_save_ctx_st
{
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
    FLASH_Status status = 0;
	uint32_t config_destination = CONFIG_START_ADDRESS + (config_save_ctx.word_offset * sizeof(uint32_t));
	uint_fast8_t tries = MAX_FLASH_WRITE_ATTEMPTS;

    FLASH_Unlock();

	do
	{
	    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

		/*
			eraseSector is false when writing out the header. We don't want to erase the first sector again
			when doing this.
		*/

		if (config_save_ctx.eraseSector == true)
		{
		    if ((config_destination & (SECTOR_SIZE-1)) == 0)
		    {
		        status = FLASH_ErasePage(config_destination);
		        if (status != FLASH_COMPLETE)
		            continue;
		    }
		}

	    status = FLASH_ProgramWord(config_destination, value);
	}
	while (status != FLASH_COMPLETE && --tries > 0);

	if (status != FLASH_COMPLETE)
		goto done;

	wrote_word = true;
	config_save_ctx.word_offset++;

done:

    FLASH_Lock();

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
    FLASH_Status status = 0;

	config_save_ctx.check_value = 0;
	/* first config data byte comes after the header */
	config_save_ctx.word_offset = (sizeof(config_header_st) + sizeof(uint32_t) - 1)/sizeof(uint32_t);
	config_save_ctx.config_size = 0;
	config_save_ctx.bufferIndex = 0;
	config_save_ctx.eraseSector = true;

	/* erase the first sector. Assumes that config starts at begining of sector */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

    status = FLASH_ErasePage(CONFIG_START_ADDRESS);

    FLASH_Lock();

	if ( status != FLASH_COMPLETE )
		initOK = false;

	return initOK;
}

bool saveConfigurationData( void const * const data, uint32_t length )
{
	uint8_t const * pdata = data;
	bool wrote_data = false;

	while ( length-- )
	{
		if ( writeConfigByte( *pdata++ ) == false )
			goto done;
	}

	wrote_data = true;

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
		if (writeConfigByte( configuration_id_reserved ) == false)
			goto done;
	}

	/* include the header in the check value */
	header.length = config_save_ctx.config_size;	/* includes header and all config data, but not padding at the end */
	header.check_value = 0;
	header.version = config_version_1;

	/* update the check value withthe header bytes */
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

	return success;
}


