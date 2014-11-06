#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <utils.h>

#include <stm32f30x_flash.h>

/* provided by linker script */

#define FLASH_START_ADDRESS		0x08000000
#define FLASH_SIZE				(256u * 1024u)
#define SECTOR_SIZE				2048u
#define NB_CONFIG_SECTORS		1u
#define CONFIG_SIZE				(NB_CONFIG_SECTORS*SECTOR_SIZE)
#define CONFIG_START_ADDRESS	(FLASH_START_ADDRESS + FLASH_SIZE - CONFIG_SIZE)

typedef struct nizfc_config_st
{
	uint32_t dummy;
} nizfc_config_st;

static nizfc_config_st configuration;

bool saveConfiguration( void )
{
	//COMPILE_TIME_ASSERT(sizeof(nizfc_config_st) > CONFIG_SIZE);
	COMPILE_TIME_ASSERT(sizeof(nizfc_config_st) <= CONFIG_SIZE);

	int success = false;
    FLASH_Status status = 0;
    uint32_t wordOffset, *config_source;
    int8_t tries = 3;

    FLASH_Unlock();

    while (tries-- > 0)
    {
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
        for (wordOffset = 0, config_source = (uint32_t *)&configuration; wordOffset < sizeof(nizfc_config_st); wordOffset += 4, config_source++)
        {
            if ((wordOffset & (SECTOR_SIZE-1)) == 0)
            {
                status = FLASH_ErasePage(CONFIG_START_ADDRESS + wordOffset);
                if (status != FLASH_COMPLETE)
                    break;
            }

            status = FLASH_ProgramWord(CONFIG_START_ADDRESS + wordOffset, *config_source);
            if (status != FLASH_COMPLETE)
                break;
        }
        if (status == FLASH_COMPLETE)
            break;
    }

    FLASH_Lock();

    if (status == FLASH_COMPLETE)
    	success = true;

    return success;
}


