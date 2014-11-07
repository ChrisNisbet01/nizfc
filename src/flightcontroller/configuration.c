#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <utils.h>

#include <stm32f30x_flash.h>

/* provided by linker script */

#define FLASH_SIZE				(256u * 1024u)
#define SECTOR_SIZE				(2048u)
#define CONFIG_SIZE				(2048u)
#define NB_CONFIG_SECTORS		((CONFIG_SIZE+SECTOR_SIZE-1)/SECTOR_SIZE))
#define FLASH_START_ADDRESS		0x08000000
#define CONFIG_START_ADDRESS	(FLASH_START_ADDRESS + FLASH_SIZE - CONFIG_SIZE)

typedef struct nizfc_config_st
{
	uint32_t dummy;
} nizfc_config_st;

static nizfc_config_st configuration;

bool saveConfiguration( void )
{
	COMPILE_TIME_ASSERT(sizeof(nizfc_config_st) <= CONFIG_SIZE);

	int success = false;
    FLASH_Status status = 0;
    uint32_t *config_source, *config_dest;
    int8_t tries = 3;

    FLASH_Unlock();

    do
    {
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

        for (config_dest = (uint32_t *)CONFIG_START_ADDRESS, config_source = (uint32_t *)&configuration;
        		config_source < (uint32_t *)((uint32_t)&configuration + sizeof(configuration));
        		config_dest++, config_source++)
        {
            if (((uint32_t)config_dest & (SECTOR_SIZE-1)) == 0)
            {
                status = FLASH_ErasePage((uint32_t)config_dest);
                if (status != FLASH_COMPLETE)
                    break;
            }

            status = FLASH_ProgramWord((uint32_t)config_dest, *config_source);
            if (status != FLASH_COMPLETE)
                break;
        }
    }
	while (status != FLASH_COMPLETE && --tries > 0);

    FLASH_Lock();

    if (status == FLASH_COMPLETE)
    	success = true;

    return success;
}


