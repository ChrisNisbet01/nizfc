#include <stdbool.h>
#include <stdint.h>
#if defined(STM32F30X)
#include <stm32f30x_flash.h>
#elif defined(STM32F10X)
#include <stm32f10x_flash.h>
#endif
#include <flash.h>

bool eraseFlashPage( uint32_t flashAddress )
{
	bool page_erased = false;
    FLASH_Status status;

    FLASH_Unlock();

	/* erase the first sector. Assumes that config starts at begining of sector */
#if defined(STM32F30X)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#elif defined(STM32F10X)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#endif

    status = FLASH_ErasePage(flashAddress);

    FLASH_Lock();

    if ( status == FLASH_COMPLETE )
    	page_erased = true;
   	else
    	page_erased = true;

	return page_erased;
}

bool writeWordToFlash( uint32_t value, uint32_t flashAddress, uint_fast8_t maximum_attempts, bool eraseSectorIfNeeded )
{
	bool wrote_word = false;
    FLASH_Status status = 0;

    FLASH_Unlock();

	do
	{
#if defined(STM32F30X)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#elif defined(STM32F10X)
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#endif

		/*
			eraseSector is false when writing out the header. We don't want to erase the first sector again
			when doing this.
		*/

		if (eraseSectorIfNeeded == true)
		{
		    if ((flashAddress & (SECTOR_SIZE-1)) == 0)
		    {
		        status = FLASH_ErasePage(flashAddress);
		        if (status != FLASH_COMPLETE)
		            continue;

#if defined(STM32F30X)
			    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
#elif defined(STM32F10X)
			    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
#endif
		    }
		}

	    status = FLASH_ProgramWord(flashAddress, value);
	}
	while (status != FLASH_COMPLETE && --maximum_attempts > 0);

	if (status == FLASH_COMPLETE)
		wrote_word = true;
	else
		wrote_word = false;

    FLASH_Lock();

	return wrote_word;
}


