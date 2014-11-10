#ifndef __FLASH_STM32F30X_H__
#define __FLASH_STM32F30X_H__

#define FLASH_SIZE				(256u * 1024u)
#define SECTOR_SIZE				(2048u)
#define FLASH_START_ADDRESS		0x08000000

bool eraseFlashPage( uint32_t flashAddress );
bool writeWordToFlash( uint32_t value, uint32_t flashAddress, uint_fast8_t maximum_attempts, bool eraseSectorIfNeeded );

#endif /*  __FLASH_STM32F30X_H__ */
