#ifndef __FLASH_H__
#define __FLASH_H__

#if defined(STM32F30X)
#define FLASH_SIZE				(256u * 1024u)
#elif defined(STM32F10X)
#define FLASH_SIZE				(128u * 1024u)
#endif

#define SECTOR_SIZE				(2048u)
#define FLASH_START_ADDRESS		0x08000000

bool eraseFlashPage( uint32_t flashAddress );
bool writeWordToFlash( uint32_t value, uint32_t flashAddress, uint_fast8_t maximum_attempts, bool eraseSectorIfNeeded );

#endif /*  __FLASH_H__ */
