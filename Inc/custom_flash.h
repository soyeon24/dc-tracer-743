#ifndef __CUSTOM_FLASH_H__
#define __CUSTOM_FLASH_H__

#include <stdint.h>
#include <stdbool.h>

#define CUSTOM_FLASH_SECTOR_ADDR  0x08040000  // 원하는 시작 주소
#define CUSTOM_FLASH_SECTOR_SIZE  0x20000     // 128KB

bool Custom_Flash_Unlock(void);
void Custom_Flash_Lock(void);
bool Custom_Flash_Erase(void);
bool Custom_Flash_Write(uint8_t *src, uint32_t length);
bool Custom_Flash_Read(uint8_t *dst, uint32_t length);

#endif
