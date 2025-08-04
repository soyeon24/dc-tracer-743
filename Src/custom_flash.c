#include "../MDK-ARM/Inc/custom_flash.h"

#include "stm32h7xx_hal.h"

bool Custom_Flash_Unlock(void) {
    return (HAL_FLASH_Unlock() == HAL_OK);
}

void Custom_Flash_Lock(void) {
    HAL_FLASH_Lock();
}

bool Custom_Flash_Erase(void) {
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t sectorError;

    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    eraseInit.Sector = FLASH_SECTOR_4;        // 원하는 Sector 번호
    eraseInit.NbSectors = 1;
    eraseInit.Banks = FLASH_BANK_1;

    if (!Custom_Flash_Unlock()) return false;

    if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK) {
        Custom_Flash_Lock();
        return false;
    }

    Custom_Flash_Lock();
    return true;
}

bool Custom_Flash_Write(uint8_t *src, uint32_t length) {
    if (length % 32 != 0) return false;

    if (!Custom_Flash_Unlock()) return false;

    for (uint32_t offset = 0; offset < length; offset += 32) {
        uint32_t* srcWord = (uint32_t*)(src + offset);
        uint32_t addr = CUSTOM_FLASH_SECTOR_ADDR + offset;

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, (uint32_t*)srcWord) != HAL_OK) {
            Custom_Flash_Lock();
            return false;
        }
    }

    Custom_Flash_Lock();
    return true;
}

bool Custom_Flash_Read(uint8_t *dst, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        dst[i] = *(__IO uint8_t *)(CUSTOM_FLASH_SECTOR_ADDR + i);
    }
    return true;
}
