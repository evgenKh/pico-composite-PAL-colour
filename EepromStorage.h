#ifndef EEPROM_STORAGE_H
#define EEPROM_STORAGE_H

#include "hardware/regs/addressmap.h"
#include "hardware/flash.h"

class EepromStorage{
    public:
    struct StorageData{
        uint32_t m_magic = 0x6969;
        uint8_t m_version = 1;
        uint32_t m_dataSize = sizeof(StorageData);

        uint32_t m_clockSpeed = 321e6;
        float m_clockDiv = (12*1.0041);
    };

    void Load();
    void Save();

    StorageData m_currentValue;

protected:
    static constexpr uint32_t m_dataSize = sizeof(StorageData);
    static constexpr uint32_t m_dataSizeAligned = FLASH_SECTOR_SIZE;
    static constexpr uint32_t m_dataStart = PICO_FLASH_SIZE_BYTES - m_dataSizeAligned;
    static_assert(m_dataSizeAligned >= m_dataSize);

    StorageData m_loadedValue;    
    uint8_t m_saveBuf[m_dataSizeAligned];

};
#endif