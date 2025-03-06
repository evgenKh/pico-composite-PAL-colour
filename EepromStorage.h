#ifndef EepromStorage_H
#define EepromStorage_H

#include "hardware/regs/addressmap.h"
#include "hardware/flash.h"

#include "pico/mutex.h"
#include "pico/critical_section.h"


extern char __flash_binary_start;  // defined in linker script
extern char __flash_binary_end;    // defined in linker script

class Option;

class EepromStorage{
    public:
    struct StorageDataHeader{
        uint32_t m_magic = 0x6969;
        uint8_t m_version = 3;
        uint32_t m_headerSize = sizeof(StorageDataHeader);
        uint32_t m_userDataSize = 0;
    };

    EepromStorage();
    bool Load(void* dstInRam, size_t size, size_t offsetInEeprom);
    bool Save(void* srcInRam, size_t size, size_t offsetInEeprom, bool flush = true);
    bool FlushSave();

    void SetExpectedUserDataSize(size_t userDataSize)
    {
        m_dataHeader.m_userDataSize = userDataSize;
    }

    bool IsStoredHeaderValid();

protected:
    //static constexpr uint32_t m_dataSize = sizeof(StorageData);
    static constexpr uint32_t m_dataSizeAligned = FLASH_SECTOR_SIZE;
    static constexpr uint32_t m_dataStart = PICO_FLASH_SIZE_BYTES - m_dataSizeAligned;
    //static_assert(m_dataSizeAligned >= m_dataSize);

    

    
    //StorageData m_loadedValue;    
    uint8_t m_saveBuf[m_dataSizeAligned];
    StorageDataHeader m_dataHeader;

};


#endif