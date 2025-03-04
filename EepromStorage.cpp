#include "EepromStorage.h"

#include "hardware/flash.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "stdio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/dma.h"
#include "pico/multicore.h"
#include "Option.h"

#include "SyncGlobals.h"

//#define printf(...) (0)
#define memcpy __builtin_memcpy

EepromStorage::EepromStorage()
{
    //Init sync primitives
    if(!mutex_is_initialized(&g_eepromMutex))
        mutex_init(&g_eepromMutex);

    if(!critical_section_is_initialized(&g_eepromCritSection))
        critical_section_init(&g_eepromCritSection);

    if(!critical_section_is_initialized(&g_eepromCritSection2))
        critical_section_init(&g_eepromCritSection2);
}

bool EepromStorage::Load(void *dstInRam, size_t bytesSize, size_t offsetInEeprom)
{
    if( !IsStoredHeaderValid() ) 
    {
        return false;
    }    
    if(offsetInEeprom + sizeof(StorageDataHeader) + bytesSize >= m_dataSizeAligned)
    {
        return false;
    }

    const uint8_t *flashReadPtr = (const uint8_t *) (XIP_BASE + m_dataStart);
    const uint8_t *userDataReadPtr = flashReadPtr + sizeof(StorageDataHeader) + offsetInEeprom;

    memcpy(dstInRam, userDataReadPtr, bytesSize);
    return true;
}

bool EepromStorage::FlushSave()
{
    static const uintptr_t g_flashBinaryStart = (uintptr_t) &__flash_binary_start;
    static const uintptr_t g_flashBinaryEnd = (uintptr_t) &__flash_binary_end;
    const uintptr_t flashBinarySpaceTaken = g_flashBinaryEnd - g_flashBinaryStart;
    const uintptr_t flashBinaryFreeSpace = PICO_FLASH_SIZE_BYTES - flashBinarySpaceTaken;
    //if(flashBinaryFreeSpace < m_dataSizeAligned)
    {
        printf("Not enough flash space for save data! Need %d, got %d", m_dataSizeAligned, flashBinaryFreeSpace);
       // return false;
    }

    //Need both sync primitives for safe Flash write!
    //Mutex to don't interfere with DMA happening on other core
    mutex_enter_blocking(&g_eepromMutex);
    //Critical section to don't interfere with Interrupts

    multicore_lockout_start_blocking();

    printf("is eeprom save mutex... savind %d bytes to %x", m_dataSizeAligned, m_dataStart);
    
        uint32_t ints = save_and_disable_interrupts();

        // disable the channel on IRQ0
        dma_channel_set_irq0_enabled(g_dmaChanToStopA, false);
        // abort the channel
        dma_channel_abort(g_dmaChanToStopA);
        // clear the spurious IRQ (if there was one)
        dma_channel_acknowledge_irq0(g_dmaChanToStopA);
        // re-enable the channel on IRQ0
        dma_channel_set_irq0_enabled(g_dmaChanToStopA, true);

        // disable the channel on IRQ0
        dma_channel_set_irq0_enabled(g_dmaChanToStop32, false);
        // abort the channel
        dma_channel_abort(g_dmaChanToStop32);
        // clear the spurious IRQ (if there was one)
        dma_channel_acknowledge_irq0(g_dmaChanToStop32);
        // re-enable the channel on IRQ0
        dma_channel_set_irq0_enabled(g_dmaChanToStop32, true);




    critical_section_enter_blocking(&g_eepromCritSection);
    {
        flash_range_erase(m_dataStart, m_dataSizeAligned);
        flash_range_program(m_dataStart, m_saveBuf, m_dataSizeAligned);
    }
    critical_section_exit(&g_eepromCritSection);
    restore_interrupts (ints);

multicore_lockout_end_blocking();

    mutex_exit(&g_eepromMutex);


    return true;
}

bool EepromStorage::Save(void *srcInRam, size_t bytesSize, size_t offsetInEeprom, bool flush)
{
    if(offsetInEeprom + sizeof(StorageDataHeader) + bytesSize >= m_dataSizeAligned)
    {
        return false;
    }
    
    memcpy(m_saveBuf, &m_dataHeader, sizeof(StorageDataHeader));

    uint8_t* saveBufUserData = m_saveBuf + sizeof(StorageDataHeader) + offsetInEeprom;
    memcpy(saveBufUserData, srcInRam, bytesSize);

    if(flush)
    {
        return FlushSave();
    }
    return true;
}

bool EepromStorage::IsStoredHeaderValid()
{    
    const uint8_t *flashReadPtr = (const uint8_t *) (XIP_BASE + m_dataStart);
    const StorageDataHeader* loadedHeader = reinterpret_cast<const StorageDataHeader*>(flashReadPtr);
    
    if(m_dataHeader.m_magic == loadedHeader->m_magic &&
        m_dataHeader.m_version == loadedHeader->m_version &&
        m_dataHeader.m_headerSize == loadedHeader->m_headerSize &&
        m_dataHeader.m_dataSize == loadedHeader->m_dataSize)
    {
        return true;
    }
    return false;
}
