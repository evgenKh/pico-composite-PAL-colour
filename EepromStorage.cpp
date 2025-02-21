#include "EepromStorage.h"

#include "hardware/flash.h"
#include "pico/stdlib.h"
#include "stdio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

#define memcpy __builtin_memcpy

void EepromStorage::Load()
{
    const uint8_t *flashReadPtr = (const uint8_t *) (XIP_BASE + m_dataStart);
    memcpy(&m_loadedValue, flashReadPtr, sizeof(m_loadedValue));
    if(m_loadedValue.m_magic == m_currentValue.m_magic &&
        m_loadedValue.m_version == m_currentValue.m_version &&
        m_loadedValue.m_dataSize == m_currentValue.m_dataSize)
    {
        m_currentValue = m_loadedValue;
        printf("EepromStorage loaded successfully");
    }

}
void EepromStorage::Save()
{
    memcpy(m_saveBuf, &m_currentValue, sizeof(m_currentValue));
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(m_dataStart, m_dataSizeAligned);
    flash_range_program(m_dataStart, m_saveBuf, m_dataSizeAligned);
    restore_interrupts (ints);
}