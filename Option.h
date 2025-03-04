#ifndef Option_H
#define Option_H

#include <pico/types.h>

class Option{
    public:
    typedef int32_t T;
    Option(const char* const name, T defaultValue, size_t offsetInEeprom = 0xFFFFFF) : 
        m_name(name),
        m_defaultValue(defaultValue),
        m_currentValue(defaultValue),
        m_min(defaultValue),
        m_max(defaultValue),
        m_incrementStep(0)
    {
    }
    Option(const char* const name, T defaultValue, T min, T max, T incrementStep, size_t offsetInEeprom = 0xFFFFFF): 
    m_name(name),
    m_defaultValue(defaultValue),
    m_currentValue(defaultValue),
    m_min(min),
    m_max(max),
    m_incrementStep(incrementStep)
    {
    }

    T m_loadedValue;
    T m_valueToSave;
    T m_currentValue;

    const T m_defaultValue;
    const T m_max;
    const T m_min;
    const T m_incrementStep;
    const char* const m_name;
    size_t m_offsetInEeprom = 0xFFFFFF;
    bool m_isSaveRequested = false;

    private:
    Option();
};

#endif