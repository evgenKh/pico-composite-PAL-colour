#include "OptionsRegistry.h"
#include "Option.h"
#include "EepromStorage.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "stdio.h"

//#define printf(...) (0)
OptionIndex OptionsRegistry::AddOption(Option &&rval)
{
    //Using index due to nature of std::vector that reallocates it's data sometimes, so pointers to items get invalid.
    m_options.emplace_back(rval);
    Option& newOpt = *m_options.rbegin();
    newOpt.m_offsetInEeprom = m_nextOptionUserDataOffset;
    m_nextOptionUserDataOffset += sizeof(Option::m_valueToSave);
    m_userDataSizeAccumulator = m_nextOptionUserDataOffset;
    return m_options.size()-1;
}

Option *OptionsRegistry::GetOption(const OptionIndex index)
{
    if(index < 0 || index >= m_options.size())
        return nullptr;

    return &m_options[index];
}

bool OptionsRegistry::LoadFromStorage(EepromStorage *storage)
{
    storage->SetExpectedUserDataSize(m_userDataSizeAccumulator);
    if(!storage->IsStoredHeaderValid())
    {
        printf("LoadFromStorage: Invalid header.");            
        return false;
    }

    bool success = true;
    for(Option& opt: m_options)
    {
        if(!storage->Load(
            &opt.m_loadedValue,
            sizeof(opt.m_loadedValue),
            opt.m_offsetInEeprom))
        {
            success = false;
            printf("LoadFromStorage: Storage::Load failed for %s opt.", opt.m_name);            
        }
    }
    //Validate all values
    for(Option& opt: m_options)
    {
        if(opt.m_loadedValue < opt.m_min ||
            opt.m_loadedValue > opt.m_max)
        {
            success = false;
            printf("LoadFromStorage: Option %s invalid value.", opt.m_name);
        }
    }

    if(!success) return false;

    //Apply loaded values
    for(Option& opt: m_options)
    {
        printf("Applying loaded '%s', %d->%d (offset %d)", opt.m_name, opt.m_loadedValue, opt.m_currentValue, opt.m_offsetInEeprom);
        opt.m_currentValue = opt.m_loadedValue;
    }
    
    return true;
}

bool OptionsRegistry::SaveToStorage(EepromStorage *storage)
{
    storage->SetExpectedUserDataSize(m_userDataSizeAccumulator);
    //if(!storage->IsStoredHeaderValid()) return false;
    bool success = true;
    
    //Validate all values
    for(Option& opt: m_options)
    {        
        if(!opt.m_isSaveRequested)
        {
            opt.m_valueToSave = opt.m_currentValue;
            opt.m_isSaveRequested = false;
        }

        if(opt.m_valueToSave < opt.m_min ||
            opt.m_valueToSave > opt.m_max)
        {
            success = false;
            printf("SaveToStorage: Option %s invalid value.", opt.m_name);
        }
    }

    if(!success) return false;

    for(Option& opt: m_options)
    {
        opt.m_isSaveRequested = false;
        storage->Save(&opt.m_valueToSave, sizeof(opt.m_valueToSave), opt.m_offsetInEeprom, false);
        
    }

    return storage->FlushSave();
}
