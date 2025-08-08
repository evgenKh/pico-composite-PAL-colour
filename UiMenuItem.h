#ifndef UiMenuItem_H
#define UiMenuItem_H

#include <pico/types.h>
#include "OptionsRegistry.h"
class Option;

class UiMenuItem
{
    public:
    
    UiMenuItem(){};

    static UiMenuItem CreateWithOption(OptionIndex optionIndex)
    {
        UiMenuItem item;
        item.m_option = optionIndex;
        return item;
    }

    static UiMenuItem CreateWithCallback(const char* title, void (*callbackFunc)(void*), void* userData)
    {
        UiMenuItem item;
        item.m_title = title;
        item.m_callbackFunc = callbackFunc;
        item.m_callbackFuncUserData = userData;
        return item;
    }
    
    static UiMenuItem CreateWithGoToPage(const char* title, int8_t pageNumToOpen = -1)
    {
        UiMenuItem item;
        item.m_title = title;
        item.m_pageNumToOpen = pageNumToOpen;
        return item;
    }

    static UiMenuItem CreateInactive(const char* title)
    {
        UiMenuItem item;
        item.m_title = title;
        return item;
    }

    public:    
    const char* m_title = "";
    int8_t m_pageNumToOpen = -1;
    OptionIndex m_option = INVALID_OPTION_INDEX;

    //pointer to callback function and param to pass to it (usually pointer to this)
    void (*m_callbackFunc) (void*) = nullptr; 
    void* m_callbackFuncUserData = nullptr; 
};


#endif