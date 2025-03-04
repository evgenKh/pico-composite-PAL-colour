#ifndef UiMenuItem_H
#define UiMenuItem_H

#include <pico/types.h>
#include "OptionsRegistry.h"
class Option;

class UiMenuItem
{
    public:
    UiMenuItem(){};
    UiMenuItem(OptionIndex option):
        m_option(option){};
        
    const char* m_title = "";
    int8_t m_pageNumToOpen = -1;
    OptionIndex m_option = INVALID_OPTION_INDEX;

    //pointer to callback function and param to pass to it (usually pointer to this)
    void (*m_callbackFunc) (void*) = nullptr; 
    void* m_callbackFuncUserData = nullptr; 
};


#endif