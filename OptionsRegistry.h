#ifndef OptionsRegistry_H
#define OptionsRegistry_H

#include <vector>
#include "pico/types.h"

class EepromStorage;
class Option;

typedef int16_t OptionIndex;
const static OptionIndex INVALID_OPTION_INDEX = -1;

class OptionsRegistry
{
public:
    OptionIndex AddOption(Option&& rval);
    Option*     GetOption(const OptionIndex index);
    bool        LoadFromStorage(EepromStorage* storage);
    bool        SaveToStorage(EepromStorage* storage);
private:
    size_t m_nextOptionUserDataOffset = 0;
    size_t m_userDataSizeAccumulator = 0;
    std::vector<Option> m_options;
};

#endif