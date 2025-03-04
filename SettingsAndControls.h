#ifndef SettingsAndControls_H
#define SettingsAndControls_H

#include "pico/types.h"
#include "OptionsRegistry.h"

class EepromStorage;
class I2cDisplayDevice;
class UiMenu;
class OptionsRegistry;
class Option;

class SettingsAndControls
{
    public:
    SettingsAndControls();
    void InitOptions();
    void InitUi();
    void Update();
    void Deinit();

    uint32_t GetClockFreq() const;
    float GetClockDiv() const;

    static void Reboot(void* dummyArg);

    protected:
    OptionIndex m_optionClockDiv = INVALID_OPTION_INDEX;
    OptionIndex m_optionClockDivAdjust = INVALID_OPTION_INDEX;
    constexpr static float adjustMulScale = 1.0f/1000000.0f;
    OptionIndex m_optionClockFreq = INVALID_OPTION_INDEX;
    OptionIndex m_optionTest = INVALID_OPTION_INDEX;

    OptionsRegistry* m_optionsRegistry = nullptr;
    EepromStorage* m_storage = nullptr;
    I2cDisplayDevice* m_display = nullptr;
    UiMenu* m_uiMenu = nullptr;
};

#endif