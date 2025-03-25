#ifndef SettingsAndControls_H
#define SettingsAndControls_H

#include "pico/types.h"
#include "OptionsRegistry.h"

//Outputs are 0-based
#define FMS_OUTPUT_AV_OUT (6-1)
#define FMS_OUTPUT_5_8G (4-1)
#define FMS_OUTPUT_1_2G (5-1)
#define FMS_OUTPUT_3_3G (3-1)
#define FMS_OUTPUT_SCRAMBLER (1-1)
//#define FMS_OUTPUT_USB (3-1)

//inputs are 1-based
#define FMS_INPUT_CAMERA 3

#define PIN_POWERON_TX_1G2 16
#define PIN_POWERON_TX_5G8 18
#define PIN_POWERON_TX_3G3 17
#define PIN_POWERON_CAMERA 19

class EepromStorage;
class I2cDisplayDevice;
class UiMenu;
class OptionsRegistry;
class Option;
class VideoSwitchDevice;

class SettingsAndControls
{
public:
    SettingsAndControls();
    void InitControls();
    void InitOptions();
    void InitUi();
    void InitTX();
    void Update();
    void UpdateTX();
    void Deinit();

    uint32_t GetClockFreq() const;
    float GetClockDiv() const;

    static void Reboot(void* dummyArg);

protected:
    OptionIndex m_optionClockDiv = INVALID_OPTION_INDEX;
    OptionIndex m_optionClockDivAdjust = INVALID_OPTION_INDEX;
    constexpr static float adjustMulScale = 1.0f/1000000.0f;
    OptionIndex m_optionClockFreq = INVALID_OPTION_INDEX;

    OptionIndex m_optionVideoTx1g2 = INVALID_OPTION_INDEX;
    OptionIndex m_optionVideoTx5g8 = INVALID_OPTION_INDEX;
    OptionIndex m_optionVideoTx3g3 = INVALID_OPTION_INDEX;
    OptionIndex m_optionVideoAvOut = INVALID_OPTION_INDEX;

    OptionsRegistry* m_optionsRegistry = nullptr;
    EepromStorage* m_storage = nullptr;
    I2cDisplayDevice* m_display = nullptr;
    UiMenu* m_uiMenu = nullptr;
    VideoSwitchDevice* m_videoSwitch = nullptr;
};

#endif