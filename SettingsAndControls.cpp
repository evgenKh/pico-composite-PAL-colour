#include "SettingsAndControls.h"

#include "pins.h"
#include "I2cDevice.h"
#include "I2cDisplayDevice.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "EepromStorage.h"
#include "UiMenu.h"
#include "UiMenuItem.h"
#include <stdio.h>
#include "pico/stdlib.h"

#include "Option.h"
#include "OptionsRegistry.h"
#include "EepromStorage.h"
#include "VideoSwitchDevice.h"

//#define printf(...) (0)

SettingsAndControls::SettingsAndControls()
{
    
    m_display = new I2cDisplayDevice();
    m_uiMenu = new UiMenu();
    m_videoSwitch = new VideoSwitchDevice();

}

void SettingsAndControls::InitControls()
{
    #if defined(PIN_BTN_1)
        gpio_init(PIN_BTN_1);
        gpio_set_dir(PIN_BTN_1, GPIO_IN);
        gpio_pull_up(PIN_BTN_1);
    #endif // PIN_BTN_1    
    #if defined(PIN_BTN_2)
        gpio_init(PIN_BTN_2);
        gpio_set_dir(PIN_BTN_2, GPIO_IN);
        gpio_pull_up(PIN_BTN_2);
    #endif // PIN_BTN_2
}

void SettingsAndControls::InitOptions()
{
    m_optionsRegistry = new OptionsRegistry();

    m_optionClockDivAdjust = m_optionsRegistry->AddOption(Option("DivAdj", 1000000, 990000, 1010000, 200) );
    m_optionClockFreq = m_optionsRegistry->AddOption(Option("Freq", 321e6, 316e6, 324e6, 200000));
    m_optionClockDiv = m_optionsRegistry->AddOption(Option("ClkDiv", 12, 12, 12, 1) );
    //m_optionTest = m_optionsRegistry->AddOption(Option("Test", 1, -5, 5, 1));

    m_optionVideoTx1g2 = m_optionsRegistry->AddOption(Option("Tx 1g2", 0, 0, 6, 1) );
    m_optionVideoTx5g8 = m_optionsRegistry->AddOption(Option("Tx 5g8", 0, 0, 6, 1) );
    m_optionVideoTx3g3 = m_optionsRegistry->AddOption(Option("Tx 3g3", 0, 0, 6, 1) );
    m_optionVideoAvOut = m_optionsRegistry->AddOption(Option("AvOut", 0, 0, 6, 1) );

    m_storage = new EepromStorage();

    
    #if defined(PIN_BTN_1) && defined(PIN_BTN_2)
    bool btnsPressed = !gpio_get(PIN_BTN_1) && !gpio_get(PIN_BTN_2);
    if(btnsPressed)
    {
        printf("Skipped loading saved options.");
        return;
    }
    #endif

    bool loadSuccess = m_optionsRegistry->LoadFromStorage(m_storage);
    if (loadSuccess)
    {
        printf("All settings loaded from storage.");
    }
    else
    {
        printf("All settings NOT loaded from storage.");
    }
}

void SettingsAndControls::InitUi()
{
    
    //I2c
    i2c_init(i2c1, 100*1000);//20kbps
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
    sleep_ms(100);
    
    
    m_display->m_address = 0x3c;
    m_display->m_i2cInstance = i2c1;

    printf("Display %s connected", (m_display->CheckIsConnected() ? "" : "not"));
    m_display->Init();
    m_display->DrawText(10, 20, "Loading..");
    m_display->Flush();

    UiMenuPage page1;
    page1.m_title = "page1";
    page1.m_items.emplace_back(UiMenuItem(m_optionVideoAvOut));
    page1.m_items.emplace_back(UiMenuItem(m_optionVideoTx1g2));
    page1.m_items.emplace_back(UiMenuItem(m_optionVideoTx5g8));
    page1.m_items.emplace_back(UiMenuItem(m_optionVideoTx3g3));

    page1.m_items.emplace_back(UiMenuItem(m_optionClockDivAdjust));
    page1.m_items.emplace_back(UiMenuItem(m_optionClockFreq));
    page1.m_items.emplace_back(UiMenuItem(m_optionClockDiv));

    UiMenuItem rebootItem;
    rebootItem.m_title = " Save&Reboot";
    rebootItem.m_callbackFunc = &SettingsAndControls::Reboot;
    page1.m_items.emplace_back(rebootItem);    

    m_uiMenu->m_pages.push_back(page1);

    UiMenuPage errorPage;
    errorPage.m_title = "errorPage";
    UiMenuItem freeSpaceErrorItem;
    freeSpaceErrorItem.m_title = "No free space to save.";
    errorPage.m_items.emplace_back(freeSpaceErrorItem);    
    m_uiMenu->m_pages.push_back(errorPage);

    m_uiMenu->m_display = m_display;
    m_uiMenu->m_optionsRegistry = m_optionsRegistry;
    m_uiMenu->OpenPage(0);    

    //m_uiMenu->Draw();

}


void SettingsAndControls::Update()
{
    UpdateTX();

    #if defined(PIN_BTN_1)
    bool btn1Pressed = !gpio_get(PIN_BTN_1);
    if(btn1Pressed)
    {
        m_uiMenu->SelectNextItem();
        //sleep_ms(50);
    }
    #endif // PIN_BTN_1 
       
    #if defined(PIN_BTN_2)
    bool btn2Pressed = !gpio_get(PIN_BTN_2);
    if(btn2Pressed)
    {
        m_uiMenu->ItemIncrement();
        
        bool saveSuccess = m_optionsRegistry->SaveToStorage(m_storage);
        if (saveSuccess)
        {
            printf("All settings saved to storage.");
        }
        else
        {
            printf("All settings NOT saved to storage.");
            m_uiMenu->OpenPage(1);    
        }
    }
    #endif // PIN_BTN_2  

    m_uiMenu->Draw();
}

void SettingsAndControls::InitTX()
{
    m_videoSwitch->m_address = 0x03;
    m_videoSwitch->m_i2cInstance = i2c1;
    m_videoSwitch->Init();

    gpio_init(PIN_POWERON_TX_1G2);
    gpio_set_dir(PIN_POWERON_TX_1G2, GPIO_OUT);
    gpio_put(PIN_POWERON_TX_1G2, false);

    gpio_init(PIN_POWERON_TX_5G8);
    gpio_set_dir(PIN_POWERON_TX_5G8, GPIO_OUT);
    gpio_put(PIN_POWERON_TX_5G8, false);

    gpio_init(PIN_POWERON_TX_3G3);
    gpio_set_dir(PIN_POWERON_TX_3G3, GPIO_OUT);
    gpio_put(PIN_POWERON_TX_3G3, false);

    gpio_init(PIN_POWERON_CAMERA);
    gpio_set_dir(PIN_POWERON_CAMERA, GPIO_OUT);
    gpio_put(PIN_POWERON_CAMERA, false);

}

void SettingsAndControls::UpdateTX()
{

    bool changed = false;
    bool cameraOn = false;
    uint8_t input1g2 = m_optionsRegistry->GetOption(m_optionVideoTx1g2)->m_currentValue;
    if(m_videoSwitch->GetInputChannel(FMS_OUTPUT_1_2G) != input1g2)
    {
        m_videoSwitch->SetChannel(FMS_OUTPUT_1_2G, input1g2);
        gpio_put(PIN_POWERON_TX_1G2, input1g2);
        changed = true;
        if(input1g2 == FMS_INPUT_CAMERA)
            cameraOn = true;
    }

    uint8_t input5g8 = m_optionsRegistry->GetOption(m_optionVideoTx5g8)->m_currentValue;
    if(m_videoSwitch->GetInputChannel(FMS_OUTPUT_5_8G) != input5g8)
    {
        m_videoSwitch->SetChannel(FMS_OUTPUT_5_8G, input5g8);
        gpio_put(PIN_POWERON_TX_5G8, input5g8);
        changed = true;
        if(input5g8 == FMS_INPUT_CAMERA)
            cameraOn = true;
    }

    uint8_t input3g3 = m_optionsRegistry->GetOption(m_optionVideoTx3g3)->m_currentValue;
    if(m_videoSwitch->GetInputChannel(FMS_OUTPUT_3_3G) != input3g3)
    {
        m_videoSwitch->SetChannel(FMS_OUTPUT_3_3G, input3g3);
        gpio_put(PIN_POWERON_TX_3G3, input3g3);
        changed = true;
        if(input3g3 == FMS_INPUT_CAMERA)
            cameraOn = true;
    }

    uint8_t inputAv = m_optionsRegistry->GetOption(m_optionVideoAvOut)->m_currentValue;
    if(m_videoSwitch->GetInputChannel(FMS_OUTPUT_AV_OUT) != inputAv)
    {
        m_videoSwitch->SetChannel(FMS_OUTPUT_AV_OUT, inputAv);
        changed = true;
        if(inputAv == FMS_INPUT_CAMERA)
            cameraOn = true;
    }

    if(changed)
    {
        m_videoSwitch->updateFMSChannels();
        gpio_put(PIN_POWERON_CAMERA, cameraOn);
    }


}

uint32_t SettingsAndControls::GetClockFreq() const
{
    Option* opt = m_optionsRegistry->GetOption(m_optionClockFreq);
    int32_t clampedValue = MAX(MIN(opt->m_currentValue, opt->m_max), opt->m_min);
    return clampedValue;
}

float SettingsAndControls::GetClockDiv() const
{
    float clockDiv = static_cast<float>(m_optionsRegistry->GetOption(m_optionClockDiv)->m_currentValue);
    float adjust = adjustMulScale * static_cast<float>(m_optionsRegistry->GetOption(m_optionClockDivAdjust)->m_currentValue);
    return clockDiv + adjustMulScale;
}

void SettingsAndControls::Reboot(void *dummyArg)
{
    watchdog_reboot(0, 0, 0);
}
