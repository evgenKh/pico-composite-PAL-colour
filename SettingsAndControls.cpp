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

}

void SettingsAndControls::InitOptions()
{
    m_optionsRegistry = new OptionsRegistry();

    m_optionClockFreq = m_optionsRegistry->AddOption(Option("Freq", 321e6, 318e6, 324e6, 500000));
    m_optionClockDiv = m_optionsRegistry->AddOption(Option("ClkDiv", 12, 12, 12, 1) );
    m_optionClockDivAdjust = m_optionsRegistry->AddOption(Option("DivAdj", 1000000, 990000, 1010000, 100) );
    m_optionTest = m_optionsRegistry->AddOption(Option("Test", 1, -5, 5, 1));

    m_storage = new EepromStorage();

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


    UiMenuPage page1;
    page1.m_title = "page1";
    page1.m_items.emplace_back(UiMenuItem(m_optionClockFreq));
    page1.m_items.emplace_back(UiMenuItem(m_optionClockDiv));
    page1.m_items.emplace_back(UiMenuItem(m_optionClockDivAdjust));
    page1.m_items.emplace_back(UiMenuItem(m_optionTest));    

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

    VideoSwitchDevice videoSwitch;
    videoSwitch.m_address = 0x03;
    videoSwitch.m_i2cInstance = i2c1;
    videoSwitch.Init();
}

void SettingsAndControls::Update()
{
    
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
uint32_t SettingsAndControls::GetClockFreq() const
{
    return m_optionsRegistry->GetOption(m_optionClockFreq)->m_currentValue;
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
