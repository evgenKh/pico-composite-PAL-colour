#include "I2cDisplayDevice.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include "pins.h"

#include <vector>
#include <string.h>

uint8_t I2cDisplayDevice::u8x8_gpio_and_delay_pico(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    if (msg == U8X8_MSG_DELAY_MILLI)
    {
        sleep_ms(arg_int); // Delay em milissegundos
    }
    if (msg == U8X8_MSG_DELAY_NANO)
    {
        int a = 0;
        a = 5;
        // sleep_us(1); // Delay em milissegundos
    }
    return 0;
}

uint8_t I2cDisplayDevice::u8x8_byte_hw_i2c_pico(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    I2cDisplayDevice *pThisDevice = static_cast<I2cDisplayDevice *>(u8x8_GetUserPtr(u8x8));
    I2cDevice *pThisI2cDevice = static_cast<I2cDevice *>(pThisDevice);

    switch (msg)
    {
    case U8X8_MSG_BYTE_SEND: // Envia dados via I²C
    {
        memcpy(&pThisDevice->m_i2cBuf[pThisDevice->m_i2cBufSize], arg_ptr, arg_int);
        pThisDevice->m_i2cBufSize += arg_int;
        break;
    }
    case U8X8_MSG_BYTE_INIT: // Inicialização (opcional)
        break;
    case U8X8_MSG_BYTE_START_TRANSFER: // Início da transferência
        break;
    case U8X8_MSG_BYTE_END_TRANSFER: // Fim da transferência
    {
        int res = pThisDevice->SendBytes(pThisDevice->m_i2cBuf, pThisDevice->m_i2cBufSize);
        pThisDevice->m_i2cBufSize = 0; // Clear buf
        break;
    }
    }
    return 0;
}

void I2cDisplayDevice::Init()
{

    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&m_u8g2, U8G2_R0, u8x8_byte_hw_i2c_pico, u8x8_gpio_and_delay_pico);

    u8g2_SetUserPtr(&m_u8g2, (void *)this);
    u8g2_SetI2CAddress(&m_u8g2, m_address); // Endereço padrão do SSD1306
    u8g2_InitDisplay(&m_u8g2);              // send init sequence to the display, display is in sleep mode after this,
    u8g2_SetPowerSave(&m_u8g2, 0);          // wake up display

    u8g2_ClearBuffer(&m_u8g2);
}

void I2cDisplayDevice::SetFontHeight(uint8_t fontHeight)
{
}

void I2cDisplayDevice::DrawText(uint16_t x, uint16_t y, const char *string)
{
    // Desenha texto
    //u8g2_SetFont(&m_u8g2, u8g2_font_fub14_tr); 
    u8g2_SetFont(&m_u8g2, u8g2_font_8x13_tr); // 1127bytes
    u8g2_DrawStr(&m_u8g2, x, y, string);  
}

void I2cDisplayDevice::SelectRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    uint8_t color_backup = m_u8g2.draw_color;
    u8g2_SetDrawColor(&m_u8g2, 2);         /* XOR */
    u8g2_DrawBox(&m_u8g2, x, y, w, h);
    u8g2_SetDrawColor(&m_u8g2, color_backup);
}

void I2cDisplayDevice::Flush()
{
    // Envia o buffer para o display
    u8g2_SendBuffer(&m_u8g2);
    // u8g2_ClearBuffer(&m_u8g2);
}

void I2cDisplayDevice::Clear()
{
    // Limpa o buffer
    u8g2_ClearBuffer(&m_u8g2);
    u8g2_ClearDisplay(&m_u8g2);
    Flush();
}
