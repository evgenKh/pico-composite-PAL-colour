#ifndef I2cDisplayDevice_H
#define I2cDisplayDevice_H

#include "I2cDevice.h"
#include "IDisplay.h"
#include <vector>
#include "u8g2.h"
#include "U8g2lib.h"
class I2cDisplayDevice : public I2cDevice, public IDisplay
{
public:
    void Init();

    virtual uint16_t GetWidth() const { return m_width; };
    virtual uint16_t GetHeight() const { return m_height; };
    virtual void SetFontHeight(uint8_t fontHeight);
    virtual void DrawText(uint16_t x, uint16_t y, const char *string);
    virtual void SelectRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    virtual void Flush();
    virtual void Clear();

    static uint8_t u8x8_byte_hw_i2c_pico(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
    static uint8_t u8x8_gpio_and_delay_pico(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

    u8g2_t m_u8g2;
    //U8G2 m_u8g2cpp;
    uint16_t m_width = 128;
    uint16_t m_height = 64;

private:
    uint8_t m_i2cBuf[340];
    uint8_t m_i2cBufSize = 0;
};

#endif