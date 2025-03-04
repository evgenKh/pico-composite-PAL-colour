#include "I2cDevice.h"

#if defined(ARDUINO)
#include "Wire.h"
#elif defined(PLATFORM_PICO)
#include "hardware/i2c.h"
#endif

void I2cDevice::Init()
{
}

size_t I2cDevice::SendBytes(const uint8_t *buf, size_t len)
{
#if defined(ARDUINO)
    m_i2cInterface->beginTransmission(m_address);
    m_i2cInterface->write(buf, len);
    uint8_t error = m_i2cInterface->endTransmission();
#elif defined(PLATFORM_PICO)
    size_t bytesWritten = i2c_write_blocking(m_i2cInstance, m_address, buf, len, false);
    return bytesWritten;
    // sleep_ms(1);
#endif
}

size_t I2cDevice::SendByte(uint8_t val)
{
    uint8_t byteBuf = val;
    return SendBytes(&byteBuf, 1);
}

bool I2cDevice::CheckIsConnected()
{
#if defined(ARDUINO)
    // Try receive 1 byte
    m_i2cInterface->beginTransmission(m_address);
    uint8_t error = m_i2cInterface->endTransmission();

    if (error == 0)
    {
        Serial.print("I2C device found at address 0x");
        Serial.print(m_address, HEX);
        Serial.print('\n');
    }
    else
    {
        Serial.print("I2C device NOT found at address 0x");
        Serial.print(m_address, HEX);
        Serial.print('\n');
    }
    return error == 0;

#elif defined(PLATFORM_PICO)
    uint8_t rxData;
    int ret = i2c_read_blocking(m_i2cInstance, m_address, &rxData, 1, false);
    return ret >= 0;
#endif
}
