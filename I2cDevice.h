#ifndef I2cDevice_H
#define I2cDevice_H

#include "IDevice.h"

#if defined(ARDUINO)
#include "Arduino.h"
#elif defined(PLATFORM_PICO)
//#include "hardware/i2c.h"
#endif // ARDUINO

#include <memory> //for shared_ptr
#include "Enums.h"

class I2cMuxDevice;
class TwoWire;
struct i2c_inst;
typedef i2c_inst i2c_inst_t;

class I2cDevice : public IDevice
{
public:
    virtual void Init() override;
    size_t SendBytes(const uint8_t *buf, size_t len);
    size_t SendByte(uint8_t val);
    bool CheckIsConnected();

#if defined(ARDUINO)
    std::shared_ptr<TwoWire> m_i2cInterface = nullptr;
#elif defined(PLATFORM_PICO)
    i2c_inst_t *m_i2cInstance = nullptr;
#endif

    std::shared_ptr<I2cMuxDevice> m_i2cMuxDevice = nullptr;
    uint8_t m_address = 0;
    I2cChannelId m_i2cMuxChannel = I2C_CHANNEL_INVALID;

    // protected:
};
#endif