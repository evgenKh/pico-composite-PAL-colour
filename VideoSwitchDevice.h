#ifndef VideoSwitchDevice_H
#define VideoSwitchDevice_H

#include "pins.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "string.h"

#include "I2cDevice.h"

//#define FMS_I2C_DEV_ADDR _u(0x03)

#define FMS_REG_OUT1_2 0x00
#define FMS_REG_OUT3_4 0x01
#define FMS_REG_OUT5_6 0x02
#define FMS_REG_CLAMP 0x03
#define FMS_REG_GAIN 0x04

#define FMS_INPUTS_COUNT 8
//#define FMS_INPUT_TESTPATTERN 6
//#define FMS_INPUT_CAMERA 1
//#define FMS_INPUT_NONE 0

#define FMS_OUTPUTS_COUNT 6


#define FMS_GAIN_6dB 0
#define FMS_GAIN_0dB 1

#define FMS_BIAS 0
#define FMS_CLAMP 1

class VideoSwitchDevice : public I2cDevice{
public:

uint8_t g_fms_outputs[FMS_OUTPUTS_COUNT] = {0, 0, 0, 0, 0, 0};




  // I2C reserves some addresses for special purposes. We exclude these from the scan.
  // These are any addresses of the form 000 0xxx or 111 1xxx
  //bool reserved_addr(uint8_t addr) {
  //    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
  //}
  void SetChannel(uint8_t output, uint8_t input)
  {
    if(output < FMS_OUTPUTS_COUNT && input < FMS_INPUTS_COUNT)
    {
      g_fms_outputs[output] = input;
    }
  }

  inline uint8_t GetInputChannel(uint8_t output) const{
    if(output < FMS_OUTPUTS_COUNT)
    {
      return g_fms_outputs[output];
    } 
    return 0;
  }
  
  void Init(){    
      //uint8_t mainInput = (false ? 3 : 6);
      memset(g_fms_outputs, 0, FMS_OUTPUTS_COUNT);
      int ret;
      uint8_t rxdata;
      printf(CheckIsConnected() ? ".0x3" : "@0x3");

      setupFMS();
  }

  void setupFMS(){
    uint8_t regOut1_2Value = (g_fms_outputs[0] & 0x0F) | ((g_fms_outputs[1] << 4) & 0xF0);
    uint8_t regOut3_4Value = (g_fms_outputs[2] & 0x0F) | ((g_fms_outputs[3] << 4) & 0xF0);
    uint8_t regOut5_6Value = (g_fms_outputs[4] & 0x0F) | ((g_fms_outputs[5] << 4) & 0xF0);
    uint8_t regClampValue = 0xFF;//Clamp all as AC-coupled
    uint8_t regGainValue = 0x00;//x2(6dB) gain for all outputs

    setFMSRegister(FMS_REG_CLAMP, regClampValue);
    setFMSRegister(FMS_REG_GAIN, regGainValue);
    setFMSRegister(FMS_REG_OUT1_2, regOut1_2Value);
    setFMSRegister(FMS_REG_OUT3_4, regOut3_4Value);
    setFMSRegister(FMS_REG_OUT5_6, regOut5_6Value);  
  }

  void updateFMSChannels(){
    uint8_t regOut1_2Value = (g_fms_outputs[0] & 0x0F) | ((g_fms_outputs[1] << 4) & 0xF0);
    uint8_t regOut3_4Value = (g_fms_outputs[2] & 0x0F) | ((g_fms_outputs[3] << 4) & 0xF0);
    uint8_t regOut5_6Value = (g_fms_outputs[4] & 0x0F) | ((g_fms_outputs[5] << 4) & 0xF0);
    setFMSRegister(FMS_REG_OUT1_2, regOut1_2Value);
    setFMSRegister(FMS_REG_OUT3_4, regOut3_4Value);
    setFMSRegister(FMS_REG_OUT5_6, regOut5_6Value); 
  }

  void setFMSRegister(uint8_t address, uint8_t value){
    
      // Skip over any reserved addresses.
      int ret=0;
      uint8_t rxdata[2];
      rxdata[0] = address;
      rxdata[1] = value;
      
      ret += SendBytes( &rxdata[0], 2);
      sleep_ms(50);
      printf("Set FMS register %X to %X. bytes written: %d\n", rxdata[0], rxdata[1], ret);
  }

};

#endif