
#define PICO_DEFAULT_I2C 1
#define PICO_DEFAULT_I2C_SDA_PIN 10
#define PICO_DEFAULT_I2C_SCL_PIN 11

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#define PICO_DEFAULT_I2C 1
#define PICO_DEFAULT_I2C_SDA_PIN 10
#define PICO_DEFAULT_I2C_SCL_PIN 11

#define PIN_I2C_SDA 10
#define PIN_I2C_SCL 11
#define I2C_INST i2c_default
#define I2C_BAUD_RATE (5*1000)


#define FMS_I2C_DEV_ADDR _u(0x03)
#define FMS_I2C_DEV_ADDR43 _u(0x43)

#define FMS_REG_OUT1_2 0x00
#define FMS_REG_OUT3_4 0x01
#define FMS_REG_OUT5_6 0x02
#define FMS_REG_CLAMP 0x03
#define FMS_REG_GAIN 0x04

#define FMS_INPUTS_COUNT 8
#define FMS_INPUT_TESTPATTERN 6
#define FMS_INPUT_CAMERA 1
#define FMS_INPUT_NONE 0

#define FMS_OUTPUTS_COUNT 6
#define FMS_OUTPUT_RCA 2
#define FMS_OUTPUT_5_8G 4
#define FMS_OUTPUT_1_2G 3
#define FMS_OUTPUT_3_3G 5
#define FMS_OUTPUT_USB 1

#define FMS_GAIN_6dB 0
#define FMS_GAIN_0dB 1

#define FMS_BIAS 0
#define FMS_CLAMP 1

uint8_t g_fms_outputs[FMS_OUTPUTS_COUNT] = {2, 1, 2, 2, 2, 2};


void setupFMS();
void updateFMSChannels();
void setFMSRegister(uint8_t address, uint8_t value);


// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
//bool reserved_addr(uint8_t addr) {
//    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
//}
 
 
void setupI2C(){
    
    // Enable UART so we can print status output
    //stdio_init_all();
    sleep_ms(2000);
    printf("Done.\n");
    i2c_init(I2C_INST, I2C_BAUD_RATE);
    
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);
    // Make the I2C pins available to picotool
    //bi_decl(bi_2pins_with_func(PIN_I2C_SDA, PIN_I2C_SCL, GPIO_FUNC_I2C));

    // Perform a 1-byte dummy read from the probe address. If a slave
    // acknowledges this address, the function returns the number of bytes
    // transferred. If the address byte is ignored, the function returns
    // -1.

    // Skip over any reserved addresses.
    int ret;
    uint8_t rxdata;
    
    ret = i2c_read_blocking(I2C_INST, FMS_I2C_DEV_ADDR, &rxdata, 1, false);

    printf(ret < 0 ? "." : "@0x3");

    ret = i2c_read_blocking(I2C_INST, FMS_I2C_DEV_ADDR43, &rxdata, 1, false);

    printf(ret < 0 ? "." : "@0x43");
    printf("Done.\n");

    setupFMS();

    i2c_deinit(I2C_INST);
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
    
    ret += i2c_write_blocking(I2C_INST, FMS_I2C_DEV_ADDR, &rxdata[0], 2, false);
    //ret += i2c_write_blocking(I2C_INST, FMS_I2C_DEV_ADDR, &rxdata[1], 1, false);
sleep_ms(100);
    printf("Set FMS register %X to %X. Errorcode: %d\n", rxdata[0], rxdata[1], ret);

    
    ret += i2c_write_blocking(I2C_INST, FMS_I2C_DEV_ADDR43, &rxdata[0], 2, false);
    //ret += i2c_write_blocking(I2C_INST, FMS_I2C_DEV_ADDR, &rxdata[1], 1, false);
sleep_ms(100);
    printf("Set 0x43 FMS register %X to %X. Errorcode: %d\n", rxdata[0], rxdata[1], ret);
}