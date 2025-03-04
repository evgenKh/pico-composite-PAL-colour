#include "pins.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "u8g2.h"
#include <vector>


//#define printf(...) (0)

#define PIN_BTN_1 21
#define PIN_BTN_2 20

#define PIN_I2C_SDA 10
#define PIN_I2C_SCL 11
#define I2C_INST i2c1
#define I2C_BAUD_RATE (10*1000)

#define SSD1306_I2C_DEV_ADDR (_u(0x3c))
#define SSD1306_I2C_DEV_ADDR_78 _u(0x78)

const unsigned char OLED_init_cmd[25]=
{
/*0xae,0X00,0X10,0x40,0X81,0XCF,0xff,0xa1,0xa4,
0xA6,0xc8,0xa8,0x3F,0xd5,0x80,0xd3,0x00,0XDA,0X12,
0x8d,0x14,0xdb,0x40,0X20,0X02,0xd9,0xf1,0xAF*/
0xAE,//关闭显示
0xD5,//设置时钟分频因子,震荡频率
0x80,//[3:0],分频因子;[7:4],震荡频率

0xA8,//设置驱动路数
0X3F,//默认0X3F(1/64)
0xD3,//设置显示偏移
0X00,//默认为0
0x40,//设置显示开始行[5:0],行数.
0x8D,//电荷泵设置
0x14,//bit2,开启/关闭
0x20,//设置内存地址模式
0x02,//[1:0],00,列地址模式;01,行地址模式;10,页地址模式;默认10;
0xA1,//段重定义设置,bit0:0,0->0;1,0->127;
0xC8,//设置COM扫描方向;bit3:0,普通模式;1,重定义模式COM[N-1]->COM0;N:驱动路数
0xDA,//设置COM硬件引脚配置
0x12,//[5:4]配置
0x81,//对比度设置
0xEF,//1~255;默认0X7F(亮度设置,越大越亮)
0xD9,//设置预充电周期
0xf1,//[3:0],PHASE1;[7:4],PHASE2;
0xDB,//设置VCOMH电压倍率
0x30,//[6:4]000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;
0xA4,//全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
0xA6,//设置显示方式;bit0:1,反相显示;0,正常显示
0xAF,//开启显示
};

void setupI2cScreen();

//https://wokwi.com/projects/417102299903413249
// Callback para comunicação I²C (necessário para U8g2)
uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
  static std::vector<uint8_t> gLastMsg;
    switch (msg) {
      case U8X8_MSG_BYTE_SEND: // Envia dados via I²C
      {
        i2c_write_burst_blocking(I2C_INST, SSD1306_I2C_DEV_ADDR, (uint8_t*)gLastMsg.data(), gLastMsg.size());
        gLastMsg.resize(arg_int);
        memcpy(gLastMsg.data(), arg_ptr, arg_int);
      }
        break;
      case U8X8_MSG_BYTE_INIT: // Inicialização (opcional)
        break;
      case U8X8_MSG_BYTE_START_TRANSFER: // Início da transferência
        break;
      case U8X8_MSG_BYTE_END_TRANSFER:   // Fim da transferência
        i2c_write_blocking(I2C_INST ,SSD1306_I2C_DEV_ADDR, gLastMsg.data(), gLastMsg.size(), false);
        gLastMsg.clear();
        break;
    }
    return 0;
  }
  
  // Callback para controle de pinos (não usado aqui, mas necessário)
  uint8_t u8x8_gpio_and_delay_pico(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    if (msg == U8X8_MSG_DELAY_MILLI) {
      sleep_ms(arg_int); // Delay em milissegundos
    }
    if (msg == U8X8_MSG_DELAY_NANO) {
      int a =0;
      a = 5;
      //sleep_us(1); // Delay em milissegundos
    }
    return 0;
  }
void setupI2cScreen(){
    

    sleep_ms(500);
    // Make the I2C pins available to picotool
    //bi_decl(bi_2pins_with_func(PIN_I2C_SDA, PIN_I2C_SCL, GPIO_FUNC_I2C));

    // Perform a 1-byte dummy read from the probe address. If a slave
    // acknowledges this address, the function returns the number of bytes
    // transferred. If the address byte is ignored, the function returns
    // -1.
    int ret;
    uint8_t rxdata;
    
    ret = i2c_read_blocking(I2C_INST, SSD1306_I2C_DEV_ADDR, &rxdata, 1, false);
    printf(ret < 0 ? "display not found" : "display found");
    ret = i2c_read_blocking(I2C_INST, SSD1306_I2C_DEV_ADDR_78, &rxdata, 1, false);
    printf(ret < 0 ? "\ndisplay 78 not found" : "display 78 found");

    sleep_ms(1000);

    u8g2_t u8g2;
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_gpio_and_delay_pico);

    u8g2_SetI2CAddress(&u8g2, SSD1306_I2C_DEV_ADDR); // Endereço padrão do SSD1306
    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
    u8g2_SetPowerSave(&u8g2, 0); // wake up display

        

    // Limpa o buffer
    u8g2_ClearBuffer(&u8g2);


    // Desenha uma linha diagonal
    u8g2_DrawLine(&u8g2, 0, 0, 127, 63);
    u8g2_DrawLine(&u8g2, 0, 0, 20, 50);
    // Envia o buffer para o display
    u8g2_SendBuffer(&u8g2);
    // Desenha texto
    u8g2_SetFont(&u8g2, u8g2_font_fub14_tr); // Define a fonte
    u8g2_DrawStr(&u8g2, 10, 32, "Hello,\n OLED!"); // Exibe texto

    // Envia o buffer para o display
    u8g2_SendBuffer(&u8g2);

    sleep_ms(1000); // Aguarda 1 segundo

    i2c_deinit(I2C_INST);
}
