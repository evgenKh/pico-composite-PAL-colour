#ifndef PINS_H
#define PINS_H

//Custom pins
#define PIN_LED_ODDEVEN 25
#define PIN_LED_INIT 13
//#define PIN_LED_PROFILING 14
#define PIN_LED_FPS 12

#define PIN_BTN_1 21
#define PIN_BTN_2 20

#define PIN_I2C_SDA 10
#define PIN_I2C_SCL 11
//i2c0 or i2c1 depends on pin selected
#define I2C_INSTANCE_ID 1
#define I2C_INSTANCE_FROM_ID(id) (__CONCAT(i2c,id))


//Default values
//If not used, comment define, don't set to 0 or -1
#ifndef PIN_LED_ODDEVEN
    #define PIN_LED_ODDEVEN 25
#endif

#ifndef PIN_LED_INIT
    #define PIN_LED_INIT 19
#endif

#ifndef PIN_LED_PROFILING
    //Uncomment to enable profiling
    //#define PIN_LED_PROFILING 26
    // Tiny2040 A0
#endif

#ifndef PIN_LED_FPS
    #define PIN_LED_FPS 20
#endif
#endif