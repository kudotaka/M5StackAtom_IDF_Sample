#pragma once
#include "freertos/FreeRTOS.h"

#if CONFIG_SOFTWARE_SK6812_SUPPORT
#include "sk6812.h"
#endif

#if CONFIG_SOFTWARE_BUTTON_SUPPORT
#include "button.h"
#endif

#if CONFIG_SOFTWARE_MPU6886_SUPPORT
#include "mpu6886.h"
#endif

#if CONFIG_SOFTWARE_EXPPORTS_SUPPORT
#include "driver/gpio.h"
#include "driver/uart.h"
#include "i2c_device.h"

#define PORT_A_SDA_PIN GPIO_NUM_26
#define PORT_A_SCL_PIN GPIO_NUM_32
#define PORT_DAC_PIN GPIO_NUM_26
#define PORT_ADC_PIN GPIO_NUM_32
#define PORT_LEVEL_HIGH 1
#define PORT_LEVEL_LOW  0

#define PORT_A_I2C_STANDARD_BAUD 100000

typedef enum {
    NONE,   //Reset GPIO to default state.
    OUTPUT, //Set GPIO to output mode.
    INPUT,  //Set GPIO to input mode.
    I2C,    //Enable I2C mode. Only available on Port A—GPIO XX(SDA) and Port A—GPIO XX (SCL).
    ADC,    //Enable ADC mode. Only available on Port B—GPIO XX
    DAC,    //Enable DAC mode. Only available on Port B—GPIO XX
    UART    //Enable UART RX/TX mode. UART TX only available on Port 
                    // C—GPIO 14 and UART RX is only available on Port C—GPIO 
                    // 13. Only supports full-duplex UART so setting one pin 
                    // to UART mode will also set the other pin to UART mode.
} pin_mode_t;
#endif

void M5Atom_Init(void);

#if CONFIG_SOFTWARE_BUTTON_SUPPORT

extern Button_t* button_a;
void M5Atom_Button_Init(void);
#endif

#if CONFIG_SOFTWARE_SK6812_SUPPORT
#define SK6812_COLOR_OFF     0x000000
#define SK6812_COLOR_BLACK   0x000000
#define SK6812_COLOR_BLUE    0x0000FF
#define SK6812_COLOR_LIME    0x00FF00
#define SK6812_COLOR_AQUA    0x00FFFF
#define SK6812_COLOR_RED     0xFF0000
#define SK6812_COLOR_MAGENTA 0xFF00FF
#define SK6812_COLOR_YELLOW  0xFFFF00
#define SK6812_COLOR_WHITE   0xFFFFFF

void M5Atom_Sk6812_Init(void);
void M5Atom_Sk6812_SetColor(uint16_t pos, uint32_t color);
void M5Atom_Sk6812_SetAllColor(uint32_t color);
void M5Atom_Sk6812_SetBrightness(uint8_t brightness);
void M5Atom_Sk6812_Show(void);
void M5Atom_Sk6812_Clear(void);
#endif

#if CONFIG_SOFTWARE_EXPPORTS_SUPPORT
esp_err_t M5Atom_Port_PinMode(gpio_num_t pin, pin_mode_t mode);
bool M5Atom_Port_Read(gpio_num_t pin);
esp_err_t M5Atom_Port_Write(gpio_num_t pin, bool level);
I2CDevice_t M5Atom_Port_A_I2C_Begin(uint8_t device_address, uint32_t baud);
esp_err_t M5Atom_Port_A_I2C_Read(I2CDevice_t device, uint32_t register_address, uint8_t *data, uint16_t length);
esp_err_t M5Atom_Port_A_I2C_Write(I2CDevice_t device, uint32_t register_address, uint8_t *data, uint16_t length);
void M5Atom_Port_A_I2C_Close(I2CDevice_t device);
uint32_t M5Atom_Port_ADC_ReadRaw(void);
uint32_t M5Atom_Port_ADC_ReadMilliVolts(void);
#endif
