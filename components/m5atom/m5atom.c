#include "stdbool.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_common.h"
#include "esp_idf_version.h"

#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

#include "m5atom.h"

#if CONFIG_SOFTWARE_EXPPORTS_SUPPORT
#include "esp_adc_cal.h"
#include "soc/dac_channel.h"

static esp_adc_cal_characteristics_t *adc_characterization;
#define DEFAULT_VREF            1100
#define ADC_CHANNEL             ADC1_CHANNEL_4
#define ADC_WIDTH               ADC_WIDTH_BIT_12
#define ADC_ATTENUATION         ADC_ATTEN_DB_11
//#define DAC_CHANNEL             DAC_GPIO26_CHANNEL

#endif

static const char *TAG = "M5Atom";

void M5Atom_Init(void) {
#if CONFIG_SOFTWARE_BUTTON_SUPPORT
    M5Atom_Button_Init();
#endif

#if CONFIG_SOFTWARE_SK6812_SUPPORT
    M5Atom_Sk6812_Init();
#endif

#if CONFIG_SOFTWARE_MPU6886_SUPPORT
    MPU6886_Init();
#endif

}

/* ===================================================================================================*/
/* --------------------------------------------- BUTTON ----------------------------------------------*/
#if CONFIG_SOFTWARE_BUTTON_SUPPORT
Button_t* button_a;

void M5Atom_Button_Init(void) {
    Button_Init();
    if (Button_Enable(GPIO_NUM_39) == ESP_OK) {
        button_a = Button_Attach(GPIO_NUM_39);
    }
}
#endif
/* ----------------------------------------------- End -----------------------------------------------*/
/* ===================================================================================================*/

/* ===================================================================================================*/
/* --------------------------------------------- SK6812 ----------------------------------------------*/
#if CONFIG_SOFTWARE_SK6812_SUPPORT
pixel_settings_t px;

void M5Atom_Sk6812_Init(void) {
#if CONFIG_SOFTWARE_MODEL_ATOM_MATRIX
    px.pixel_count = 25;
#else
    px.pixel_count = 1;
#endif
    px.brightness = 10;
    sprintf(px.color_order, "GRBW");
    px.nbits = 24;
    px.timings.t0h = (350);
    px.timings.t0l = (800);
    px.timings.t1h = (600);
    px.timings.t1l = (700);
    px.timings.reset = 80000;
    px.pixels = (uint8_t *)malloc((px.nbits / 8) * px.pixel_count);
    neopixel_init(GPIO_NUM_27, RMT_CHANNEL_0);
    np_clear(&px);
}

void M5Atom_Sk6812_SetColor(uint16_t pos, uint32_t color) {
    np_set_pixel_color(&px, pos, color << 8);
}

void M5Atom_Sk6812_SetAllColor(uint32_t color) {
    for (uint8_t i = 0; i < px.pixel_count; i++) {
        np_set_pixel_color(&px, i, color << 8);
    }
}

void M5Atom_Sk6812_SetBrightness(uint8_t brightness) {
    px.brightness = brightness;
}

void M5Atom_Sk6812_Show(void) {
    np_show(&px, RMT_CHANNEL_0);
}

void M5Atom_Sk6812_Clear(void) {
    np_clear(&px);
}
#endif
/* ----------------------------------------------- End -----------------------------------------------*/
/* ===================================================================================================*/

/* ===================================================================================================*/
/* ----------------------------------------- Expansion Ports -----------------------------------------*/
#if CONFIG_SOFTWARE_EXPPORTS_SUPPORT

static esp_err_t check_pins(gpio_num_t pin, pin_mode_t mode){
    esp_err_t err = ESP_ERR_NOT_SUPPORTED;
    if (pin != PORT_A_SDA_PIN && pin != PORT_A_SCL_PIN)
    {
        ESP_LOGE(TAG, "Only Port A (GPIO 26 and 32) are supported. Pin selected: %d", pin);
    }
    else if (mode == I2C && (pin != PORT_A_SDA_PIN && pin != PORT_A_SCL_PIN))
    {
        ESP_LOGE(TAG, "I2C is only supported on GPIO 26 (SDA) and GPIO 32 (SCL).");
    }
    else if (mode == ADC && pin != PORT_ADC_PIN)
    {
        ESP_LOGE(TAG, "ADC is only supported on GPIO 32.");
    } 
    else 
    {
        err = ESP_OK;
    }
    return err;
}

esp_err_t M5Atom_Port_PinMode(gpio_num_t pin, pin_mode_t mode){
    esp_err_t err = check_pins(pin, mode);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Invalid mode selected for GPIO %d. Error code: 0x%x.", pin, err);
        return err;
    }

    if (mode == OUTPUT || mode == INPUT){
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
        io_conf.pin_bit_mask = (1ULL << pin);

        if (mode == OUTPUT){
            io_conf.mode = GPIO_MODE_OUTPUT; 
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            err = gpio_config(&io_conf);
            if (err != ESP_OK){
                ESP_LOGE(TAG, "Error configuring GPIO %d. Error code: 0x%x.", pin, err);
            }
        } else{
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            err = gpio_config(&io_conf);
            if (err != ESP_OK){
                ESP_LOGE(TAG, "Error configuring GPIO %d. Error code: 0x%x.", pin, err);
            }
        }  
    }
    else if (mode == ADC)
    {
        err = adc1_config_width(ADC_WIDTH);
        if(err != ESP_OK){
            ESP_LOGE(TAG, "Error configuring ADC width on pin %d. Error code: 0x%x.", pin, err);
            return err;
        }
        
        err = adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTENUATION);
        if(err != ESP_OK){
            ESP_LOGE(TAG, "Error configuring ADC channel attenuation on pin %d. Error code: 0x%x.", pin, err);
        }
        
        adc_characterization = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTENUATION, ADC_WIDTH, DEFAULT_VREF, adc_characterization);
    }
/*    else if (mode == DAC){
        dac_output_enable(DAC_CHANNEL);
    }
    else if (mode == UART){
        err = uart_driver_install(PORT_C_UART_NUM, UART_RX_BUF_SIZE, 0, 0, NULL, 0);
        if(err != ESP_OK){
            ESP_LOGE(TAG, "UART driver installation failed for UART num %d. Error code: 0x%x.", PORT_C_UART_NUM, err);
            return err;
        }

        err = uart_set_pin(PORT_C_UART_NUM, PORT_C_UART_TX_PIN, PORT_C_UART_RX_PIN, 
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if(err != ESP_OK){
            ESP_LOGE(TAG, "Failed to set pins %d, %d, to  UART%d. Error code: 0x%x.", PORT_C_UART_RX_PIN, PORT_C_UART_TX_PIN, PORT_C_UART_NUM, err);
        }
    }
    else if (mode == NONE){
        dac_output_disable(DAC_CHANNEL);
        gpio_reset_pin(pin);
        uart_driver_delete(UART_NUM_2);
        i2c_free_port(I2C_NUM_0);
    }
*/
    return err;
}

bool M5Atom_Port_Read(gpio_num_t pin){
    ESP_ERROR_CHECK_WITHOUT_ABORT(check_pins(pin, INPUT));

    return gpio_get_level(pin);
}

esp_err_t M5Atom_Port_Write(gpio_num_t pin, bool level){
    esp_err_t err = check_pins(pin, OUTPUT);
    
    err = gpio_set_level(pin, level);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Error setting GPIO %d state. Error code: 0x%x.", pin, err);
    }
    return err;
}

I2CDevice_t M5Atom_Port_A_I2C_Begin(uint8_t device_address, uint32_t baud){
    return i2c_malloc_device(I2C_NUM_0, PORT_A_SDA_PIN, PORT_A_SCL_PIN, baud < 100000 ? baud : PORT_A_I2C_STANDARD_BAUD, device_address);
}

esp_err_t M5Atom_Port_A_I2C_Read(I2CDevice_t device, uint32_t register_address, uint8_t *data, uint16_t length){
    return i2c_read_bytes(device, register_address, data, length);
}

esp_err_t M5Atom_Port_A_I2C_Write(I2CDevice_t device, uint32_t register_address, uint8_t *data, uint16_t length){
    return i2c_write_bytes(device, register_address, data, length);
}

void M5Atom_Port_A_I2C_Close(I2CDevice_t device){
    i2c_free_device(device);
}

uint32_t M5Atom_Port_ADC_ReadRaw(void){
    return adc1_get_raw(ADC_CHANNEL);
}

uint32_t M5Atom_Port_ADC_ReadMilliVolts(void){
    uint32_t voltage;
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_adc_cal_get_voltage(ADC_CHANNEL, adc_characterization, &voltage));
    return voltage;
}

#endif
/* ----------------------------------------------- End -----------------------------------------------*/
/* ===================================================================================================*/