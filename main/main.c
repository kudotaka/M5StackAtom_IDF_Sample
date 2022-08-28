#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "m5atom.h"

//#define EX_CONFIG_SOFTWARE_EXTERNAL_LED
//#define EX_CONFIG_SOFTWARE_EXTERNAL_ADC
//#define EX_CONFIG_SOFTWARE_EXTERNAL_DIGITALREAD

static const char *TAG = "MAIN";

#ifdef CONFIG_SOFTWARE_SK6812_SUPPORT
TaskHandle_t xRGBLedBlink;
void vLoopRGBLedBlinkTask(void *pvParametes)
{
    ESP_LOGI(TAG, "start vLoopRGBLedBlinkTask");

    uint8_t blink_count = 5;
    uint32_t colors[] = {SK6812_COLOR_BLUE, SK6812_COLOR_LIME, SK6812_COLOR_AQUA
                    , SK6812_COLOR_RED, SK6812_COLOR_MAGENTA, SK6812_COLOR_YELLOW
                    , SK6812_COLOR_WHITE};

    while (1) {
        for (uint8_t c = 0; c < sizeof(colors)/sizeof(uint32_t); c++) {
            for (uint8_t i = 0; i < blink_count; i++) {
                M5Atom_Sk6812_SetAllColor(colors[c]);
                M5Atom_Sk6812_Show();
                vTaskDelay(pdMS_TO_TICKS(1000));

                M5Atom_Sk6812_SetAllColor(SK6812_COLOR_OFF);
                M5Atom_Sk6812_Show();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }
}
#endif

#ifdef CONFIG_SOFTWARE_BUTTON_SUPPORT
TaskHandle_t xButton;
static void button_task(void* pvParameters) {
    ESP_LOGI(TAG, "start button_task");

    while(1){
        if (Button_WasPressed(button_a)) {
            ESP_LOGI(TAG, "BUTTON A PRESSED!");
        }
        if (Button_WasReleased(button_a)) {
            ESP_LOGI(TAG, "BUTTON A RELEASED!");
        }
        if (Button_WasLongPress(button_a, pdMS_TO_TICKS(1000))) { // 1Sec
            ESP_LOGI(TAG, "BUTTON A LONGPRESS!");
        }
    
        vTaskDelay(pdMS_TO_TICKS(80));
    }
    vTaskDelete(NULL); // Should never get to here...
}
#endif

#ifdef CONFIG_SOFTWARE_MPU6886_SUPPORT
TaskHandle_t xMPU6886;
static void mpu6886_task(void* pvParameters) {
    ESP_LOGI(TAG, "start mpu6886_task");

    float ax, ay, az;
    while (1) {
        MPU6886_GetAccelData(&ax, &ay, &az);
        ESP_LOGI(TAG, "MPU6886 Acc x: %.2f, y: %.2f, z: %.2f", ax, ay, az);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    vTaskDelete(NULL); // Should never get to here...
}
#endif

#ifdef EX_CONFIG_SOFTWARE_EXTERNAL_LED
TaskHandle_t xExtLED;
static void external_led_task(void* pvParameters) {
    ESP_LOGI(TAG, "start external_led_task");

    if (M5Atom_Port_PinMode(GPIO_NUM_26, OUTPUT) != ESP_OK) {
        ESP_LOGI(TAG, "M5Atom_Port_PinMode() is error. port:%d", GPIO_NUM_26);
    } else {
        vTaskDelay(pdMS_TO_TICKS(100));
        while (1) {
            if (M5Atom_Port_Write(GPIO_NUM_26, PORT_LEVEL_LOW) != ESP_OK) {
                ESP_LOGE(TAG, "M5Atom_Port_Write(LOW) is error. port:%d", GPIO_NUM_26);
                break;
            }
//            ESP_LOGI(TAG, "M5Atom_Port_Write(LOW) is OFF");
            vTaskDelay(pdMS_TO_TICKS(800));

            if (M5Atom_Port_Write(GPIO_NUM_26, PORT_LEVEL_HIGH) != ESP_OK) {
                ESP_LOGE(TAG, "M5Atom_Port_Write(HIGH) is error. port:%d", GPIO_NUM_26);
                break;
            }
//            ESP_LOGI(TAG, "M5Atom_Port_Write(HIGH) is ON");
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
    vTaskDelete(NULL); // Should never get to here...
}
#endif

#ifdef EX_CONFIG_SOFTWARE_EXTERNAL_ADC
TaskHandle_t xExtADC;
static void external_adc_task(void* pvParameters) {
    ESP_LOGI(TAG, "start external_adc_task");

    if (M5Atom_Port_PinMode(GPIO_NUM_32, ADC) != ESP_OK) {
        ESP_LOGE(TAG, "M5Atom_Port_PinMode() is error. port:%d", GPIO_NUM_32);
    } else {
        vTaskDelay(pdMS_TO_TICKS(100));
        uint32_t adc_row = 0;
        uint32_t adc_millvolts = 0;
        while (1) {
            adc_row = M5Atom_Port_ADC_ReadRaw();
            adc_millvolts = M5Atom_Port_ADC_ReadMilliVolts();
            ESP_LOGI(TAG, "M5Atom_Port_ADC_Read( ) adc_row:%d, adc_millvolts:%d", adc_row, adc_millvolts);
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
    vTaskDelete(NULL); // Should never get to here...
}
#endif

#ifdef EX_CONFIG_SOFTWARE_EXTERNAL_DIGITALREAD
TaskHandle_t xExtDIGITALREAD;
static void external_digitalread_task(void* pvParameters) {
    ESP_LOGI(TAG, "start external_digitalread_task");

    if (M5Atom_Port_PinMode(GPIO_NUM_32, INPUT) != ESP_OK) {
        ESP_LOGE(TAG, "M5Atom_Port_PinMode() is error. port:%d", GPIO_NUM_32);
    } else {
        vTaskDelay(pdMS_TO_TICKS(100));
        while (1) {
            if (M5Atom_Port_Read(GPIO_NUM_32) == PORT_LEVEL_LOW) {
                ESP_LOGI(TAG, "M5Atom_Port_Read( ) is PORT_LEVEL_LOW");
            } else {
                ESP_LOGI(TAG, "M5Atom_Port_Read( ) is PORT_LEVEL_HIGH");
            }
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
    vTaskDelete(NULL); // Should never get to here...
}
#endif


void app_main()
{
    ESP_LOGI(TAG, "app_main() start.");

    M5Atom_Init();

#ifdef CONFIG_SOFTWARE_SK6812_SUPPORT
    // RGB LED BLINK
    xTaskCreatePinnedToCore(&vLoopRGBLedBlinkTask, "rgb_led_blink_task", 4096 * 1, NULL, 2, &xRGBLedBlink, 1);
#endif

#ifdef CONFIG_SOFTWARE_BUTTON_SUPPORT
    // BUTTON
    xTaskCreatePinnedToCore(&button_task, "button_task", 4096 * 1, NULL, 2, &xButton, 1);
#endif

#ifdef CONFIG_SOFTWARE_MPU6886_SUPPORT
    // MPU6886
    xTaskCreatePinnedToCore(&mpu6886_task, "mpu6886_task", 4096 * 1, NULL, 2, &xMPU6886, 1);
#endif

#ifdef EX_CONFIG_SOFTWARE_EXTERNAL_LED
    // External LED
    xTaskCreatePinnedToCore(&external_led_task, "external_led_task", 4096 * 1, NULL, 2, &xExtLED, 1);
#endif

#ifdef EX_CONFIG_SOFTWARE_EXTERNAL_ADC
    // External ADC
    xTaskCreatePinnedToCore(&external_adc_task, "external_adc_task", 4096 * 1, NULL, 2, &xExtADC, 1);
#endif

#ifdef EX_CONFIG_SOFTWARE_EXTERNAL_DIGITALREAD
    // External DIGITAL READ
    xTaskCreatePinnedToCore(&external_digitalread_task, "external_digitalread_task", 4096 * 1, NULL, 2, &xExtDIGITALREAD, 1);
#endif

}
