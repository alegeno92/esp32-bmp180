#include "esp_log.h"
#include "bmp180.h"

#define TAG "BMP"

void app_main(void)
{
    ESP_LOGI(TAG, "MAIN");
    BMP180_Initialize(I2C_DEVICE_0);
    float temp = BMP180_ReadTemperature();
    ESP_LOGI(TAG, "Temperature: %f", temp);
    BMP180_Deinitialize();
}
