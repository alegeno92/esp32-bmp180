//
// Created by Alessandro Genovese on 23/01/22.
//
#include <string.h>
#include "bmp180.h"
#include "i2c/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "BMP"

#define BMP180 0x77U
#define BMP180_CMD_CHIP_ID 0xD0U
#define BMP180_CMD_TEMP_WR 0xF4U
#define BMP180_CMD_TEMP_RD 0xF6U

static int16_t AC1 = 0U;
static short AC2 = 0U;
static short AC3 = 0U;
static ushort AC4 = 0U;
static ushort AC5 = 0U;
static ushort AC6 = 0U;
static short B1 = 0U;
static short B2 = 0U;
static short MB = 0U;
static short MC = 0U;
static short MD = 0U;

static I2CDeviceId_t DeviceId = I2C_DEVICE_TOTAL;

static int32_t CalculateB5(uint16_t ut);

esp_err_t BMP180_Initialize(I2CDeviceId_t deviceId) {
    esp_err_t ret = ESP_FAIL;
    if (deviceId >= I2C_DEVICE_TOTAL) {

    } else {
        DeviceId = deviceId;
        ret = i2c_init(DeviceId);
        if (ret != ESP_OK) {
        } else {
            uint8_t rdBuff[22];
            memset(rdBuff, 0x0, 22);
            if (i2c_read(DeviceId, BMP180, 0xAA, 22, rdBuff) != ESP_OK) {
                ESP_LOGE(TAG, "Error while reading constants");
            } else {
                AC1 = (int16_t) ((rdBuff[0] << 8) | rdBuff[1]);
                AC2 = (int16_t) ((rdBuff[2] << 8) | rdBuff[3]);
                AC3 = (int16_t) ((rdBuff[4] << 8) | rdBuff[5]);
                AC4 = (uint16_t) ((rdBuff[6] << 8) | rdBuff[7]);
                AC5 = (uint16_t) ((rdBuff[8] << 8) | rdBuff[9]);
                AC6 = (uint16_t) ((rdBuff[10] << 8) | rdBuff[11]);
                B1 = (int16_t) ((rdBuff[12] << 8) | rdBuff[13]);
                B2 = (int16_t) ((rdBuff[14] << 8) | rdBuff[15]);
                MB = (int16_t) ((rdBuff[16] << 8) | rdBuff[17]);
                MC = (int16_t) ((rdBuff[18] << 8) | rdBuff[19]);
                MD = (int16_t) ((rdBuff[20] << 8) | rdBuff[21]);
#ifdef DEBUG_ENABLED
                ESP_LOGD(TAG, "AC1: %d ", AC1);
                ESP_LOGD(TAG, "AC2: %d ", AC2);
                ESP_LOGD(TAG, "AC3: %d ", AC3);
                ESP_LOGD(TAG, "AC4: %d ", AC4);
                ESP_LOGD(TAG, "AC5: %d ", AC5);
                ESP_LOGD(TAG, "AC6: %d ", AC6);
                ESP_LOGD(TAG, "B1: %d ", B1);
                ESP_LOGD(TAG, "B2: %d ", B2);
                ESP_LOGD(TAG, "MB: %d ", MB);
                ESP_LOGD(TAG, "MC: %d ", MC);
                ESP_LOGD(TAG, "MD: %d ", MD);
#endif /*DEBUG_ENABLED*/
            }
        }
    }
    return ret;
}

esp_err_t BMP180_CheckStatus(void) {
    bool ret = false;
    uint8_t *data = malloc(1);

    if (i2c_read(DeviceId, BMP180, BMP180_CMD_CHIP_ID, 1, data) != ESP_OK) {
        ESP_LOGE(TAG, "Error while reading chip id");
    } else {
        uint8_t chipId = data[0];
        if (chipId == 0x55U) {
#ifdef DEBUG_ENABLED
            ESP_LOGI(TAG, "Chip ready! ChipId 0x%02x", chipId);
#endif /*DEBUG_ENABLED*/
            ret = true;
        }
    }

    free(data);
    return ret;
}

float BMP180_ReadTemperature(void) {
    uint8_t cmd = 0x2E;
    i2c_write(DeviceId, BMP180, BMP180_CMD_TEMP_WR, 1, &cmd);
    vTaskDelay(5 / portTICK_RATE_MS);
    uint8_t rdBuff[2];
    i2c_read(DeviceId, BMP180, BMP180_CMD_TEMP_RD, 2, rdBuff);
    uint16_t ut = rdBuff[1] | (rdBuff[0] << 8);

#ifdef DEBUG_ENABLED
    ESP_LOGI(TAG, "I2C - RawTemp: %d", ut);
#endif /*DEBUG_ENABLED*/
    float temp = (float) ((CalculateB5(ut) + 8) >> 4);
    temp /= 10;
#ifdef DEBUG_ENABLED
    ESP_LOGI(TAG, "I2C - Temp: %f", temp);
#endif /*DEBUG_ENABLED*/
    return temp;
}

esp_err_t BMP180_Deinitialize(void)
{
    esp_err_t ret = i2c_deinit(DeviceId);
    return ret;
}

static int32_t CalculateB5(uint16_t ut) {
    int32_t x1 = (ut - (int32_t) AC6) * ((int32_t) AC5) >> 15;
    if (x1 == 0 && MD == 0) {
        return 0;
    }

    /* executed only the divisor is not zero*/
    int32_t x2 = ((int32_t) MC << 11) / (x1 + (int32_t) MD);
    int32_t b5 = x1 + x2;

    return b5;
}