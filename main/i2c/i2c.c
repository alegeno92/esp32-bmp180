//
// Created by Alessandro Genovese on 23/01/22.
//
#include <string.h>
#include "esp_log.h"
#include "i2c.h"
#include "driver/i2c.h"

#define TAG "i2c"

struct I2CDevice_t
{
    bool inited;
    uint8_t port;
    uint8_t sda;
    uint8_t scl;
    uint32_t frequency;
};

static struct I2CDevice_t I2CDevices[I2C_DEVICE_TOTAL] = {
        {false, I2C_NUM_0, 21, 22, 100000U},
        {false, I2C_NUM_1, 33, 32, 100000U}
};

esp_err_t i2c_init(I2CDeviceId_t deviceId) {
    esp_err_t ret = ESP_FAIL;
    if(deviceId >= I2C_DEVICE_TOTAL)
    {
        ESP_LOGE(TAG, "Cannot initialize device %d", deviceId);
    }
    else if(I2CDevices[deviceId].inited == true)
    {
        ESP_LOGW(TAG, "Device %d already inited", deviceId);

    }
    else
    {
        struct I2CDevice_t* i2cDevice = &I2CDevices[deviceId];
        i2c_config_t config = {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = i2cDevice->sda,
                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_io_num = i2cDevice->scl,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                .master.clk_speed = i2cDevice->frequency,
        };
        ret = i2c_param_config(i2cDevice->port, &config);
        if ( ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C - Error while set config");
            return ret;
        }

        ret = i2c_driver_install(i2cDevice->port, I2C_MODE_MASTER, 0, 0, 0);

        if(ret != ESP_OK)
        {
            ESP_LOGE(TAG, "I2C - Error while install driver");
        }
        else
        {
            i2cDevice->inited = true;
#ifdef DEBUG_ENABLED
            ESP_LOGI(TAG, "I2C Device %d inited", deviceId);
#endif /*DEBUG_ENABLED*/
        }
    }
    return ret;
}

esp_err_t i2c_deinit(I2CDeviceId_t deviceId){
    esp_err_t ret = ESP_FAIL;
    if(deviceId >= I2C_DEVICE_TOTAL)
    {
        ESP_LOGE(TAG, "Cannot initialize device %d", deviceId);
    }
    else if(I2CDevices[deviceId].inited == true)
    {
        ret = i2c_driver_delete(I2CDevices[deviceId].port);
        if(ret == ESP_OK)
        {
#ifdef DEBUG_ENABLED
            ESP_LOGI(TAG, "I2C device %d deinited", deviceId);
#endif /*DEBUG_ENABLED*/
        } else{
            ESP_LOGE(TAG, "Failed to deinit I2C device %d", deviceId);
        }
    }
    return ret;
}

esp_err_t i2c_read(I2CDeviceId_t deviceId, uint8_t chip, uint8_t addr, uint8_t len, uint8_t *rdBuff) {
    esp_err_t ret = ESP_FAIL;
    if(deviceId >= I2C_DEVICE_TOTAL)
    {
        ESP_LOGE(TAG, "Error while selecting device");
    }
    else if (I2CDevices[deviceId].inited != true) {
        ESP_LOGE(TAG, "I2C driver not initialized");
    }
    else
    {
        struct I2CDevice_t* i2cDevice = &I2CDevices[deviceId];
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (chip << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, addr, ACK_CHECK_EN);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, chip << 1 | I2C_MASTER_READ, ACK_CHECK_EN);

        if (len > 1) {
            i2c_master_read(cmd, rdBuff, len - 1, ACK_VAL);
        }

        i2c_master_read(cmd, &rdBuff[len - 1], 1, NACK_VAL);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2cDevice->port, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
#ifdef DEBUG_ENABLED
            ESP_LOGI(TAG, "Read ok!");
            for (int i = 0; i < len; i++) {
                ESP_LOGD(TAG, "0x%02x ", rdBuff[i]);
            }
#endif /*DEBUG_ENABLED*/
        } else if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Bus is busy");
        } else {
            ESP_LOGW(TAG, "Read failed");
        }
    }
    return ret;
}

esp_err_t i2c_write(I2CDeviceId_t deviceId, uint8_t chip, uint8_t addr, uint8_t len, uint8_t *wrBuff) {
    esp_err_t ret = ESP_FAIL;
    if(deviceId >= I2C_DEVICE_TOTAL)
    {
        ESP_LOGE(TAG, "Error while selecting device");
    }
    else if (I2CDevices[deviceId].inited != true) {
        ESP_LOGE(TAG, "I2C driver not initialized");
    }
    else
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (chip << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, addr, ACK_CHECK_EN);
        if(len > 0)
        {
            i2c_master_write(cmd, wrBuff, len, ACK_CHECK_EN);
        }

        i2c_master_stop(cmd);

        ret = i2c_master_cmd_begin(I2CDevices[deviceId].port, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
#ifdef DEBUG_ENABLED
            ESP_LOGI(TAG, "Write ok!");
#endif /*DEBUG_ENABLED*/
        } else if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Bus is busy");
        } else {
            ESP_LOGW(TAG, "Write Failed");
        }
    }

    return ret;
}
