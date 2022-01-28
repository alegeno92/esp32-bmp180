//
// Created by Alessandro Genovese on 28/01/22.
//

#ifndef I2C_I2C_H
#define I2C_I2C_H

#include "esp_err.h"

#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

typedef enum{
    I2C_DEVICE_0 = 0,
    I2C_DEVICE_1,
    I2C_DEVICE_TOTAL,
} I2CDeviceId_t;

esp_err_t i2c_init(I2CDeviceId_t deviceId);
esp_err_t i2c_deinit(I2CDeviceId_t deviceId);
esp_err_t i2c_read(I2CDeviceId_t deviceId, uint8_t chip, uint8_t addr, uint8_t len, uint8_t *rdBuff);
esp_err_t i2c_write(I2CDeviceId_t deviceId, uint8_t chip, uint8_t addr, uint8_t len, uint8_t *wrBuff);

#endif //I2C_I2C_H
