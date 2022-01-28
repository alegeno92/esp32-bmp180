//
// Created by Alessandro Genovese on 23/01/22.
//

#ifndef BMP180_BMP180_H
#define BMP180_BMP180_H
#include "esp_types.h"
#include "i2c/i2c.h"

esp_err_t BMP180_Initialize(I2CDeviceId_t deviceId);

esp_err_t BMP180_CheckStatus(void);

float BMP180_ReadTemperature(void);

esp_err_t BMP180_Deinitialize(void);


#endif //BMP180_BMP180_H
