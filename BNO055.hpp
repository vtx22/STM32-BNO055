#pragma once

#include <cstdint>

class BNO055
{
public:
    BNO055(I2C_HandleTypeDef *hi2c);

private:
    I2C_HandleTypeDef *_hi2c = nullptr;
    uint8_t _address;
};