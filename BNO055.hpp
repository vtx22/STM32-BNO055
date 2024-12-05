#pragma once

#include "BNO055_reg.hpp"

#include <cstdint>
#include "I2C.h"

// Use the following flags for compiling the right library, e.g.: -D STM32F1
#if defined(STM32F0)
#include "stm32f0xx_hal.h"
#elif defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F2)
#include "stm32f2xx_hal.h"
#elif defined(STM32F3)
#include "stm32f3xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#elif defined(STM32F7)
#include "stm32f7xx_hal.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#else
#error "Unsupported STM32 microcontroller. Make sure you build with -D STM32F1 for example!"
#endif

class BNO055
{
public:
    BNO055(I2C_HandleTypeDef *hi2c, uint8_t address);

    uint16_t device_id();
    uint8_t chip_id();

    int8_t temperature();

    void set_operation_mode(BNO_OPERATION_MODE mode);
    void set_power_mode(BNO_POWER_MODE mode);
    uint8_t get_system_status();

    void set_acceleration_unit(BNO_ACC_UNIT unit);
    void set_angular_rate_unit(BNO_ANG_RATE_UNIT unit);
    void set_angle_unit(BNO_ANG_UNIT unit);
    void set_temperature_unit(BNO_TEMP_UNIT unit);
    void set_temperature_source(BNO_TEMP_SOURCE source);

private:
    void set_page_id(bool page);
    void write_page_id(bool page);

    bool get_page_id();
    bool read_page_id();

    uint16_t read_reg(const bno_reg_t &reg);
    void write_reg(const bno_reg_t &reg, uint16_t value);
    void set_reg_bit(const bno_reg_t &reg, uint8_t n, bool value);

    I2C_HandleTypeDef *_hi2c = nullptr;
    uint8_t _address;
    bool _page = false;
};