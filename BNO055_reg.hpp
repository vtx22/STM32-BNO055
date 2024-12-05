#pragma once

#include <cstdint>

struct bno_reg_t
{
    uint8_t address;
    uint8_t byte_length;
    bool page = 0;
};

constexpr bno_reg_t PAGE_ID = {0x07, 1, 0};

// PAGE 0 Registers
constexpr bno_reg_t CHIP_ID = {0x00, 1, 0};
constexpr bno_reg_t ACC_ID = {0x01, 1, 0};
constexpr bno_reg_t MAG_ID = {0x02, 1, 0};
constexpr bno_reg_t GYR_ID = {0x03, 1, 0};
constexpr bno_reg_t SW_REV_ID = {0x04, 2, 0};

constexpr bno_reg_t ACC_DATA_X = {0x08, 2, 0};
constexpr bno_reg_t ACC_DATA_Y = {0x0A, 2, 0};
constexpr bno_reg_t ACC_DATA_Z = {0x0C, 2, 0};

constexpr bno_reg_t MAG_DATA_X = {0x0E, 2, 0};
constexpr bno_reg_t MAG_DATA_Y = {0x10, 2, 0};
constexpr bno_reg_t MAG_DATA_Z = {0x12, 2, 0};

constexpr bno_reg_t GYR_DATA_X = {0x14, 2, 0};
constexpr bno_reg_t GYR_DATA_Y = {0x16, 2, 0};
constexpr bno_reg_t GYR_DATA_Z = {0x18, 2, 0};

constexpr bno_reg_t EUL_HEADING = {0x1A, 2, 0};
constexpr bno_reg_t EUL_ROLL = {0x1C, 2, 0};
constexpr bno_reg_t EUL_PITCH = {0x1E, 2, 0};

constexpr bno_reg_t QUA_DATA_W = {0x20, 2, 0};
constexpr bno_reg_t QUA_DATA_X = {0x22, 2, 0};
constexpr bno_reg_t QUA_DATA_Y = {0x24, 2, 0};
constexpr bno_reg_t QUA_DATA_Z = {0x26, 2, 0};

constexpr bno_reg_t LIA_DATA_X = {0x28, 2, 0};
constexpr bno_reg_t LIA_DATA_Y = {0x2A, 2, 0};
constexpr bno_reg_t LIA_DATA_Z = {0x2C, 2, 0};

constexpr bno_reg_t GRV_DATA_X = {0x2E, 2, 0};
constexpr bno_reg_t GRV_DATA_Y = {0x30, 2, 0};
constexpr bno_reg_t GRV_DATA_Z = {0x32, 2, 0};

constexpr bno_reg_t BNO_TEMP = {0x34, 1, 0};
constexpr bno_reg_t CALIB_STAT = {0x35, 1, 0};
constexpr bno_reg_t ST_RESULT = {0x36, 1, 0};
constexpr bno_reg_t INT_STAT = {0x37, 1, 0};
constexpr bno_reg_t SYS_CLK_STAT = {0x38, 1, 0};
constexpr bno_reg_t SYS_STATUS = {0x39, 1, 0};
constexpr bno_reg_t SYS_ERR = {0x3A, 1, 0};

constexpr bno_reg_t UNIT_SEL = {0x3B, 1, 0};
constexpr bno_reg_t OPR_MODE = {0x3D, 1, 0};
constexpr bno_reg_t PWR_MODE = {0x3E, 1, 0};
constexpr bno_reg_t SYS_TRIGGER = {0x3F, 1, 0};
constexpr bno_reg_t TEMP_SOURCE = {0x40, 1, 0};
constexpr bno_reg_t AXIS_MAP_CONFIG = {0x41, 1, 0};
constexpr bno_reg_t AXIS_MAP_SIGN = {0x42, 1, 0};

constexpr bno_reg_t SIC_MATRIX[] = {
    {0x43, 2, 0}, {0x45, 2, 0}, {0x47, 2, 0}, {0x49, 2, 0}, {0x4B, 2, 0}, {0x4D, 2, 0}, {0x4F, 2, 0}, {0x51, 2, 0}, {0x53, 2, 0}};

constexpr bno_reg_t ACC_OFFSET_X = {0x55, 2, 0};
constexpr bno_reg_t ACC_OFFSET_Y = {0x57, 2, 0};
constexpr bno_reg_t ACC_OFFSET_Z = {0x59, 2, 0};

constexpr bno_reg_t MAG_OFFSET_X = {0x5B, 2, 0};
constexpr bno_reg_t MAG_OFFSET_Y = {0x5D, 2, 0};
constexpr bno_reg_t MAG_OFFSET_Z = {0x5F, 2, 0};

constexpr bno_reg_t GYR_OFFSET_X = {0x61, 2, 0};
constexpr bno_reg_t GYR_OFFSET_Y = {0x63, 2, 0};
constexpr bno_reg_t GYR_OFFSET_Z = {0x65, 2, 0};

constexpr bno_reg_t ACC_RADIUS = {0x68, 2, 0};
constexpr bno_reg_t MAG_RADIUS = {0x6A, 2, 0};

// PAGE 1 Registers
constexpr bno_reg_t ACC_CONFIG = {0x08, 1, 1};
constexpr bno_reg_t MAG_CONFIG = {0x09, 1, 1};
constexpr bno_reg_t GYR_CONFIG_0 = {0x0A, 1, 1};
constexpr bno_reg_t GYR_CONFIG_1 = {0x0B, 1, 1};
constexpr bno_reg_t ACC_SLEEP_CONFIG = {0x0C, 1, 1};
constexpr bno_reg_t GYR_SLEEP_CONFIG = {0x0D, 1, 1};
constexpr bno_reg_t INT_MSK = {0x0F, 1, 1};
constexpr bno_reg_t INT_EN = {0x10, 1, 1};
constexpr bno_reg_t ACC_AM_THRES = {0x11, 1, 1};
constexpr bno_reg_t ACC_INT_SETTINGS = {0x12, 1, 1};
constexpr bno_reg_t ACC_HG_DURATION = {0x13, 1, 1};
constexpr bno_reg_t ACC_HG_THRES = {0x14, 1, 1};
constexpr bno_reg_t ACC_NM_THRES = {0x15, 1, 1};
constexpr bno_reg_t ACC_NM_SET = {0x16, 1, 1};
constexpr bno_reg_t GYR_INT_SET = {0x17, 1, 1};
constexpr bno_reg_t GYR_HR_X_SET = {0x18, 1, 1};
constexpr bno_reg_t GYR_DUR_X = {0x19, 1, 1};
constexpr bno_reg_t GYR_HR_Y_SET = {0x1A, 1, 1};
constexpr bno_reg_t GYR_DUR_Y = {0x1B, 1, 1};
constexpr bno_reg_t GYR_HR_Z_SET = {0x1C, 1, 1};
constexpr bno_reg_t GYR_DUR_Z = {0x1D, 1, 1};
constexpr bno_reg_t GYR_AM_THRES = {0x1E, 1, 1};
constexpr bno_reg_t GYR_AM_SET = {0x1F, 1, 1};
constexpr bno_reg_t BNO_UNIQUE_ID = {0x50, 1, 1};
