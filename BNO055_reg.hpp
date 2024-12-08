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
constexpr bno_reg_t BOOTL_VER = {0x06, 1, 0};

constexpr bno_reg_t ACC_DATA = {0x08, 6, 0};
constexpr bno_reg_t ACC_DATA_X = {0x08, 2, 0};
constexpr bno_reg_t ACC_DATA_Y = {0x0A, 2, 0};
constexpr bno_reg_t ACC_DATA_Z = {0x0C, 2, 0};

constexpr bno_reg_t MAG_DATA = {0x0E, 6, 0};
constexpr bno_reg_t MAG_DATA_X = {0x0E, 2, 0};
constexpr bno_reg_t MAG_DATA_Y = {0x10, 2, 0};
constexpr bno_reg_t MAG_DATA_Z = {0x12, 2, 0};

constexpr bno_reg_t GYR_DATA = {0x14, 6, 0};
constexpr bno_reg_t GYR_DATA_X = {0x14, 2, 0};
constexpr bno_reg_t GYR_DATA_Y = {0x16, 2, 0};
constexpr bno_reg_t GYR_DATA_Z = {0x18, 2, 0};

constexpr bno_reg_t EULER_DATA = {0x1A, 6, 0};
constexpr bno_reg_t EUL_HEADING = {0x1A, 2, 0};
constexpr bno_reg_t EUL_ROLL = {0x1C, 2, 0};
constexpr bno_reg_t EUL_PITCH = {0x1E, 2, 0};

constexpr bno_reg_t QUA_DATA = {0x20, 8, 0};
constexpr bno_reg_t QUA_DATA_W = {0x20, 2, 0};
constexpr bno_reg_t QUA_DATA_X = {0x22, 2, 0};
constexpr bno_reg_t QUA_DATA_Y = {0x24, 2, 0};
constexpr bno_reg_t QUA_DATA_Z = {0x26, 2, 0};

constexpr bno_reg_t LIA_DATA = {0x28, 6, 0};
constexpr bno_reg_t LIA_DATA_X = {0x28, 2, 0};
constexpr bno_reg_t LIA_DATA_Y = {0x2A, 2, 0};
constexpr bno_reg_t LIA_DATA_Z = {0x2C, 2, 0};

constexpr bno_reg_t GRV_DATA = {0x2E, 6, 0};
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

constexpr bno_reg_t ACC_OFFSET = {0x55, 6, 0};
constexpr bno_reg_t ACC_OFFSET_X = {0x55, 2, 0};
constexpr bno_reg_t ACC_OFFSET_Y = {0x57, 2, 0};
constexpr bno_reg_t ACC_OFFSET_Z = {0x59, 2, 0};

constexpr bno_reg_t MAG_OFFSET = {0x5B, 6, 0};
constexpr bno_reg_t MAG_OFFSET_X = {0x5B, 2, 0};
constexpr bno_reg_t MAG_OFFSET_Y = {0x5D, 2, 0};
constexpr bno_reg_t MAG_OFFSET_Z = {0x5F, 2, 0};

constexpr bno_reg_t GYR_OFFSET = {0x61, 6, 0};
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
constexpr bno_reg_t BNO_UNIQUE_ID = {0x50, 2, 1};

enum BNO_OPERATION_MODE
{
    CONFIGMODE,
    ACC_ONLY,
    MAG_ONLY,
    GYRO_ONLY,
    ACC_MAG,
    ACC_GYRO,
    MAG_GYRO,
    AMG,
    IMU,
    COMPASS,
    M4G,
    NDOF_FMC_OFF,
    NDOF,
};

enum BNO_POWER_MODE
{
    NORMAL,
    LOW_POWER,
    SUSPEND,
};

enum BNO_ORI_FORMAT
{
    WINDOWS,
    ANDROID,
};

enum BNO_TEMP_UNIT
{
    CELSIUS,
    FAHRENHEIT,
};

enum BNO_ACC_UNIT
{
    METERS_PER_SECOND2,
    MILLI_G,
};

enum BNO_ANG_RATE_UNIT
{
    DEG_PER_SECOND,
    RAD_PER_SECOND,
};

enum BNO_ANG_UNIT
{
    DEG,
    RAD,
};

enum BNO_TEMP_SOURCE
{
    TEMP_ACCELEROMETER,
    TEMP_GYROSCOPE,
};

enum BNO_CLK_SOURCE
{
    INTERNAL,
    EXTERNAL,
};

enum BNO_AXIS
{
    BNO_X_AXIS,
    BNO_Y_AXIS,
    BNO_Z_AXIS,
};

enum BNO_INT_MASK
{
    ACC_BSX_DRDY = (1 << 0),
    MAG_DRDY = (1 << 1),
    GYRO_AM = (1 << 2),
    GYR_HIGH_RATE = (1 << 3),
    GYR_DRDY = (1 << 4),
    ACC_HIGH_G = (1 << 5),
    ACC_AM = (1 << 6),
    ACC_NM = (1 << 7),
};

enum BNO_CALIB_MASK
{
    BNO_MAG_CALIBRATED = (3 << 0),
    BNO_ACC_CALIBRATED = (3 << 2),
    BNO_GYR_CALIBRATED = (3 << 4),
    BNO_SYS_CALIBRATED = (3 << 6),

};

struct BNO_UNIT_CONFIG
{
    BNO_ORI_FORMAT ori = WINDOWS;
    BNO_TEMP_UNIT temp = CELSIUS;
    BNO_ACC_UNIT acc = METERS_PER_SECOND2;
    BNO_ANG_UNIT angle = DEG;
    BNO_ANG_RATE_UNIT angle_rate = DEG_PER_SECOND;
};

struct bno_vec_3_t
{
    float x, y, z;
};

struct bno_vec_4_t
{
    float w, x, y, z;
};

struct bno_vec_3i16_t
{
    int16_t x, y, z;
};

struct bno_sensor_offsets_t
{
    bno_vec_3i16_t accelerometer;
    bno_vec_3i16_t magnetometer;
    bno_vec_3i16_t gyroscope;
};