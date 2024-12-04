#pragma once

// PAGE 0
#define CHIP_ID 0x00
#define ACC_ID 0x01
#define MAG_ID 0x02
#define GYR_ID 0x03
#define SW_REV_ID 0x04
#define PAGE_ID 0x07

#define ACC_DATA_X 0x08
#define ACC_DATA_Y 0x0A
#define ACC_DATA_Z 0x0C

#define MAG_DATA_X 0x0E
#define MAG_DATA_Y 0x10
#define MAG_DATA_Z 0x12

#define GYR_DATA_X 0x14
#define GYR_DATA_Y 0x16
#define GYR_DATA_Z 0x18

#define EUL_HEADING 0x1A
#define EUL_ROLL 0x1C
#define EUL_PITCH 0x1E

#define QUA_DATA_W 0x20
#define QUA_DATA_X 0x22
#define QUA_DATA_Y 0x24
#define QUA_DATA_Z 0x26

#define LIA_DATA_X 0x28
#define LIA_DATA_Y 0x2A
#define LIA_DATA_Z 0x2C

#define GRV_DATA_X 0x2E
#define GRV_DATA_Y 0x30
#define GRV_DATA_Z 0x32

#define BNO_TEMP 0x34
#define CALIB_STAT 0x35
#define ST_RESULT 0x36
#define INT_STAT 0x37
#define SYS_CLK_STAT 0x38
#define SYS_STATUS 0x39
#define SYS_ERR 0x3A

#define UNIT_SEL 0x3B
#define OPR_MODE 0x3D
#define PWR_MODE 0x3E
#define SYS_TRIGGER 0x3F
#define TEMP_SOURCE 0x40
#define AXIS_MAP_CONFIG 0x41
#define AXIS_MAP_SIGN 0x42

#define SIC_MATRIX_0 0x43
#define SIC_MATRIX_1 0x45
#define SIC_MATRIX_2 0x47
#define SIC_MATRIX_3 0x49
#define SIC_MATRIX_4 0x4B
#define SIC_MATRIX_5 0x4D
#define SIC_MATRIX_6 0x4F
#define SIC_MATRIX_7 0x51
#define SIC_MATRIX_8 0x53

#define ACC_OFFSET_X 0x55
#define ACC_OFFSET_Y 0x57
#define ACC_OFFSET_Z 0x59

#define MAG_OFFSET_X 0x5B
#define MAG_OFFSET_Y 0x5D
#define MAG_OFFSET_Z 0x5F

#define GYR_OFFSET_X 0x61
#define GYR_OFFSET_Y 0x63
#define GYR_OFFSET_Z 0x65

#define ACC_RADIUS 0x68
#define MAG_RADIUS 0x6A

// PAGE 1
#define ACC_HG_THRES 0x14
#define ACC_NM_THRES 0x15
#define ACC_NM_SET 0x16
#define GYR_INT_SET 0x17
#define GYR_HR_X_SET 0x18
#define GYR_DUR_X 0x19
#define GYR_HR_Y_SET 0x1A
#define GYR_DUR_Y 0x1B
#define GYR_HR_Z_SET 0x1C
#define GYR_DUR_Z 0x1D
#define GYR_AM_THRES 0x1E
#define GYR_AM_SET 0x1F
#define BNO_UNIQUE_ID 0x50