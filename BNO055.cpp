#include "BNO055.hpp"

BNO055::BNO055(I2C_HandleTypeDef *hi2c, uint8_t address) : _hi2c(hi2c), _address(address)
{
}

void BNO055::set_page_id(bool page)
{
    if (_page == page)
    {
        return;
    }

    write_page_id(page);
}

void BNO055::write_page_id(bool page)
{
    write_reg(PAGE_ID, page ? 1 : 0);
    _page = page;
}

bool BNO055::get_page_id()
{
    return _page;
}

bool BNO055::read_page_id()
{
    return (bool)read_reg(PAGE_ID);
}

uint16_t BNO055::unique_id()
{
    return read_reg(BNO_UNIQUE_ID);
}

uint8_t BNO055::bno_chip_id()
{
    return read_reg(CHIP_ID);
}

uint8_t BNO055::acc_chip_id()
{
    return read_reg(ACC_ID);
}

uint8_t BNO055::mag_chip_id()
{
    return read_reg(MAG_ID);
}

uint8_t BNO055::gyro_chip_id()
{
    return read_reg(GYR_ID);
}

/*
Get the current device software revision as a fixed point number
where the second byte represents the integer after the decimal point.

@return The software revision in fixed point format
*/
uint16_t BNO055::software_revision()
{
    return read_reg(SW_REV_ID);
}

/*
Get the BNO bootloader version

@return Bootloader version as integer
*/
uint8_t BNO055::bootloader_version()
{
    return read_reg(BOOTL_VER);
}

/*
Get the measured temperature from TEMP_SOURCE in Degrees Celsius

Note: The returned value is only correct if the TEMP_UNIT is
configured to be CELSIUS. See set_temperature_unit
@return Temperature in Degrees Celsius
*/
int8_t BNO055::temperature_c()
{
    return (int8_t)read_reg(BNO_TEMP);
}

/*
Get the measured temperature from TEMP_SOURCE in Degree Fahrenheit

Note: The returned value is only correct if the TEMP_UNIT is
configured to be FAHRENHEIT. See set_temperature_unit
@return Temperature in Degrees Fahrenheit
*/
int16_t BNO055::temperature_f()
{
    return ((int8_t)read_reg(BNO_TEMP)) * 2;
}

/*
Set the BNO operation mode
@param mode Operation mode, see BNO_OPERATION_MODE
*/
void BNO055::set_operation_mode(BNO_OPERATION_MODE mode)
{
    return write_reg(OPR_MODE, mode);
}

/*
Set the BNO power mode
@param mode Power mode, see BNO_POWER_MODE
*/
void BNO055::set_power_mode(BNO_POWER_MODE mode)
{
    return write_reg(PWR_MODE, mode);
}

/*
Get the BNO system status byte
@return Current status byte
*/
uint8_t BNO055::get_system_status()
{
    return read_reg(SYS_STATUS);
}

/*
Specify the output orientation format

Note: Changes only take effect when the sensor is in CONFIGMODE
@param mode Orientation output format, see BNO_ORI_FORMAT
*/
void BNO055::set_orientation_format(BNO_ORI_FORMAT format)
{
    return set_reg_bit(UNIT_SEL, 7, format);
}

/*
Specify the acceleration output unit

Note: Changes only take effect when the sensor is in CONFIGMODE
@param unit Acceleration unit, see BNO_ACC_UNIT
*/
void BNO055::set_acceleration_unit(BNO_ACC_UNIT unit)
{
    return set_reg_bit(UNIT_SEL, 0, unit);
}

/*
Specify the angular rate output unit

Note: Changes only take effect when the sensor is in CONFIGMODE
@param unit Angular rate unit, see BNO_ANG_RATE_UNIT
*/
void BNO055::set_angular_rate_unit(BNO_ANG_RATE_UNIT unit)
{
    return set_reg_bit(UNIT_SEL, 1, unit);
}

/*
Specify the angle output unit

Note: Changes only take effect when the sensor is in CONFIGMODE
@param unit Angle unit, see BNO_ANG_UNIT
*/
void BNO055::set_angle_unit(BNO_ANG_UNIT unit)
{
    return set_reg_bit(UNIT_SEL, 2, unit);
}

/*
Specify the temperature output unit

Note: Changes only take effect when the sensor is in CONFIGMODE
@param unit Temperature unit, see BNO_TEMP_UNIT
*/
void BNO055::set_temperature_unit(BNO_TEMP_UNIT unit)
{
    return set_reg_bit(UNIT_SEL, 4, unit);
}

/*
Specify from which sensor temperature readings are taken

Note: Changes only take effect when the sensor is in CONFIGMODE
@param source Temperature source, see BNO_TEMP_SOURCE
*/
void BNO055::set_temperature_source(BNO_TEMP_SOURCE source)
{
    return set_reg_bit(TEMP_SOURCE, 0, source);
}

uint16_t BNO055::read_reg(const bno_reg_t &reg)
{
    set_page_id(reg.page);

    if (reg.byte_length == 1)
    {
        return read_i2c_reg_8(_hi2c, _address, reg.address);
    }

    return read_i2c_reg_16(_hi2c, _address, reg.address);
}

void BNO055::write_reg(const bno_reg_t &reg, uint16_t value)
{
    set_page_id(reg.page);

    if (reg.byte_length == 1)
    {
        return write_i2c_reg_8(_hi2c, _address, reg.address, (uint8_t)value);
    }

    return write_i2c_reg_16(_hi2c, _address, reg.address, value);
}

void BNO055::set_reg_bit(const bno_reg_t &reg, uint8_t n, bool value)
{
    uint16_t current = read_reg(reg);
    current = (current & ~(1 << n)) | ((uint8_t)value << n);
    return write_reg(reg, current);
}

bool BNO055::get_reg_bit(const bno_reg_t &reg, uint8_t n)
{
    uint16_t current = read_reg(reg);
    return (current >> n) & 0x01;
}