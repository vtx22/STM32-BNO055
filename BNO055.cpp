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

uint16_t BNO055::device_id()
{
    return read_reg(BNO_UNIQUE_ID);
}

uint8_t BNO055::chip_id()
{
    return read_reg(CHIP_ID);
}

int8_t BNO055::temperature()
{
    return (int8_t)read_reg(BNO_TEMP);
}

void BNO055::set_operation_mode(BNO_OPERATION_MODE mode)
{
    return write_reg(OPR_MODE, mode);
}

void BNO055::set_power_mode(BNO_POWER_MODE mode)
{
    return write_reg(PWR_MODE, mode);
}

uint8_t BNO055::get_system_status()
{
    return read_reg(SYS_STATUS);
}

void BNO055::set_acceleration_unit(BNO_ACC_UNIT unit)
{
    return set_reg_bit(UNIT_SEL, 0, unit);
}

void BNO055::set_angular_rate_unit(BNO_ANG_RATE_UNIT unit)
{
    return set_reg_bit(UNIT_SEL, 1, unit);
}

void BNO055::set_angle_unit(BNO_ANG_UNIT unit)
{
    return set_reg_bit(UNIT_SEL, 2, unit);
}

void BNO055::set_temperature_unit(BNO_TEMP_UNIT unit)
{
    return set_reg_bit(UNIT_SEL, 4, unit);
}

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