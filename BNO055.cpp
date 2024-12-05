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

uint8_t BNO055::temperature()
{
    return read_reg(BNO_TEMP);
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