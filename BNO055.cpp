#include "BNO055.hpp"

BNO055::BNO055(I2C_HandleTypeDef *hi2c, uint8_t address) : _hi2c(hi2c), _address(address)
{
}

/*
Specify a GPIO pin connected to the BNO reset pin. This pin will be used for hardware resetting the sensor.
@param port GPIO port
@param pin GPIO pin number
@param invert If true, the GPIO pin is pulled high for a reset and then stays low.
*/
void BNO055::set_reset_pin(GPIO_TypeDef *port, uint16_t pin, bool invert)
{
    _rst_port = port;
    _rst_pin = pin;
    _rst_invert = invert;
}

/*
Specify a GPIO pin connected to the BNO reset pin. This pin will be used for hardware resetting the sensor.
@param port GPIO port
@param pin GPIO pin number
*/
void BNO055::set_reset_pin(GPIO_TypeDef *port, uint16_t pin)
{
    return set_reset_pin(port, pin, false);
}

/*
Performs a hardware reset or a software reset if no reset pin was specified
*/
void BNO055::reset()
{
    if (_rst_port != nullptr)
    {
        return hardware_reset();
    }

    return software_reset();
}

/*
Triggers a hardware reset by pulling the reset pin low for 100ms.
After that the method blocks for 900ms to ensure the sensor has booted before sending commands.
If the reset pin was not specified, the function returns immediately.
If invert was set to true, the pin is pulled high and then stays low.
*/
void BNO055::hardware_reset()
{
    if (_rst_port == nullptr)
    {
        return;
    }

    HAL_GPIO_WritePin(_rst_port, _rst_pin, _rst_invert ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(_rst_port, _rst_pin, _rst_invert ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_Delay(900);
}

/*
Trigger a software reset by setting the RST_SYS bit in the SYS_TRIGGER register to 1
*/
void BNO055::software_reset()
{
    set_reg_bit(SYS_TRIGGER, 5, true);
    HAL_Delay(900);
}

/*
Trigger a sensor self test by setting the Self_Test bit in the SYS_TRIGGER register to 1
*/
void BNO055::self_test()
{
    return set_reg_bit(SYS_TRIGGER, 0, true);
}

/*
Reset all system interrupt bits
*/
void BNO055::reset_interrupts()
{
    return set_reg_bit(SYS_TRIGGER, 6, true);
}

/*
Set the page id to switch between register pages.
@param page Page id to switch to, 0 (false) or 1 (true)
*/
void BNO055::set_page_id(bool page)
{
    if (_page == page)
    {
        return;
    }

    write_page_id(page);
}

/*
Write the page id to the register (0x07)
@param page Page id to write, 0 (false) or 1 (true)
*/
void BNO055::write_page_id(bool page)
{
    write_reg(PAGE_ID, page ? 1 : 0);
    _page = page;
}

/*
Get the currently selected page id
@return Current page id, 0 (false) or 1 (true)
*/
bool BNO055::get_page_id()
{
    return _page;
}

/*
Read the current page id from the register
@return Current page id, 0 (false) or 1 (true)
*/
bool BNO055::read_page_id()
{
    return (bool)read_reg(PAGE_ID);
}

/*
Read the unique id of the BNO
@return Unique id
*/
uint16_t BNO055::unique_id()
{
    return read_reg(BNO_UNIQUE_ID);
}

/*
Read the chip id of the BNO
@return BNO chip id
*/
uint8_t BNO055::bno_chip_id()
{
    return read_reg(CHIP_ID);
}

/*
Read the chip id of the accelerometer
@return Accelerometer chip id
*/
uint8_t BNO055::acc_chip_id()
{
    return read_reg(ACC_ID);
}

/*
Read the chip id of the magnetometer
@return Magnetometer chip id
*/
uint8_t BNO055::mag_chip_id()
{
    return read_reg(MAG_ID);
}

/*
Read the chip id of the gyroscope
@return Gyroscope chip id
*/
uint8_t BNO055::gyro_chip_id()
{
    return read_reg(GYR_ID);
}

/*
Set the system oscillator source (internal or external)
@param clk_source Clock source , either INTERNAL or EXTERNAL
@return True if change was successful, see the datasheet when clock source changes are allowed.
*/
bool BNO055::set_sys_clk(BNO_CLK_SOURCE clk_source)
{
    if (!get_sys_clk_status())
    {
        return false;
    }

    return set_reg_bit_checked(SYS_TRIGGER, 7, (bool)clk_source);
}

/*
Get the system clock status
@return False: Clock source free to configure, True: In configuration state
*/
bool BNO055::get_sys_clk_status()
{
    return get_reg_bit(SYS_CLK_STAT, 0);
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
Get the measured temperature from TEMP_SOURCE

@return Temperature in degrees celsius or fahrenheit based on selected TEMP_UNIT
*/
int16_t BNO055::temperature()
{
    int16_t temp = read_reg(BNO_TEMP);

    return (_unit_config.temp == BNO_TEMP_UNIT::CELSIUS) ? temp : temp * 2;
}

/*
Set the BNO operation mode
@param mode Operation mode, see BNO_OPERATION_MODE
*/
void BNO055::set_operation_mode(BNO_OPERATION_MODE mode)
{
    write_reg(OPR_MODE, (uint16_t)mode);
    HAL_Delay(20);
}

/*
Get the current BNO operation mode
@return Current operation mode
*/
BNO_OPERATION_MODE BNO055::get_operation_mode()
{
    return (BNO_OPERATION_MODE)read_reg(OPR_MODE);
}

/*
Set the BNO power mode
@param mode Power mode, see BNO_POWER_MODE
*/
void BNO055::set_power_mode(BNO_POWER_MODE mode)
{
    return write_reg(PWR_MODE, (uint16_t)mode);
}

/*
Get the current BNO power mode
@return Current power mode
*/
BNO_POWER_MODE BNO055::get_power_mode()
{
    return (BNO_POWER_MODE)read_reg(PWR_MODE);
}

/*
Set the axis signs for axis inversion
@param x X axis sign - False: no inversion, True: invert axis
@param y Y axis sign - False: no inversion, True: invert axis
@param z Z axis sign - False: no inversion, True: invert axis
*/
void BNO055::set_axis_sign_invert(bool x, bool y, bool z)
{
    uint8_t raw = (x << 2) + (y << 1) + (uint8_t)z;

    return write_reg(AXIS_MAP_SIGN, raw);
}

/*
Get the current axis sign bits.
@return Axis sign byte (AXIS_MAP_SIGN) - b0: Z axis sign, b1: Y axis sign, b2: Z axis sign
*/
uint8_t BNO055::get_axis_sign_invert()
{
    return read_reg(AXIS_MAP_SIGN);
}

/*
Remap BNO axes
@param new_x Axis that will be mapped to the x axis
@param new_y Axis that will be mapped to the y axis
@param new_z Axis that will be mapped to the z axis
*/
void BNO055::set_axis_remap(BNO_AXIS new_x, BNO_AXIS new_y, BNO_AXIS new_z)
{
    uint8_t raw = ((uint8_t)new_z << 4) + ((uint8_t)new_y << 2) + (uint8_t)new_x;
    return write_reg(AXIS_MAP_CONFIG, raw);
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
Get the BNO system error code byte
@return Current error code byte
*/
uint8_t BNO055::get_system_error()
{
    return read_reg(SYS_ERR);
}

/*
Get the interrupt status byte from register INT_STA (0x37).
See the datasheet for all bits available.
@return Interrupt status byte
*/
uint8_t BNO055::get_interrupt_status()
{
    return read_reg(INT_STAT);
}

/*
Get the selftest (st) result from register ST_RESULT (0x36)

b0: Accelerometer st result;
b1: Magnetometer st result;
b2 GYroscope st result;
b3 Microcontroller st result;

1 indicates selftest success, 0 selftest failure

@return Selftest result byte
*/
uint8_t BNO055::get_selftest_results()
{
    return read_reg(ST_RESULT);
}

/*
Get the calibration status byte from register CALIB_STAT (0x35)

b[1..0]: MAG calib status;
b[3..2]: ACC calib status;
b[5..4]: GYR calib status;
b[7..6]: SYS calib status;

Both bits true indicates calibrated, both bits false indicates not calibrated
@return Calibration status byte
*/
uint8_t BNO055::get_calib_status()
{
    return read_reg(CALIB_STAT);
}

/*
Check if the magnetometer is calibrated
@return Calib status from 0 - 3 where 0 means not calibrated and 3 means fully calibrated
*/
uint8_t BNO055::get_mag_calibrated()
{
    return (get_calib_status() & (uint8_t)BNO_CALIB_MASK::BNO_MAG_CALIBRATED);
}

/*
Check if the accelerometer is calibrated
@return Calib status from 0 - 3 where 0 means not calibrated and 3 means fully calibrated
*/
uint8_t BNO055::get_acc_calibrated()
{
    return ((get_calib_status() & (uint8_t)BNO_CALIB_MASK::BNO_ACC_CALIBRATED) >> 2);
}

/*
Check if the gyroscope is calibrated
@return Calib status from 0 - 3 where 0 means not calibrated and 3 means fully calibrated
*/
uint8_t BNO055::get_gyr_calibrated()
{
    return ((get_calib_status() & (uint8_t)BNO_CALIB_MASK::BNO_GYR_CALIBRATED) >> 4);
}

/*
Check if the system is calibrated
@return Calib status from 0 - 3 where 0 means not calibrated and 3 means fully calibrated
*/
uint8_t BNO055::get_sys_calibrated()
{
    return ((get_calib_status() & (uint8_t)BNO_CALIB_MASK::BNO_SYS_CALIBRATED) >> 6);
}

/*
Set which interrupts to enable
@param int_en_bits Interrupt enable bitmap, see BNO_INT_MASK
*/
void BNO055::set_interrupt_enable(uint8_t int_en_bits)
{
    return write_reg(INT_EN, int_en_bits);
}

/*
Set which interrupts to mask
@param int_msk_bits Interrupt mask bitmap, see BNO_INT_MASK
*/
void BNO055::set_interrupt_mask(uint8_t int_msk_bits)
{
    return write_reg(INT_MSK, int_msk_bits);
}

/*
Specify the output orientation format

Note: Changes only take effect when the sensor is in CONFIGMODE
@param mode Orientation output format, see BNO_ORI_FORMAT
*/
void BNO055::set_orientation_format(BNO_ORI_FORMAT format)
{
    return set_reg_bit(UNIT_SEL, 7, (bool)format);
}

/*
Specify the acceleration output unit

Note: Changes only take effect when the sensor is in CONFIGMODE
@param unit Acceleration unit, see BNO_ACC_UNIT
@return True if register update was successful
*/
bool BNO055::set_acceleration_unit(BNO_ACC_UNIT unit)
{
    if (set_reg_bit_checked(UNIT_SEL, 0, (bool)unit))
    {
        _unit_config.acc = unit;
        return true;
    }

    return false;
}

/*
Specify the angular rate output unit

Note: Changes only take effect when the sensor is in CONFIGMODE
@param unit Angular rate unit, see BNO_ANG_RATE_UNIT
@return True if register update was successful
*/
bool BNO055::set_angular_rate_unit(BNO_ANG_RATE_UNIT unit)
{
    if (set_reg_bit_checked(UNIT_SEL, 1, (bool)unit))
    {
        _unit_config.angle_rate = unit;
        return true;
    }

    return false;
}

/*
Specify the angle output unit

Note: Changes only take effect when the sensor is in CONFIGMODE
@param unit Angle unit, see BNO_ANG_UNIT
@return True if register update was successful
*/
bool BNO055::set_angle_unit(BNO_ANG_UNIT unit)
{
    if (set_reg_bit_checked(UNIT_SEL, 2, (bool)unit))
    {
        _unit_config.angle = unit;
        return true;
    }

    return false;
}

/*
Specify the temperature output unit

Note: Changes only take effect when the sensor is in CONFIGMODE
@param unit Temperature unit, see BNO_TEMP_UNIT
@return True if register update was successful
*/
bool BNO055::set_temperature_unit(BNO_TEMP_UNIT unit)
{
    if (set_reg_bit_checked(UNIT_SEL, 4, (bool)unit))
    {
        _unit_config.temp = unit;
        return true;
    }

    return false;
}

/*
Specify from which sensor temperature readings are taken

Note: Changes only take effect when the sensor is in CONFIGMODE
@param source Temperature source, see BNO_TEMP_SOURCE
@return True if register update was successful
*/
void BNO055::set_temperature_source(BNO_TEMP_SOURCE source)
{
    return set_reg_bit(TEMP_SOURCE, 0, (bool)source);
}

/*
Get the current accelerometer data
@return Accelerations for x, y, z in m/s^2 or mg depending on selected unit
*/
bno_vec_3_t BNO055::get_acceleration()
{
    int16_t raw[3];
    read_triple_reg(ACC_DATA, raw);

    bno_vec_3_t data;

    bool unit = (_unit_config.acc == BNO_ACC_UNIT::MILLI_G);
    data.x = unit ? raw[0] : raw[0] / 100.f;
    data.y = unit ? raw[1] : raw[1] / 100.f;
    data.z = unit ? raw[2] : raw[2] / 100.f;

    return data;
}

/*
Get the current linear acceleration data
@return Linear accelerations for x, y, z in m/s^2 or mg depending on selected unit
*/
bno_vec_3_t BNO055::get_linear_acceleration()
{
    int16_t raw[3];
    read_triple_reg(LIA_DATA, raw);

    bno_vec_3_t data;
    bool unit = (_unit_config.acc == BNO_ACC_UNIT::MILLI_G);
    data.x = unit ? raw[0] : raw[0] / 100.f;
    data.y = unit ? raw[1] : raw[1] / 100.f;
    data.z = unit ? raw[2] : raw[2] / 100.f;

    return data;
}

/*
Get the current fused euler angles.
Fusion has to be enabled!
@return Euler angles x, y, z (roll, pitch, yaw) from fusion algorithm in deg or rad depending on selected unit
*/
bno_vec_3_t BNO055::get_euler()
{
    int16_t raw[3];
    read_triple_reg(EULER_DATA, raw);

    bno_vec_3_t data;

    // Switch order to roll, pitch, yaw
    bool unit = (_unit_config.angle == BNO_ANG_UNIT::DEG);
    data.x = unit ? raw[1] / 16.f : raw[1] / 900.f;
    data.y = unit ? raw[2] / 16.f : raw[2] / 900.f;
    data.z = unit ? raw[0] / 16.f : raw[0] / 900.f;

    return data;
}

/*
Get the current magnetometer data
@return Magnetometer data for x, y, z in micro tesla (uT)
*/
bno_vec_3_t BNO055::get_mag_data()
{
    int16_t raw[3];
    read_triple_reg(MAG_DATA, raw);

    bno_vec_3_t data;

    data.x = raw[0] / 16.f;
    data.y = raw[1] / 16.f;
    data.z = raw[2] / 16.f;

    return data;
}

/*
Get the current gyroscope data
@return Angular rotation rates for x, y, z in dps or rps depending on selected unit
*/
bno_vec_3_t BNO055::get_gyro_data()
{
    int16_t raw[3];
    read_triple_reg(GYR_DATA, raw);

    bno_vec_3_t data;

    bool unit = (_unit_config.angle_rate == BNO_ANG_RATE_UNIT::DEG_PER_SECOND);
    data.x = unit ? raw[0] / 16.f : raw[0] / 900.f;
    data.y = unit ? raw[1] / 16.f : raw[1] / 900.f;
    data.z = unit ? raw[2] / 16.f : raw[2] / 900.f;

    return data;
}

/*
Get the current gravity vector data
@return Gravity vector x,y,z in m/s^2 or mg depending on selected unit
*/
bno_vec_3_t BNO055::get_gravity_vector()
{
    int16_t raw[3];
    read_triple_reg(GRV_DATA, raw);

    bno_vec_3_t data;

    bool unit = (_unit_config.acc == BNO_ACC_UNIT::MILLI_G);
    data.x = unit ? raw[0] : raw[0] / 100.f;
    data.y = unit ? raw[1] : raw[1] / 100.f;
    data.z = unit ? raw[2] : raw[2] / 100.f;

    return data;
}

/*
Get the current fused rotation quaternion
@return Rotation unit quaternion
*/
bno_vec_4_t BNO055::get_quaternion_data()
{
    int16_t raw[4];
    read_quad_reg(QUA_DATA, raw);

    bno_vec_4_t data;

    data.w = raw[0] / (float)(2 << 13);
    data.x = raw[1] / (float)(2 << 13);
    data.y = raw[2] / (float)(2 << 13);
    data.z = raw[3] / (float)(2 << 13);

    return data;
}

/*
Get the current sensor offsets

No unit conversion happens, data is in LSB!
@return struct containing the current offsets for all three sensors
*/
bno_sensor_offsets_t BNO055::get_sensor_offsets()
{
    // Should be combined into a single request in the future
    int16_t off[9];
    read_triple_reg(ACC_OFFSET, off);
    read_triple_reg(MAG_OFFSET, off + 3);
    read_triple_reg(GYR_OFFSET, off + 6);

    bno_sensor_offsets_t offsets;

    offsets.accelerometer.x = off[0];
    offsets.accelerometer.y = off[1];
    offsets.accelerometer.z = off[2];
    offsets.magnetometer.x = off[3];
    offsets.magnetometer.y = off[4];
    offsets.magnetometer.z = off[5];
    offsets.gyroscope.x = off[6];
    offsets.gyroscope.y = off[7];
    offsets.gyroscope.z = off[8];

    return offsets;
}

/*
Set the sensor offsets. Only in CONFIGMODE!

No unit conversion happens, data should be LSB!
@param offsets Offsets struct
*/
void BNO055::set_sensor_offsets(const bno_sensor_offsets_t &offsets)
{
    int16_t off[3];
    off[0] = offsets.accelerometer.x;
    off[1] = offsets.accelerometer.y;
    off[2] = offsets.accelerometer.z;
    write_triple_reg(ACC_OFFSET, off);

    off[0] = offsets.magnetometer.x;
    off[1] = offsets.magnetometer.y;
    off[2] = offsets.magnetometer.z;
    write_triple_reg(MAG_OFFSET, off);

    off[0] = offsets.gyroscope.x;
    off[1] = offsets.gyroscope.y;
    off[2] = offsets.gyroscope.z;
    write_triple_reg(GYR_OFFSET, off);
}

/*
Get the current calibration data. Includes all offsets, magnetometer radius and accelerometer radius.
@return Current calibration data as struct
*/
bno_sensor_calibration_t BNO055::get_calibration_data()
{
    bno_sensor_calibration_t calib;

    // Should be a single access for all bytes
    calib.offsets = get_sensor_offsets();

    uint16_t acc_radius = read_reg(ACC_RADIUS);
    uint16_t mag_radius = read_reg(MAG_RADIUS);
    calib.acc_radius = *(int16_t *)&acc_radius;
    calib.mag_radius = *(int16_t *)&mag_radius;

    return calib;
}

void BNO055::set_calibration_data(const bno_sensor_calibration_t &calib_data)
{
    set_sensor_offsets(calib_data.offsets);
    write_reg(ACC_RADIUS, calib_data.acc_radius);
    write_reg(MAG_RADIUS, calib_data.mag_radius);
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

void BNO055::read_triple_reg(const bno_reg_t &reg, int16_t *data)
{
    read_i2c_bytes(_hi2c, _address, reg.address, (uint8_t *)data, 6);
}

void BNO055::write_triple_reg(const bno_reg_t &reg, const int16_t *data)
{
    set_page_id(reg.page);
    uint8_t raw[7];

    raw[0] = reg.address;
    raw[1] = (data[0] >> 8);
    raw[2] = (data[0] & 0xFF);
    raw[3] = (data[1] >> 8);
    raw[4] = (data[1] & 0xFF);
    raw[5] = (data[2] >> 8);
    raw[6] = (data[2] & 0xFF);

    write_i2c_bytes(_hi2c, _address, raw, 7);
}

void BNO055::read_quad_reg(const bno_reg_t &reg, int16_t *data)
{
    set_page_id(reg.page);
    read_i2c_bytes(_hi2c, _address, reg.address, (uint8_t *)data, 8);
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

bool BNO055::set_reg_bit_checked(const bno_reg_t &reg, uint8_t n, bool value)
{
    set_reg_bit(reg, n, value);
    return (get_reg_bit(reg, n) == value);
}