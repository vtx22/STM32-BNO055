# STM32-BNO055
STM32 library for the Bosch BNO055 IMU with integrated sensor fusion algorithm

## Usage example
### Simple Code
```c++
#include "BNO055.hpp"

// Create BNO object with I2C interface and I2C address
BNO055 bno(&hi2c1, 0x28);

// (Optional) Specify the reset GPIO pin if connected
bno.set_reset_pin(BNO_RST_GPIO_Port, BNO_RST_Pin);

// Perform a sensor reset
bno.reset();

// Perform your configurations here while the sensor is in CONFIGMODE
// [...]

// Start the sensor in full fusion mode, configurations are now disabled
bno.set_operation_mode(BNO_OPERATION_MODE::NDOF);

// Simple loop to read fused angle data every 100ms
while(true)
{
  bno_vec_3_t euler_angles = bno.euler_angles();
  HAL_Delay(100);
}
```

### Build
Copy the `BNO055.cpp`, `BNO55.hpp`, `BNO55_reg.hpp`, `I2C.cpp` and `I2C.hpp` to your source and include directory.

To build, specify a build flag for your STM32 MCU. For a STM32F1XXX for example, use `-D STM32F1`.

In the **Cube IDE**, paste the flag in the `Preprocessor` tab in the C/C++ build settings under `Project > Properties`

## API
### Constructor
Construct a BNO sensor object using the hi2c interface and the I2C address (e.g. 0x28)
```c++
BNO055(I2C_HandleTypeDef *hi2c, uint8_t address)
```
---
### Specify a reset pin
Specify a GPIO pin that is connected to the BNO reset pin for hardware resets.
The reset is performed by setting the pin low for 100ms and then back to high. If invert is true, the pin is set high and then kept low.
```c++
void set_reset_pin(GPIO_TypeDef *port, uint16_t pin, bool invert);   // Specify a STM32 GPIO pin using port and pin number
void set_reset_pin(GPIO_TypeDef *port, uint16_t pin);                // Same as above with invert=false
```
---
### Sensor Reset
The sensor can be reset via the reset pin or through a reset bit. After a reset, the method blocks for 900ms to ensure that the sensor has started.
```c++
void reset();            // Perform a hardware reset or a software reset if no reset pin is defined
void hardware_reset();   // Perform a hardware reset by toggeling the reset pin, then wait 900ms. If no pin is defined, method returns.
void software_reset();   // Perform a software reset by setting RST_SYS in SYS_TRIGGER and wait 900ms
```
---
### Get sensor IDs
```c++
uint16_t unique_id();     // Get the BNO           unique id
uint8_t  bno_chip_id();   // Get the BNO           chip id, fixed value: 0xA0
uint8_t  acc_chip_id();   // Get the accelorometer chip id, fixed value: 0xFB
uint8_t  mag_chip_id();   // Get the magnetometer  chip id, fixed value: 0x32
uint8_t  gyro_chip_id();  // Get the gyroscope     chip id, fixed value: 0x0F
```
---
### Software versions
```c++
uint16_t  software_revision();   // Get the current BNO software version, format: [MSB].[LSB]
uint8_t   bootloader_version();  // Get the current BNO bootloader version
```
---
### Sensor self test
```c++
void     self_test();             // Trigger a self test
uint8_t  get_selftest_results();  // Returns selftest result bits
```
---
### Operation Mode
The operation mode defines which sensors and algorithms are active.
| **BNO_OPERATION_MODE**      | **Description**                                                                          | **ACC**            | **MAG**            | **GYR**            | **Fusion**         |
| :---------------------------| :----------------------------------------------------------------------------------------|--------------------|--------------------|--------------------|--------------------|
| CONFIGMODE                  | This is the only mode in which setting registers are writable, default mode after reset  | :x:                | :x:                | :x:                | :x:                | 
| ACC_ONLY                    | Only accelerometer enabled                                                               | :heavy_check_mark: | :x:                | :x:                | :x:                | 
| MAG_ONLY                    | Only magnetometer enabled                                                                | :x:                | :heavy_check_mark: | :x:                | :x:                | 
| GYRO_ONLY                   | Only gyroscope enabled                                                                   | :x:                | :x:                | :heavy_check_mark: | :x:                | 
| ACC_MAG                     | Both accelerometer and magnetometer enabled                                              | :heavy_check_mark: | :heavy_check_mark: | :x:                | :x:                | 
| ACC_GYRO                    | Both accelerometer and gyroscope enabled                                                 | :heavy_check_mark: | :x:                | :heavy_check_mark: | :x:                | 
| MAG_GYRO                    | Both magnetometer and gyroscope enabled                                                  | :x:                | :heavy_check_mark: | :heavy_check_mark: | :x:                | 
| AMG                         | All three sensors enabled                                                                | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :x:                | 
| IMU                         | Relative orientation from accelerometer and gyroscope                                    | :heavy_check_mark: | :x:                | :heavy_check_mark: | :heavy_check_mark: | 
| COMPASS                     | Heading calculation from gravity vector and magnetometer                                 | :heavy_check_mark: | :heavy_check_mark: | :x:                | :heavy_check_mark: | 
| M4G                         | Same as IMU, but magnetometer is used instead of gyroscope                               | :heavy_check_mark: | :heavy_check_mark: | :x:                | :heavy_check_mark: | 
| NDOF_FMC_OFF                | 9 DoF fusion without fast magnetometer calibration (FMC)                                 | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | 
| NDOF                        | 9 DoF fusion with fast magentometer calibration (FMC)                                    | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | 

:warning: Switching operation modes takes around 20 ms
```c++
void                 set_operation_mode(BNO_OPERATION_MODE mode);  // Sets the operation mode and blocks for 20 ms
BNO_OPERATION_MODE   get_operation_mode();                         // Get the current operation mode
```
---
### Power Mode
The BNO allows for three general power modes.
| **BNO_POWER_MODE**          | **Description**                                                                                                         |
| :---------------------------| :-----------------------------------------------------------------------------------------------------------------------|
| NORMAL                      | All sensors required for the selected operation mode are always switched ON                                             | 
| LOW_POWER                   | If no activity, low power mode is entered. Only accelerometer is active. If motion is detected, normal mode is entered. | 
| SUSPEND                     | System is paused and all sensors are halted. Microcontroller is in sleep mode. No registers are updated.                |

```c++
void             set_power_mode(BNO_POWER_MODE mode);   // Set the power mode
BNO_POWER_MODE   get_power_mode();                      // Get the current power mode
```
---
### Sensor calibration
```c++
uint8_t get_calib_status();      // Get the sensor calibration status, 0xFF means fully calibrated
```
Check individual sensor calibration status:
```c++
uint8_t get_mag_calibrated();    // calibration state, 0 - 3 where 3 means fully calibrated
uint8_t get_acc_calibrated();    // calibration state, 0 - 3 where 3 means fully calibrated
uint8_t get_gyr_calibrated();    // calibration state, 0 - 3 where 3 means fully calibrated
uint8_t get_sys_calibrated();    // calibration state, 0 - 3 where 3 means fully calibrated
```

---
### Interrupts
#### Interrupt Enable
To enable certain interrupts, set the respective bits. For all bits see `BNO_INT_MASK` or the datasheet.
```c++
void set_interrupt_enable(uint8_t int_en_bits);   // Set the interrupt enable bits, see BNO_INT_MASK for all bits
```
Example:
```c++
set_interrupt_enable(ACC_NM | ACC_HIGH_G); // Enable accelerometer no motion and high-g interrupt
```
To disable all interrupts, set the interrupt enable byte to `0`.
#### Interrupt Mask
With the interrupt mask you can control which interrupts trigger a change of the INT pin. When disabled, only the INT_STA bit will be updated.
```c++
void set_interrupt_mask(uint8_t int_msk_bits);   // Set the interrupt mask, see BNO_INT_MASK for all bits
```
To clear all mask bits, set the interrupt mask byte to `0`.
#### Interrupt Status and Reset
```c++
uint8_t get_interrupt_status();  // Get the interrupt status bits
void reset_interrupts();         // Reset all interrupt status bits
```
---
### System Clock
The BNO can run using its internal or an external oscillator.
```c++
bool set_sys_clk(BNO_CLK_SOURCE clk_source);  // Set the system oscillator source (INTERNAL or EXTERNAL)
bool get_sys_clk_status();                    // Get the system clock status
```
---
### Temperature
Both the accelerometer and the gyroscope have an integrated temperature sensor. You can select which one should be used for readings.
```c++
void set_temperature_source(BNO_TEMP_SOURCE source); // TEMP_ACCELEROMETER or TEMP_GYROSCOPE
```
Reading the temperature is done using
```c++
int16_t temperature(); // Returns temperature in °C or °F, see set_temperature_unit
```
---
### Unit selection

You can select different output units for different data types by calling the `set_[...]_unit` methods. The following units are available.
| **Unit Struct Name**          | **Influenced Data**                            | **Available units** |
| :---------------------------- | :--------------------------------------------- | :------------------ |
| BNO_ACC_UNIT                  | acceleration<br>linear acceleration<br>gravity | `m/s^2` or `mg`     |
| BNO_ANG_UNIT                  | euler angles                                   | `deg` or `rad`      |
| BNO_ANG_RATE_UNIT             | angular rate                                   | `deg/s` or `rad/s`  |
| BNO_TEMP_UNIT                 | temperature                                    | `°C` or `°F`        |

:warning: Changing units is only possible in `CONFIGMODE`<br>
:information_source: Quaternion data is unitless<br>
:information_source: Magnetometer data unit is always `uT` and cannot be changed

The selected units are tracked internally by the sensor class. When you read the data the correct unit conversion is performed.<br>
`temperature()` for example will return either `°C` or `°F`, depending on your selection.

```c++
bool set_acceleration_unit(BNO_ACC_UNIT unit);          // either METERS_PER_SECOND2 or MILLI_G
bool set_angle_unit(BNO_ANG_UNIT unit);                 // either DEG                or RAD
bool set_angular_rate_unit(BNO_ANG_RATE_UNIT unit);     // either DEG_PER_SECOND     or RAD_PER_SECOND
bool set_temperature_unit(BNO_TEMP_UNIT unit);          // either CELSIUS            or FAHRENHEIT
```

All functions return `true` if the unit was updated sucessfully. If false, make sure that you are in `CONFIGMODE`

---
### Axis output settings
:warning: Changing axis settings is only possible in `CONFIGMODE`<br>
#### Orientation output format
Two different fusion data output formats are available.
| **Rotation Angle** | **Range (Android)**                     | **Range (Windows)**                     |
|--------------------|-----------------------------------------|-----------------------------------------|
| Pitch              | +180° to -180°<br>(cw = value decrease) | -180° to +180°<br>(cw = value increase) |
| Roll               | -90° to +90°                            | -90° to +90°                            |
| Yaw                | 0° to 360°                              | 0° to 360°                              |

```c++
void set_orientation_format(BNO_ORI_FORMAT format); // either WINDOWS or ANDROID
```
#### Axis signs
You can change the sign of every axis to invert its direction.
```c++
void set_axis_sign_invert(bool x, bool y, bool z);  // false = no invert, true = invert axis
uint8_t get_axis_sign_invert();                     // Get the current signs, bits are: 0bxxxxxXYZ
```
#### Axis remap
You can remap axes to new axes. The default map is: X Axis = X, Y Axis = Y, Z Axis = Z. Mapping one axis to two or more is not allowed and will be ignored by the BNO.
```c++
void set_axis_remap(BNO_AXIS new_x, BNO_AXIS new_y, BNO_AXIS new_z); // use BNO_X_AXIS, BNO_Y_AXIS or BNO_Z_AXIS
```
---
### System Status
Get the system status byte.
```c++
uint8_t get_system_status();
```
---
### System Error
Get the system error byte.
```c++
uint8_t get_system_error();
```
---
### Output IMU data
#### Accelerometer data
Get the acceleration data for x, y and z axis.
```c++
bno_vec_3_t get_acceleration();
```
:information_source: The unit is either `m/s^2` or `mg`, see Unit selection 
#### Linear Acceleration
Get the linear acceleration for x, y and z axis.
```c++
bno_vec_3_t get_linear_acceleration();
```
:information_source: The unit is either `m/s^2` or `mg`, see Unit selection<br>
:warning: Linear acceleration integration for velocity and position estimation is too inaccurate.
#### Gravity vector
Get the gravity vector estimation.
```c++
bno_vec_3_t get_gravity_vector();
```
:information_source: The unit is either `m/s^2` or `mg`, see Unit selection<br>
:warning: Only available in fusion mode
#### Gyroscope data
Get the gyroscope data (angular rate) for x, y and z axis.
```c++
bno_vec_3_t get_gyro_data();
```
:information_source: The unit is either `deg/s` or `rad/s`, see Unit selection 
#### Magnetometer data
Get the magnetometer data for x, y and z axis.
```c++
bno_vec_3_t get_mag_data();
```
:information_source: The unit is always `uT`
#### Euler angles
Get the fused orientation as euler angles for each axis.
```c++
bno_vec_3_t get_euler();
```
:information_source: The unit is either `deg` or `rad`, see Unit selection<br>
:warning: Only available in fusion mode
#### Quaternion data
Get the fused orientation as quaternion (w + ix + jy + kz)
```c++
bno_vec_4_t get_quaternion_data();
```
:information_source: Quaternions are unitless<br>
:warning: Only available in fusion mode

---
