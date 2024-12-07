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

// Start the sensor, configurations are now disabled 
bno.set_power_mode(NORMAL);
bno.set_operation_mode(NDOF);

// Simple loop to read fused angle data every 100ms
while(true)
{
  bno_vec_3_t euler_angles = bno.euler_angles();
  HAL_Delay(100);
}
```

### Build
Copy the `BNO055.cpp`, `BNO55.hpp`, `I2C.cpp` and `I2C.hpp` to your source and include directory.

To build, specify a build flag for your STM32 MCU. For a STM32F1XXX for example, use `-D STM32F1`.

In the **Cube IDE**, paste the flag in the `Preprocessor` tab in the C/C++ build settings under `Project > Properties`

## API
### Constructor
Construct a BNO sensor object using the hi2c interface and the I2C address (e.g. 0x28)
```c++
BNO055(I2C_HandleTypeDef *hi2c, uint8_t address)
```
### Specify a reset pin
Specify a GPIO pin that is connected to the BNO reset pin for hardware resets.
The reset is performed by setting the pin low for 100ms and then back to high. If invert is true, the pin is set high and then kept low.
```c++
void set_reset_pin(GPIO_TypeDef *port, uint16_t pin, bool invert);   // Specify a STM32 GPIO pin using port and pin number
void set_reset_pin(GPIO_TypeDef *port, uint16_t pin);                // Same as above with invert=false
```
### Sensor Reset
The sensor can be reset via the reset pin or through a reset bit. After a reset, the method blocks for 900ms to ensure that the sensor has started.
```c++
void reset();            // Perform a hardware reset or a software reset if no reset pin is defined
void hardware_reset();   // Perform a hardware reset by toggeling the reset pin, then wait 900ms. If no pin is defined, method returns.
void software_reset();   // Perform a software reset by setting RST_SYS in SYS_TRIGGER and wait 900ms
```
### Get sensor IDs
```c++
uint16_t unique_id();     // Get the BNO           unique id
uint8_t  bno_chip_id();   // Get the BNO           chip id, fixed value: 0xA0
uint8_t  acc_chip_id();   // Get the accelorometer chip id, fixed value: 0xFB
uint8_t  mag_chip_id();   // Get the magnetometer  chip id, fixed value: 0x32
uint8_t  gyro_chip_id();  // Get the gyroscope     chip id, fixed value: 0x0F
```
### Software versions
```c++
uint16_t  software_revision();   // Get the current BNO software version, format: [MSB].[LSB]
uint8_t   bootloader_version();  // Get the current BNO bootloader version
```

### Sensor self test
```c++
void     self_test();             // Trigger a self test
uint8_t  get_selftest_results();  // Returns selftest result bits
```

### Operation Mode
The operation mode defines what sensors and algorithms are active.
```c++
void                 set_operation_mode(BNO_OPERATION_MODE mode);  // Set the operation mode
BNO_OPERATION_MODE   get_operation_mode();                         // Get the current operation mode
```

### Power Mode
The BNO allows for three general power modes
```c++
void             set_power_mode(BNO_POWER_MODE mode);   // Set the power mode
BNO_POWER_MODE   get_power_mode();                      // Get the current power mode
```
### Interrupts
```c++
void reset_interrupts(); // Reset all interrupt status bits
```

### System Clock
The BNO can run using its internal or an external oscillator.
```c++
bool set_sys_clk(BNO_CLK_SOURCE clk_source);  // Set the system oscillator source (INTERNAL or EXTERNAL)
bool get_sys_clk_status();                    // Get the system clock status
```

### Temperature
Both the accelerometer and the gyroscope have an integrated temperature sensor. You can select which one should be used for readings.
```c++
void set_temperature_source(BNO_TEMP_SOURCE source); // TEMP_ACCELEROMETER or TEMP_GYROSCOPE
```
Reading the temperature is done using
```c++
int16_t temperature(); // Returns temperature in degrees celsius or fahrenheit, see set_temperature_unit
```
