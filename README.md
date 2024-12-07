# STM32-BNO055
STM32 library for the Bosch BNO055 IMU with integrated sensor fusion algorithm

## Usage example
Copy the `BNO055.cpp`, `BNO55.hpp`, `I2C.cpp` and `I2C.hpp` to your source and include directory.

To build, specify a build flag for your STM32 MCU. If you use a STM32F1XXX for example, use `-D STM32F1`. In the Cube IDE, edit the C/C++ build settings.
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
