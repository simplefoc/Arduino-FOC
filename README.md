# Arduino Simple FOC library minimal example 

![MinimalBuild](https://github.com/askuric/Arduino-FOC/workflows/MinimalBuild/badge.svg?branch=minimal)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![arduino-library-badge](https://www.ardu-badge.com/badge/Simple%20FOC.svg?)](https://www.ardu-badge.com/badge/Simple%20FOC.svg)

This is the repository of the [*SimpleFOClibrary*](https://github.com/askuric/Arduino-FOC) intended to be used to crete the projects with minimal code possible which is specific for certain **motor+sensor+driver** combination. 

### Repository structure
```shell
├─── library_source
| |
| ├─ BLDCMotor.cpp/h           # BLDCMotor class implementing all the FOC operation
| ├─ FOCutils.cpp/h            # Utility functions 
| ├─ defaults.h                # Default configuration values
│ ├─ Sensor.h                  # Abstract Sensor class that all the sensors implement
│ │ 
│ ├─ Encoder.cpp/h                # Encoder class implementing the Quadrature encoder operations
│ ├─ MagneticSensorSPI.cpp/h      # class implementing SPI communication for Magnetic sensors
│ ├─ MagneticSensorI2C.cpp/h      # class implementing I2C communication for Magnetic sensors
│ ├─ MagneticSensorAnalog.cpp/h   # class implementing Analog output for Magnetic sensors
│ └─ HallSensor.cpp/h             # class implementing Hall sensor 
|
└─── minimal_project_examples
  ├─ arduino_foc_minimal_openloop       # Arduino minimal code for running a motor in the open loop
  ├─ arduino_foc_minimal_encoder        # Arduino minimal code for running a motor with Encoder
  ├─ arduino_foc_minimal_hall           # Arduino minimal code for running a motor with Hall sensors
  ├─ arduino_foc_minimal_magnetic_i2c   # Arduino minimal code for running a motor with I2C magnetic sensor 
  └─ arduino_foc_minimal_magnetic_spi   # Arduino minimal code for running a motor with SPI magnetic sensor 
```

# Creating your own minimal project
First you need to download this repository to your computer. 

#### Github website download
- Make sure you are in [minimal branch](https://github.com/askuric/Arduino-FOC/tree/minimal) 
- Download the code by clicking on the `Clone or Download > Download ZIP`.
- Unzip it 

#### Using terminal
- Open the terminal:
  ```sh
  cd *to you desired directory*
  git clone -b minimal https://github.com/askuric/Arduino-FOC.git
  ```

After this step you will be able to open the example projects directly with Arduino IDE. This code is completely independent and you can run it as any other Arduino Sketch without the need for any libraries. 

> **BEWARE** In some cases this minimal version of the code will produce conflicts with the *Simple FOC* library if it is installed through Arduino library manager. So you might need to uninstall the library to run minimal projects.

## BLDC motor support code

In the `library_source` folder you will find all *SimpleFOClibrary* source files. From those files you will choose just the ones you need for your own project. 

In each Arduino project using the *SimpleFOClibrary* you will need to copy these six files into your project directory.

```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| |
| ├─ BLDCMotor.cpp/h           # BLDCMotor class implementing all the FOC operation
| ├─ FOCutils.cpp/h            # Utility functions 
| ├─ defaults.h                # Default configuration values
  └─ Sensor.h                  # Abstract Sensor class that all the sensors implement
```

And in your arduino code you need to add the include:
```cpp
#include "BLDCMotor.h"
```

If you wish to run your motor in the open loop mode these are all the files that you will need. See the `arduino_foc_minimal_openloop.ino` project example.

## Sensor support
In order to support the different position sensors you will have to add their `*.cpp` and `*.h` files into the directory. All you need to do is copy the header files from the `library_source` directory.


### Example: Encoder sensor 
For example if you wish to use encoder, your arduino project will have structure:

```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| |
| ├─ BLDCMotor.cpp/h           # BLDCMotor class implementing all the FOC operation
| ├─ FOCutils.cpp/h            # Utility functions 
│ ├─ defaults.h                # Default configuration values
│ ├─ Sensor.h                  # Abstract Sensor class that all the sensors implement
│ |
  └─ Encoder.cpp/h                # Encoder class implementing the Quadrature encoder operations
```
And your includes will be:
```cpp
#include "BLDCMotor.h"
#include "Encoder.h"
```
See `arduino_foc_minimal_encoder.ino`.

### Example: SPI Magnetic sensor 
If you wish to use SPI magnetic sensor with your project, your folder structure will be:

```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| |
| ├─ BLDCMotor.cpp/h           # BLDCMotor class implementing all the FOC operation
| ├─ FOCutils.cpp/h            # Utility functions 
│ ├─ defaults.h                # Default configuration values
│ ├─ Sensor.h                  # Abstract Sensor class that all the sensors implement
│ |
  └─ MagneticSensorSPI.cpp/h      # class implementing SPI communication for Magnetic sensors
```
And your includes will be:
```cpp
#include "BLDCMotor.h"
#include "MagneticSensorSPI.h"
```
See `arduino_foc_minimal_magnetic_spi.ino`.


### Example: Analog magnetic sensor and encoder
For example if you wish to use magnetic sensor with SPI communication, your arduino project will have structure:

```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| |
| ├─ BLDCMotor.cpp/h           # BLDCMotor class implementing all the FOC operation
| ├─ FOCutils.cpp/h            # Utility functions 
│ ├─ defaults.h                # Default configuration values
│ ├─ Sensor.h                  # Abstract Sensor class that all the sensors implement
│ |
│ ├─ Encoder.cpp/h                # Encoder class implementing the Quadrature encoder operations
  └─ MagneticSensorAnalog.cpp/h   # class implementing Analog output for Magnetic sensors
```
And your includes will be:
```cpp
#include "BLDCMotor.h"
#include "MagneticSensorAnalog.h"
#include "Encoder.h"
```

## Documentation
Find out more information about the Arduino *SimpleFOCproject* in [docs website](https://docs.simplefoc.com/) 


## Arduino FOC repo structure
Branch  | Description | Status
------------ | ------------- | ------------ 
[master](https://github.com/simplefoc/Arduino-FOC) | Stable and tested library version | ![Library Compile](https://github.com/simplefoc/Arduino-FOC/workflows/Library%20Compile/badge.svg)
[dev](https://github.com/simplefoc/Arduino-FOC/tree/dev) | Development library version | ![Library Dev Compile](https://github.com/simplefoc/Arduino-FOC/workflows/Library%20Dev%20Compile/badge.svg?branch=dev)
[minimal](https://github.com/simplefoc/Arduino-FOC/tree/minimal) | Minimal Arduino example with integrated library | ![MinimalBuild](https://github.com/simplefoc/Arduino-FOC/workflows/MinimalBuild/badge.svg?branch=minimal)
