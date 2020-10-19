# Arduino *SimpleFOClibrary* v1.6.0 - minimal project builder 

![MinimalBuild](https://github.com/askuric/Arduino-FOC/workflows/MinimalBuild/badge.svg?branch=minimal)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![arduino-library-badge](https://www.ardu-badge.com/badge/Simple%20FOC.svg?)](https://www.ardu-badge.com/badge/Simple%20FOC.svg)

This is the branch of the [*SimpleFOClibrary*](https://github.com/askuric/Arduino-FOC) repository intended to be used to simplify the creation of the projects with minimal code possible which is specific for certain **motor+sensor+driver** combination. 

### Repository structure
Library source code structure
```shell
├─── library_source
| |
| ├─── common                  # Contains all the common utility classes and functions
| | |
| | ├─ defaults.h              # default motion control parameters
| | |
| | ├─ foc_utils.cpp./h        # utility functions of the FOC algorithm
| | ├─ hardware_utils.cpp./h   # all the hardware specific implementations are in these files
| | ├─ pid.cpp./h              # class implementing PID controller
| | ├─ lowpass_filter.cpp./h   # class implementing Low pass filter
| | |
| | ├─ FOCMotor.cpp./h         # common class for all implemented motors  
| | └─ Sensor./h               # common class for all implemented sensors    
| |
| ├─ BLDCMotor.cpp/h           # BLDC motor handling class  
| ├─ StepperMotor.cpp/h        # Stepper motor handling class  
│ │ 
│ ├─ Encoder.cpp/h                # Encoder class implementing the Quadrature encoder operations
│ ├─ MagneticSensorSPI.cpp/h      # class implementing SPI communication for Magnetic sensors
│ ├─ MagneticSensorI2C.cpp/h      # class implementing I2C communication for Magnetic sensors
│ ├─ MagneticSensorAnalog.cpp/h   # class implementing Analog output for Magnetic sensors
  └─ HallSensor.cpp/h             # class implementing Hall sensor 
```

Minimal project examples provided for quick start:
```shell
├─── minimal_project_examples       # Project examples
│ ├─ arduino_foc_bldc_openloop       # BLDC motor + Open loop control
│ ├─ arduino_foc_bldc_encoder        # BLDC motor + Encoder
│ ├─ arduino_foc_bldc_hall           # BLDC motor + Hall sensors
│ ├─ arduino_foc_bldc_magnetic_i2c   # BLDC motor + I2C magnetic sensor 
│ ├─ arduino_foc_bldc_magnetic_spi   # BLDC motor + SPI magnetic sensor 
│ |
│ ├─ arduino_foc_stepper_openloop       # Stepper motor + Open loop control
│ ├─ arduino_foc_stepper_encoder        # Stepper motor + Encoder
  └─ arduino_foc_stepper_magnetic_i2c   # Stepper motor + I2C magnetic sensor 
```



# Creating your own minimal project

Creating your own minimal project version is very simple and is done in four steps:
- Step 0: Download minimal branch contents to your PC
- Step 1: Create your the arduino project
- Step 2: Add motor specific code
- Step 3: Add sensor specific code

## Step 0. Download the code
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

## Step 1. Creating the Arduino project

Open a directory you want to use as your arduino project directory `my_arduino_project` and create `my_arduino_project.ino` file. After this you create `src` folder in this directory and copy the folder named `common` from the `library_source` folder.   Your project directory should now have structure:

```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| └─── src
| | ├─── common               # Contains all the common utility classes and functions
| | ├─ defaults.h             # default motion control parameters
| | ├─ foc_utils.cpp./h       # utility functions of the FOC algorithm
| | ├─ hardware_utils.cpp./h  # all the hardware specific implementations are in these files
| | ├─ pid.cpp./h             # class implementing PID controller
| | ├─ lowpass_filter.cpp./h  # class implementing Low pass filter
| | ├─ FOCMotor.cpp./h        # common class for all implemented motors  
| | └─ Sensor./h              # common class for all implemented sensors  
```

## Step 2. Add motor specific code
If you wish to use the BLDC motor with your setup you will have to copy the `BLDCMotor.cpp/h` from the `library_source` folder, and if you wish to use the stepper motor make sure to copy the `StepperMotor.cpp/h` files and place them to the `src` folder
```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| └─── src
| | ├─── common             # Common utility classes and functions
| | |
| | └─ BLDCMotor.cpp/h      # BLDC motor handling class  
```
And in your Arduino code in the `my_arduino_project.ino` file make sure to add the the include:
```cpp
#include "src/BLDCMotor.h"
```
For stepper motors the procedure is equivalent:

```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| └─── src
| | ├─── common             # Common utility classes and functions
| | |
| | └─ StepperMotor.cpp/h   # Stepper motor handling class 
```
And the include:
```cpp
#include "src/StepperMotor.h"
```
If you wish to run your motor in the open loop mode these are all the files that you will need. See the `arduino_foc_bldc_openloop` and  `arduino_foc_stepper_openloop` project example.

## Step 3. Add sensor specific code
In order to support the different position sensors you will have to copy their `*.cpp` and `*.h` into your `src` directory. All you need to do is copy the header files from the `library_source` directory.


### Example: Encoder sensor 
For example if you wish to use BLDC motor and encoder as a sensor, your arduino project will have structure:
```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| └─── src
| | ├─── common             # Common utility classes and functions
| | |
| | ├─ BLDCMotor.cpp/h      # BLDC motor handling class 
| | └─ Encoder.cpp/h        # Encoder class implementing the Quadrature encoder operations
```
And your includes will be:
```cpp
#include "src/BLDCMotor.h"
#include "src/Encoder.h"
```
See `arduino_foc_bldc_encoder` project example or `arduino_foc_stepper_encoder` for stepper equivalent.

### Example: SPI Magnetic sensor 
If you wish to use Stepper motor and SPI magnetic sensor in your project, your folder structure will be:

```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| └─── src
| | ├─── common                # Common utility classes and functions
| | |
| | ├─ StepperMotor.cpp/h      # Stepper motor handling class  
| | └─ MagneticSensorSPI.cpp/h  # class implementing SPI communication for Magnetic sensors
```
And your includes will be:
```cpp
#include "src/StepperMotor.h"
#include "src/MagneticSensorSPI.h"
```
See `arduino_foc_stepper_magnetic_spi` project example or `arduino_foc_bldc_magnetic_spi` for BLDC motor equivalent.


### Example: Multiple sensors: analog magnetic sensor and encoder
For example if you wish to use magnetic sensor with SPI communication, your arduino project will have structure:

```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| └─── src
| | ├─── common                # Common utility classes and functions
| | |
| | ├─ BLDCMotor.cpp/h              # BLDC motor handling class  
| | ├─ Encoder.cpp/h                # Encoder class implementing the Quadrature encoder operations
| | └─ MagneticSensorAnalog.cpp/h   # class implementing Analog output for Magnetic sensors
```
And your includes will be:
```cpp
#include "src/BLDCMotor.h"
#include "src/MagneticSensorAnalog.h"
#include "src/Encoder.h"
```


## Documentation
Find out more information about the Arduino *Simple**FOC**library* and *Simple**FOC**project* in [docs website](https://docs.simplefoc.com/) 


## Arduino FOC repo structure
Branch  | Description | Status
------------ | ------------- | ------------ 
[master](https://github.com/simplefoc/Arduino-FOC) | Stable and tested library version | ![Library Compile](https://github.com/simplefoc/Arduino-FOC/workflows/Library%20Compile/badge.svg)
[dev](https://github.com/simplefoc/Arduino-FOC/tree/dev) | Development library version | ![Library Dev Compile](https://github.com/simplefoc/Arduino-FOC/workflows/Library%20Dev%20Compile/badge.svg?branch=dev)
[minimal](https://github.com/simplefoc/Arduino-FOC/tree/minimal) | Minimal Arduino example with integrated library | ![MinimalBuild](https://github.com/simplefoc/Arduino-FOC/workflows/MinimalBuild/badge.svg?branch=minimal)
