# Arduino Simple FOC library minimal example 

![MinimalBuild](https://github.com/askuric/Arduino-FOC/workflows/MinimalBuild/badge.svg?branch=minimal)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![arduino-library-badge](https://www.ardu-badge.com/badge/Simple%20FOC.svg?)](https://www.ardu-badge.com/badge/Simple%20FOC.svg)

This is the minimal Arduino example of the [Simple FOC](https://github.com/askuric/Arduino-FOC) arduino library intended for mostly for easier experimentation and modification!

### Minimal repository structure
```shell
├───arduino_foc_minimal_encoder    # Arduino minimal code for running a motor with Encoder
│
└───arduino_foc_minimal_magnetic   # Arduino minimal code for running a motor with magnetic sensor 
                                   # AS5048/47   
```

Each of the examples will give you the opportunity to change the PI velocity parameters `P` and `I`, Low pass filter time constant `Tf`, change the control loop in real time and check the average loop execution time, all from the serial terminal. 

List of commands:
- **P**: velocity PI controller P gain
- **I**: velocity PI controller I gain
- **L**: velocity PI controller voltage limit
- **R**: velocity PI controller voltage ramp
- **F**: velocity Low pass filter time constant
- **K**: angle P controller P gain
- **N**: angle P controller velocity limit
- **C**: control loop 
  - **0**: voltage 
  - **1**: velocity 
  - **2**: angle
- **V**: get motor variables
  - **0**: currently set voltage
  - **1**: current velocity
  - **2**: current angle
  - **3**: current target value


Find more information about the **motor commands** in the [docs.simplefoc.com](https://docs.simplefoc.com/communication)

###  Installation
For those willing to experiment and to modify the code I suggest using the [minimal version](https://github.com/askuric/Arduino-FOC/tree/minimal) of the code. 
 > This code is completely independent and you can run it as any other Arduino Sketch without the need for any libraries. 

#### Github website download
- Make sure you are in [minimal branch](https://github.com/askuric/Arduino-FOC/tree/minimal) 
- Download the code by clicking on the `Clone or Download > Download ZIP`.
- Unzip it and open the sketch in Arduino IDE. 

#### Using terminal
- Open the terminal:
  ```sh
  cd *to you desired directory*
  git clone -b minimal https://github.com/askuric/Arduino-FOC.git
  ```
- Then you just open it with the Arduino IDE and run it.

## Documentation
Find out more information about the Arduino SimpleFOC project in [docs website](https://askuric.github.io/Arduino-FOC/) 


## Arduino FOC repo structure
Branch  | Description | Status
------------ | ------------- | ------------ 
[master](https://github.com/askuric/Arduino-FOC) | Stable and tested library version | ![Library Compile](https://github.com/askuric/Arduino-FOC/workflows/Library%20Compile/badge.svg)
[dev](https://github.com/askuric/Arduino-FOC/tree/dev) | Development library version | ![Library Dev Compile](https://github.com/askuric/Arduino-FOC/workflows/Library%20Dev%20Compile/badge.svg?branch=dev)
[minimal](https://github.com/askuric/Arduino-FOC/tree/minimal) | Minimal Arduino example with integrated library | ![MinimalBuild](https://github.com/askuric/Arduino-FOC/workflows/MinimalBuild/badge.svg?branch=minimal)
