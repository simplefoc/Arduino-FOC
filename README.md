# Arduino Simple FOC library minimal example 

![MinimalBuild](https://github.com/askuric/Arduino-FOC/workflows/MinimalBuild/badge.svg?branch=minimal)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![arduino-library-badge](https://www.ardu-badge.com/badge/Simple%20FOC.svg?)](https://www.ardu-badge.com/badge/Simple%20FOC.svg)

This is the minimal Arduino example of the [Simple FOC](https://github.com/askuric/Arduino-FOC) arduino library intended for mostly for easier experiemtation and modification!

### Minimal repository structure
```shell
├───arduino_foc_minimal_encoder    # Arduino minimal code for running a motor with Encoder
│
└───arduino_foc_minimal_magnetic   # Arduino minimal code for running a motor with magentic sensor 
                                   # AS5048/47   
```

Each of the examples will give you the opportunity to change the PI velocity parameters `P` and `I`, Low pass filter time constant `Tf`, change the control loop in real time and check the average loop execution time, all from the serial terminal.
```
PI controller parameters change:
- P value : Prefix P (ex. P0.1)
- I value : Prefix I (ex. I0.1)

Velocity filter:
- Tf value : Prefix F (ex. F0.001)

Average loop execution time:
- Type T

Control loop type:
- C0 - angle control
- C1 - velocity control
- C2 - voltage control

Initial parameters:
PI velocity P: 0.20,	 I: 20.00,	 Low passs filter Tf: 0.0000
```

###  Installation
For those willing to experiment and to modify the code I suggest using the [minimal version](https://github.com/askuric/Arduino-FOC/tree/minimal) of the code. 
 > This code is completely indepenedet and you can run it as any other Arduino Schetch without the need for any libraries. 

#### Github webiste downlaod
- Make sure you are in [minimal branch](https://github.com/askuric/Arduino-FOC/tree/minimal) 
- Downlaod the code by clicking on the `Clone or Download > Download ZIP`.
- Unzip it and open the schetch in Arduino IDE. 

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
[dev](https://github.com/askuric/Arduino-FOC/tree/dev) | Developement library version | ![Library Dev Compile](https://github.com/askuric/Arduino-FOC/workflows/Library%20Dev%20Compile/badge.svg?branch=dev)
[minimal](https://github.com/askuric/Arduino-FOC/tree/minimal) | Minimal Arduino example with integrated library | ![MinimalBuild](https://github.com/askuric/Arduino-FOC/workflows/MinimalBuild/badge.svg?branch=minimal)
