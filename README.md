# Arduino Simple Field Oriented Control (FOC) library 


![Library Compile](https://github.com/askuric/Arduino-FOC/workflows/Library%20Compile/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![arduino-library-badge](https://www.ardu-badge.com/badge/Simple%20FOC.svg?)](https://www.ardu-badge.com/badge/Simple%20FOC.svg)

Proper low-cost and low-power FOC supporting boards are very hard to find these days and even may not exist. Even harder to find is a stable and simple FOC algorithm code capable of running on Arduino devices. 
Therefore this is an attempt to: 
- Demystify FOC algorithm and make a robust but simple Arduino library: [Arduino SimpleFOC library](https://askuric.github.io/Arduino-FOC/arduino_simplefoc_library_showcase)
- Develop a modular BLDC driver board: [Arduino SimpleFOC shield](https://askuric.github.io/Arduino-FOC/arduino_simplefoc_shield_showcase).

## Arduino SimpleFOCShield

<p align="">
<a href="https://youtu.be/G5pbo0C6ujE">
<img src="https://askuric.github.io/Arduino-FOC/extras/Images/foc_shield_video.jpg"  height="320px">
</a>
</p>

### Features
- **Plug & play**: In combination with Arduino <span class="simple">Simple<span class="foc">FOC</span>library</span> 
- **Low-cost**: Price of €20 - [Check the pricing](https://askuric.github.io/simplefoc_shield_product) 
- **Open Source**: Fully available [Gerber files and BOM](https://askuric.github.io/Arduino-FOC/arduino_simplefoc_shield_fabrication)
- **Stackable**: running 2 motors in the same time
- **Encoder interface**: Integrated 3kOhm pullups (configurable)
- **Configurable pinout**: Hardware configuration - soldering connections

##### If you are interested in this board, preorder your version on this link: [Arduino Simple FOC Shield](https://askuric.github.io/simplefoc_shield_product)

<p align=""><img src="https://askuric.github.io/Arduino-FOC/extras/Images/shield_to_v13.jpg" height="180px">   <img src="https://askuric.github.io/Arduino-FOC/extras/Images/shield_bo_v13.jpg"  height="180px"> <img src="https://askuric.github.io/Arduino-FOC/extras/Images/simple_foc_shield_v13_small.gif"  height="180x"></p>


## Arduino SimpleFOClibrary

<p align="">
<a href="https://youtu.be/N_fRYf7Z80k">
<img src="https://askuric.github.io/Arduino-FOC/extras/Images/youtube.png"  height="320px">
</a>
</p>

This video demonstrates the Simple FOC library basic usage, electronic connections and shows its capabilities.

### Features
- **Arduino compatible**: Arduino library code
- **Easy to setup and configure**: 
  - Easy hardware configuration
  - Easy [tuning the control loops](https://askuric.github.io/Arduino-FOC/control_loops)
- **Modular**:
  - Supports as many [sensors ,  BLDC motors  and  driver boards](https://askuric.github.io/Arduino-FOC/electrical_connections) as possible
  - Supports as many application requirements as possible
- **Plug & play**: Arduino SimpleFOC shield

<p align=""> <img src="https://askuric.github.io/Arduino-FOC/extras/Images/uno_l6234.jpg"  height="170px">  <img src="https://askuric.github.io/Arduino-FOC/extras/Images/hmbgc_v22.jpg" height="170px">  <img src="https://askuric.github.io/Arduino-FOC/extras/Images/foc_shield_v12.jpg"  height="170px"></p>

## Getting Started
Depending on if you want to use this library as the plug and play Arduino library or you want to get insight in the algorithm and make changes there are two ways to install this code.

- Full library installation [Docs](https://askuric.github.io/Arduino-FOC/library_download)
- Minimal code installation [Docs](https://askuric.github.io/Arduino-FOC/minimal_download)

### Arduino SimpleFOC library installation to Arduino IDE
#### Arduino Library Manager 
The simplest way to get hold of the library is directly by using Arduino IDE and its integrated Library Manager. 
- Open Arduino IDE and start Arduino Library Manager by clicking: `Tools > Manage Libraries...`.
- Search for `Simple FOC` library and install the latest version.
- Reopen Arduino IDE and you should have the library examples in `File > Examples > Simple FOC`.

### Using Github website 
- Go to the [github repository](https://github.com/askuric/Arduino-FOC)
- Click first on `Clone or Download > Download ZIP`. 
- Unzip it and place it in `Arduino Libraries` folder. Windows: `Documents > Arduino > libraries`.  
- Reopen Arduino IDE and you should have the library examples in `File > Examples > Simple FOC`.

### Using terminal
- Open terminal and run
```sh  
cd *arduino libraries folder*
git clone https://github.com/askuric/Arduino-FOC.git
```
- Reopen Arduino IDE and you should have the library examples in `File > Examples > Simple FOC`.

###  SimpleFOC library minimal sketch example

For those willing to experiment and to modify the code I suggest using the [minimal version](https://github.com/askuric/Arduino-FOC/tree/minimal) of the code. 
 > This code is completely independent and you can run it as any other Arduino Sketch without the need for any libraries. 

#### Github website download
- Go to [minimal branch](https://github.com/askuric/Arduino-FOC/tree/minimal) 
- Download the code by clicking on the `Clone or Download > Download ZIP`.
- Unzip it and open the sketch in Arduino IDE. 

#### Using terminal
- Open the terminal:
  ```sh
  cd *to you desired directory*
  git clone -b minimal https://github.com/askuric/Arduino-FOC.git
  ```
- Then you just open it with the Arduino IDE and run it.

## Arduino code example
This is a simple Arduino code example implementing the velocity control program of a BLDC motor with encoder. 

NOTE: This program uses all the default control parameters.

```cpp
#include <SimpleFOC.h>

//  BLDCMotor( pin_pwmA, pin_pwmB, pin_pwmC, pole_pairs, enable (optional))
BLDCMotor motor = BLDCMotor(9, 10, 11, 11, 8);
//  Encoder(pin_A, pin_B, CPR)
Encoder encoder = Encoder(arduinoInt1, arduinoInt2, 2048);
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}


void setup() {  
  // initialize encoder hardware
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);

  // set control loop type to be used
  motor.controller = ControlType::velocity;
  
  // use debugging with the BLDCMotor
  Serial.begin(115200);
  // debugging port
  motor.useDebugging(Serial);

  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();
}

void loop() {
  // FOC algorithm function
  motor.loopFOC();

  // velocity control loop function
  // setting the target velocity or 2rad/s
  motor.move(2);

  // debugging function outputting motor variables to the serial terminal 
  motor.monitor();
}
```
You can find more details in the [SimpleFOC documentation](https://askuric.github.io/Arduino-FOC/).

## Example projects
Here are some of the SimpleFOC application examples. 
### Arduino Field Oriented Controlled Reaction Wheel Inverted Pendulum
This is a very cool open-source project of one of the simplest setups of the Reaction wheel inverted pendulum. Check out all the components and projects notes in the [github repository](https://github.com/askuric/Arduino-FOC-reaction-wheel-inverted-pendulum).  
<p align="">
<a href="https://youtu.be/Ih-izQyXJCI">
<img src="https://askuric.github.io/Arduino-FOC/extras/Images/youtube_pendulum.png"  height="320px">
</a>
</p>

**The main benefits of using the BLDC motor in this project are:**
- High torque to weight ratio
  - The lighter the better
- Lots of torque for low angular velocities
  - No need to spin the motor to very high PRM to achieve high torques
- No gearboxes and backlash
  - Very smooth operation = very stable pendulum


## Documentation
Find out more information about the Arduino SimpleFOC project in [docs website](https://askuric.github.io/Arduino-FOC/) 


## Arduino FOC repo structure
Branch  | Description | Status
------------ | ------------- | ------------ 
[master](https://github.com/askuric/Arduino-FOC) | Stable and tested library version | ![Library Compile](https://github.com/askuric/Arduino-FOC/workflows/Library%20Compile/badge.svg)
[dev](https://github.com/askuric/Arduino-FOC/tree/dev) | Development library version | ![Library Dev Compile](https://github.com/askuric/Arduino-FOC/workflows/Library%20Dev%20Compile/badge.svg?branch=dev)
[minimal](https://github.com/askuric/Arduino-FOC/tree/minimal) | Minimal Arduino example with integrated library | ![MinimalBuild](https://github.com/askuric/Arduino-FOC/workflows/MinimalBuild/badge.svg?branch=minimal)
