# Arduino Simple Field Oriented Control (FOC) library 


![Library Compile](https://github.com/simplefoc/Arduino-FOC/workflows/Library%20Compile/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![arduino-library-badge](https://www.ardu-badge.com/badge/Simple%20FOC.svg?)](https://www.ardu-badge.com/badge/Simple%20FOC.svg)

We live in very exciting times 😃! BLDC motors are entering the hobby community more and more and many great projects have already emerged leveraging their far superior dynamics and power capabilities. BLDC motors have numerous advantages over regular DC motors but they have one big disadvantage, the complexity of control. Even though it has become relatively easy to design and manufacture PCBs and create our own hardware solutions for driving BLDC motors the proper low-cost solutions are yet to come. One of the reasons for this is the apparent complexity of writing the BLDC driving algorithms, Field oriented control (FOC) being an example of one of the most efficient ones.
The solutions that can be found on-line are almost exclusively very specific for certain hardware configuration and the microcontroller architecture used.
Additionally, most of the efforts at this moment are still channeled towards the high-power applications of the BLDC motors and proper low-cost and low-power FOC supporting boards are very hard to find today and even may not exist. <br>
Therefore this is an attempt to: 
- 🎯 Demystify FOC algorithm and make a robust but simple Arduino library: [Arduino *SimpleFOClibrary*](https://docs.simplefoc.com/arduino_simplefoc_library_showcase)
  - <i>Support as many <b>motor + sensor + driver + mcu</b> combinations out there</i>
- 🎯 Develop a modular FOC supporting BLDC driver boards:
   - *Low-power* gimbal driver (<5Amps) :  [*Arduino Simple**FOC**Shield*](https://docs.simplefoc.com/arduino_simplefoc_shield_showcase).
   - ***NEW*** 📢: *Medium-power* BLDC driver (<30Amps): [Arduino <span class="simple">Simple<b>FOC</b>PowerShield</span> ](https://github.com/simplefoc/Arduino-SimpleFOC-PowerShield).
   - See also [@byDagor](https://github.com/byDagor)'s *fully-integrated* ESP32 based board: [Dagor Brushless Controller](https://github.com/byDagor/Dagor-Brushless-Controller)

##### <b> NEW RELEASE 📢:</b> <i>Simple<b>FOC</b>library v2.1
> - **Initial current sensing support**🎉
>   - Inline current sensors 
>   - adaptive zero finding and shunt direction
> - **Implemented real torque control** 
>   - using voltage
>   - using current magnitude (one current)
>   - using FOC currents ( d-q currents ) - real foc control
> - SVPWM full implementation  d+q axis
> - **Simplified sensor implementation**📢
>    - For new sensor implementation only one function necessary `getAngle()`
> - Upgrade of the HallSensor implementation by [@owennewo](https://github.com/owennewo)
> - Support for Arduino DUE - everything except the 6PWM mode
> - Support for ATMega328pb
> - bugfix for the Teensy boards ( setting 3pwm )
> - extended support for 2PWM stepper drivers - by [@zjor](https://github.com/zjor)
> - included F macro for shrinking string memory usage - moved to programming memory
> - disable phase support for 3pwm driver
>    - not yet for 6pwm
> - rewritten `initFOC()`
>    - can be skipped and outputs much more info
>    - align sensor: direction + zero offset + pole pair check
>    - align current sense
> - sensor offset supported (`motor.sensor_offset`)
> - **refactored motor commands interface**
>   - much more flexible and easy to extend
>   - very easy to add new commands and function callbacks
>   - implemented motor+pid+lpf commands of-the-shelf
> - Added **step/dir interface**
>   - integrated as an optional communication channel
>
> BEWARE 📢 slight API changes included
>   - `ControlType` renamed into `MotionControlType`
>   - `ControlType::voltage` does not exist any more now - `MotionControlType::torque`


## Arduino *SimpleFOCShield* v2.0.3
<p align="">
<a href="https://youtu.be/G5pbo0C6ujE">
<img src="https://docs.simplefoc.com/extras/Images/foc_shield_video.jpg"  height="320px">
</a>
</p>

### Features
- **Plug & play**: In combination with Arduino *Simple**FOC**library* - [github](https://github.com/simplefoc/Arduino-FOC)
- **Low-cost**: Price of €15 - [Check the pricing](https://www.simplefoc.com/shop) 
- **In-line current sensing**: Up to 3Amps/5Amps bidirectional
   - configurable: 3.3Amps - 3.3V adc, 5Amps - 5V adc
- **Integrated 8V regulator**: 
   - Enable/disable by soldering pads
- **Max power 120W** - max current 5A, power-supply 12-24V
   - Designed for Gimbal motors with the internal resistance >10 Ωs. 
- **Stackable**: running 2 motors in the same time
- **Encoder/Hall sensors interface**: Integrated 3.3kΩ pullups (configurable)
- **I2C interface**: Integrated 4.7kΩ pullups (configurable)
- **Configurable pinout**: Hardware configuration - soldering connections
- **Arduino headers**: Arduino UNO, Arduino MEGA, STM32 Nucleo boards...
- **Open Source**: Fully available fabrication files - [how to make it yourself](https://docs.simplefoc.com/arduino_simplefoc_shield_fabrication)

<p align=""><img src="https://simplefoc.com/assets/img/v2.jpg" height="180px">   <img src="https://simplefoc.com/assets/img/v1.jpg"  height="180px"> <img src="https://docs.simplefoc.com/extras/Images/simple_foc_shield_v13_small.gif"  height="180x"></p>


## Arduino *SimpleFOClibrary* v2.1

<p align="">
<a href="https://youtu.be/Y5kLeqTc6Zk">
<img src="https://docs.simplefoc.com/extras/Images/youtube.png"  height="320px">
</a>
</p>

This video demonstrates the *Simple**FOC**library* basic usage, electronic connections and shows its capabilities.


### Features
- **Arduino compatible**: 
   - Arduino library code
  - Arduino Library Manager integration
- **Open-Source**: Full code and documentation available on github
- **Easy to setup and configure**: 
  - Easy hardware configuration
  - Easy [tuning the control loops](https://docs.simplefoc.com/motion_control)
- **Modular**:
  - Supports as many [sensors,  BLDC motors  and  driver boards](https://docs.simplefoc.com/supported_hardware) as possible
  - Supports multiple [MCU architectures](https://docs.simplefoc.com/microcontrollers):
     - Arduino: UNO, MEGA, any board with ATMega328 chips
     - STM32 boards: [Nucleo](https://www.st.com/en/evaluation-tools/stm32-nucleo-boards.html), [Bluepill](https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill.html) ...
     - ESP32
     - Teensy boards
- **Plug & play**: Arduino <span class="simple">Simple<span class="foc">FOC</span>Shield</span>  

<p align=""> <img src="https://docs.simplefoc.com/extras/Images/uno_l6234.jpg"  height="170px">  <img src="https://docs.simplefoc.com/extras/Images/hmbgc_v22.jpg" height="170px">  <img src="https://docs.simplefoc.com/extras/Images/foc_shield_v13.jpg"  height="170px"></p>

## Getting Started
Depending on if you want to use this library as the plug and play Arduino library or you want to get insight in the algorithm and make changes there are two ways to install this code.

- Full library installation [Docs](https://docs.simplefoc.com/library_download)
- Minimal project builder [Docs](https://docs.simplefoc.com/minimal_download)

### Arduino *SimpleFOClibrary* installation to Arduino IDE
#### Arduino Library Manager 
The simplest way to get hold of the library is directly by using Arduino IDE and its integrated Library Manager. 
- Open Arduino IDE and start Arduino Library Manager by clicking: `Tools > Manage Libraries...`.
- Search for `Simple FOC` library and install the latest version.
- Reopen Arduino IDE and you should have the library examples in `File > Examples > Simple FOC`.

#### Using Github website 
- Go to the [github repository](https://github.com/simplefoc/Arduino-FOC)
- Click first on `Clone or Download > Download ZIP`. 
- Unzip it and place it in `Arduino Libraries` folder. Windows: `Documents > Arduino > libraries`.  
- Reopen Arduino IDE and you should have the library examples in `File > Examples > Simple FOC`.

#### Using terminal
- Open terminal and run
```sh  
cd #Arduino libraries folder
git clone https://github.com/simplefoc/Arduino-FOC.git
```
- Reopen Arduino IDE and you should have the library examples in `File > Examples > Simple FOC`.

###  *SimpleFOClibrary* minimal project builder

For those willing to experiment and to modify the code I suggest using the minimal project builder [minimal branch](https://github.com/simplefoc/Arduino-FOC/tree/minimal). 
 > This code is completely independent and you can run it as any other Arduino Sketch without the need for any libraries. 

All you need to do is:
- Go to [minimal branch](https://github.com/simplefoc/Arduino-FOC/tree/minimal) 
- Follow the tutorial in the README file and choose only the library files that are necessary for your application.

## Arduino code example
This is a simple Arduino code example implementing the velocity control program of a BLDC motor with encoder. 

NOTE: This program uses all the default control parameters.

```cpp
#include <SimpleFOC.h>

//  BLDCMotor( pole_pairs )
BLDCMotor motor = BLDCMotor(11);
//  BLDCDriver( pin_pwmA, pin_pwmB, pin_pwmC, enable (optional) )
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
//  Encoder(pin_A, pin_B, CPR)
Encoder encoder = Encoder(2, 3, 2048);
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}


void setup() {  
  // initialize encoder hardware
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);
  
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // initialise driver hardware
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // set control loop type to be used
  motor.controller = MotionControlType::velocity;
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
}
```
You can find more details in the [SimpleFOC documentation](https://docs.simplefoc.com/).

## Example projects
Here are some of the *Simple**FOC**library* and *Simple**FOC**Shield* application examples. 
<p align="center">
<a href="https://youtu.be/Ih-izQyXJCI">
<img src="https://docs.simplefoc.com/extras/Images/youtube_pendulum.png"  height="200px" >
</a>
<a href="https://youtu.be/xTlv1rPEqv4">
<img src="https://docs.simplefoc.com/extras/Images/youtube_haptic.png"  height="200px" >
</a>
<a href="https://youtu.be/RI4nNMF608I">
<img src="https://docs.simplefoc.com/extras/Images/youtube_drv8302.png"  height="200px" >
</a>
<a href="https://youtu.be/zcb86TRxTxc">
<img src="https://docs.simplefoc.com/extras/Images/youtube_stepper.png"  height="200px" >
</a>
</p>


## Documentation
Find out more information about the Arduino SimpleFOC project in [docs website](https://docs.simplefoc.com/) 


## Arduino FOC repo structure
Branch  | Description | Status
------------ | ------------- | ------------ 
[master](https://github.com/simplefoc/Arduino-FOC) | Stable and tested library version | ![Library Compile](https://github.com/simplefoc/Arduino-FOC/workflows/Library%20Compile/badge.svg)
[dev](https://github.com/simplefoc/Arduino-FOC/tree/dev) | Development library version | ![Library Dev Compile](https://github.com/simplefoc/Arduino-FOC/workflows/Library%20Dev%20Compile/badge.svg?branch=dev)
[minimal](https://github.com/simplefoc/Arduino-FOC/tree/minimal) | Minimal Arduino example with integrated library | ![MinimalBuild](https://github.com/simplefoc/Arduino-FOC/workflows/MinimalBuild/badge.svg?branch=minimal)
