# Arduino Simple FOC library minimal example 

![MinimalBuild](https://github.com/askuric/Arduino-FOC/workflows/MinimalBuild/badge.svg?branch=minimal)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![arduino-library-badge](https://www.ardu-badge.com/badge/Simple%20FOC.svg?)](https://www.ardu-badge.com/badge/Simple%20FOC.svg)

This is the minimal Arduino example of the [Simple FOC](https://github.com/askuric/Arduino-FOC) arduino library intended for mostly for easier experiemtation and modification!

## Arduino FOC repo structure
Branch  | Description | Status
------------ | ------------- | ------------ 
[master](https://github.com/askuric/Arduino-FOC) | Stable and tested library version | ![Library Compile](https://github.com/askuric/Arduino-FOC/workflows/Library%20Compile/badge.svg)
[dev](https://github.com/askuric/Arduino-FOC/tree/dev) | Developement library version | ![Library Dev Compile](https://github.com/askuric/Arduino-FOC/workflows/Library%20Dev%20Compile/badge.svg?branch=dev)
[minimal](https://github.com/askuric/Arduino-FOC/tree/minimal) | Minimal Arduino example with integrated library | ![MinimalBuild](https://github.com/askuric/Arduino-FOC/workflows/MinimalBuild/badge.svg?branch=minimal)

 # Contents
 - [Installation](#arduino-simple-foc-instalation)
  - [Installing the full library](#installing-simple-foc-full-library)
  - [Installing the minimal Arduino example](#download-minimal-simple-foc-arduino-example)
- [Electrical connecitons and schematic](#electrical-connections)
  - [Minimal setup](#all-you-need-for-this-project-is-an-exaple-in-brackets)
  - [Arduino Simple FOC Shield V1.2](#arduino-simple-foc-shield-v12)
  - [Arduino UNO + L6234 driver](#arduino-uno--l6234-breakout-broad)
  - [HMBGC gimbal contorller example](#hmbgc-v22)
- [Code explanation and examples](#arduino-simple-foc-library-code)
  - [Encoder setup](#encoder-setup)
  - [BLDC motor setup](#motor-setup)
  - [Control loop setup](#control-loop-setup)
    - [Voltage control loop](#voltage-control-loop)
    - [Velcoity control loop](#velocity-control-loop)
    - [Angle control loop](#angle-control-loop)
    - [Utra Slow Velocity control loop](#ultra-slow-velocity-control-loop)
  - [Debugging practice](#debugging)
  - [Future work and work in progress](#future-work-roadmap)
- [Contact](#contact)


# Arduino Simple FOC instalation
Depending on if you want to use this library as the plug and play Arduino library or you want to get insight in the algorithm and make changes simply there are two ways to install this code.

## Installing Simple FOC full library
### Arduino IDE - Library manager
The simplest way to get hold of the library is direclty by using Arduino IDE and its integrated Library Manager. Just serarch for `Simple FOC` library and install the lates version.

### Download library directly
If you don't want to use the Arduino IDE and Library manager you can direclty download the library from this website. 
- Simplest way to do it is to download the `zip` aerchieve directly on the top of this webiste. Click first on `Clone or Download` and then on `Download ZIP`. Once you have the zip ardhieve downloaded, unzip it and place it in your Arduino Libraries forlder. On Windows it is usually in `Documents > Arduino > libraries`.  
  - Now reopen your Arduino IDE and you should have the library examples in `File > Examples > Simple FOC`.

- If you are more experienced with the terminal you can open your terminal in the Arduino libraries folder direclty and clone the Arduino FOC git repsitory:
```bash
git clone https://github.com/askuric/Arduino-FOC.git
```
  - Now reopen your Arduino IDE and you should have the library examples in `File > Examples > Simple FOC`.

## Download minimal Simple FOC Arduino example
To download the minmial verison of Simple FOC intended for those willing to experiment and extend the code I suggest using this version over the full library. 
This code is completely indepenedet and you can run it as any other Arduino Schetch without the need for any libraries. 
The code is place in the [minimal branch](https://github.com/askuric/Arduino-FOC/tree/minimal). 

- You can download it directly form the [minimal branch](https://github.com/askuric/Arduino-FOC/tree/minimal) by clicking on the `Clone or Download > Download ZIP`.
  - Then you just unzip it and open the schetch in Arduino IDE. 

- You can also clone it using the terminal:
  ```bash
  git clone -b minimal https://github.com/askuric/Arduino-FOC.git
  ```
  - Then you just open it with the Arduino IDE and run it.



# Electrical connections

### All you need for this project is (an exaple in brackets):
 - Brushless DC motor - 3 pahse    (IPower GBM4198H-120T [Ebay](https://www.ebay.com/itm/iPower-Gimbal-Brushless-Motor-GBM4108H-120T-for-5N-7N-GH2-ILDC-Aerial-photo-FPV/252025852824?hash=item3aade95398:g:q94AAOSwPcVVo571:rk:2:pf:1&frcectupt=true))
 - Encoder  
   - Incremental 2400cpr [Ebay](https://www.ebay.com/itm/600P-R-Photoelectric-Incremental-Rotary-Encoder-5V-24V-AB-2-Phases-Shaft-6mm-New/173145939999?epid=19011022356&hash=item28504d601f:g:PZsAAOSwdx1aKQU-:rk:1:pf:1)
   - Magnetic 14bit [Aliexpress](https://fr.aliexpress.com/item/4000034013999.html?spm=a2g0o.productlist.0.0.4a7f5c25mYwpN3&algo_pvid=8f452506-7081-4d0a-8f66-d0b725d6de66&algo_expid=8f452506-7081-4d0a-8f66-d0b725d6de66-0&btsid=0b0a0ad815873142372227604ed134&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_)
- BLDC motor driver 
  -  L6234 driver [Drotek](https://store-drotek.com/212-brushless-gimbal-controller-l6234.html), [Ebay](https://www.ebay.fr/itm/L6234-Breakout-Board-/153204519965)
  -  Alternatively the library supports the arduino based gimbal controllers such as: HMBGC V2.2 ([Ebay](https://www.ebay.com/itm/HMBGC-V2-0-3-Axle-Gimbal-Controller-Control-Plate-Board-Module-with-Sensor/351497840990?hash=item51d6e7695e:g:BAsAAOSw0QFXBxrZ:rk:1:pf:1))


## Arduino Simple FOC Shield V1.2

At this moment we are developing an open source version of Arduin shiled specifically for FOC motor control. 
We already have prototypes of the board and we are in the testing phase. We will be coming out with the details very soon!

### Features
- Plug and play capability with the Arduino Simple FOC library
- Price in the range of \$20-\$40
- Gerber files and BOM available Open Source
  
***Let me know if you are interested! antun.skuric@outlook.com***
You can explore the [3D model of the board in the PDF form](https://raw.githubusercontent.com/askuric/Arduino-FOC/master/extras/ArduinoFOCShieldV12.pdf).

<img src="https://raw.githubusercontent.com/askuric/Arduino-FOC/master/extras/Images/AFSV11_side.png" height="300px">  <img src="https://raw.githubusercontent.com/askuric/Arduino-FOC/master/extras/Images/AFSV11_top.png" height="200px">   <img src="https://raw.githubusercontent.com/askuric/Arduino-FOC/master/extras/Images/AFSV11_bottom.png" height="200px">


## Arduino UNO + L6234 breakout broad
The code is simple enough to be run on Arudino Uno board. 

<p>
 <img src="https://raw.githubusercontent.com/askuric/Arduino-FOC/master/extras/Images/arduino_connection.png" height="">
</p>  

### Encoder
- Encoder channels `A` and `B` are connected to the Arduino's external intrrupt pins `2` and `3`. 
- Optionally if your encoder has `index` signal you can connect it to any available pin, figure shows pin `4`.
  - If you can choose preferably connect it to an `A0-A5` due to the interrupt rutine, it will have better performance (but any other pin will work as well).  
### L6234 breakout board 
- Connected to the arduino pins `9`,`10` and `11` (you can use also pins `5` and `6`).  
- Additionally you can connect the `enable` pin to the any digital pin of the arduino the picture shows pin `8` but this is optional. You can connect the driver enable directly to 5v. 
- Make sure you connect the common ground of the power supply and your Arduino
### Motor
- Motor phases `a`, `b` and `c` are connected directly to the driver outputs

Motor phases `a`,`b`,`c` and encoder channels `A` and `B` have to be oriented right for the algorightm to work. But don't worry about it too much. Connect it in initialy as you wish and then if it doesnt move reverse pahse `a` and `b` of the motor, that should be enogh.


## HMBGC V2.2
To use HMBGC controller for vector control (FOC) you need to connect motor to one of the motor terminals and connect the Encoder. The shema of connection is shown on the figures above, I also took a (very bad) picture of my setup.

<p>
	<img src="https://raw.githubusercontent.com/askuric/Arduino-FOC/master/extras/Images/hmbgc_connection.png" height="">
	<img src="https://raw.githubusercontent.com/askuric/Arduino-FOC/master/extras/Images/setup1.jpg" height="400px">
</p>
 
 
### Encoder
Since HMBGC doesn't have acces to the arduinos external interrupt pins `2` and `3` and additionally we only have acces to the analog pins, we need to read the encoder using the software interrupt. To show the functionallity we provide one example of the HMBGC code (`HMBGC_example.ino`) using the [PciManager library](https://github.com/prampec/arduino-pcimanager).

- Encoder channels `A` and `B` are connected to the pins `A0` and `A1`.
- Optionally if your encoder has `index` signal you can connect it to any available pin, figure shows pin `A2`.  
### Motor
- Motor phases `a`,`b` and `c` are connected directly to the driver outputs

Motor phases `a`,`b`,`c` and encoder channels `A` and `B` have to be oriented right for the algorightm to work. But don't worry about it too much. Connect it in initialy as you wish and then if it doesnt move reverse pahse `a` and `b` of the motor, that should be enogh.



# Arduino Simple FOC library code
The code is organised into a library. The library contains two classes `BLDCmotor` and `Endcoder`. `BLDCmotor` contains all the necessary FOC algorithm funcitons as well as PI controllers for the velocity and angle control.  `Encoder`  deals with the encoder interupt funcitons, calcualtes motor angle and velocity ( using the [Mixed Time Frequency Method](https://github.com/askuric/Arduino-Mixed-Time-Frequency-Method)). The `Encoder` class will support any type of otpical and magnetic encoder.

## Encoder setup
To initialise the encoder you need to provide the encoder `A` and `B` channel pins, encoder `PPR` and optionally `index` pin.
```cpp
//  Encoder(int encA, int encB , int cpr, int index)
//  - encA, encB    - encoder A and B pins
//  - ppr           - impulses per rotation  (cpr=ppr*4)
//  - index pin     - (optional input)
Encoder encoder = Encoder(2, 3, 8192, A0);
```
Next important feature of the encoder is enabling or disabling the `Quadrature` more. If the Encoder is run in the quadratue more its number of impulses per rotation(`PPR`) is quadrupled by detecting each `CHANGE` of the signals `A` and `B` - `CPR = 4xPPR`. In some applicaitons, when the encoder `PPR` is high it can be too much for the Arudino to handle so it is preferable not to use `Quadrature` mode. By default all the encoders use `Quadrature` mode. If you would like to enable or disable this paramter do it in the Arduino setup funciton by running:
```cpp
// check if you need internal pullups
//  Quadrature::ENABLE - CPR = 4xPPR  - default
//  Quadrature::DISABLE - CPR = PPR
encoder.quadrature = Quadrature::ENABLE;
```
Additionally the encoder has one more important parameters which is whether you want to use Arduino's internal pullup or you have external one. That is set by changing the value of the `encoder.pullup` variuable. The default value is set to `Pullup::EXTERN`
```cpp
// check if you need internal pullups
// Pullup::EXTERN - external pullup added  - dafault
// Pullup::INTERN - needs internal arduino pullup
encoder.pullup = Pullup::EXTERN;
```
### Encoder interrupt configuration
There are two ways you can run encoders with Simple FOC libtrary.
- Using Arduino hardware external interrupt - for Arduino UNO pins  `2` and `3` 
- Using software pin chnage interrupt by using a library such as [PciManager library](https://github.com/prampec/arduino-pcimanager)

> Using the hardware external interrupts usualy results in a bit better and more realible performance but software interrupts will work very good as well. 

#### Arduino Hardware external interrupt
Arduino hadrware external interrupt pins are pin `2` and `3`. And in order to use its functionallities the encoder channels `A` and `B` will have to be connected exacly on these pins.

Simple FOC `Encoder` class already has implemented initialisation and encoder `A` and `B` channel callbacks. 
All you need to do is define two funcitons `doA()` and `doB()`, the buffering functions of encoder callback funcitons `encoder.handleA()` and `encoder.handleB()`. 
```cpp
// interrupt ruotine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
```
And supply those functions to the encoder initialiasation fucntion `encoder.init()`
```cpp
// initialise encoder hardware
encoder.init(doA, doB);
```
You can name the buffering funcitons as you wish. It is just important to supply them to the `encoder.init()` funciton. This procedure is a tradeoff in between scalability and simplicity. This allows you to have more than one encoder connected to the same arduino. All you need to do is to instantiate new `Encoder` class and create new buffer functions. For example:
```cpp
// encoder 1
Encoder enc1 =  Encoder(...);
void doA1(){enc1.handleA();}
void doB1(){enc1.handleB();}
// encoder 2
Encoder enc2 =  Encoder(...);
void doA2(){enc2.handleA();}
void doB2(){enc2.handleB();}

void setup(){
...
	enc1.init(doA1,doB1);
	enc2.init(doA2,doB2);
...
}
```

##### Index pin configuration
In order to read index pin efficienlty Simple FOC algorithm uses Arduino interrupt routine as well. 
Simple FOC has implemented the index pin callback  `encoder.handleIndex()`. In order to enable the index pin utilisation just add the index pin in the encoder constructor:
```cpp
Encoder encoder = Encoder(pinA, pinB, cpr, index_pin);
```
The library make sure to enable the propper interrupts and initialise the `pinMode` based on your [pullup configuration](#encoder-setup).

The last peace of the index pin enabling puzzle is the Arduino's `ISR` funciton call:
```cpp
// index calback interrupt code 
// please set the right PCINT(0,1,2)_vect parameter
//  PCINT0_vect - index pin in between D8 and D13
//  PCINT1_vect - index pin in between A0 and A5 (recommended)
//  PCINT2_vect - index pin in between D0 and D7
ISR (PCINT1_vect) { encoder.handleIndex(); }
```
Make sure you use the right `PCINT(1,2,3)_vect` paramereter of the `ISR` function. 
`ISR` Parameter | Arduino pin 
-----| -----
PCINT0_vect | D8 - D13
PCINT1_vect | A0 - A5
PCINT2_vect | D0 - D7

The `ISR` with the vector `PCINT0_vect` will be called each time one of the pins `D8-D13` changes. Each time one of the pins `A0-A5` chenges the Arduino interrupt `ISR` function with the `PCINT1_vect` will be called and finally if any one of the `D0-D7` pins changes the `ISR` function with the parameter `PCINT2_vect` will be called. So therefore it is important to use the index callback `encoder.handleIndex()` funciton in the rigth `ISR` function. 
Here is an eaxmple of the setup od two encoders with different `index` locations:
```cpp
Encoder encoder =  Encoder(2,3,600,A0);
// A and B interrupt rutine 
void doA1(){encoder.handleA();}
void doB1(){encoder.handleB();}
// index  interrupt rutine 
ISR (PCINT1_vect) { encoder.handleIndex(); }

void setup(){...}
void loop(){...}
```
Or for example:
```cpp
Encoder encoder =  Encoder(2,3,600,13);
// A and B interrupt rutine 
void doA1(){encoder.handleA();}
void doB1(){encoder.handleB();}
// index  interrupt rutine 
ISR (PCINT0_vect) { encoder.handleIndex(); }

void setup(){...}
void loop(){...}
```
To explore better the encoder algorithm an example is provided `encoder_example.ino`.

#### Arduino software pin change interrupt
If you are not able to access your pins `2` and `3` of your Arduino or if you want to use more than none encoder you will have to use the software interrupt approach. 
I suggest using the [PciManager library](https://github.com/prampec/arduino-pcimanager).

The stps of using this library in code are very similar to [harware interrupt](#arduino-hardware-external-interrupt).
The SimpleFOC `Encoder` class still provides you with all the callbacks `A`, `B` and `Index` channels but the Simple FOC library will not initialise the interrupts for you. 

In order to use the `PCIManager` library you will need to include it in your code:
```cpp
#include <PciManager.h>
#include <PciListenerImp.h>
```
Next step is the same as before, you will just intialise the new `Encoder` instance.
```cpp
Encoder encoder = Encoder(10, 11, 8192);
// A and B interrupt callback buffers
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
```
Then you declare listeners `PciListenerImp `:
```cpp
// encoder interrupt init
PciListenerImp listenerA(encoder.pinA, doA);
PciListenerImp listenerB(encoder.pinB, doB);
```
And finally instead of supplying the functions `doA` and `doB` to the `encoder.init()` you call the init without parameters and  use `PCIManager` library to register the interrupts
```cpp
// initialise encoder hardware
encoder.init();
// interrupt intitialisation
PciManager.registerListener(&listenerA);
PciManager.registerListener(&listenerB);
```
And that is it, it is very simple. It if you wnat more than one encoder, you just initialise the new class instance, create the new `A` and `B` callbacks, intialise the new listeners. Here is a quick example:
```cpp
// encoder 1
Encoder enc1 =  Encoder(9, 10, 8192);
void doA1(){enc1.handleA();}
void doB1(){enc1.handleB();}
PciListenerImp listA1(enc1.pinA, doA1);
PciListenerImp listB1(enc1.pinB, doB1);

// encoder 2
Encoder enc2 =  Encoder(13, 12, 8192);
void doA2(){enc2.handleA();}
void doB2(){enc2.handleB();}
PciListenerImp listA2(enc2.pinA, doA2);
PciListenerImp listB2(enc2.pinB, doB2);

void setup(){
...
  // encoder 1
  enc1.init();
  PciManager.registerListener(&listA1);
  PciManager.registerListener(&listB1);
  // encoder 2
  enc2.init();
  PciManager.registerListener(&listA2);
  PciManager.registerListener(&listB2);
...
}
```
You can look into the `HMBGC_example.ino` ecxample to see this code in action. 
##### Index pin configuration
Enabling index pin in the case of the software interrupt is very simple. You just need to provide it to the `Encoder` class intialisation as additional parameter. 
```cpp
Encoder encoder = Encoder(pinA, pinB, cpr, index_pin);
```
Afterward you create the same type of callback buffering function as for `A` and `B` channels and using the `PCIManager` tools initialise and register the listener for the `index` channel as for the `A` and `B`. Here is a quick example:
xample:
```cpp
// class init
Encoder encoder =  Encoder(9, 10, 8192,11);
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doIndex(){encoder.handleIndex();}
// listeners init
PciListenerImp listenerA(encoder.pinA, doA);
PciListenerImp listenerB(encoder.pinB, doB);
PciListenerImp listenerIndex(encoder.index_pin, doIndex);

void setup(){
...
  // enable the hardware
  enc1.init();
  // enable interrupt
  PciManager.registerListener(&listenerA);
  PciManager.registerListener(&listenerB);
  PciManager.registerListener(&listenerIndex);
...
}
```

## Motor setup
To intialise the motor you need to input the `pwm` pins, number of `pole pairs` and optionally driver `enable` pin.
```cpp
//  BLDCMotor( int phA, int phB, int phC, int pp, int en)
//  - phA, phB, phC - motor A,B,C phase pwm pins
//  - pp            - pole pair number
//  - enable pin    - (optional input)
BLDCMotor motor = BLDCMotor(9, 10, 11, 11, 8);
```
If you are not sure what your `pole_paris` number is I included an example code to estimate your `pole_paris` number in the examples `find_pole_pairs_number.ino`. I hope it helps. 

To finalise the motor setup the encoder is added to the motor and the `init` function is called.
```cpp
// link the motor to the sensor
motor.linkEncoder(&encoder);
// intialise motor
motor.init();
```


### Power supply voltage
The default  `voltage_power_supply`  value is set to `12V`. If you set your power supply to some other vlaue, chnage it here for the control loops to adapt.
```cpp
// power supply voltage
motor.voltage_power_supply = 12;
```
The `voltage_power_supply` value tells the FOC algorithm what is the maximum voltage it can output. Additioanlly since the FOC algotihm implemented in the Simple FOC library uses sinusoidal voltages the magnitudes of the sine waves exiting the Drvier circuit is going to be  `[-voltage_power_supply/2, voltage_power_supply/2]`.

<img src="https://raw.githubusercontent.com/askuric/Arduino-FOC/master/extras/Images/sine_foc.png" >

## Control loop setup
The SimpleFOC library gives you the choice of using 4 different plug and play control loops: 
- voltage control loop
- velocity control loop
- angle control loop
- ultra slow velocity control loop

You set it by changing the `motor.controller` variable. If you want to control the motor angle you will set the `controller` to `ControlType::angle`, if you seek the DC motor behavior behaviour by controlling the voltage use `ControlType::voltage`, if you wish to control motor angular velocity `ControlType::velocity`. If you wish to control velocities which are very very slow, typically around ~0.01 rad/s you can use the `ControlType::velocity_ultra_slow` controller.
```cpp
// set FOC loop to be used
// ControlType::voltage
// ControlType::velocity
// ControlType::velocity_ultra_slow
// ControlType::angle
motor.controller = ControlType::angle;
```
### Voltage control loop
This control loop allows you to run the BLDC motor as it is simple DC motor using Park transformation. This mode is enabled by:
```cpp
// voltage control loop
motor.controller = ControlType::voltage;
```
 <a name="foc_image"></a><img src="https://raw.githubusercontent.com/askuric/Arduino-FOC/master/extras/Images/voltage.png">

You rcan test this algoithm by running the example `voltage_control.ino`.
The FOC algorithm reads the angle <i>a</i> from the motor and sets appropriate <i>u<sub>a</sub></i>, <i>u<sub>b</sub></i> and <i>u<sub>c</sub></i> voltages such to always have <i>90 degree</i> angle in between the magnetic fields of the permanent magents in rotor and the stator. What is exaclty the principle of the DC motor.
> This control loop will give you the motor which spins freely with the velocity depending on the voltage $U_q$ you set and the disturbance it is facing. *It will turn slower if you try to hold it*.


### Velocity control loop
This control loop allows you to spin your BLDC motor with desired velocity.  This mode is enabled by:
```cpp
// velocity control loop
motor.controller = ControlType::velocity;
```

<img src="https://raw.githubusercontent.com/askuric/Arduino-FOC/master/extras/Images/velocity.png" >

You can test this algorithm by running the example `velocity_control.ino`.
The velocity control is created by adding a PI velocity controller. This controller reads the motor velocity <i>v</i> and sets the <i>u<sub>q</sub></i> voltage to the motor in a such maner that it reaches and maintains the target velocity <i>v<sub>d</sub></i>, set by the user. 
#### PI controller parameters
To change the parameters of your PI controller to reach desired behaiour you can change `motor.PI_velocity` structure:
```cpp
// velocity PI controller parameters
// default K=0.5 Ti = 0.01
motor.PI_velocity.K = 0.2;
motor.PI_velocity.Ti = 0.01;
motor.PI_velocity.voltage_limit = 6; 
// jerk control using voltage voltage ramp
// default value is 300 volts per sec  ~ 0.3V per millisecond
motor.PI_velocity.voltage_ramp = 6;
```
The parameters of the PI controller are proportional gain `K`, integral time constant `Ti`, voltage limit `voltage_limit`  and `voltage_ramp`. 
- The `voltage_limit` parameter is intended if, for some reason, you wish to limit the voltage that can be sent to your motor.  
- In general by raising the proportional constant `K`  your motor controller will be more reactive, but too much will make it unstable. 
- The same goes for integral time constant `Ti` the smaller it is the faster motors reaction to disturbance will be, but too small value will make it unstable. 
- The `voltage_ramp` value it intended to reduce the maximal change of the voltage value which is sent to the motor. The higher the value the PI controller will be able to change faster the <i>U<sib>q</sub></i> value. The lower the value the smaller the possible change and the less responsive your controller becomes. The value of this parameter is set to be `Volts per second[V/s` or in other words how many volts can your controller raise the voltage in one time unit. If you set your `voltage_ramp` value to `10 V/s`, and on average your contol loop will run each `1ms`. Your controller will be able to chnage the <i>U<sib>q</sub></i> value each time `10[V/s]*0.001[s] = 0.01V` waht is not a lot.


So in order to get optimal performance you will have to fiddle a bit with with the parameters. :)

### Angle control loop
This control loop allows you to move your BLDC motor to the desired angle in real time.   This mode is enabled by:
```cpp
// angle control loop
motor.controller = ControlType::angle;
```

<img src="https://raw.githubusercontent.com/askuric/Arduino-FOC/master/extras/Images/position.png">

You can test this algorithm by running the example `angle_control.ino`.
The angle control loop is done by adding one more control loop in cascade on the velocity control loop like showed on the figure above. The loop is closed by using simple P controller. The controller reads the angle <i>a</i> from the motor and determins which velocity <i>v<sub>d</sub></i> the motor should move to reach desire angle <i>a<sub>d</sub></i> set by the user. And then the velocity controller reads the current velocity from the motor <i>v</i> and sets the voltage <i>u<sub>q</sub></i> that is neaded to reach the velocity <i>v<sub>d</sub></i>, set by the angle loop. 

#### Controller parameters
To tune this control loop you can set the parameters to both angle P controller and velocity PI controller. 
```cpp
// velocity PI controller parameters
// default K=1.0 Ti = 0.003
motor.PI_velocity.K = 0.5;
motor.PI_velocity.Ti = 0.01;
motor.PI_velocity.voltage_limit = 6;
// jerk control using voltage voltage ramp
// default value is 300 volts per sec  ~ 0.3V per millisecond
motor.PI_velocity.voltage_ramp = 6;
// angle P controller 
// default K=70
motor.P_angle.K = 20;
//  maximal velocity of the poisition control
// default 20
motor.P_angle.velocity_limit = 10;
```
It is important to paramter both velocity PI and angle P controller to have the optimal performance.
The velocity PI controller is parametrisized by updating the `motor.PI_velcity` structure as expalined before. 
- Rough rule should be to lower the proportional gain `K` and raise time constant `Ti` in order to achieve less vibrations.
  
The angle P controller can be updated by changign the `motor.P_angle` structure. 
- Roughly proportional gain `K` will make it more responsive, but too high value will make it unstable.
  
Additionally you can configure the `velocity_limit` value of the controller. This value prevents the contorller to set too high velocities $v_d$ to the motor. 
- If you make your `velocity_limit` very low your motor will be moving in between desired positions with exactly this velocity. If you keep it high, you will not notice that this variable even exists. :D  

Finally, each application is a bit different and the chances are you will have to tune the controller values a bit to reach desired behaviour.


### Ultra slow velocity control loop
This control loop allows you to spin your BLDC motor with desired velocity as well as the [velocity loop](#velocity-control-loop) but it is intended for very smooth operation in very low velocityes (< 0.1 rad/s).  This mode is enabled by:
```cpp
// velocity ultra slow control loop
motor.controller = ControlType::velocity_ultra_slow;
```

<img src="https://raw.githubusercontent.com/askuric/Arduino-FOC/master/extras/Images/velocity_ultraslow_loop.png" >

You can test this algorithm by running the example `velocity_ultrasloaw_control_serial.ino` .
This type of the velocity control is nothing more but motor angle control. It works particularly well for the purposes of very slow movements because regular velocity calculation techniques are not vel suited for this application and regular [velocity control loop](#velocity-control-loop) would not work well. 
The behavior is achieved by integrating the user set target velocity <i>v<sub>d</sub></i> to get the necessary angle <i>a<sub>d</sub></i>. And then controlling the motor angle <i>v</i> with high-gain PI controller. This controller reads the motor angle <i>v</i> and sets the <i>u<sub>q</sub></i> voltage to the motor in a such maner that it closely follows the target angle <i>a<sub>d</sub></i>, to achieve the velocity profile <i>v<sub>d</sub></i>, set by the user. 
#### PI controller parameters
To change the parameters of your PI controller to reach desired behaiour you can change `motor.PI_velocity` structure:
```cpp
// velocity PI controller parameters
  // default K=120.0 Ti = 100.0
motor.PI_velocity_ultra_slow.K = 120;
motor.PI_velocity_ultra_slow.Ti = 100;
motor.PI_velocity_ultra_slow.voltage_limit = 12;
// jerk control using voltage voltage ramp
// default value is 300 volts per sec  ~ 0.3V per millisecond
motor.PI_velocity_ultra_slow.voltage_ramp = 6;
```
The parameters of the PI controller are proportional gain `K`, integral time constant `Ti` and voltage limit `voltage_limit` which is by default set to the `voltage_power_supply/2` and `voltage_ramp`. 
- The `voltage_limit` parameter is intended if some reason you wish to limit the voltage that can be sent to your motor.  
- In general by raising the proportional constant `K`  your motor controller will be more reactive, but too much will make it unstable. 
- The same goes for integral time constant `Ti` the smaller it is the faster motors reaction to disturbance will be, but too small value will make it unstable. By defaualt the integral time constant `Ti` is set  `100s`. Which means that it is extreamply slow, meaning that it is not effecting the behvior of the controlle, making it basically a P controller.
- The `voltage_ramp` value it intended to reduce the maximal change of the voltage value which is sent to the motor. The higher the value the PI controller will be able to change faster the <i>U<sib>q</sub></i> value. The lower the value the smaller the possible change and the less responsive your controller becomes. The value of this parameter is set to be `Volts per second[V/s` or in other words how many volts can your controller raise the voltage in one time unit. If you set your `voltage_ramp` value to `10 V/s`, and on average your contol loop will run each `1ms`. Your controller will be able to chnage the <i>U<sib>q</sub></i> value each time `10[V/s]*0.001[s] = 0.01V` waht is not a lot.

From the PI controller parameters you can see that the values are much higher than in the [velocity control loop](#velocity-control-loop). The reason is because the angle control loop is not the main loop and we need it to follow the profile as good as possible as fast as possible. Therefore we need much higher gain than before.

### Index search routine
Finding the encoder index is performed only if the constructor of the `Encoder` class has been provided with the `index` pin. The search is performed by setting a constant velocity of the motor until it reaches the index pin. To set the desired searching velocity alter the paramterer:
```cpp
// index search velocity - default 1rad/s
motor.index_search_velocity = 2;
```

This velocity control loop is implemented exaclty the same as [velocity control loop](#velocity-control-loop) but it has different contorller paramters which can be set by:
```cpp
// index search PI contoller parameters
// default K=0.5 Ti = 0.01
motor.PI_velocity_index_search.K = 0.1;
motor.PI_velocity_index_search.Ti = 0.01;
motor.PI_velocity_index_search.voltage_limit = 3;
// jerk control using voltage voltage ramp
// default value is 300 volts per sec  ~ 0.3V per millisecond
motor.PI_velocity_index_search.voltage_ramp = 6;
```
If you are having problems during the finding procedure, try tuning the PI controller constants. The same parameters as the `PI_velocity` should work well, but you can put it a bit more conservative to avoid high jumps.


## FOC routine 
### Intialisation - `setup()`
After the motor and encoder are intialised and the driver and control loops are configured you intialise the FOC algorithm. 
```cpp
// align encoder and start FOC
motor.initFOC();
```
This function aligns encoder and motor zero positions and intialises FOC variables. It is intended to be run in the `setup` function of the Arudino. After the call of this funciton FOC is ready to start following your instructions.

### Real-time execution `loop()`

The real time execution of the Arduino Simple FOC library is govenred by two funcitons `motor.loopFOC()` and `motor.move(float target)`.
```cpp
// iterative setting FOC pahse voltage
// the faster you run this funciton the better
// in arduino loop it should have ~1kHz
// the best would be to be in ~10kHz range
motor.loopFOC();
```
The funciton `loopFOC()` gets the current motor angle from the encoder, turns in into the electrical angle and computes Clarke transfrom to set the desired <i>U<sub>q</sub></i> voltage to the motor. Basically it implements the funcitonality of the [voltage control loop](#voltage-control-loop).
- The faster you can run this funciton the better 
- In the empty arduino loop it runs at ~1kHz but idealy it would be around ~10kHz


```cpp
// iterative function setting the outter loop target
// velocity, position or voltage
// this funciton can be run at much lower frequency than loopFOC funciton
// it can go as low as ~50Hz
motor.move(target);
```
The `move()` method executes the control loops of the algorihtm. If is governed by the `motor.controller` variable. It executes eigther pure voltage loop, velocity loop, angle loop or ultra slow velocity loop.

It receives one parameter `BLDCMotor::move(float target)` which is current user define target value.
- If the user runs [velocity loop](#velocity-control-loop), `move` funciton will interpret `target` as the target velocity <i>v<sub>d</sub></i>.
- If the user runs [angle loop](#angle-control-loop), `move` will interpret `target` parameter as the target angle<i>a<sub>d</sub></i>. 
- If the user runs the [voltage loop](#voltage-control-loop), `move` funciton will interpret the `target` parameter as voltage <i>u<sub>d</sub></i>.


> At this point because we are oriented to simplicity we did not implement synchornious version of this code. Uing timer interrupt. The main reason for the moment is that Arduino UNO doesn't have enough timers to run it. 
> *But in future we are planning to include this functionality.*

## Examples
Examples folder structure
```
├───examples
│   ├───voltage_control                       # example of the voltage control loop with configuraiton
│   ├───angle_control                         # example of angle control loop with configuraiton
│   ├───velocity_control                      # example of velocity control loop with configuraiton
│   ├───velocity_ultraslow_control            # example of ultra slow velocity control using  with configuraiton
│   ├───encoder_example                       # simple example of encoder usage 
│   ├───minimal_example                       # example of code without using configuration
│   ├───HMBGC_example                         # example of code to be used with HMBGC controller with
│   └───find_pole_pairs_number                # simple code example estimating pole pair number of the motor
```


# Debugging
To debug control loop exection in the examples we added a funciton `motor_monitor()` which log the motor variables to the serial port. The funciton logs different variables based for differenc control loops.
```cpp
// utility function intended to be used with serial plotter to monitor motor variables
// significantly slowing the execution down!!!!
void motor_monitor() {
  switch (motor.controller) {
    case ControlType::velocity_ultra_slow:
    case ControlType::velocity:
      Serial.print(motor.voltage_q);
      Serial.print("\t");
      Serial.print(motor.shaft_velocity_sp);
      Serial.print("\t");
      Serial.println(motor.shaft_velocity);
      break;
    case ControlType::angle:
      Serial.print(motor.voltage_q);
      Serial.print("\t");
      Serial.print(motor.shaft_angle_sp);
      Serial.print("\t");
      Serial.println(motor.shaft_angle);
      break;
    case ControlType::voltage:
      Serial.print(motor.voltage_q);
      Serial.print("\t");
      Serial.println(motor.shaft_velocity);
      break;
  }
}
```
This is just a template funciton to help you debug and create your own functions in future.
The funciton accesses the motor variables:
```cpp

class BLDCMotor
{
  public:
  ...
    // current elelctrical angle
    float elctric_angle;
    // current motor angle
    float shaft_angle;
    // current motor velocity 
    float shaft_velocity;
    // current target velocity
    float shaft_velocity_sp;
    // current target angle
    float shaft_angle_sp;
    // current voltage u_q set
    float voltage_q;
...
}
```
Additionally it is possible to use encoder api directly to get the encoder angle and velocity. 
```cpp

class Encoder{
 public:
    // shaft velocity getter
    float getVelocity();
	// shaft angle getter
    float getAngle();
}
```
### BLDCMotor debugging
Additonally a `BLDCMotor` calss now supports debugging using `Serial` port which is enabled by:
```cpp
motor.useDebugging(Serial);
```
before running `motor.init()`.


# Work Roadmap
## Future work
- [ ] Proper introduction of the **Arudino FOC Shield V1.2**
- [ ] Publish a video tutorial fir using the library and the samples  

## Work in progress
- [x] Make the library accesible in the Arduino Library Manager 
- [x] Make minimal version of the arduino code - all in one arduino file
- [x] Encoder index proper implementation
- [x] Enable more dirver types 
- [x] Make support for magnetic encoder AS5048 and similar
- [x] Add support for acceleration ramping
- [x] Timer interrupt execution rather than in the `loop()`
  - FAIL: Perfromance not improved

# Contact
Please do not hesitate to leave an issue or contact me direclty by email.
I will be very happy to hear your experiences.
antun.skuric@outlook.com