# Arduino Field Oriented Control (FOC) library


![Library Compile](https://github.com/askuric/Arduino-FOC/workflows/Library%20Compile/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)


Proper low cost FOC supporting boards are very hard to find these days and even may not exist. The reason may be that the hobby community has not yet dug into it properly. Therefore this is the attempt to demistify the Field Oriented Control (FOC) algorithm and make a robust but simple implementation for usage with Arduino hadrware.

### This project aims to close the gap in the areas:
- Low cost applications <50$
- Low current operation < 5A
- Simple usage and scalability (Arduino)
 and demistify FOC control in a simple way. 


#### The closest you can get to FOC support and low cost (I was able to find) is:

<a href="https://odriverobotics.com/" >Odroid</a> | <a href="https://www.youtube.com/watch?v=g2BHEdvW9bU">Trinamic</a>
------------ | -------------
<img src="https://static1.squarespace.com/static/58aff26de4fcb53b5efd2f02/t/5c2c766921c67c143049cbd3/1546417803031/?format=1200w" width="400px"> | <img src="http://i3.ytimg.com/vi/g2BHEdvW9bU/maxresdefault.jpg" width="400px">
:heavy_check_mark: Open Source | :x: Open Source
:heavy_check_mark:Simple to use | :heavy_check_mark: Simple to use
:x: Low cost ($100) | :x: Low cost ($100)
:x: Low power (>50A) | :heavy_check_mark: Low power 

<a href="https://www.infineon.com/cms/en/product/evaluation-boards/bldc_shield_tle9879/" >Infineon</a> | <a href="https://github.com/gouldpa/FOC-Arduino-Brushless">FOC-Arduino-Brushless</a>
------------ | -------------
<img src="https://www.infineon.com/export/sites/default/_images/product/evaluation-boards/BLDC_Motor_Shild_with_TLE9879QXA40.jpg_1711722916.jpg" height="300px" width="400px">| <img src="https://hackster.imgix.net/uploads/attachments/998086/dev_kit_89eygMekks.jpg?auto=compress%2Cformat&w=1280&h=960&fit=max" width="400px">
:x: Open Source | :heavy_check_mark: Open Source
:heavy_check_mark:Simple to use | :x: Simple to use
:heavy_check_mark:Low cost ($40) | :heavy_check_mark: Low cost
:heavy_check_mark:  Low power | :heavy_check_mark: Low power



# Electrical connections

### All you need for this project is (an exaple in brackets):
 - Brushless DC motor - 3 pahse    (IPower GBM4198H-120T [Ebay](https://www.ebay.com/itm/iPower-Gimbal-Brushless-Motor-GBM4108H-120T-for-5N-7N-GH2-ILDC-Aerial-photo-FPV/252025852824?hash=item3aade95398:g:q94AAOSwPcVVo571:rk:2:pf:1&frcectupt=true))
 - Encoder - ( Incremental 2400cpr [Ebay](https://www.ebay.com/itm/600P-R-Photoelectric-Incremental-Rotary-Encoder-5V-24V-AB-2-Phases-Shaft-6mm-New/173145939999?epid=19011022356&hash=item28504d601f:g:PZsAAOSwdx1aKQU-:rk:1:pf:1))
- Arduino + BLDC motor driver ( L6234 driver [Drotek](https://store-drotek.com/212-brushless-gimbal-controller-l6234.html), [Ebay](https://www.ebay.fr/itm/L6234-Breakout-Board-/153204519965))

Alternatively the library supports the arduino based gimbal controllers such as:
- HMBGC V2.2 ([Ebay](https://www.ebay.com/itm/HMBGC-V2-0-3-Axle-Gimbal-Controller-Control-Plate-Board-Module-with-Sensor/351497840990?hash=item51d6e7695e:g:BAsAAOSw0QFXBxrZ:rk:1:pf:1))


## Arduino FOC Shield V1.2

At this moment we are developing an open source version of Arduin shiled specifically for FOC motor control. 
We already have prototypes of the board and we are in the testing phase. We will be coming out with the details very soon!

### Features
- Plug and play capability with the Arduino FOC library
- Price in the range of \$20-\$40
- Gerber files and BOM available Open Source
  
***Let me know if you are interested! antun.skuric@outlook.com***
You can explore the [3D model of the board in the PDF form](extras/ArduinoFOCShieldV12.pdf).

<img src="extras/Images/AFSV11_side.png" height="300px">  <img src="extras/Images/AFSV11_top.png" height="200px">   <img src="extras/Images/AFSV11_bottom.png" height="200px">


## Arduino UNO + L6234 breakout broad
The code is simple enough to be run on Arudino Uno board. 

<p>
 <img src="extras/Images/arduino_connection.png" height="">
</p>  

### Encoder
- Encoder channels `A` and `B` are connected to the Arduino's external intrrupt pins `2` and `3`. 
- Optionally if your encoder has `index` signal you can connect it to any available pin, figure shows pin `4`.  
		- The library doesnt support the Index pin for now (version v1.1.0)
### L6234 breakout board 
- Connected to the arduino pins `9`,`10` and `11`. 
- Additionally you can connect the `enable` pin to the any digital pin of the arduino the picture shows pin `8` but this is optional. You can connect the driver enable directly to 5v. 
- Make sure you connect the common ground of the power supply and your Arduino
### Motor
- Motor phases `a`, `b` and `c` are connected directly to the driver outputs

Motor phases `a`,`b`,`c` and encoder channels `A` and `B` have to be oriented right for the algorightm to work. But don't worry about it too much. Connect it in initialy as you wish and then if it doesnt move reverse pahse `a` and `b` of the motor, that should be enogh.


## HMBGC V2.2
To use HMBGC controller for vector control (FOC) you need to connect motor to one of the motor terminals and connect the Encoder. The shema of connection is shown on the figures above, I also took a (very bad) picture of my setup.

<p>
	<img src="extras/Images/hmbgc_connection.png" height="">
	<img src="extras/Images/setup1.jpg" height="400px">
</p>
 
 
### Encoder
Since HMBGC doesn't have acces to the arduinos external interrupt pins `2` and `3` and additionally we only have acces to the analog pins, we need to read the encoder using the software interrupt. To show the functionallity we provide one example of the HMBGC code (`HMBGC_example.ino`) using the [PciManager library](https://github.com/prampec/arduino-pcimanager).

- Encoder channels `A` and `B` are connected to the pins `A0` and `A1`.
- Optionally if your encoder has `index` signal you can connect it to any available pin, figure shows pin `A2`.  
		- The library doesnt support the Index pin for now (version v1.1.0)
### Motor
- Motor phases `a`,`b` and `c` are connected directly to the driver outputs

Motor phases `a`,`b`,`c` and encoder channels `A` and `B` have to be oriented right for the algorightm to work. But don't worry about it too much. Connect it in initialy as you wish and then if it doesnt move reverse pahse `a` and `b` of the motor, that should be enogh.




# Arduino FOC library code
The code is organised into a librarie. The library contains two classes `BLDCmotor` and `Endcoder`. `BLDCmotor` contains all the necessary FOC algorithm funcitons as well as PI controllers for the velocity and angle control.  `Encoder`  deals with the encoder interupt funcitons, calcualtes motor angle and velocity( using the [Mixed Time Frequency Method](https://github.com/askuric/Arduino-Mixed-Time-Frequency-Method)).

## Encoder setup
To initialise the encoder you need to provide the encoder `A` and `B` channel pins, encoder `PPR` and optionally `index` pin.
```cpp
//  Encoder(int encA, int encB , int cpr, int index)
//  - encA, encB    - encoder A and B pins
//  - ppr           - impulses per rotation  (cpr=ppr*4)
//  - index pin     - (optional input)
Encoder encoder = Encoder(2, 3, 8192, 4);
```
Additionally the encoder has one more important parameters which is whether you want to use Arduino's internal pullup or you have external one. That is set by changing the value of the `encoder.pullup` variuable. The default value is set to `Pullup::EXTERN`
```cpp
// check if you need internal pullups
// Pullup::EXTERN - external pullup added  - dafault
// Pullup::INTERN - needs internal arduino pullup
encoder.pullup = Pullup::EXTERN;
```
Finally to start the encoder counter and intialise all the periphery pins you need the call of `encoder.init()` is made.
```cpp
// initialise encoder hardware
encoder.init(doA, doB);
```
Where the functions `doA()` and `doB()` are buffering functions of encoder callback funcitons `encoder.handleA()` and `encoder.handleB()`. 
```cpp
// interrupt ruotine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
```
You can name the funcitons as you wish. It is just important to supply them to the `encoder.init()` funciton. This procedure is a tradeoff in between scalability and simplicity. This allows you to have more than one encoder connected to the same arduino. All you need to do is to instantiate new `Encoder` class and create new buffer functions. For example:
```cpp
// encoder 1
Encoder enc1 =  Encoder(2, 3, 8192, 4);
void doA1(){enc1.handleA();}
void doB1(){enc1.handleB();}
// encoder 2
Encoder enc2 =  Encoder(5, 6, 8192, 7);
void doA2(){enc2.handleA();}
void doB2(){enc2.handleB();}

void setup(){
...
	enc1.init(doA1,doB1);
	enc2.init(doA2,doB2);
...
}
```

To explore better the encoder algorithm an example is provided `encoder_example.ino`.

## Motor setup
To intialise the motor you need to input the `pwm` pins, number of `pole pairs` and optionally driver `enable` pin.
```cpp
//  BLDCMotor( int phA, int phB, int phC, int pp, int en)
//  - phA, phB, phC - motor A,B,C phase pwm pins
//  - pp            - pole pair number
//  - enable pin    - (optional input)
BLDCMotor motor = BLDCMotor(9, 10, 11, 11, 8);
```
To finalise the motor setup the encoder is added to the motor and the `init` function is called.
```cpp
// link the motor to the sensor
motor.linkEncoder(&encoder);
// intialise motor
motor.init();
```


## BLDC Driver parameters
First thing you can configure is your `power_supply_voltage` value. The default value is set to `12V`. If you set your power supply to some other vlaue, chnage it here for the control loops to adapt.
```cpp
// power supply voltage
motor.power_supply_voltage = 12;
```
You can also change driver type by changing the value of the variable `motor.driver`. It tells the algorithm to generate unipolar of bipolar FOC voltages. This basically means if your BLDC driver can only output voltages in range `[0,power_supply_voltage]` your driver is `DriverType::unipolar` and if it can output voltage in range `[-power_supply_voltage, power_supply_voltage]` than you driver is `DriverType::bipolar` what is case in most of the drivers and what is default value as well.
```cpp
// set driver type
//  DriverType::unipolar   // HMBGC
//  DriverType::bipolar    // L6234 (default)
motor.driver = DriverType::bipolar;
```
## Control loop setup
First parameter you can change is the variable you want to control. You set it by changing the `motor.controller` variable. If you want to control the motor angle you will set the `controller` to `ControlType::angle`, if youy seek the DC motor behavior behaviour by controlling the voltage use `ControlType::voltage`, if you wish to control motor angular velocity `ControlType::velocity`. If you wish to control velocities which are very very slow, typically around ~0.01 rad/s you can use the `ControlType::velocity_ultra_slow` controller.
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
 <a name="foc_image"></a><img src="extras/Images/voltage.png">

You rcan test this algoithm by running the example `voltage_control.ino`.
The FOC algorithm reads the angle $\textsf{a}$ from the motor and sets appropriate $\textsf{u}_a$, $\textsf{u}_b$ and $\textsf{u}_c$ voltages such to always have $90\degree$ angle in between the magnetic fields of the permanent magents in rotor and the stator. What is exaclty the principle of the DC motor.
> This control loop will give you the motor which spins freely with the velocity depending on the voltage $U_q$ you set and the disturbance it is facing. *It will turn slower if you try to hold it*.


### Velocity control loop
This control loop allows you to spin your BLDC motor with desired velocity.  This mode is enabled by:
```cpp
// velocity control loop
motor.controller = ControlType::velocity;
```

<img src="extras/Images/velocity.png" >

You can test this algorithm by running the example `velocity_control.ino` and `velocity_control_serial.ino` .
The velocity control is created by adding a PI velocity controller. This controller reads the motor velocity $\textsf{v}$ and sets the $\textsf{u}_q$ voltage to the motor in a such maner that it reaches and maintains the target velocity $\textsf{v}_d$, set by the user. 
#### PI controller parameters
To change the parameters of your PI controller to reach desired behaiour you can change `motor.PI_velocity` structure:
```cpp
// velocity PI controller parameters
// default K=1.0 Ti = 0.003
motor.PI_velocity.K = 1;
motor.PI_velocity.Ti = 0.003;
motor.PI_velocity.u_limit = 12;
```
The parameters of the PI controller are proportional gain `K`, integral time constant `Ti` and voltage limit `u_limit` which is by default set to the `power_supply_voltage`. 
- The `u_limit` parameter is intended if some reason you wish to limit the voltage that can be sent to your motor.  
- In general by raising the proportional constant `K`  your motor controller will be more reactive, but too much will make it unstable. 
- The same goes for integral time constant `Ti` the smaller it is the faster motors reaction to disturbance will be, but too small value will make it unstable. 

So in order to get optimal performance you will have to fiddle a bit with with the parameters. :)

### Angle control loop
This control loop allows you to move your BLDC motor to the desired angle in real time.   This mode is enabled by:
```cpp
// angle control loop
motor.controller = ControlType::angle;
```

<img src="extras/Images/position.png">

You can test this algorithm by running the example `angle_control.ino` and `angle_control_serial.ino` .
The angle control loop is done by adding one more control loop in cascade on the velocity control loop like showed on the figure above. The loop is closed by using simple P controller. The controller reads the angle $\textsf{a}$ from the motor and determins which velocity $\textsf{v}_d$ the motor should move to reach desire angle $\textsf{a}_d$ set by the user. And then the velocity controller reads the current velocity from the motor $\textsf{v}$ and sets the voltage $\textsf{u}_q$ that is neaded to reach the velocity $\textsf{v}_d$, set by the angle loop. 

#### Controller parameters
To tune this control loop you can set the parameters to both angle P controller and velocity PI controller. 
```cpp
// velocity PI controller parameters
// default K=1.0 Ti = 0.003
motor.PI_velocity.K = 0.5;
motor.PI_velocity.Ti = 0.01;
motor.PI_velocity.u_limit = 12;
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

<img src="extras/Images/velocity_ultraslow_loop.png" >

You can test this algorithm by running the example `velocity_ultrasloaw_control_serial.ino` .
This type of the velocity control is nothing more but motor angle control. It works particularly well for the purposes of very slow movements because regular velocity calculation techniques are not vel suited for this application and regular [velocity control loop](#velocity-control-loop) would not work well. 
The behavior is achieved by integrating the user set target velocity $\textsf{v}_d$ to get the necessary angle $\textsf{a}_d$. And then controlling the motor angle $\textsf{a}$ with high-gain PI controller. This controller reads the motor angle $\textsf{a}$ and sets the $\textsf{u}_q$ voltage to the motor in a such maner that it closely follows the target angle $\textsf{a}_d$, to achieve the velocity profile $\textsf{v}_d$, set by the user. 
#### PI controller parameters
To change the parameters of your PI controller to reach desired behaiour you can change `motor.PI_velocity` structure:
```cpp
// velocity PI controller parameters
  // default K=120.0 Ti = 100.0
motor.PI_velocity_ultra_slow.K = 120;
motor.PI_velocity_ultra_slow.Ti = 100;
motor.PI_velocity_ultra_slow.u_limit = 12;
```
The parameters of the PI controller are proportional gain `K`, integral time constant `Ti` and voltage limit `u_limit` which is by default set to the `power_supply_voltage`. 
- The `u_limit` parameter is intended if some reason you wish to limit the voltage that can be sent to your motor.  
- In general by raising the proportional constant `K`  your motor controller will be more reactive, but too much will make it unstable. 
- The same goes for integral time constant `Ti` the smaller it is the faster motors reaction to disturbance will be, but too small value will make it unstable. By defaualt the integral time constant `Ti` is set  `100s`. Which means that it is extreamply slow, meaning that it is not effecting the behvior of the controlle, making it basically a P controller.

From the PI controller parameters you can see that the values are much higher than in the [velocity control loop](#velocity-control-loop). The reason is because the angle control loop is not the main loop and we need it to follow the profile as good as possible as fast as possible. Therefore we need much higher gain than before.

## FOC routine 
### Intialisation - `setup()`
After the motor and encoder are intialised and the driver and control loops are configured you intialise the FOC algorithm. 
```cpp
// align encoder and start FOC
motor.initFOC();
```
This function aligns encoder and motor zero positions and intialises FOC variables. It is intended to be run in the `setup` function of the Arudino. After the call of this funciton FOC is ready to start following your instructions.

### Real-time execution `loop()`

The real time execution of the Arduino FOC library is govenred by two funcitons `motor.loopFOC()` and `motor.move(float target)`.
```cpp
// iterative state calculation calculating angle
// and setting FOC pahse voltage
// the faster you run this funciton the better
// in arduino loop it should have ~1kHz
// the best would be to be in ~10kHz range
motor.loopFOC();
```
The funciton `loopFOC()` gets the current motor angle from the encoder, turns in into the electrical angle and computes Clarke transfrom to set the desired $U_q$ voltage to the motor. Basically it implements the funcitonality of the [velocity control loop](#voltage-control-loop).
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
- If the user runs [velocity loop](#velocity-control-loop), `move` funciton will interpret `target` as the target velocity $\textsf{v}_d$.
- If the user runs [angle loop](#angle-control-loop), `move` will interpret `target` parameter as the target angle $\textsf{a}_d$. 
- If the user runs the [voltage loop](#voltage-control-loop), `move` funciton will interpret the `target` parameter as voltage $\textbf{u}_q$.


> At this point because we are oriented to simplicity we did not implement synchornious version of this code. Uing timer interrupt. The main reason for the moment is that Arduino UNO doesn't have enough timers to run it. 
> *But in future we are planning to include this functionality.*

## Examples
Examples folder structure
```
├───examples
│   ├───voltage_control                       # example of the voltage control loop with configuraiton
│   ├───angle_control                         # example of angle control loop with configuraiton
│   ├───angle_control_serial                  # example of angle control using serial port with configuraiton
│   ├───velocity_control                      # example of velocity control loop with configuraiton
│   ├───velocity_control_serial               # example of velocity control using serial port with configuraiton
│   ├───velocity_ultraslow_control_serial     # example of ultra slow velocity control using serial  port with configuraiton
│   ├───encoder_example                       # simple example of encoder usage 
│   ├───minimal_example                       # example of code without using configuration
│   └───HMBGC_example                         # example of code to be used with HMBGC controller with configuraiton
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



# Future Work Roadmap
#### Library maintenance
- [ ] Proper introduction of the **Arudino FOC Shield V1.2**
- [ ] Make the library accesible in the Arduino Library Manager 
- [ ] Publish a video utilising the library and the samples  
- [ ] Make minimal version of the arduino code - all in one arduino file

#### Code developement
- [ ] Encoder index proper implementation
- [ ] Enable more dirver types 
- [ ] Timer interrupt execution rather than in the `loop()`
- [ ] Make support for magnetic encoder AS5048 and similar

