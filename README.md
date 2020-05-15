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
<img src="https://askuric.github.io/Arduino-FOC/extras/Images/foc_shield_video.jpg"  height="350px">
</a>
</p>

### Features

- **Plug & play**: Arduino <span class="simple">Simple<span class="foc">FOC</span>library</span> 
- **Low-cost**: Price in the range of \$20-\$40
- **Open Source**: Gerber files and BOM available
- **Stackable**: running 2 motors in the same time
##### If you are interested in this board, preorder your version on this link: [Arduino Simple FOC Shield](https://askuric.github.io/simplefoc_shield_product)

<p align=""><img src="https://askuric.github.io/Arduino-FOC/extras/Images/shield_to_v13.jpg" height="180px">   <img src="https://askuric.github.io/Arduino-FOC/extras/Images/shield_bo_v13.jpg"  height="180px"> <img src="https://askuric.github.io/Arduino-FOC/extras/Images/simple_foc_shield_v13_small.gif"  height="180x"></p>


## Arduino SimpleFOClibrary

<p align="">
<a href="https://youtu.be/N_fRYf7Z80k">
<img src="https://askuric.github.io/Arduino-FOC/extras/Images/youtube.png"  height="350px">
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

## Documentation
Find out more information about the Arduino SimpleFOC project in [docs website](https://askuric.github.io/Arduino-FOC/) 


## Arduino FOC repo structure
Branch  | Description | Status
------------ | ------------- | ------------ 
[master](https://github.com/askuric/Arduino-FOC) | Stable and tested library version | ![Library Compile](https://github.com/askuric/Arduino-FOC/workflows/Library%20Compile/badge.svg)
[dev](https://github.com/askuric/Arduino-FOC/tree/dev) | Development library version | ![Library Dev Compile](https://github.com/askuric/Arduino-FOC/workflows/Library%20Dev%20Compile/badge.svg?branch=dev)
[minimal](https://github.com/askuric/Arduino-FOC/tree/minimal) | Minimal Arduino example with integrated library | ![MinimalBuild](https://github.com/askuric/Arduino-FOC/workflows/MinimalBuild/badge.svg?branch=minimal)
