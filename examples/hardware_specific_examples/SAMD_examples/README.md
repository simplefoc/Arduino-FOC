
# SAMD Support

SimpleFOC supports many SAMD21 MCUs, really any SAMD21 supported by Arduino core should work.

## Pin assignments

The SAMD chips have some very powerful PWM features, but do not have flexible pin assignments.

You should be able to use *most* (but not all!), pin combinations for attaching your motor's PWM pins. Please ignore the board descriptions and pinout diagrammes regarding PWM-pins on SAMD boards. They are pretty much all incorrect to varying degrees of awfulness.

On SAMD we use TCC and TC timer peripherals (built into the SAMD chip) to control the PWM. Depending on the chip there are various timer units, whose PWM outputs are attached to various different pins, and it is all very complicated. Luckily SimpleFOC sets it all up automatically *if* there is a compatible configuration for those pins.

Not all timers are created equal. The TCC timers are pretty awesome for PWM motor control, while the TC timers are just ok for the job. So to get best performance, you want to use just TCC timer pins if you can.

By enabling

```
 #define SIMPLEFOC_SAMD_DEBUG
```

in drivers/hardware_specific/samd_mcu.cpp<br>
you will see a table of pin assignments printed on the serial console, as well as the timers SimpleFOC was able to find and configure on the pins you specified. You can use this to optimize your choice of pins if you want.

You can configure up to 12 pins for PWM motor control, i.e. 6x 2-PWM motors, 4x 3-PWM motors, 3x 4-PWM motors or 2x 6-PWM motors. 

## PWM control modes

All modes (3-PWM, 6-PWM, Stepper 2-PWM and Stepper 4-PWM) are supported.

For 2-, 3- amd 4- PWM, any valid pin-combinations can be used. If you stick to TCC timers rather than using TC timers, then you'll get getter PWM waveforms. If you use pins which are all on the same TCC unit, you'll get the best result, with the PWM signals all perfectly aligned as well.

For 6-PWM, the situation is much more complicated:<br>
TC timers cannot be used for 6-PWM, only TCC timers.

For Hardware Dead-Time insertion, you must use H and L pins for one phase from the same TCC unit, and on the same channel, but using complementary WOs (Waveform Outputs, i.e. PWM output pins). Check the table to find pins on the same channel (like TCC0-0) but complementary WOs (like TCC0-0[0] and TCC0-0[4] or TCC1-0[0] and TCC1-0[2]).

For Software Dead-Time insertion, you must use the same TCC and different channels for the H and L pins of the same phase.

Note: in all of the above note that you *cannot* set the timers or WOs used - they are fixed, and determined by the pins you selected. SimpleFOC will find the best combination of timers given the pins, trying to use TCC timers before TC, and trying to keep things on the same timers as much as possible. If you configure multiple motors, it will take into account the pins already assigned to other motors.
So it is matter of choosing the right pins, nothing else.

Note also: Unfortunately you can't set the PWM frequency. It is currently fixed at 24KHz. This is a tradeoff between limiting PWM resolution vs
increasing frequency, and also due to keeping the pin assignemts flexible, which would not be possible if we ran the timers at different rates.

## Status

Currently, SAMD21 is supported, and SAMD51 is unsupported. SAMD51 support is in progress.

Boards tested:

 * Arduino Nano 33 IoT
 * Arduino MKR1000
 * Arduino MKR1010 Wifi
 * Seeduino XIAO
 * Feather M0 Basic

Environments tested:

 * Arduino IDE
 * Arduino Pro IDE
 * Sloeber
