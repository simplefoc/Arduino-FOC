# SimpleFOC Examples

The SimpleFOC library includes 88+ fully documented examples covering various hardware configurations and control modes.

## Hardware-Specific Examples

Examples optimized for specific hardware platforms:

- **[B_G431B_ESC1](hardware_specific_examples/B_G431B_ESC1/)** - STM32 B-G431B-ESC1 development board
- **[Bluepill](hardware_specific_examples/Bluepill_examples/)** - STM32 Bluepill boards
- **[DRV8302](hardware_specific_examples/DRV8302_driver/)** - DRV8302/DRV8305 motor drivers
- **[ESP32](hardware_specific_examples/ESP32/)** - ESP32 microcontrollers with MCPWM and LEDC
- **[SAMD](hardware_specific_examples/SAMD_examples/)** - SAMD21/51 boards (Arduino Zero, MKR series)
- **[Silabs](hardware_specific_examples/Silabs/)** - Silicon Labs (Arduino Nano Matter)
- **[Teensy](hardware_specific_examples/Teensy/)** - Teensy 3.x and 4.x boards
- **[SimpleFOCShield](hardware_specific_examples/SimpleFOCShield/)** - SimpleFOCShield v1, v2, and v3
- **[SimpleFOCMini](hardware_specific_examples/SimpleFOCMini/)** - SimpleFOCMini board
- **[PowerShield](hardware_specific_examples/SimpleFOC-PowerShield/)** - SimpleFOC PowerShield
- **[HMBGC](hardware_specific_examples/HMBGC_example/)** - HMBGC gimbal controller
- **[ODrive](hardware_specific_examples/Odrive_examples/)** - ODrive boards
- **[SmartStepper](hardware_specific_examples/Smart_Stepper/)** - Smart Stepper configurations

## Motion Control Examples

Different control modes with various sensor types:

### Torque Control
- **Voltage-based torque control** - [encoder](motion_control/torque_control/encoder/voltage_control/) | [hall sensor](motion_control/torque_control/hall_sensor/voltage_control/) | [magnetic sensor](motion_control/torque_control/magnetic_sensor/voltage_control/)
- **Current-based torque control** (FOC with current sensing) - [encoder](motion_control/torque_control/encoder/current_control/)

### Velocity Control
- [Velocity control with encoder](motion_control/velocity_motion_control/encoder/velocity_control/)
- [Velocity control with hall sensor](motion_control/velocity_motion_control/hall_sensor/velocity_control/)
- [Velocity control with magnetic sensor](motion_control/velocity_motion_control/magnetic_sensor/velocity_control/)

### Position Control
- [Position/angle control with encoder](motion_control/position_motion_control/encoder/angle_control/)
- [Position control with hall sensor](motion_control/position_motion_control/hall_sensor/angle_control/)
- [Position control with magnetic sensor](motion_control/position_motion_control/magnetic_sensor/angle_control/)

### Open-loop Control
- [Open-loop velocity control](motion_control/open_loop_motor_control/open_loop_velocity_example/) (sensorless)
- [Open-loop position control](motion_control/open_loop_motor_control/open_loop_position_example/) (sensorless)

### Custom Control
- [Custom motion control loop](motion_control/custom_motion_control/) implementation

## Utility Examples

### Calibration and Testing
- [Find pole pair number](utils/find_pole_pairs_number/)
- [Find KV rating](utils/find_kv_rating/)
- [Find sensor offset and direction](utils/sensor_test/find_sensor_offset_and_direction/)
- [Measure inductance and resistance](utils/find_resistance_inductance/)
- [Alignment and cogging test](utils/alignment_procedure/)

### Current Sensing
- [Align current sense with motor phases](utils/current_sense_alignment/)
- [Generic current sense testing](utils/current_sense_test/generic_test/)
- [Inline current sense testing](utils/current_sense_test/inline_test/)

### Driver Testing
- [BLDC 3PWM driver standalone test](utils/driver_standalone_test/bldc_driver_3pwm_standalone/)
- [BLDC 6PWM driver standalone test](utils/driver_standalone_test/bldc_driver_6pwm_standalone/)
- [Stepper 2PWM driver standalone test](utils/driver_standalone_test/stepper_driver_2pwm_standalone/)
- [Stepper 4PWM driver standalone test](utils/driver_standalone_test/stepper_driver_4pwm_standalone/)

### Sensor Testing
- [Encoder testing](utils/sensor_test/encoder/) (hardware and software interrupts)
- [Hall sensor testing](utils/sensor_test/hall_sensors/)
- [Magnetic sensor testing](utils/sensor_test/magnetic_sensors/) (SPI, I2C, Analog)

### Communication
- [Commander interface examples](utils/communication_test/commander/)
- [Step/Dir listener examples](utils/step_dir_listener/)

## Serial Command Examples

Examples showing full motor control via serial interface:
- [Full control with encoder](motor_commands_serial_examples/encoder/full_control_serial/)
- [Full control with hall sensor](motor_commands_serial_examples/hall_sensor/full_control_serial/)
- [Full control with magnetic sensor](motor_commands_serial_examples/magnetic_sensor/full_control_serial/)

## OSC Control Examples

Open Sound Control (OSC) integration:
- [ESP32 3PWM with OSC](osc_control_examples/osc_esp32_3pwm/)
- [ESP32 full control with OSC](osc_control_examples/osc_esp32_fullcontrol/)

## How to Use Examples

1. **Arduino IDE**: Go to `File > Examples > Simple FOC > [category] > [example]`
2. **PlatformIO**: Copy example code from the `examples/` directory
3. **GitHub**: Browse examples at https://github.com/simplefoc/Arduino-FOC/tree/master/examples

Each example includes:
- Detailed comments explaining the code
- Hardware connection diagrams (where applicable)
- Expected behavior description
- Common troubleshooting tips

## Getting Help

- **Documentation**: https://docs.simplefoc.com
- **Community Forum**: https://community.simplefoc.com
- **GitHub Issues**: https://github.com/simplefoc/Arduino-FOC/issues

