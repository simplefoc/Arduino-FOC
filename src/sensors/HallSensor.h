#ifndef HALL_SENSOR_LIB_H
#define HALL_SENSOR_LIB_H

#include "Arduino.h"
#include "../common/base_classes/Sensor.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"


class HallSensor: public Sensor{
 public:


    enum class HallType : uint8_t
    {
      HALL_120 = 0,
      HALL_60C = 0b001,
      HALL_60B = 0b010,
      HALL_60A = 0b100,
      UNKNOWN = 0b111
    };

    /**
    HallSensor class constructor
    @param encA  HallSensor A pin
    @param encB  HallSensor B pin
    @param encC  HallSensor C pin
    @param pp  pole pairs  (e.g hoverboard motor has 15pp and small gimbals often have 7pp)
    @param hall_60deg Indicate if the hall sensors are 60 degrees apart electrically (means that they can all be one or off at the same time). In 60deg mode, B needs to lead, so you may need to swap the connections until you find one that works
    */
    HallSensor(int encA, int encB, int encC, int pp, HallType hall_type = HallType::UNKNOWN);

    /** HallSensor initialise pins */
    void init();
    /**
     *  function enabling hardware interrupts for the HallSensor channels with provided callback functions
     *  if callback is not provided then the interrupt is not enabled
     * 
     * @param doA pointer to the A channel interrupt handler function
     * @param doB pointer to the B channel interrupt handler function
     * @param doIndex pointer to the Index channel interrupt handler function
     * 
     */
    void enableInterrupts(void (*doA)() = nullptr, void(*doB)() = nullptr, void(*doC)() = nullptr);
    
    //  HallSensor interrupt callback functions
    /** A channel callback function */
    void handleA();
    /** B channel callback function */
    void handleB();
    /** C channel callback function */
    void handleC();
    
    
    // pins A and B
    int pinA; //!< HallSensor hardware pin A
    int pinB; //!< HallSensor hardware pin B
    int pinC; //!< HallSensor hardware pin C
    bool use_interrupt; //!< True if interrupts have been attached
    HallType hall_type, last_print_type; //!< Connectivity of hall sensor. The type indicates the pin to be swapped. Hall120 has no swapped pin

    // HallSensor configuration
    Pullup pullup; //!< Configuration parameter internal or external pullups
    int cpr;//!< HallSensor cpr number

    // Abstract functions of the Sensor class implementation
    /** Interrupt-safe update */
    void update() override;
    /** get current angle (rad) */
    float getSensorAngle() override;
    /**  get current angular velocity (rad/s) */
    float getVelocity() override;

    /** 
     * returns 0 if it does need search for absolute zero
     * 0 - magnetic sensor (& encoder with index which is found)
     * 1 - ecoder with index (with index not found yet)
     */
    int needsSearch() override;


    // whether last step was CW (+1) or CCW (-1).  
    Direction direction;
    Direction old_direction;

    void attachSectorCallback(void (*onSectorChange)(int a) = nullptr);

    //last unique previous states, 0 = recent, used to detect the hall type
    volatile uint8_t previous_states[3]; 
    // the current 3bit state of the hall sensors, without any line flipped
    volatile int8_t hall_state_raw;
    // the current sector of the sensor. Each sector is 60deg electrical
    volatile int8_t electric_sector;
    // the number of electric rotations
    volatile long electric_rotations;
    // this is sometimes useful to identify interrupt issues (e.g. weak or no pullup resulting in 1000s of interrupts)
    volatile long total_interrupts; 

    // variable used to filter outliers - rad/s
    float velocity_max = 1000.0f;

  private:
    
    Direction decodeDirection(int oldState, int newState);
    void updateState();

    volatile unsigned long pulse_timestamp;//!< last impulse timestamp in us
    volatile int A_active; //!< current active states of A channel
    volatile int B_active; //!< current active states of B channel
    volatile int C_active; //!< current active states of C channel

    // function pointer for on sector change call back
    void (*onSectorChange)(int sector) = nullptr;

    volatile long pulse_diff;
};


#endif
