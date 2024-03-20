#ifndef LINEARHALLSENSOR_LIB_H
#define LINEARHALLSENSOR_LIB_H

#include "Arduino.h"
#include "../common/base_classes/Sensor.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"

/**
 * This sensor has LinearHallSensor.
 * This approach is very simple.
 */
class LinearHallSensor : public Sensor
{
public:
  /**
   * LinearHallSensor class constructor
   * @param _pinAnalog_a  the pin to read the HALL A signal
   * @param _pinAnalog_b  the pin to read the HALL B signal
   */
  LinearHallSensor(uint8_t _pinAnalog_a, uint8_t _pinAnalog_b);

  /** sensor initialise pins */
  void init(int _min_raw_count_a = NOT_SET, int _max_raw_count_a = NOT_SET,
            int _min_raw_count_b = NOT_SET, int _max_raw_count_b = NOT_SET);

  uint8_t pinAnalog_a; //!< Linear Hall pin A
  uint8_t pinAnalog_b; //!< Linear Hall pin B

  /** Encoder configuration */
  Pullup pullup;

  /** implementation of abstract functions of the Sensor class */
  /** get current angle (rad) */
  float getSensorAngle() override;
  /** raw count (typically in range of 0-1023), useful for debugging resolution issues */
  int raw_count_a;
  int raw_count_b;

  /** 获取霍尔滤波后的值[-1,1] */
  float getHallFiledA();
  float getHallFiledB();

  /** 数据映射 */
  float map_float(float in_x, float in_min, float in_max, float out_min, float out_max);

private:
  /** LinearHall 实际输出模拟值范围 */
  int min_raw_count_a;
  int max_raw_count_a;
  int min_raw_count_b;
  int max_raw_count_b;

  /** LinearHall 滤波后值映射到[-1,1] */
  float hall_filtered_a;
  float hall_filtered_b;

  int cpr;
  int read();

  /**
   * Function getting current angle register value
   * it uses angle_register variable
   */
  int getRawCountA();
  int getRawCountB();
};

#endif
