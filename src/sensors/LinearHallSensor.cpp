#include "LinearHallSensor.h"
#include "./communication/SimpleFOCDebug.h"

/** LinearHallSensor(uint8_t _pinAnalog, int _min, int _max)
 * @param _pinAnalog  the pin that is reading the pwm from magnetic sensor
 * @param _min_raw_count  the smallest expected reading.  Whilst you might expect it to be 0 it is often ~15.  Getting this wrong results in a small click once per revolution
 * @param _max_raw_count  the largest value read.  whilst you might expect it to be 2^10 = 1023 it is often ~ 1020. Note: For ESP32 (with 12bit ADC the value will be nearer 4096)
 */
LinearHallSensor::LinearHallSensor(uint8_t _pinAnalog_a, uint8_t _pinAnalog_b)
{
  pinAnalog_a = _pinAnalog_a;
  pinAnalog_b = _pinAnalog_b;

  if (pullup == Pullup::USE_INTERN)
  {
    pinMode(pinAnalog_a, INPUT_PULLUP);
    pinMode(pinAnalog_b, INPUT_PULLUP);
  }
  else
  {
    pinMode(pinAnalog_a, INPUT);
    pinMode(pinAnalog_b, INPUT);
  }
}

/**
 * @brief 线性传感器初始化
 *
 */
void LinearHallSensor::init(int _min_raw_count_a, int _max_raw_count_a,
                            int _min_raw_count_b, int _max_raw_count_b)
{
  raw_count_a = getRawCountA();
  raw_count_b = getRawCountB();

  if (_isset(_min_raw_count_a) && _isset(_max_raw_count_a) && _isset(_min_raw_count_b) && _isset(_max_raw_count_b))
  {
    /** LinearHall A */
    min_raw_count_a = _min_raw_count_a;
    max_raw_count_a = _max_raw_count_a;
    /** LinearHall B */
    min_raw_count_b = _min_raw_count_b;
    max_raw_count_b = _max_raw_count_b;
  }
  else
  {
    /** 开始旋转电机一周校准线性霍尔的最大最小值 */
    int minA, maxA, minB, maxB, centerA, centerB;

    minA = raw_count_a;
    maxA = minA;
    centerA = (minA + maxA) / 2;
    minB = raw_count_b;
    maxB = minB;
    centerB = (minB + maxB) / 2;

    SIMPLEFOC_DEBUG("Sensor start init");

    // move one electrical revolution forward
    for (int i = 0; i <= 5000; i++)
    {
      // float angle = _3PI_2 + _2PI * i / 500.0f;
      // BLDCMotor::setPhaseVoltage(FOCMotor::voltage_sensor_align, 0, angle);

      //手动旋转电机一周来检测霍尔最大最小值
      int tempA = getRawCountA();
      if (tempA < minA)
        minA = tempA;
      if (tempA > maxA)
        maxA = tempA;
      centerA = (minA + maxA) / 2;

      int tempB = getRawCountB();
      if (tempB < minB)
        minB = tempB;
      if (tempB > maxB)
        maxB = tempB;
      centerB = (minB + maxB) / 2;

      if (i % 500 == 0)
      {
        static int num = 10;
        SIMPLEFOC_DEBUG("Count down: ", num);
        SIMPLEFOC_DEBUG("A: ", centerA);
        SIMPLEFOC_DEBUG("B: ", centerB);
        SIMPLEFOC_DEBUG("min A: ", minA);
        SIMPLEFOC_DEBUG("max A: ", maxA);
        SIMPLEFOC_DEBUG("min B: ", minB);
        SIMPLEFOC_DEBUG("max B: ", maxB);

        num--;
      }

      _delay(2);
    }

    min_raw_count_a = minA;
    max_raw_count_a = maxA;

    min_raw_count_b = minB;
    max_raw_count_b = maxB;

    SIMPLEFOC_DEBUG("Sensor init ok:");
    SIMPLEFOC_DEBUG("A: ", centerA);
    SIMPLEFOC_DEBUG("B: ", centerB);
    SIMPLEFOC_DEBUG("min A: ", minA);
    SIMPLEFOC_DEBUG("max A: ", maxA);
    SIMPLEFOC_DEBUG("min B: ", minB);
    SIMPLEFOC_DEBUG("max B: ", maxB);
  }

  cpr = max_raw_count_a - min_raw_count_a;

  this->Sensor::init(); // call base class init
}

/**
 * @brief Shaft angle calculation
 *
 * @return angle is in radians [rad] float [0,2PI]
 */
float LinearHallSensor::getSensorAngle()
{
  /* 1.获取双霍尔原始模拟值 */
  raw_count_a = getRawCountA();
  raw_count_b = getRawCountB();

  /* 2.双霍尔模拟值滤波 */

  /* 3.双霍尔滤波后值映射到[-1,1] */
  this->hall_filtered_a = map_float(raw_count_a, min_raw_count_a, max_raw_count_a, -1, 1);
  this->hall_filtered_b = map_float(raw_count_b, min_raw_count_b, max_raw_count_b, -1, 1);

  /* 4.反正切函数计算轴角度[-180,180] */
  float angle = atan2(hall_filtered_a, hall_filtered_b) * 180 / PI;

  /* 5.轴角转为弧度制[0,2PI] */
  return (angle + 180) * PI / 180.0f;
}

/**
 * @brief Reading the raw counter of the linear hall sensor A
 *
 * @return int
 */
int LinearHallSensor::getRawCountA()
{
  return analogRead(pinAnalog_a);
}

/**
 * @brief Reading the raw counter of the linear hall sensor B
 *
 * @return int
 */
int LinearHallSensor::getRawCountB()
{
  return analogRead(pinAnalog_b);
}

/**
 * @brief 数据映射 float
 *
 * @param in_x    输入值
 * @param in_min  输入值的最小幅值
 * @param in_max  输入值的最大幅值
 * @param out_min 输出值的最小幅值
 * @param out_max 输出值的最大幅值
 * @return float  映射后的输出值
 */
float LinearHallSensor::map_float(float in_x, float in_min, float in_max, float out_min, float out_max)
{
  return (in_x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief 获取线性霍尔A滤波后的值
 *
 * @return float [-1,1]
 */
float LinearHallSensor::getHallFiledA()
{
  return this->hall_filtered_a;
}

/**
 * @brief 获取线性霍尔B滤波后的值
 *
 * @return float [-1,1]
 */
float LinearHallSensor::getHallFiledB()
{
  return this->hall_filtered_b;
}
