#include <SimpleFOC.h>

// alternative pinout
#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_SCLK 14
#define HSPI_SS 15

// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
// config           - SPI config
//  cs              - SPI chip select pin 
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, HSPI_SS);

// for esp 32, it has 2 spi interfaces VSPI (default) and HPSI as the second one
// to enable it instatiate the object
SPIClass SPI_2(HSPI);

void setup() {
  // monitoring port
  Serial.begin(115200);

  // start the newly defined spi communication
  SPI_2.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); //SCLK, MISO, MOSI, SS 
  // initialise magnetic sensor hardware
  sensor.init(&SPI_2);

  Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and 
  // has to be called before getAngle nad getVelocity
  sensor.update();
  // display the angle and the angular velocity to the terminal
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
}
