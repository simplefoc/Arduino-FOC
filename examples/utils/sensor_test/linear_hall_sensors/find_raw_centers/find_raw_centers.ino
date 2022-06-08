/**
 * An example to find the center offsets for both ADC channels used in the LinearHall sensor constructor
 * Spin your motor through at least one full revolution to average out all of the variations in magnet strength.
 */

//Change these defines to match the analog input pins that your hall sensors are connected to
#define LINEAR_HALL_CHANNEL_A 39
#define LINEAR_HALL_CHANNEL_B 33


//program variables
int minA, maxA, minB, maxB, centerA, centerB;
unsigned long timestamp;

void setup() {
  // monitoring port
  Serial.begin(115200);

  // initialise magnetic sensor hardware
  pinMode(LINEAR_HALL_CHANNEL_A, INPUT);
  pinMode(LINEAR_HALL_CHANNEL_B, INPUT);
  
  minA = analogRead(LINEAR_HALL_CHANNEL_A);
  maxA = minA;
  centerA = (minA + maxA) / 2;
  minB = analogRead(LINEAR_HALL_CHANNEL_B);
  maxB = minB;
  centerB = (minB + maxB) / 2;

  Serial.println("Sensor ready");
  delay(1000);
  timestamp = millis();
}

void loop() {
  //read sensors and update variables
  int tempA = analogRead(LINEAR_HALL_CHANNEL_A);
  if (tempA < minA) minA = tempA;
  if (tempA > maxA) maxA = tempA;
  centerA = (minA + maxA) / 2;
  int tempB = analogRead(LINEAR_HALL_CHANNEL_B);
  if (tempB < minB) minB = tempB;
  if (tempB > maxB) maxB = tempB;
  centerB = (minB + maxB) / 2;

  if (millis() > timestamp + 100) {
    timestamp = millis();
    // display the center counts, and max and min count
    Serial.print("A:");
    Serial.print(centerA);
    Serial.print("\t, B:");
    Serial.print(centerB);
    Serial.print("\t, min A:");
    Serial.print(minA);
    Serial.print("\t, max A:");
    Serial.print(maxA);
    Serial.print("\t, min B:");
    Serial.print(minB);
    Serial.print("\t, max B:");
    Serial.println(maxB);
  }
}
