/**
 * Simple example of custom commands that have nothing to do with the simple foc library
 */

#include <SimpleFOC.h>

// instantiate the commander
Commander command = Commander(Serial);

// led control function
void doLed(char* cmd){ 
    if(atoi(cmd)) digitalWrite(LED_BUILTIN, HIGH); 
    else digitalWrite(LED_BUILTIN, LOW); 
};
// get analog input 
void doAnalog(char* cmd){ 
    if (cmd[0] == '0') Serial.println(analogRead(A0));
    else if (cmd[0] == '1') Serial.println(analogRead(A1));
    else if (cmd[0] == '2') Serial.println(analogRead(A2));
    else if (cmd[0] == '3') Serial.println(analogRead(A3));
    else if (cmd[0] == '4') Serial.println(analogRead(A4));
};

void setup() {
    // define pins
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(A4, INPUT);

    // Serial port to be used
    Serial.begin(115200);

    // add new commands
    command.add('L', doLed, "led on/off");
    command.add('A', doAnalog, "analog read A0-A4");

    Serial.println(F("Commander listening"));
    Serial.println(F(" - Send ? to see the node list..."));
    Serial.println(F(" - Send L0 to turn the led off and L1 to turn it off"));
    Serial.println(F(" - Send A0-A4 to read the analog pins"));
    _delay(1000);
}


void loop() {

    // user communication
    command.run(); 
    _delay(10);
}