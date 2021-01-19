/**
 *
 * Control 2 motors on ESP32 using OSC
 *
 * In this example, all the commands available on the serial command interface are also available via OSC.
 * Also, the example is for 2 motors. If you have only 1 motor, comment out the lines for the second motor.
 *
 * There is a simple GUI that works with 2 motors and all the commands, which you can find in the file
 * "osc_fullcontrol.pd". You run it using purr Data: https://github.com/jonwwilkes/purr-data
 *
 * The nice thing about pD is that you can very easily extend the example using the graphical programming environment,
 * and that is quick and easy to do. Everything is live, and you can change the program as it runs. There is a huge
 * variety of existing functions and extensions, and its a very quick way to prototype some ideas. pD + OSC ss a good
 * way to quickly connect to other devices / protocols.
 *
 * It is definately a solid choice for any kind of music/art project.
 *
 *
 */


#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include "ssid.h" // create this file, which defines the constants MYSSID and MYPASS
#include <Wifi.h>
#include <WiFiUdp.h>

#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCBoards.h>


const char ssid[] = MYSSID;					// your network SSID (name)
const char pass[] = MYPASS;					// your network password
WiFiUDP Udp;
IPAddress outIp(192,168,1,13);        		// remote IP (not needed for receive)
const unsigned int outPort = 8000;          // remote port (not needed for receive)
const unsigned int inPort = 8000;        	// local port to listen for UDP packets (here's where we send the packets)




MagneticSensorI2C sensor1 = MagneticSensorI2C(0x41, 14, 0xFE, 8);
MagneticSensorI2C sensor2 = MagneticSensorI2C(0x40, 14, 0xFE, 8);
BLDCMotor motor1 = BLDCMotor(11);
BLDCMotor motor2 = BLDCMotor(11);
BLDCDriver3PWM driver1 =  BLDCDriver3PWM(25, 26, 27);
BLDCDriver3PWM driver2 =  BLDCDriver3PWM(0, 4, 16);

String m1Prefix("/M1");
String m2Prefix("/M2");


// we store seperate set-points for each motor, of course
float set_point1 = 2.0;
float set_point2 = 2.0;


OSCErrorCode error;
OSCBundle bundleOUT;			// outgoing message, gets re-used





void setup() {
	// set pins low - motor initialization can take some time,
	// and you don't want current flowing through the other motor while it happens...
	pinMode(0,OUTPUT);
	pinMode(4,OUTPUT);
	pinMode(16,OUTPUT);
	pinMode(25,OUTPUT);
	pinMode(26,OUTPUT);
	pinMode(27,OUTPUT);
	digitalWrite(0, 0);
	digitalWrite(4, 0);
	digitalWrite(16, 0);
	digitalWrite(25, 0);
	digitalWrite(26, 0);
	digitalWrite(27, 0);

	Serial.begin(115200);
    delay(200);

	WiFi.begin(ssid, pass);

    Serial.print("Connecting WiFi ");
    Serial.println(ssid);
    while (WiFi.status() != WL_CONNECTED) {
    	delay(500);
    	Serial.print(".");
    }
    Udp.begin(inPort);
    Serial.println();
    Serial.print("WiFi connected. Local OSC address: ");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(inPort);


    Serial.println("Initializing motors.");

    Wire.setClock(400000);

	sensor1.init();
	motor1.linkSensor(&sensor1);
	driver1.voltage_power_supply = 9;
	driver1.init();
	motor1.linkDriver(&driver1);
	motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
	motor1.controller = ControlType::velocity;
	motor1.PID_velocity.P = 0.2;
	motor1.PID_velocity.I = 20;
	motor1.PID_velocity.D = 0.001;
	motor1.PID_velocity.output_ramp = 1000;
	motor1.LPF_velocity.Tf = 0.01;
	motor1.voltage_limit = 8;
	motor1.P_angle.P = 20;
	motor1.init();
	motor1.initFOC();

	sensor2.init();
	motor2.linkSensor(&sensor2);
	driver2.voltage_power_supply = 9;
	driver2.init();
	motor2.linkDriver(&driver2);
	motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;
	motor2.controller = ControlType::velocity;
	motor2.PID_velocity.P = 0.2;
	motor2.PID_velocity.I = 20;
	motor2.PID_velocity.D = 0.001;
	motor2.PID_velocity.output_ramp = 1000;
	motor2.LPF_velocity.Tf = 0.01;
	motor2.voltage_limit = 8;
	motor2.P_angle.P = 20;
	motor2.init();
	motor2.initFOC();

	sendMotorParams(motor1, m1Prefix);
	sendMotorParams(motor2, m2Prefix);
	Serial.println("All initialization complete.");
	_delay(1000);
}




template <typename T>
void sendMessage(String& addr, T datum) {
	bundleOUT.add(addr.c_str()).add(datum);
	Udp.beginPacket(outIp, outPort);
	bundleOUT.send(Udp);
	Udp.endPacket();
	bundleOUT.empty(); // empty the bundle to free room for a new one
}





void motorCmd(OSCMessage &msg, int offset, BLDCMotor& motor, float* set_point, String& prefix){
	Serial.print("Command for ");
	Serial.println(prefix);
	if (msg.fullMatch("/P", offset)) {
		motor.PID_velocity.P = msg.getFloat(0);
		sendMessage(prefix+"/P", motor.PID_velocity.P);
	}
	else if (msg.fullMatch("/I", offset)) {
		motor.PID_velocity.I = msg.getFloat(0);
		sendMessage(prefix+"/I", motor.PID_velocity.I);
	}
	else if (msg.fullMatch("/D", offset)) {
		motor.PID_velocity.D = msg.getFloat(0);
		sendMessage(prefix+"/D", motor.PID_velocity.D);
	}
	else if (msg.fullMatch("/R", offset)) {
		motor.PID_velocity.output_ramp = msg.getFloat(0);
		sendMessage(prefix+"/R", motor.PID_velocity.output_ramp);
	}
	else if (msg.fullMatch("/F", offset)) {
		motor.LPF_velocity.Tf = msg.getFloat(0);
		sendMessage(prefix+"/F", motor.LPF_velocity.Tf);
	}
	else if (msg.fullMatch("/K", offset)) {
		motor.P_angle.P = msg.getFloat(0);
		sendMessage(prefix+"/K", motor.P_angle.P);
	}
	else if (msg.fullMatch("/N", offset)) {
		motor.velocity_limit = msg.getFloat(0);
		sendMessage(prefix+"/N", motor.velocity_limit);
	}
	else if (msg.fullMatch("/L", offset)) {
		motor.voltage_limit = msg.getFloat(0);
		sendMessage(prefix+"/L", motor.voltage_limit);
	}
	else if (msg.fullMatch("/C", offset)) {
		motor.controller = (ControlType)msg.getInt(0);
		sendMessage(prefix+"/C", motor.controller);
	}
	else if (msg.fullMatch("/t", offset)) {
		*set_point = msg.getFloat(0);
		sendMessage(prefix+"/3", *set_point); // TODO is the set-point the same as motor.target?
		Serial.print("Setting set-point to ");
		Serial.println(*set_point);
	}
	else if (msg.fullMatch("/params", offset)) {
		sendMotorParams(motor, prefix);
	}
}






void sendMotorMonitoring() {
	//Serial.println("Sending monitoring...");
	bundleOUT.add("/M1/0").add(motor1.voltage_q);
	bundleOUT.add("/M1/1").add(motor1.shaft_velocity);
	bundleOUT.add("/M1/2").add(motor1.shaft_angle);
	bundleOUT.add("/M1/3").add(motor1.target);
	bundleOUT.add("/M2/0").add(motor2.voltage_q);
	bundleOUT.add("/M2/1").add(motor2.shaft_velocity);
	bundleOUT.add("/M2/2").add(motor2.shaft_angle);
	bundleOUT.add("/M2/3").add(motor2.target);
	// TODO pack it into one message bundleOUT.add("/M2/i").add(motor2.voltage_q).add(motor2.shaft_velocity).add(motor2.shaft_angle).add(motor2.target);
	Udp.beginPacket(outIp, outPort);
	bundleOUT.send(Udp);
	Udp.endPacket();
	bundleOUT.empty(); // empty the bundle to free room for a new one
}



void sendMotorParams(BLDCMotor& motor, String& prefix) {
	bundleOUT.add((prefix+"/P").c_str()).add(motor.PID_velocity.P);
	bundleOUT.add((prefix+"/I").c_str()).add(motor.PID_velocity.I);
	bundleOUT.add((prefix+"/D").c_str()).add(motor.PID_velocity.D);
	bundleOUT.add((prefix+"/R").c_str()).add(motor.PID_velocity.output_ramp);
	bundleOUT.add((prefix+"/F").c_str()).add(motor.LPF_velocity.Tf);
	bundleOUT.add((prefix+"/K").c_str()).add(motor.P_angle.P);
	bundleOUT.add((prefix+"/N").c_str()).add(motor.velocity_limit);
	bundleOUT.add((prefix+"/L").c_str()).add(motor.voltage_limit);
	bundleOUT.add((prefix+"/C").c_str()).add(motor.controller);
	Udp.beginPacket(outIp, outPort);
	bundleOUT.send(Udp);
	Udp.endPacket();
	bundleOUT.empty(); // empty the bundle to free room for a new one
}





long lastSend = 0;
OSCMessage bundleIN;
int size;


void loop() {

	// FOC algorithm function
	motor1.loopFOC();
	motor1.move(set_point1);
	motor2.loopFOC();
	motor2.move(set_point2);


	int size = Udp.parsePacket();
	if (size > 0) {
	    while (size--) {
	    	bundleIN.fill(Udp.read());
	    }
	    if (!bundleIN.hasError()) {
			bundleIN.route("/M1", [](OSCMessage& msg, int offset){ motorCmd(msg, offset, motor1, &set_point1, m1Prefix); }, 0);
			bundleIN.route("/M2", [](OSCMessage& msg, int offset){ motorCmd(msg, offset, motor2, &set_point2, m2Prefix); }, 0);
	        IPAddress ip = Udp.remoteIP();
	        if (!( ip==outIp )) {
	        	Serial.print("New connection from ");
	        	Serial.println(ip);
	        	outIp = ip;
	        	sendMotorParams(motor1, m1Prefix);
	        	sendMotorParams(motor2, m2Prefix);
	        }
		}
	    else {
	    	error = bundleIN.getError();
	    	Serial.print("error: ");
	    	Serial.println(error);
	    }
		bundleIN.empty();
	}
	else { // don't receive and send in the same loop...
		long now = millis();
		if (now - lastSend > 100) {
			sendMotorMonitoring();
		    lastSend = now;
		}
	}

}
