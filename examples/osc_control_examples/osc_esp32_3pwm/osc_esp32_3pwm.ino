/**
 * Arduino SimpleFOC + OSC control example
 *
 * Simple example to show how you can control SimpleFOC via OSC.
 *
 * It's a nice way to work, easier than changing speeds via Seerial text input. It could also be the basis for
 * a functional UI, for example in a lab, art-gallery or similar situation.
 *
 * For this example we used an ESP32 to run the code, a AS5048B I2C absolute encoder
 * and a generic L298 driver board to drive a Emax 4114 gimbal motor. But there is no reason the OSC part will
 * not work with any other setup.
 *
 * You will need:
 *
 * - a working SimpleFOC setup - motor, driver, encoder
 * - a MCU with WiFi and UDP support, for example an ESP32, or an Arduino with Ethernet Shield
 * - a device to run an OSC UI
 * - a configured OSC UI
 * - a WiFi network
 *
 * To do the OSC UI I used TouchOSC from https://hexler.net/products/touchosc
 * There is an example UI file that works with this sketch, see "layout1.touchosc"
 * You can open the UI file in 'TouchOSC Editor' (free program) and transfer it to the TouchOSC app on your device.
 *
 * Alternatively, there are other OSC UIs which may work, e.g. http://opensoundcontrol.org/implementations
 *
 * Using:
 *
 * Change the values below to match the WiFi ssid/password of your network.
 * Load and run the code on your ESP32. Take a note of the IP address of your ESP32.
 * Load and run the UI in TouchOSC.
 * Configure TouchOSC to connect to your ESP32.
 * The first command you send will cause the ESP32 to start sending responses to your TouchOSC device.
 * After this you will see motor position and speed in the UI.
 * Have fun controlling your SimpleFOC motors from your smartphone!
 *
 */


#include "Arduino.h"
#include <SimpleFOC.h>

#include <WiFi.h>
#include <WiFiUdp.h>

#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCBoards.h>
#include <Math.h>


const char ssid[] = "myssid";		// your network SSID (name)
const char pass[] = "mypassword";	// your network password
WiFiUDP Udp;
IPAddress outIp(192,168,1,17);        // remote IP (not needed for receive)
const unsigned int outPort = 8000;    // remote port (not needed for receive)
const unsigned int inPort = 8000;     // local port to listen for UDP packets (here's where we send the packets)


OSCErrorCode error;

MagneticSensorI2C sensor = MagneticSensorI2C(0x40, 14, 0xFE, 8);
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver =  BLDCDriver3PWM(25, 26, 27);

// commander interface
Commander command = Commander(Serial);

void setup() {
	Serial.begin(115200);

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

    delay(2000);
	Serial.println("Initializing motor.");

	sensor.init();
	motor.linkSensor(&sensor);
	driver.voltage_power_supply = 9;
	driver.init();
	motor.linkDriver(&driver);
	motor.controller = MotionControlType::velocity;
	motor.PID_velocity.P = 0.2f;
	motor.PID_velocity.I = 20;
	motor.PID_velocity.D = 0.001f;
	motor.PID_velocity.output_ramp = 1000;
	motor.LPF_velocity.Tf = 0.01f;
	motor.voltage_limit = 8;
	//motor.P_angle.P = 20;
	motor.init();
	motor.initFOC();

	Serial.println("All initialization complete.");
}

// velocity set point variable
float target_velocity = 2.0f;
// angle set point variable
float target_angle = 1.0f;


void motorControl(OSCMessage &msg){
	if (msg.isInt(0))
		target_velocity = radians(msg.getInt(0));
	else if (msg.isFloat(0))
		target_velocity = radians(msg.getFloat(0));
	else if (msg.isDouble(0))
		target_velocity = radians(msg.getDouble(0));
	Serial.print("Velocity set to ");
	Serial.println(target_velocity);
}

void cmdControl(OSCMessage &msg){
	char cmdStr[16];
	if (msg.isString(0)) {
		msg.getString(0,cmdStr,16);
		command.motor(&motor,cmdStr);
	}
}

long lastSend = 0;
OSCMessage bundleIN;

void loop() {
    OSCBundle bundleOUT;

	// FOC algorithm function
	motor.move(target_velocity);
	motor.loopFOC();


	int size = Udp.parsePacket();
	if (size > 0) {
	    while (size--) {
	    	bundleIN.fill(Udp.read());
	    }
	    if (!bundleIN.hasError()) {
			bundleIN.dispatch("/mot1/S", motorControl);
			bundleIN.dispatch("/mot1/C", cmdControl);
	        IPAddress ip = Udp.remoteIP();
	        if (!( ip==outIp )) {
	        	Serial.print("New connection from ");
	        	Serial.println(ip);
	        	outIp = ip;
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
			int ang = (int)degrees(motor.shaftAngle()) % 360;
			if (ang<0) ang = 360-abs(ang);
		    //BOSCBundle's add' returns the OSCMessage so the message's 'add' can be composed together
			bundleOUT.add("/mot1/A").add((int)ang);
			bundleOUT.add("/mot1/V").add((int)degrees(motor.shaftVelocity()));
			Udp.beginPacket(outIp, outPort);
		    bundleOUT.send(Udp);
		    Udp.endPacket();
		    bundleOUT.empty(); // empty the bundle to free room for a new one
		    lastSend = now;
		}
	}

}
