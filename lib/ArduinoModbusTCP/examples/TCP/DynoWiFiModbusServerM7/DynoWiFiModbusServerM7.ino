/*
  WiFi Modbus TCP Server LED

  This sketch creates a Modbus TCP Server with a simulated coil.
  
*/

#include <Arduino.h>
#include <RPC.h>
#include <SPI.h>
#include <WiFi.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include "arduino_secrets.h"
#include "Servo.h"
//#include <Arduino_AdvancedAnalog.h>

#define Servo_O D9


char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

//const int ledPin = LED_BUILTIN;

int status = WL_IDLE_STATUS;
int rpm, brakeTorque;

//Servo vars
//int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin
 
WiFiServer wifiServer(502);

ModbusTCPServer modbusTCPServer;
Servo myservo;  // create servo object to control a servo

void setup() {

  RPC.begin();
  analogReadResolution(12);
  analogWriteResolution(12);
  pinMode(A0, INPUT);
  pinMode(D9, OUTPUT);
 
  Serial.begin(115200);
  //Initialize serial and wait for port to open:
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Modbus TCP Server LED");
  
  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
  
    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the status:
//  printWifiStatus();

  // start the server
  wifiServer.begin();
  // start the Modbus TCP server
  if (!modbusTCPServer.begin()) {
    Serial.println("Failed to start Modbus TCP Server!");
    while (1);
  }

  // configure a single coil at address 0x00
  modbusTCPServer.configureCoils(0x00, 1);
  modbusTCPServer.configureHoldingRegisters(0x00, 10);
  myservo.attach(9);  // attaches the servo output on pin 9 to the servo object


   //pinMode(LED_PIN, OUTPUT);

}

void loop() {
  // listen for incoming clients
  WiFiClient client = wifiServer.available();
  if (client) {
    // a new client connected
    Serial.println("new client");
    modbusTCPServer.accept(client);

    while (client.connected()) {
      // poll for Modbus TCP requests, while client connected
      modbusTCPServer.poll();
      
      // update the modbus registers
      rpm = RPC.call("rpmRead",1).as<int>();
      delay(1);
      brakeTorque = RPC.call("rpmRead",0).as<int>();
      delay(1);
      Serial.println("RPM data received:"+ String( rpm ));
      modbusTCPServer.holdingRegisterWrite(1, rpm);//rpm -> Holding rpm
      modbusTCPServer.holdingRegisterWrite(2, brakeTorque);//brake ->
      //myservo---------------------------------------------------------------------------------------
      val = analogRead(A0);            // reads the value of the potentiometer (value between 0 and 1023)
      val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
      myservo.write(val);                  // sets the servo position according to the scaled value
      delay(15);

    }
    Serial.println("client disconnected");
  }
}

