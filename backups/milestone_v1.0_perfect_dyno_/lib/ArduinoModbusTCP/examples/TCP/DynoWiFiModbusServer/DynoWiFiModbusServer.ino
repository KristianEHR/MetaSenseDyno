

/*
  WiFi Modbus TCP Server LED

  This sketch creates a Modbus TCP Server with a simulated coil.
  
*/
#include "Arduino.h"
#include "RPC.h"
#include <SPI.h>
#include <WiFi.h>
#include "ModbusTCPServer.h"
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include "arduino_secrets.h"
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
int result1, result2;
//const int ledPin = LED_BUILTIN;

int status = WL_IDLE_STATUS;

WiFiServer wifiServer(502);

ModbusTCPServer modbusTCPServer;

//using namespace rtos;
//Thread servoThread;


void setup() {
  // RPC initiate
  RPC.begin();
  Serial.begin(115200); 


  // attempt to connect to WiFi network:   // Start the server
  wifiServer.begin();
  while (status != WL_CONNECTED) {
  //  Serial.print("Attempting to connect to SSID: ");
  //  Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
  
    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the status:
  // printWifiStatus();

  
  // start the Modbus TCP server
  if (!modbusTCPServer.begin()) {
    Serial.println("Failed to start Modbus TCP Server!");
    while (1);
  }
  
  // configure a single coil at address 0x00
  modbusTCPServer.configureCoils(0x00, 1);
  modbusTCPServer.configureHoldingRegisters(0x00, 10);
}
//  Serial.println("Modbus TCP Server LED");


void loop() {

  // listen for incoming clients
  WiFiClient client = wifiServer.available();
  if (client) {
    // a new client connected
    Serial.println("new client");

    // let the Modbus TCP accept the connection 
    modbusTCPServer.accept(client);

    // poll for Modbus TCP requests, while client connected
    while (client.connected()) {
      modbusTCPServer.poll();    
      delay(1);
      // update the modbus registers
      result1 = RPC.call("rpmRead",1).as<int>();
      delay(1);
      result2 = RPC.call("rpmRead",0).as<int>();
      delay(1);
//      Serial.println(result1);
//      Serial.println(result2);
      modbusTCPServer.holdingRegisterWrite(1, result1);//rpm -> Holding rpm
      modbusTCPServer.holdingRegisterWrite(2, result2);//brake ->
    }    
//    Serial.println("client disconnected");
  }
}
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
void blink(int led, int delaySeconds) {
  for (int i; i < 10; i++) {
    digitalWrite(led, LOW);
    delay(delaySeconds);
    digitalWrite(led, HIGH);
    delay(delaySeconds);
  }
}
