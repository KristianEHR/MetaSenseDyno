#include "Arduino.h"
#include "RPC.h"

/*
  WiFi Modbus TCP Server LED

  This sketch creates a Modbus TCP Server with a simulated coil.
  
*/
#include <SPI.h>
#include <WiFi.h>

#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include "arduino_secrets.h"
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

//const int ledPin = LED_BUILTIN;

int status = WL_IDLE_STATUS;

WiFiServer wifiServer(502);

ModbusTCPServer modbusTCPServer;

void setup() {

  RPC.begin();
  
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
  printWifiStatus();

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

}

void loop() {

  // listen for incoming clients
  WiFiClient client = wifiServer.available();
  if (client) {
    // a new client connected
    Serial.println("new client");

    // let the Modbus TCP accept the connection 
    modbusTCPServer.accept(client);

    while (client.connected()) {
      // poll for Modbus TCP requests, while client connected
      modbusTCPServer.poll();
      
      // update the modbus registers
      requestReading();
    }

    Serial.println("client disconnected");
  }
}

