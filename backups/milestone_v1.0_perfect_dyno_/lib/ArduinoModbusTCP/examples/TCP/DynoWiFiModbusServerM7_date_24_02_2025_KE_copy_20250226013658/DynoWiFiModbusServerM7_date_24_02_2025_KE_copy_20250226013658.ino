
/****************************************************
  WiFi Modbus TCP Server

  This sketch creates a Modbus TCP Server for Dyno control and acquttion
  It is connected to the Modbusmaster on a Wifi ethernet
  The Wifi logon parameters are kept in the "arduino_secrets.h"
  This modbus TCP/Wifi client is compiled to Arduino GIGA R1 core 7 (main core)
  It communicates with the DynoAdaptivePIDcontroller running on M4 core
  where data are fetched by RPC and inserted in the modbus H registers.
  Date: 24-02-2025, first edition
  Author: Kristian Ehrhorn, MetaSense
  Record:  
*/
#include <Arduino.h>
#include <RPC.h>
#include <SPI.h>
#include <WiFi.h>
#include <ArduinoRS485.h>  // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include "arduino_secrets.h"

char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;           // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;

WiFiServer wifiServer(502);
ModbusTCPServer modbusTCPServer;

void setup() {

  Serial.begin(115200);
  //Initialize serial and wait for port to open:
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  RPC.begin();

  Serial.println("Modbus TCP Server Wifi");
  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(1000);
  }
 
  // start the server
  wifiServer.begin();

  // start the Modbus TCP server
  if (!modbusTCPServer.begin()) {
    Serial.println("Failed to start Modbus TCP Server!");
    while (1)
      ;
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
      /************************************************************************
      Update the modbus holding registers
      RPC.call with parameter = 0 reads rpm from M4 and write it to H_0adr
      RPC.call with parameter = 1 reads brake ----------------------------
      RPC.call with parameter = 2 reads rpm_ref---------------------------
      *************************************************************************/ 
      int i;
      for(i = 0; i < 3; i++)  {
        modbusTCPServer.holdingRegisterWrite(i, (RPC.call("rpmRead", i).as<int>()));
      }
      delay(10);
    }
    Serial.println("client disconnected");
  }
}  
