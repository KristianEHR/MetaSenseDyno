
/*----------------------------------------------------------------------------
// This sketch controls a dyno and measures data from this.
 It runs on Arduino GIGA R1 wifi on core M7 and has following features:
// PID
// Modbus TCP Client for communication with WAGO PLC
// Modbus TCP Server for communication with e.g. Modbus Poll
// RPC to core M4
// Author: Kristian Ehrhorn
// Date:17-07-2025
---------------------------------------------------------------------------------*/
#include "Arduino.h"
#include "RPC.h"
#include <Arduino_BuiltIn.h>
#include <SPI.h>
#include <WiFi.h>
#include "ModbusTCPServer.h"
#include "ModbusTCPClient.h"
#include <ArduinoRS485.h>  // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include "arduino_secrets.h"


#define GR 7.2727  //16/11 * 5
#define Brake_K 5.5555e-2
#define RPM_K 4.4
#define TC1_K 0.19536                   //800C/4095 scale tc1
#define TC2_K 0.03663                   //150C/4095 scale tc2
#define MB_CLIENT_CONNECT_INTERVAL 200  //Poll every 200 ms even if the mosbus poll scan rate is faster

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS, i;
double h_reg[20];
const double k_reg[20] = { RPM_K, 1, RPM_K, 1 / GR, 1, 0, 0, Brake_K, 1, 1, 4.579e-2, 2.4415e-1, 1, 1, 1, 1, 1, 1, 1, 1 };
unsigned long prev = 0, now;  // will store last time LED was updated

//IPAddress server(192, 168, 0, 181);  // local numeric IP used by modbus poll
// Create wifi Client
WiFiClient M_client;
// numeric IP for WAGO PLC
IPAddress WAGO_server(192, 168, 0, 164);
// Create local wifiServer Port 502
WiFiServer wifiServer(502);
// Create Modbus local server
ModbusTCPServer modbusTCPServer;
// Create Modbus local client
ModbusTCPClient modbusTCPClient(M_client);

void setup() {

  RPC.begin();

  //Serial.begin(115200);
  //Initialize serial and wait for port to open:
  //  while (!Serial) {
  //    ;  // wait for serial port to connect. Needed for native USB port only
  //  }
  //  Serial.println("Modbus TCP Server and client");

  // connect to WiFi network:
  while (status != WL_CONNECTED) {
    //    Serial.print("Attempting to connect to SSID: ");
    //    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    wifiServer.begin();
    //  delay(10);
    // start the Modbus TCP server
    if (!modbusTCPServer.begin()) {
      // Serial.println("Failed to start Modbus TCP Server!");
      //    while (1);
    }
    //Serial.println("server running");
    // configure a single coil at address 0x00
    modbusTCPServer.configureCoils(0x00, 1);
    // configure 20 holding registers at address 0x00
    modbusTCPServer.configureHoldingRegisters(0x00, 20);
    // wait 2 seconds for connection:
    delay(10000);
  }
  // you're connected now, so print out the data:
  //  Serial.print("You're connected to the network");
  //  printCurrentNet();
  //printWiFiStatus();
}

void loop() {

  // Server listen for incoming clients
  WiFiClient Sclient = wifiServer.available();
  if (Sclient) {
    // a new client connected
    //  Serial.println("New client connected");
    modbusTCPServer.accept(Sclient);

    while (Sclient.connected()) {
      // poll for Modbus TCP requests, while client connected
      now = millis();
      modbusTCPServer.poll();
      // fetch, scale and update the modbus registers with analog channels 0-8
      for (int j = 0; j < 20; j++) {
        if (j < 10) {
          h_reg[j] = RPC.call("rpmRead", j).as<int>() * k_reg[j];
        }
        // Power (kW) calculation P= 2 pi RPM/60 * Torque
        if (j == 9) h_reg[9] = 0.010472 * 0.1 * h_reg[0] * h_reg[3];  //Power  2 * 3.1416 * h_reg[0] * h_reg[3] / 60
        modbusTCPServer.holdingRegisterWrite(j, h_reg[j]);            // write to modbus holding registers
      }
      if ((now - prev) > MB_CLIENT_CONNECT_INTERVAL) {
        prev = now;
        // connect modbusTCPClient
        if (!modbusTCPClient.connected()) {
          // client not connected, start the Modbus TCP client
          //Serial.println("Attempting to connect to Modbus TCP server");
          if (!modbusTCPClient.begin(WAGO_server)) {
            //Serial.println("Modbus TCP Client failed to connect!");
          } else {
            //Serial.println("Modbus TCP Client connected");
          }
        } else {
          -- -- -- -- -- -- -- -- -- -delay(5);
        }
      }
    }

    //Read Holding Register values from the modbus server and store- in h_reg[]
    void readHoldingRegisterValues(int id, int function, int startadr, int number) {
      // Serial.print("Reading Holding Register values ... ");
      // Read 10 Holding  Register values from (server) id 255, address 0x00
      if (!modbusTCPClient.requestFrom(id, function, startadr, number)) {
        //Serial.print("Failed to read holding regs! ");
        //Serial.println(modbusTCPClient.lastError());
      } else {
        int i = 0;
        while (modbusTCPClient.available()) {
          h_reg[i + 10] = k_reg[i + 10] * (int)modbusTCPClient.read();
          i++;
        }
      }
    }

    void printWiFiStatus() {
      // print the SSID of the network you're attached to:
      Serial.print("SSID: ");
      Serial.println(WiFi.SSID());

      // print your board's IP address:
      IPAddress ip = WiFi.localIP();
      Serial.print("IP Address: ");
      Serial.println(ip);

      // print the received signal strength:
      long rssi = WiFi.RSSI();
      Serial.print("signal strength (RSSI):");
      Serial.print(rssi);
      Serial.println(" dBm");
    }
    void printWifiData() {
      // print your board's IP address:
      IPAddress ip = WiFi.localIP();
      Serial.print("IP Address: ");
      Serial.println(ip);

      // print your MAC address:
      byte mac[6];
      WiFi.macAddress(mac);
      Serial.print("MAC address: ");
      printMacAddress(mac);
    }

    void printCurrentNet() {
      // print the SSID of the network you're attached to:
      Serial.print("SSID: ");
      Serial.println(WiFi.SSID());

      // print the MAC address of the router you're attached to:
      byte bssid[6];
      WiFi.BSSID(bssid);
      Serial.print("BSSID: ");
      printMacAddress(bssid);

      // print the received signal strength:
      long rssi = WiFi.RSSI();
      Serial.print("signal strength (RSSI):");
      Serial.println(rssi);

      // print the encryption type:
      byte encryption = WiFi.encryptionType();
      Serial.print("Encryption Type:");
      Serial.println(encryption, HEX);
      Serial.println();
    }

    void printMacAddress(byte mac[]) {
      for (int i = 5; i >= 0; i--) {
        if (mac[i] < 16) {
          Serial.print("0");
        }
        Serial.print(mac[i], HEX);
        if (i > 0) {
          Serial.print(":");
        }
      }
      Serial.println();
    }
