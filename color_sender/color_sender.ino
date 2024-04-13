#include <WiFi.h>
#include <WiFiUdp.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRsend.h>


const char* ssid = "RoBorregosHome";
const char* password = ""; // Set password
const int udpPort = 1234;

IRsend irsend(16);
IRrecv irrecv(17);
decode_results results;

WiFiUDP udp;

IPAddress staticIP(192, 168, 31, 10); // Set the desired static IP address
IPAddress gateway(192, 168, 31, 1);    // Set the gateway of your network
IPAddress subnet(255, 255, 255, 0);    // Set the subnet mask of your network

void setup() {
  Serial.begin(115200);
  Serial.println();
  irsend.begin();
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  
//  Serial.print("Default Gateway: ");
//  Serial.println(WiFi.gatewayIP());
//  Serial.print("Subnet Mask: ");
//  Serial.println(WiFi.subnetMask());
//  Serial.print("IP Address: ");
//  Serial.println(WiFi.localIP());

  WiFi.config(staticIP, gateway, subnet);

  Serial.print("Current ip: ");
  Serial.println(WiFi.localIP());
  

  udp.begin(udpPort);
  Serial.println("UDP server started");
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, udp.remoteIP().toString().c_str(), udp.remotePort());
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
      Serial.print("Entering first if");
      // Trim leading and trailing whitespaces
      String message_completed;
      for (int i = 0; i < len; i++) {
        char letter = incomingPacket[i];
        if (letter != ' ') {
          message_completed += letter;
        Serial.println(letter);
        Serial.println(message_completed);
        }
      }
      Serial.printf("UDP packet contents: %s\n", message_completed.c_str());
      // Send confirmation message
      if (message_completed == "OFF")
        irsend.sendNEC(0X00FFE21D);
      else if (message_completed == "ON")
        irsend.sendNEC(0x00FFA25D);
      else if (message_completed == "RED")
        irsend.sendNEC(0X00FF6897);
      else if (message_completed == "GREEN")
        irsend.sendNEC(0X00FF9867);
      else if (message_completed == "BLUE")
        irsend.sendNEC(0X00FFB04F);
    }
  }
  delay(10);
}