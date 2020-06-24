#include <WiFi.h>
#include <WiFiMulti.h>

#include <HardwareSerial.h>


const char* ssid1 = "UPSTAIRS";
const char* ssid2 = "VADER";

const char* password = "";

WiFiMulti wifiMulti;

void setup_wifi()
{
  Serial.println();
  Serial.print("Connecting to wifi");

  wifiMulti.addAP(ssid1, password);
  wifiMulti.addAP(ssid2, password);
  if(wifiMulti.run() == WL_CONNECTED) {
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("SSID: ");
        Serial.println(WiFi.SSID());
  }
}