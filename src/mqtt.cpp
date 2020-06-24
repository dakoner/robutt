#include <PubSubClient.h>
#include <WiFi.h>
#include "PID_v1.h"

// MQTT
const char *mqtt_server = "gork";
WiFiClient espClient;
PubSubClient pubsub_client(espClient);



extern double Kp, Kd, Ki;
extern double setpoint;

extern PID pid;

void callback(char *topic, byte *message, unsigned int length)
{
  Serial.println("callback");
  String messageTemp;

  for (int i = 0; i < length; i++)  {
    messageTemp += (char)message[i];
  }

Serial.println("Got message:");
Serial.println(topic);
Serial.println(messageTemp);
String st = String(topic);
  if (st == "robutt/p") {
    Kp = messageTemp.toDouble();
    pid.SetTunings(Kp, Ki, Kd);
  } else if (st == "robutt/i") {
    Ki = messageTemp.toDouble();
    pid.SetTunings(Kp, Ki, Kd);

  } else if (st == "robutt/d") {
    Kd = messageTemp.toDouble();
    pid.SetTunings(Kp, Ki, Kd);
  }
  else if (st == "robutt/sp") {
    setpoint = messageTemp.toDouble();
  }
}

void setup_mqtt() {
  pubsub_client.setServer(mqtt_server, 1883);
  pubsub_client.setCallback(callback);

}

void reconnect_mqtt()
{
  // Loop until we're reconnected
  while (!pubsub_client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (pubsub_client.connect("ESP8266Client"))
    {
      Serial.println("connected");
      // Subscribe
      pubsub_client.subscribe("robutt/p");
      pubsub_client.subscribe("robutt/i");
      pubsub_client.subscribe("robutt/d");
      pubsub_client.subscribe("robutt/sp");

    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(pubsub_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
