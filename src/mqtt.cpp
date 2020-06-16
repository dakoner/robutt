#include <PubSubClient.h>
#include <WiFi.h>

// MQTT
const char *mqtt_server = "gork";
WiFiClient espClient;
PubSubClient pubsub_client(espClient);

void setup_mqtt() {
  pubsub_client.setServer(mqtt_server, 1883);
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
      pubsub_client.subscribe("esp32/output");
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