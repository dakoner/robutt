#include "PID_v1.h"
#include "DRV8835MotorShield_esp32.h"
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

DRV8835MotorShieldEsp32 motors;

//PID
double setpoint = 0;
double input, output;
double Kp = 3;
double Kd = 0.1;
double Ki = 0.5;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


void setup_wifi();
void setup_mqtt();
void reconnect_mqtt();
void setup_arduino_ota();

void mpu_9250_setup();
void mpu_9250_loop();

extern float yaw, pitch, roll;
extern PubSubClient pubsub_client;

long publish_delay = 5;


void setup()
{
  Serial.begin(115200);
  Serial.println("setup");

  setup_wifi();
  setup_arduino_ota();

  mpu_9250_setup();
  Serial.println("IMU active");

  if (MDNS.begin("robutt")) {
    Serial.println("MDNS responder started");
  }
  setup_mqtt();

  motors.setM1Speed(0);
  motors.setM2Speed(0);

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(publish_delay);
  pid.SetOutputLimits(-100, 100 );
}

long c = millis();

void loop()
{
   if (!pubsub_client.connected()) {
    reconnect_mqtt();
  }
  pubsub_client.loop();

  ArduinoOTA.handle();

  long delt_t = millis() - c;
  if (delt_t > publish_delay) { 
    
    mpu_9250_loop();
  
    input = pitch;
    pid.Compute();

    motors.setM1Speed(output);
    motors.setM2Speed(output);

    String msg = "{ \"setpoint\":" + String(setpoint) + ", \"pitch\":" + String(pitch) + ", \"output\":" + String(output) + " }";
    pubsub_client.publish("robutt/motor", msg.c_str());
    c = millis();
  }
}