#include "PID_v1.h"
#include "DRV8835MotorShield_esp32.h"
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Encoder.h>

Encoder knobLeft(16, 17);
Encoder knobRight(18, 23);

long positionLeft, positionRight;

DRV8835MotorShieldEsp32 motors;

bool drive_mode = false;

//PID
double angle_setpoint = 0;

double velocity_setpoint_left = 0;
double velocity_setpoint_right = 0;

double setpoint_left = 0;
double setpoint_right = 0;
double input_left, output_left;
double input_right, output_right;

double Kp = 3;
double Kd = 0.1;
double Ki = 0.5;
PID pid_left(&input_left, &output_left, &setpoint_left, Kp, Ki, Kd, DIRECT);
PID pid_right(&input_right, &output_right, &setpoint_right, Kp, Ki, Kd, DIRECT);


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

  pid_left.SetMode(AUTOMATIC);
  pid_left.SetSampleTime(publish_delay);
  pid_left.SetOutputLimits(-400, 400 );
  pid_right.SetMode(AUTOMATIC);
  pid_right.SetSampleTime(publish_delay);
  pid_right.SetOutputLimits(-400, 400 );

  positionLeft = knobLeft.read();
  positionRight = knobRight.read();
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

    long newLeft, newRight;
    newLeft = knobLeft.read();
    newRight = knobRight.read();
    long dleft = newLeft - positionLeft;
    long dright = newRight - positionRight;

    if (!drive_mode) {
      input_left = pitch;
      input_right = pitch;
      setpoint_left = angle_setpoint;
      setpoint_right = angle_setpoint;
    } else {
      input_left = dleft;
      input_right = dright;
      setpoint_left = velocity_setpoint_left;
      setpoint_right = velocity_setpoint_right;
    }
    pid_left.Compute();
    pid_right.Compute();

    motors.setM1Speed(output_left);
    motors.setM2Speed(output_right);

    positionLeft = newLeft;
    positionRight = newRight;


    String msg = "{ \"pitch\":" + String(pitch) + 
                  ", \"drive_mode\":" + String(drive_mode) + 
                  ", \"setpoint_left\":" + String(setpoint_left) + 
                  ", \"output_left\":" + String(output_left) + 
                  ", \"setpoint_right\":" + String(setpoint_right) + 
                  ", \"output_right\":" + String(output_right) + 
                  ", \"dleft\":" + String(dleft) +
                  ", \"dright\":" + String(dright) +
                  ", \"positionLeft\":" + String(positionLeft) +
                  ", \"positionRight\":" + String(positionRight) +
                  " }";
    pubsub_client.publish("robutt/motor", msg.c_str());
    c = millis();
  }
}