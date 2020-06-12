#include "PID_v1.h"

#include "DRV8835MotorShield_esp32.h"

#include "Wire.h"

#include <WiFi.h>
#include <ESPmDNS.h>
DRV8835MotorShieldEsp32 motors;

const char* ssid = "UPSTAIRS";
const char* password = "Recurser";

//PID

double setpoint = 0;
double input, output;
double Kp = 5;
double Kd = 0.5;
double Ki = 0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


void mpu_9250_setup();
void mpu_9250_loop();

void webserver_setup();
void webserver_loop();

extern float yaw, pitch, roll;

void setup()
{
  Serial.begin(115200);
  Serial.println("setup");

  mpu_9250_setup();
  Serial.println("IMU active");


  WiFi.begin(ssid, password);
  int i = 0;
  while (WiFi.waitForConnectResult() != WL_CONNECTED && i < 10) {
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    delay(250);
    Serial.print(".");
    ++i;
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("robutt")) {
    Serial.println("MDNS responder started");
  }


  webserver_setup();

  motors.setM1Speed(0);
  motors.setM2Speed(0);

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-128, 128 );

}

long c = millis();

void loop()
{
  mpu_9250_loop();
  
  input = pitch;
  pid.Compute();
  long delt_t = millis() - c;
  if (delt_t > 10) { // update LCD once per half-second independent of read rate
   // Serial.print("Motor speed: ");
    //Serial.println(output);
    c = millis();
  }
  motors.setM1Speed(output);
  motors.setM2Speed(output);
  webserver_loop();
  
}

