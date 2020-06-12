#include "PID_v1.h"

#include <ESP32WebServer.h>

extern double Kp, Kd, Ki;
extern double setpoint;
extern PID pid;

ESP32WebServer server(80);
bool mpu_9250_probe();

void handleRoot() {
  server.send(200, "text/plain", "hello from esp8266!");
}

void handleParams() {
  String params;
  params += String(setpoint);
  params += " ";
  params += String(Kp);
  params += " ";
  params += String(Kd);
  params += " ";
  params += String(Ki);
  server.send(200, "text/plain", params);
}
extern double input, output;

void handleState() {
  String state;
  state += String(input);
  state += " ";
  state += String(output);
  server.send(200, "text/plain", state);
}


void handleImuState() {
  String state;
  state += String(mpu_9250_probe());

  server.send(200, "text/plain", state);
}

extern int ENA, IN1, IN2, IN3, IN4, ENB;

void handleSetParams() {
  String message;
  for (uint8_t i=0; i<server.args(); i++){
    if (server.argName(i) == "p") {
      Kp = server.arg(i).toFloat(); 
    } else if (server.argName(i) == "i") {
      Ki = server.arg(i).toFloat(); 
    } else if (server.argName(i) == "d") {
      Kd = server.arg(i).toFloat(); 
    } else if (server.argName(i) == "sp") {
      setpoint = server.arg(i).toFloat(); 
    } else {
      message += "Unknown arg; ";
      message += server.argName(i);
      message += "\n";
    }
  }
  
  pid.SetTunings(Kp, Ki, Kd);

  Serial.println(message);
  server.send(200, "text/plain", message);
}


void handleNotFound(){
  server.send(404, "text/plain", "Not found");
}

void webserver_setup() {
  server.on("/", handleRoot);
  server.on("/params", handleParams);
  server.on("/state", handleState);
  server.on("/imustate", handleImuState);
  server.on("/setParams", handleSetParams);

  server.onNotFound(handleNotFound);

  server.begin();
}

void webserver_loop() {
  server.handleClient();
}

