#ifndef CarServoController_h
#define CarServoController_h

#include <Arduino.h>
#include <ESP32Servo.h>

class CarServoController {
public:
  CarServoController(int servo1Pin, int servo2Pin, int servo3Pin, int minUs, int maxUs, int delayMsec);
  void setup();
  void loop();

private:
  Servo servo1;
  Servo servo2;
  Servo servo3;
  int servo1Pin;
  int servo2Pin;
  int servo3Pin;
  int minUs;
  int maxUs;
  int delayMsec;
  int pos;
  ESP32PWM pwm;
};

#endif
