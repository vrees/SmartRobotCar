/*
 * ESP32 Servo Example Using Arduino ESP32 Servo Library
 * John K. Bennett
 * March, 2017
 *
 * This sketch uses the Arduino ESP32 Servo Library to sweep 4 servos in sequence.
 *
 * Different servos require different pulse widths to vary servo angle, but the range is
 * an approximately 500-2500 microsecond pulse every 20ms (50Hz). In general, hobbyist servos
 * sweep 180 degrees, so the lowest number in the published range for a particular servo
 * represents an angle of 0 degrees, the middle of the range represents 90 degrees, and the top
 * of the range represents 180 degrees. So for example, if the range is 1000us to 2000us,
 * 1000us would equal an angle of 0, 1500us would equal 90 degrees, and 2000us would equal 1800
 * degrees.
 *
 * Circuit:
 * Servo motors have three wires: power, ground, and signal. The power wire is typically red,
 * the ground wire is typically black or brown, and the signal wire is typically yellow,
 * orange or white. Since the ESP32 can supply limited current at only 3.3V, and servos draw
 * considerable power, we will connect servo power to the VBat pin of the ESP32 (located
 * near the USB connector). THIS IS ONLY APPROPRIATE FOR SMALL SERVOS.
 *
 * We could also connect servo power to a separate external
 * power source (as long as we connect all of the grounds (ESP32, servo, and external power).
 * In this example, we just connect ESP32 ground to servo ground. The servo signal pins
 * connect to any available GPIO pins on the ESP32 (in this example, we use pins
 * 22, 19, 23, & 18).
 *
 * In this example, we assume four Tower Pro SG90 small servos.
 * The published min and max for this servo are 500 and 2400, respectively.
 * These values actually drive the servos a little past 0 and 180, so
 * if you are particular, adjust the min and max values to match your needs.
 * Experimentally, 550 and 2350 are pretty close to 0 and 180.
 */

#include <Arduino.h>
#include "CarServoController.h"
#include "MotorController.h"

// PWM control pin
const int PWM1_PIN = 16; // 5
const int PWM2_PIN = 27; // 6

// 74HCT595N Chip pins
const int SHCP_PIN = 26; // 2
const int EN_PIN = 14;   // 7
const int DATA_PIN = 12; // 8
const int STCP_PIN = 17; // 4

MotorController motorController(PWM1_PIN, PWM2_PIN, SHCP_PIN, EN_PIN, DATA_PIN, STCP_PIN);

CarServoController carServoController(13, 5, 23, 1000, 2000, 10);

void setup()
{
  // carServoController.setup();
  motorController.begin();
}

void loop()
{
  /* Forward */
  motorController.motor(Forward, 250);
  delay(2000);

  /* Backward */
  motorController.motor(Backward, 250);
  delay(2000);

  /* Turn_Left */
  motorController.motor(TurnLeft, 250);
  delay(2000);

  /* Turn_Right */
  motorController.motor(TurnRight, 250);
  delay(2000);

  /* Clockwise */
  motorController.motor(Clockwise, 250);
  delay(2000);
  /* Contrarotate */
  motorController.motor(Contrarotate, 250);
  delay(2000);

  /* Stop */
  motorController.stop();
  delay(2000);
}
