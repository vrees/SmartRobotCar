#include "MotorController.h"

MotorController::MotorController(int pwmPin1, int pwmPin2, int shcpPin, int enPin, int dataPin, int stcpPin)
    : pwmPin1_(pwmPin1), pwmPin2_(pwmPin2), shcpPin_(shcpPin), enPin_(enPin), dataPin_(dataPin), stcpPin_(stcpPin) {}

void MotorController::begin() {
    pinMode(shcpPin_, OUTPUT);
    pinMode(enPin_, OUTPUT);
    pinMode(dataPin_, OUTPUT);
    pinMode(stcpPin_, OUTPUT);
    pinMode(pwmPin1_, OUTPUT);
    pinMode(pwmPin2_, OUTPUT);

    // Initialize PWM channels for motor control
    ledcSetup(0, 5000, 8); // PWM channel 0, 5000 Hz frequency, 8-bit resolution
    ledcAttachPin(pwmPin1_, 0); // Attach PWM channel 0 to pwmPin1_
    ledcAttachPin(pwmPin2_, 1); // Attach PWM channel 1 to pwmPin2_
}

void MotorController::motor(int dir, int speed) {
    digitalWrite(enPin_, LOW);

    // Map speed to the appropriate PWM range (0-255)
    int pwmSpeed = map(speed, 0, 255, 0, 255);

    ledcWrite(0, pwmSpeed); // Set pwmPin1_ speed
    ledcWrite(1, pwmSpeed); // Set pwmPin2_ speed

    digitalWrite(stcpPin_, LOW);
    shiftOut(dataPin_, shcpPin_, MSBFIRST, dir);
    digitalWrite(stcpPin_, HIGH);
}

void MotorController::stop() {
    motor(0, 0); // Stop the motor by setting speed to 0
}
