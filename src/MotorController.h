#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

const int Forward = 92;       // forward
const int Backward = 163;     // back
const int TurnLeft = 149;     // left translation
const int TurnRight = 106;    // Right translation
const int TopLeft = 20;       // Upper left mobile
const int BottomLeft = 129;   // Lower left mobile
const int TopRight = 72;      // Upper right mobile
const int BottomRight = 34;   // The lower right move
const int Stop = 0;           // stop
const int Contrarotate = 172; // Counterclockwise rotation
const int Clockwise = 83;     // Rotate clockwise

class MotorController
{
public:
    MotorController(int pwmPin1, int pwmPin2, int shcpPin, int enPin, int dataPin, int stcpPin);
    void begin();
    void motor(int dir, int speed);
    void stop();

private:
    int pwmPin1_;
    int pwmPin2_;
    int shcpPin_;
    int enPin_;
    int dataPin_;
    int stcpPin_;
};

#endif
