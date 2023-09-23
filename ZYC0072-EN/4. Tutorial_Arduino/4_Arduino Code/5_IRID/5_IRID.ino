#include "IR_remote.h"
#include "keymap.h"
#include <Servo.h>

// Infrared receiving control pin
#define RECV_PIN            3
// PWM control pin
#define PWM1_PIN            5
#define PWM2_PIN            6
// 74HCT595N chip pin
#define SHCP_PIN            2                               // The displacement of the clock
#define EN_PIN              7                               // Can make control
#define DATA_PIN            8                               // Serial data
#define STCP_PIN            4                               // Memory register clock         
// Servo control pin
#define CLAW_PIN            9
#define ARM_PIN             10
#define BASE_PIN            11
// Ultrasonic control pin
const int Trig       =      12;
const int Echo       =      13;
// Trace the control pin
#define LEFT_LINE_TRACJING      A0
#define CENTER_LINE_TRACJING    A1
#define right_LINE_TRACJING     A2

const int Forward       = 92;                               // Forward
const int Backward      = 163;                              // Backward
const int Turn_Left     = 149;                              // Left translation
const int Turn_Right    = 106;                              // Right translation
const int Top_Left      = 20;                               // Upper left mobile
const int Bottom_Left   = 129;                              // Lower left mobile
const int Top_Right     = 72;                               // Upper right mobile
const int Bottom_Right  = 34;                               // The lower right move
const int Stop          = 0;                                // Stop
const int Contrarotate  = 172;                              // Counterclockwise rotation
const int Clockwise     = 83;                               // Rotate clockwise

int base_degress = 90;
int arm_degress = 90;
int claw_degress = 90;
boolean Line_tracking_Function_flag = false;
boolean Avoidance_Function_flag = false;
boolean Following_Function_flag = false;
int Left_Tra_Value;
int Center_Tra_Value;
int Right_Tra_Value;
int Black_Line = 500;

IRremote IR(RECV_PIN);
Servo   clawservo;
Servo   armservo;
Servo   baseservo;

void setup()
{
    pinMode(SHCP_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    pinMode(DATA_PIN, OUTPUT);
    pinMode(STCP_PIN, OUTPUT);
    pinMode(PWM1_PIN, OUTPUT);
    pinMode(PWM2_PIN, OUTPUT);

    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);

    pinMode(LEFT_LINE_TRACJING, INPUT);
    pinMode(CENTER_LINE_TRACJING, INPUT);
    pinMode(right_LINE_TRACJING, INPUT);

    clawservo.attach(CLAW_PIN);
    armservo.attach(ARM_PIN);
    baseservo.attach(BASE_PIN);
    clawservo.write(135);
    armservo.write(45);
    baseservo.write(90);
    delay(500);

    Motor(Stop, 0);

    Serial.begin(9600);

}

void loop()
{
    switch (IR.getIrKey(IR.getCode(), IR_TYPE_EM))
    {
        case EM_IR_KEYCODE_UP:      // Forward
            Motor(Forward, 200);
            delay(200);
            break;
        case EM_IR_KEYCODE_DOWN:    // Backward
            Motor(Backward, 200);
            delay(200);
            break;
        case EM_IR_KEYCODE_LEFT:    // Turn_Left
            Motor(Turn_Left, 200);
            delay(200);
            break;
        case EM_IR_KEYCODE_RIGHT:    // Turn_Right
            Motor(Turn_Right, 200);
            delay(200);
            break;
        case EM_IR_KEYCODE_D:       // Top_Left
            Motor(Top_Left, 200);
            delay(200);
            break;
        case EM_IR_KEYCODE_PLUS:    // Top_Right
            Motor(Top_Right, 200);
            delay(200);
            break;
        case EM_IR_KEYCODE_0:       // Bottom_Left
            Motor(Bottom_Left, 200);
            delay(200);
            break;
        case EM_IR_KEYCODE_REDUCE:       // Bottom_Right
            Motor(Bottom_Right, 200);
            delay(200);
            break;
        case EM_IR_KEYCODE_OK:       // Clockwise
            Motor(Clockwise, 200);
            delay(200);
            break;
        case EM_IR_KEYCODE_4:       // Claws open
            claw_degress = claw_degress - 3;
            if (claw_degress <= 50) 
            {
                claw_degress = 50;
            }
            clawservo.write(claw_degress);
            delay(2);
            break;
        case EM_IR_KEYCODE_7:       // Paw closed
            claw_degress = claw_degress + 3;
            if (claw_degress >= 180) 
            {
                claw_degress = 180;
            }
            clawservo.write(claw_degress);
            delay(2);
            break;
        case EM_IR_KEYCODE_5:       // Arm forward
            arm_degress = arm_degress + 3;
            if (arm_degress >= 150) 
            {
                arm_degress = 150;
            }
            armservo.write(arm_degress);
            delay(2);
            break;
        case EM_IR_KEYCODE_8:       // Arm backwards
            arm_degress = arm_degress - 3;
            if (arm_degress <= 0) 
            {
                arm_degress = 0;
            }
            armservo.write(arm_degress);
            delay(2);
            break;
        case EM_IR_KEYCODE_6:       // Chassis left pendulum
            base_degress = base_degress + 3;
            if (base_degress >= 180) 
            {
                base_degress = 180;
            }
            baseservo.write(base_degress);
            delay(2);
            break;
        case EM_IR_KEYCODE_9:       // Chassis side
            base_degress = base_degress - 5;
            if (base_degress <= 0) 
            {
                base_degress = 0;
            }
            baseservo.write(base_degress);
            delay(2);
            break;
        case EM_IR_KEYCODE_1:       // Tracking mode
            Motor(Stop, 0);
            Line_tracking_Function();
            delay(500);
            break;
        case EM_IR_KEYCODE_2:       // Obstacle avoidance mode
            Motor(Stop, 0);
            Avoidance_Function();
            delay(500);
            break;
        case EM_IR_KEYCODE_3:       // Tracking mode
            Motor(Stop, 0);
            Following_Function();
            delay(500);
            break;
        default:
            Motor(Stop, 0);
            break;
    }
}

void Motor(int Dir, int Speed)
{
    digitalWrite(EN_PIN, LOW);
    analogWrite(PWM1_PIN, Speed);
    analogWrite(PWM2_PIN, Speed);

    digitalWrite(STCP_PIN, LOW);
    shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, Dir);
    digitalWrite(STCP_PIN, HIGH);
}

float SR04(int Trig, int Echo)
{
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);
    float distance = pulseIn(Echo, HIGH) / 58.00;
    delay(10);
    return distance;
}

void Line_tracking_Function()           // Tracking mode
{
    Line_tracking_Function_flag = true;
    delay(500);
    while (Line_tracking_Function_flag)
    {
        Left_Tra_Value = analogRead(LEFT_LINE_TRACJING);
        Center_Tra_Value = analogRead(CENTER_LINE_TRACJING);
        Right_Tra_Value = analogRead(right_LINE_TRACJING);
        if (Left_Tra_Value < Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line)
        {
            Motor(Forward, 175);
        }
        else if (Left_Tra_Value >= Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line)
        {
            Motor(Contrarotate, 165);
        }
        else if (Left_Tra_Value >= Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value < Black_Line)
        {
            Motor(Contrarotate, 190);
        }
        else if (Left_Tra_Value < Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line)
        {
            Motor(Clockwise, 190);
        }
        else if (Left_Tra_Value < Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line)
        {
            Motor(Clockwise, 165);
        }
        else if (Left_Tra_Value >= Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line)
        {
            Motor(Stop, 0);
        }
        if (IR.getIrKey(IR.getCode(), IR_TYPE_EM) == EM_IR_KEYCODE_1) 
        {
            Line_tracking_Function_flag = false;
            Motor(Stop, 0);
            delay(1000);
        }
    }
}

void Following_Function()
{
    int Following_distance = 0;
    Following_Function_flag = true;
    delay(500);
    while (Following_Function_flag)
    {
        Following_distance = SR04(Trig, Echo);
        if (Following_distance < 15)
        {
            Motor(Backward, 200);
        }
        else if (15 <= Following_distance && Following_distance <= 20)
        {
            Motor(Stop, 0);
        }
        else if (20 <= Following_distance && Following_distance <= 25)
        {
            Motor(Forward, 180);
        }
        else if (25 <= Following_distance && Following_distance <= 50)
        {
            Motor(Forward, 220);
        }
        else
        {
            Motor(Stop, 0);
        }
        if (IR.getIrKey(IR.getCode(), IR_TYPE_EM) == EM_IR_KEYCODE_3)
        {
            Following_Function_flag = false;
            Motor(Stop, 0);
            delay(1000);
        }
    }
}

void Avoidance_Function()
{
    int Avoidance_distance = 0;
    Avoidance_Function_flag = true;
    delay(500);
    while (Avoidance_Function_flag)
    {
        Avoidance_distance = SR04(Trig, Echo);
        if (Avoidance_distance <= 25) 
        {
            if (Avoidance_distance <= 15) 
            {
                Motor(Stop, 0);
                delay(100);
                Motor(Backward, 180);
                delay(600);
                Motor(Clockwise, 180);
                delay(200);
            } 
            else
            {
                Motor(Stop, 0);
                delay(100);
                Motor(Backward, 180);
                delay(300);
                Motor(Contrarotate, 180);
                delay(600);
            }
        } 
        else 
        {
            Motor(Forward, 180);
        }
        if (IR.getIrKey(IR.getCode(), IR_TYPE_EM) == EM_IR_KEYCODE_2)
        {
            Avoidance_Function_flag = false;
            Motor(Stop, 0);
            delay(1000);
        }
    }
}