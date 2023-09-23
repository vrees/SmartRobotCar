#include <MsTimer2.h>
#include    <Servo.h>

// PWM control pin
#define PWM1_PIN            5
#define PWM2_PIN            6
// 74HCT595N chip pin
#define SHCP_PIN            2                               // The displacement of the clock
#define EN_PIN              7                               // Can make control
#define DATA_PIN            8                               // Serial data
#define STCP_PIN            4                               // Memory register clock                  
// Ultrasonic control pin
const int Trig       =      12;
const int Echo       =      13;
// Trace the control pin
#define LEFT_LINE_TRACJING      A0
#define CENTER_LINE_TRACJING    A1
#define right_LINE_TRACJING     A2
// Servo control pin
#define CLAW_PIN            9
#define ARM_PIN             10
#define BASE_PIN            11

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


byte TX_package[5] = {0xA5, 0, 0, 0, 0x5A};                 // Packet header(0xA5) +  original data (n*byte) + inspection sum(1byte) + Package the tail(0x5A)
byte RX_package[10] = {0};
int UT_distance = 0;
int Serialcount = 0;
int base_degrees = 90;
int arm_degrees = 90;
int claw_degrees = 90;
boolean menory_action_flag;
boolean Line_tracking_Function_flag = false;
boolean Avoidance_Function_flag = false;
boolean Following_Function_flag = false;
boolean Jail_Function_flag = false;
int Left_Tra_Value;
int Center_Tra_Value;
int Right_Tra_Value;
int Black_Line = 500;
int actions_count = 0;
int auto_count;
int claw_read_degress[20] = {0, 0, 0};
int arm_read_degress[20] = {0, 0, 0};
int base_read_degress[20] = {0, 0, 0};

typedef struct 
{
    byte mode1;             // Bit0: free mode;Bit1: tracking mode;Bit2: Obstacle avoidance mode;Bit3: Follow mode;
                            // Bit4: Dungeon Mode;Bit5: Save button;Bit6: Automatic button;Bit7: empty
    byte mode2;             // Bit0: fluke;Bit1: closed claw;Bit2: clockwise rotation;Bit3: reverse;
    char x_axis = 0;        // Store variables on the X axis
    char y_axis = 0;        // Store the variables on the Y axis
    byte C_speed = 127;     // Speed of storage cart
    char x_Base = 0;        // Store the steering gear on the X axis
    char y_Arm = 0;         // Store the steering gear on the Y-axis
}rx_buffer;

rx_buffer RX_buffer;

Servo clawservo;
Servo armservo;
Servo baseservo;

void Timer2Isr()
{
    sei();
    UT_distance = SR04(Trig, Echo);
}
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
    clawservo.write(claw_degrees);
    delay(500);
    armservo.write(arm_degrees);
    delay(500);
    baseservo.write(base_degrees);
    delay(500);

    Motor(Stop, 0);

    Serial.begin(9600);

    //5ms timed interrupt Settings use timer2    
    MsTimer2::set(100, Timer2Isr);
    MsTimer2::start();
}

void loop()
{
    TX_Information(UT_distance, auto_count);                    // Send ultrasonic data
    RX_Information();                                           // Receiving Bluetooth data
    switch (RX_buffer.mode1)
    {
        case 0x02:       //tracking mode
            Motor(Stop, 0);
            Line_tracking_Function();
            delay(10);
            break;
        case 0x04:       //Obstacle avoidance mode
            Motor(Stop, 0);
            Avoidance_Function();
            delay(10);
            break;
        case 0x08:       //Follow the pattern
            Motor(Stop, 0);
            Following_Function();
            delay(10);
            break;
        case 0x10:       //Dungeon mode
            Motor(Stop, 0);
            Jail_Function();
            delay(10);
            break;
        case 0x40:       //Automatic mode
            Motor(Stop, 0);
            auto_doit();
            delay(10);
            break;
        default:
            free_mode();
            break;
    }
}

void Line_tracking_Function()       // tracking mode
{
    Line_tracking_Function_flag = true;
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
        RX_Information();
        if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
        {
            Line_tracking_Function_flag = false;
        }
    }
}

void Avoidance_Function()           // Obstacle avoidance mode
{
    int i;
    Avoidance_Function_flag = true;
    while (Avoidance_Function_flag)
    {
        if (UT_distance <= 25)
        {
            if (UT_distance <= 15)
            {
                Motor(Stop, 0);
                /* 为了避免堵塞串口接收，延时不能太长，于是分步延时 */
                for(i = 0; i < 2; i++)
                {
                    delay(50);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        Avoidance_Function_flag = false;
                        return;
                    }
                }
                Motor(Backward, 180);
                /* 为了避免堵塞串口接收，延时不能太长，于是分步延时 */
                for(i = 0; i < 12; i++)
                {
                    delay(50);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        Avoidance_Function_flag = false;
                        return;
                    }
                }
                Motor(Clockwise, 180);
                /* 为了避免堵塞串口接收，延时不能太长，于是分步延时 */
                for(i = 0; i < 4; i++)
                {
                    delay(50);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        Avoidance_Function_flag = false;
                        return;
                    }
                }
            }
            else
            {
                Motor(Stop, 0);
                /* 为了避免堵塞串口接收，延时不能太长，于是分步延时 */
                for(i = 0; i < 2; i++)
                {
                    delay(50);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        Avoidance_Function_flag = false;
                        return;
                    }
                }
                Motor(Backward, 180);
                /* 为了避免堵塞串口接收，延时不能太长，于是分步延时 */
                for(i = 0; i < 6; i++)
                {
                    delay(50);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        Avoidance_Function_flag = false;
                        return;
                    }
                }
                Motor(Contrarotate, 180);
                /* 为了避免堵塞串口接收，延时不能太长，于是分步延时 */
                for(i = 0; i < 12; i++)
                {
                    delay(50);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        Avoidance_Function_flag = false;
                        return;
                    }
                }
            }
        }
        else
        {
            Motor(Forward, 180);
        }
        RX_Information();
        if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
        {
            Avoidance_Function_flag = false;
        }
    }
}

void Following_Function()           // Follow the pattern
{
    Following_Function_flag = true;
    while (Following_Function_flag)
    {
        if (UT_distance < 15)
        {
            Motor(Backward, 200);
        }
        else if (15 <= UT_distance && UT_distance <= 20)
        {
            Motor(Stop, 0);
        }
        else if (20 <= UT_distance && UT_distance <= 25)
        {
            Motor(Forward, 180);
        }
        else if (25 <= UT_distance && UT_distance <= 50)
        {
            Motor(Forward, 220);
        }
        else
        {
            Motor(Stop, 0);
        }
        RX_Information();
        if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x10)
        {
            Following_Function_flag = false;
        }
    }
}

void Jail_Function()                // Dungeon mode
{
    int i;
    Jail_Function_flag = true;
    while (Jail_Function_flag)
    {
        Left_Tra_Value = analogRead(LEFT_LINE_TRACJING);
        Center_Tra_Value = analogRead(CENTER_LINE_TRACJING);
        Right_Tra_Value = analogRead(right_LINE_TRACJING);
        if (Left_Tra_Value < Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value < Black_Line)
        {
            Motor(Forward, 150);
        }
        else if (Left_Tra_Value >= Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value < Black_Line)
        {
            Motor(Backward,150);
            /* 为了避免堵塞串口接收，延时不能太长，于是分步延时 */
            for(i = 0; i < 4; i++)
            {
                delay(50);
                RX_Information();
                if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08)
                {
                    Jail_Function_flag = false;
                    return;
                }
            }
            Motor(Clockwise,160);
            /* 为了避免堵塞串口接收，延时不能太长，于是分步延时 */
            for(i = 0; i < 10; i++)
            {
                delay(50);
                RX_Information();
                if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08)
                {
                    Jail_Function_flag = false;
                    return;
                }
            }
        }
        else if (Left_Tra_Value >= Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line)
        {
            Motor(Backward,150);
             /* 为了避免堵塞串口接收，延时不能太长，于是分步延时 */
            for(i = 0; i < 4; i++)
            {
                delay(50);
                RX_Information();
                if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08)
                {
                    Jail_Function_flag = false;
                    return;
                }
            }
            Motor(Clockwise,160);
            for(i = 0; i < 12; i++)
            {
                delay(50);
                RX_Information();
                if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08)
                {
                    Jail_Function_flag = false;
                    return;
                }
            }
        }
        else if (Left_Tra_Value < Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line)
        {
            Motor(Backward,150);
            for(i = 0; i < 4; i++)
            {
                delay(50);
                RX_Information();
                if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08)
                {
                    Jail_Function_flag = false;
                    return;
                }
            }
            Motor(Contrarotate,160);
            for(i = 0; i < 10; i++)
            {
                delay(50);
                RX_Information();
                if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08)
                {
                    Jail_Function_flag = false;
                    return;
                }
            }
        }
        else if (Left_Tra_Value < Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line)
        {
            Motor(Backward,150);
             /* 为了避免堵塞串口接收，延时不能太长，于是分步延时 */
            for(i = 0; i < 4; i++)
            {
                delay(50);
                RX_Information();
                if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08)
                {
                    Jail_Function_flag = false;
                    return;
                }
            }
            Motor(Contrarotate,160);
            for(i = 0; i < 12; i++)
            {
                delay(50);
                RX_Information();
                if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08)
                {
                    Jail_Function_flag = false;
                    return;
                }
            }
        }
        else
        {
            Motor(Backward,150);
            for(i = 0; i < 12; i++)
            {
                delay(50);
                RX_Information();
                if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08)
                {
                    Jail_Function_flag = false;
                    return;
                }
            }
            Motor(Clockwise,160);
            for(i = 0; i < 10; i++)
            {
                delay(50);
                RX_Information();
                if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08)
                {
                    Jail_Function_flag = false;
                    return;
                }
            }
        }
        RX_Information();
        if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08)
        {
            Jail_Function_flag = false;
        }
    }
}

void auto_doit()                      // Automatic mode
{
    if (0 != auto_count)
    {
        menory_action_flag = true;
    }
    actions_count = 0;
    claw_degrees = clawservo.read();
    arm_degrees = armservo.read();
    base_degrees = baseservo.read();
    while (menory_action_flag)
    {
        for (int i = (1); i <= (auto_count); i = i + (1))
        {
            RX_Information();
            if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
            {
                menory_action_flag = false;
                return;
            }
            if (claw_degrees < claw_read_degress[(int)(i - 1)])
            {
                while (claw_degrees < claw_read_degress[(int)(i - 1)])
                {
                    claw_degrees = claw_degrees + 1;
                    clawservo.write(claw_degrees);
                    delay(15);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        menory_action_flag = false;
                        return;
                    }
                }
            }
            else
            {
                while (claw_degrees > claw_read_degress[(int)(i - 1)])
                {
                    claw_degrees = claw_degrees - 1;
                    clawservo.write(claw_degrees);
                    delay(15);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        menory_action_flag = false;
                        return;
                    }
                }
            }
            RX_Information();
            if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
            {
                menory_action_flag = false;
                return;
            }
            if (arm_degrees < arm_read_degress[(int)(i - 1)])
            {
                while (arm_degrees < arm_read_degress[(int)(i - 1)])
                {
                    arm_degrees = arm_degrees + 1;
                    armservo.write(arm_degrees);
                    delay(15);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        menory_action_flag = false;
                        return;
                    }
                }
            }
            else
            {
                while (arm_degrees > arm_read_degress[(int)(i - 1)])
                {
                    arm_degrees = arm_degrees - 1;
                    armservo.write(arm_degrees);
                    delay(15);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        menory_action_flag = false;
                        return;
                    }
                }
            }
            RX_Information();
            if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
            {
                menory_action_flag = false;
                return;
            }
            if (base_degrees < base_read_degress[(int)(i - 1)])
            {
                while (base_degrees < base_read_degress[(int)(i - 1)])
                {
                    base_degrees = base_degrees + 1;
                    baseservo.write(base_degrees);
                    delay(15);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        menory_action_flag = false;
                        return;
                    }
                }
            }
            else
            {
                while (base_degrees > base_read_degress[(int)(i - 1)])
                {
                    base_degrees = base_degrees - 1;
                    baseservo.write(base_degrees);
                    delay(15);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        menory_action_flag = false;
                        return;
                    }
                }
            }
            RX_Information();
            if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
            {
                menory_action_flag = false;
                return;
            }
        }
    }
}

void free_mode()                    // free mode
{
    if(RX_buffer.x_axis >= -30 && RX_buffer.x_axis <= 30 && RX_buffer.y_axis >= 30)     //Forward
    {
        Motor(Forward, RX_buffer.C_speed);
        delay(5);
    }
    else if (RX_buffer.x_axis >= -30 && RX_buffer.x_axis <= 30 && RX_buffer.y_axis <= -30)     //Backward
    {
        Motor(Backward, RX_buffer.C_speed);
        delay(5);
    }
    else if (RX_buffer.y_axis >= -30 && RX_buffer.y_axis <= 30 && RX_buffer.x_axis <= -30)     //Turn_Left
    {
        Motor(Turn_Left, RX_buffer.C_speed);
        delay(5);
    }
    else if (RX_buffer.y_axis >= -30 && RX_buffer.y_axis <= 30 && RX_buffer.x_axis >= 30)     //Turn_Right
    {
        Motor(Turn_Right, RX_buffer.C_speed);
        delay(5);
    }
    else if (RX_buffer.x_axis <= -30 && RX_buffer.y_axis >= 30)     //Top_Left
    {
        Motor(Top_Left, RX_buffer.C_speed);
        delay(5);
    }
    else if (RX_buffer.x_axis <= -30 && RX_buffer.y_axis <= -30)     //Bottom_Left
    {
        Motor(Bottom_Left, RX_buffer.C_speed);
        delay(5);
    }
    else if (RX_buffer.x_axis >= 30 && RX_buffer.y_axis >= 30)     //Top_Right
    {
        Motor(Top_Right, RX_buffer.C_speed);
        delay(5);
    }
    else if (RX_buffer.x_axis >= 30 && RX_buffer.y_axis <= -30)     //Bottom_Right
    {
        Motor(Bottom_Right, RX_buffer.C_speed);
        delay(5);
    }
    else if (RX_buffer.mode2 & 0x04)     //Contrarotate
    {
        Motor(Contrarotate, RX_buffer.C_speed);
        delay(5);
    }
    else if (RX_buffer.mode2 & 0x08)     //Clockwise
    {
        Motor(Clockwise, RX_buffer.C_speed);
        delay(5);
    }
    else if (RX_buffer.y_Arm <= 30 && RX_buffer.y_Arm >= -30 && RX_buffer.x_Base <= -30)    // base_left
    {
        base_degrees = base_degrees + 1;
        if (base_degrees >= 185) 
        {
            base_degrees = 185;
        }
        baseservo.write(base_degrees);
        delay(5);
    }
    else if (RX_buffer.y_Arm <= 30 && RX_buffer.y_Arm >= -30 && RX_buffer.x_Base >= 30)     // base_right
    {
        base_degrees = base_degrees - 1;
        if (base_degrees <= 0) 
        {
            base_degrees = 0;
        }
        baseservo.write(base_degrees);
        delay(5);
    }
    else if (RX_buffer.x_Base <= 30 && RX_buffer.x_Base >= -30 && RX_buffer.y_Arm <= -30)    // Arm contraction
    {
        arm_degrees = arm_degrees - 1;
        if (arm_degrees <= 0) 
        {
            arm_degrees = 0;
        }
        armservo.write(arm_degrees);
        delay(5);
    }
    else if (RX_buffer.x_Base <= 30 && RX_buffer.x_Base >= -30 && RX_buffer.y_Arm >= 30)     // Arm elongation
    {
        arm_degrees = arm_degrees + 1;
        if (arm_degrees >= 150) 
        {
            arm_degrees = 150;
        }
        armservo.write(arm_degrees);
        delay(5);
    }
    else if (RX_buffer.mode2 & 0x01)            // claws open                                        
    {
        claw_degrees = claw_degrees - 1;
        if (claw_degrees <= 50) 
        {
            claw_degrees = 50;
        }
        clawservo.write(claw_degrees);
        delay(5);
    }
    else if (RX_buffer.mode2 & 0x02)            // claws closed
    {
        claw_degrees = claw_degrees + 1;
        if (claw_degrees >= 150) 
        {
            claw_degrees = 150;
        }
        clawservo.write(claw_degrees);
        delay(5);
    }
    else if (RX_buffer.mode1 & 0x20)            // save
    {
        read_degress();
    }  
    else
    {
        Motor(Stop, 0);
    }
}

void read_degress()
{
    if (actions_count <= 19)
    {
        claw_read_degress[(int)((actions_count + 1) - 1)] = clawservo.read();
        delay(50);
        RX_Information(); 
        arm_read_degress[(int)((actions_count + 1) - 1)] = armservo.read();
        delay(50);
        RX_Information(); 
        base_read_degress[(int)((actions_count + 1) - 1)] = baseservo.read();
        delay(50);
        RX_Information(); 
        actions_count = actions_count + 1;
        auto_count = actions_count;
    }
}

void auto_do()
{
    if (0 != auto_count)
    {
        menory_action_flag = true;
    }
    actions_count = 0;
    claw_degrees = clawservo.read();
    arm_degrees = armservo.read();
    base_degrees = baseservo.read();
    while (menory_action_flag)
    {
        for (int i = (1); i <= (auto_count); i = i + (1))
        {
            RX_Information();
            if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
            {
                menory_action_flag = false;
                return;
            }
            if (claw_degrees < claw_read_degress[(int)(i - 1)])
            {
                while (claw_degrees < claw_read_degress[(int)(i - 1)])
                {
                    claw_degrees = claw_degrees + 1;
                    clawservo.write(claw_degrees);
                    delay(15);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        menory_action_flag = false;
                        return;
                    }
                }
            }
            else
            {
                while (claw_degrees > claw_read_degress[(int)(i - 1)])
                {
                    claw_degrees = claw_degrees - 1;
                    clawservo.write(claw_degrees);
                    delay(15);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        menory_action_flag = false;
                        return;
                    }
                }
            }
            RX_Information();
            if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
            {
                menory_action_flag = false;
                return;
            }
            if (arm_degrees < arm_read_degress[(int)(i - 1)])
            {
                while (arm_degrees < arm_read_degress[(int)(i - 1)])
                {
                    arm_degrees = arm_degrees + 1;
                    armservo.write(arm_degrees);
                    delay(15);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        menory_action_flag = false;
                        return;
                    }
                }
            }
            else
            {
                while (arm_degrees > arm_read_degress[(int)(i - 1)])
                {
                    arm_degrees = arm_degrees - 1;
                    armservo.write(arm_degrees);
                    delay(15);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        menory_action_flag = false;
                        return;
                    }
                }
            }
            RX_Information();
            if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
            {
                menory_action_flag = false;
                return;
            }
            if (base_degrees < base_read_degress[(int)(i - 1)])
            {
                while (base_degrees < base_read_degress[(int)(i - 1)])
                {
                    base_degrees = base_degrees + 1;
                    baseservo.write(base_degrees);
                    delay(15);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        menory_action_flag = false;
                        return;
                    }
                }
            }
            else
            {
                while (base_degrees > base_read_degress[(int)(i - 1)])
                {
                    base_degrees = base_degrees - 1;
                    baseservo.write(base_degrees);
                    delay(15);
                    RX_Information();
                    if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
                    {
                        menory_action_flag = false;
                        return;
                    }
                }
            }
            RX_Information();
            if(RX_buffer.mode1 == 0x01 || RX_buffer.mode1 == 0x02 || RX_buffer.mode1 == 0x04 || RX_buffer.mode1 == 0x08 || RX_buffer.mode1 == 0x10)
            {
                menory_action_flag = false;
                return;
            }
        }
    }
}

void Motor(int Dir, int Speed)      // Motor drive
{
    digitalWrite(EN_PIN, LOW);
    analogWrite(PWM1_PIN, Speed);
    analogWrite(PWM2_PIN, Speed);

    digitalWrite(STCP_PIN, LOW);
    shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, Dir);
    digitalWrite(STCP_PIN, HIGH);
}

float SR04(int Trig, int Echo)      // Ultrasonic distance measurement
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

void TX_Information(byte dis, byte act)     // Sending data packets
{
    int check;
    if(dis>127) dis = 127;
    if(act>127) act = 127;
    TX_package[1] = dis;
    TX_package[2] = act;
    TX_package[3] = byte(dis + act);                        // The sum of inspection                              
    Serial.write(TX_package, 5);                            // Sending data packets
}

void RX_Information(void)                   // Receiving data packet
{
    if(Serial.available() > 0)
    {
        delay(1);                                           // delay 1MS
        if(Serial.readBytes(RX_package, 10))
        {
            if (RX_package[0] == 0xA5 && RX_package[9] == 0x5A)     // The header and tail of the packet are verified
            {
                Serialcount = 0;
                RX_buffer.mode1 = RX_package[1];
                RX_buffer.mode2 = RX_package[2];
                RX_buffer.x_axis = RX_package[3];
                RX_buffer.y_axis = RX_package[4];
                RX_buffer.C_speed = RX_package[5] + 127;
                RX_buffer.x_Base = RX_package[6];
                RX_buffer.y_Arm = RX_package[7];
            }
            else
            {
                Serialcount++;
                return;
            }
        }
    }
    else
    {
        delay(1);
        Serialcount++;
        if(Serialcount > 500)
        {
            Serialcount = 0;
            RX_buffer.mode1 = 0; 
            RX_buffer.mode2 = 0;
            RX_buffer.x_axis = 0;
            RX_buffer.y_axis = 0;
            RX_buffer.x_Base = 0;
            RX_buffer.y_Arm = 0;
            Line_tracking_Function_flag = false;
            Avoidance_Function_flag = false;
            Following_Function_flag = false;
            Jail_Function_flag = false;
        }
    }
}