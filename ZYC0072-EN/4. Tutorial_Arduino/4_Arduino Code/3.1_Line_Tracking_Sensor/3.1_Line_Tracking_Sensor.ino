#define LEFT_LINE_TRACJING          A0
#define CENTER_LINE_TRACJING        A1
#define right_LINE_TRACJING         A2


void setup()
{
    Serial.begin(9600);
    pinMode(LEFT_LINE_TRACJING, INPUT);
    pinMode(CENTER_LINE_TRACJING, INPUT);
    pinMode(right_LINE_TRACJING, INPUT);
}

void loop()
{
    Infrared_Tracing();
}

void Infrared_Tracing()
{
    int Left_Tra_Value;
    int Center_Tra_Value;
    int Right_Tra_Value;
    Left_Tra_Value = analogRead(LEFT_LINE_TRACJING);
    Center_Tra_Value = analogRead(CENTER_LINE_TRACJING);
    Right_Tra_Value = analogRead(right_LINE_TRACJING);
    Serial.print("Left Tracking value:");
    Serial.println(Left_Tra_Value);
    Serial.print("Center Tracking value:");
    Serial.println(Center_Tra_Value);
    Serial.print("Right Tracking value:");
    Serial.println(Right_Tra_Value);
    Serial.println("");
    delay(1000);
}