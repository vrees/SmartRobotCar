#include    <Servo.h>

#define CLAW_PIN    9
#define ARM_PIN     10
#define BASE_PIN    11

#define CLAW_CLOSE  128
#define CLAW_OPEN   50
#define DELAY       15

Servo clawservo;//Clamp servo
Servo armservo;//Arm servo
Servo baseservo;//Turntable servo

int pos = 0;

void move(Servo &servo, int pos, int msec) {
  servo.write(pos);    
  delay(msec);
}

void setup()
{
    clawservo.attach(CLAW_PIN);
    armservo.attach(ARM_PIN);
    baseservo.attach(BASE_PIN);

    move(clawservo, CLAW_CLOSE, DELAY);
    move(armservo, 135, DELAY);
    move(baseservo, 90, DELAY);

    delay(500);
}

void moveClaw() {
      for (pos = CLAW_CLOSE; pos >= CLAW_OPEN; pos -= 1) 
    { 
        clawservo.write(pos);    
        delay(DELAY);
    } 

    for (pos = CLAW_OPEN; pos <= CLAW_CLOSE; pos += 1) 
    { 
        clawservo.write(pos);            
        delay(DELAY);                     
    }
}

void moveArm() {      
    for (pos = 150; pos >= 0; pos -= 1) 
    { 
        armservo.write(pos);    
        delay(DELAY);
    }

    for (pos = 0; pos <= 150; pos += 1) 
    { 
        armservo.write(pos);            
        delay(DELAY);                     
    }
}

void moveBase() {
    for (pos = 90; pos >= 0; pos -= 1) 
    { 
        baseservo.write(pos);    
        delay(DELAY);
    }

    for (pos = 0; pos <= 180; pos += 1) 
    { 
        baseservo.write(pos);            
        delay(DELAY);                     
    }

    for (pos = 180; pos >= 90; pos -= 1) 
    { 
        baseservo.write(pos);    
        delay(DELAY);
    }

}

void loop()
{
  moveClaw();
  moveArm();
  moveBase();
}
