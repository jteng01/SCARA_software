#include <math.h>

volatile int encoderPos1 = 0;
volatile int encoderPos2 = 0;
volatile byte encoderA1;
volatile byte encoderA2;
volatile byte encoderB1;
volatile byte encoderB2;

void setup() {
    Serial.begin(115200);

    TCCR1A = 0;			    //Reset entire TCCR1A register
    TCCR1B = 0;			    //Reset entire TCCR1B register
    TCCR1A |= B00000100;    //Set CS12 to 1 so we get Prescalar = 256
    TCNT1 = 0;			    //Reset Timer 1 value to 0

    pinMode(pin, INPUT);
    pinMode(pin, INPUT);
    pinMode(pin, INPUT);
    pinMode(pin, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(interruptPin), ChangePosA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(interruptPin), ChangePosB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(interruptPin), ChangePosA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(interruptPin), ChangePosB, CHANGE);

    
}



void loop() {
    
    //initialize variables
    
    //run the code indefinetly
    while(1){
        
    }
}

ISR(TIMER1_COMPA_vect) {

}

void ChangeMotor1PosA() {
    encoderPos = 
}

void ChangeMotor1PosB() {

}

void ChangeMotor2PosA() {
    encoderPos =
}

void ChangeMotor2PosB() {

}
