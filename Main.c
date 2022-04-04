#include <math.h>
#include <stdlib.h>

#define PI 3.14159265358979323846
#define TRUE 1
#define FALSE 0

#define MOTOR1PWM 13
#define MOTOR2PWM 12
#define MOTOR1DIRECTION 6
#define MOTOR2DIRECTION 7
#define ENCODERA1 2
#define ENCODERB1 3
#define ENCODERA2 20
#define ENCODERB2 21
#define HOMING1 A1
#define HOMING2 A2
#define LIMIT2LOW A2

#define CCW 1
#define CW 0

#define TX 18
#define RX 19

#define HIGH 1
#define LOW 0

#define A1B1 1
#define A0B0 3
#define A1B0 4
#define A0B1 2

//create a data structure holding the position and angle data of each arm
struct SCARAArm {
    float length;
    float armVector[2];
    float desiredAngle;
    float encoderAngle;
};
typedef struct SCARAArm SCARAArm;


/*
InitializeArm

Inputs:
struct SCARAArm* arm
float length

Purpose:function to modify the gains of the PID values

*/
void InitializeArm(SCARAArm* arm, float length) {
    arm->length = length;
    arm->armVector[0] = length;
    arm->armVector[1] = 0;
    arm->encoderAngle = 0;
    arm->desiredAngle = 0;
}

/*
RotateVector

Inputs:
float theta
float posvec[] SIZE 2

Purpose: Rotates the arm to a desired angle counter clockwise

*/
void RotateVector(float theta, float posvec[]) {
    posvec[0] = cos(theta) * posvec[0] - sin(theta) * posvec[0];
    posvec[1] = sin(theta) * posvec[1] + cos(theta) * posvec[1];
}

/*
PositionToAngle
Inputs:
SCARAArm lowArm
SCARAArm upArm
float xpos
float ypos

purpose: find the desired angles required to move the arms to the desired position,
the c code implements the function PostoAngle.m written in matlab
*/
void PositionToAngle(SCARAArm* lowArm, SCARAArm* upArm, float xpos, float ypos) {

    //set l3 as the length of the origin to the desired coordinate and express as a coordinate vector and obtain the angle
    float l3 = pow(xpos, 2) + pow(ypos, 2);
    float posangle = atan2(xpos, ypos);
    float posVec[2] = { xpos, ypos };

    //Do not do anything if the coordinate is out of bounds which is the sum of the arm lengths
    if (pow(posVec[0], 2) + pow(posVec[1], 2) > (double)upArm->length + lowArm->length)
        return;

    //find the angle difference of the lower arm using the cosine law
    float thetal1l3 = acos((pow(lowArm->length, 2) + pow(l3, 2) - pow(upArm->length, 2)) / ((double)2 * lowArm->length * l3));
    //subtract the position vector by the angle difference
    lowArm->desiredAngle = posangle - thetal1l3;

    //rotate the arm coordinates to be used in finding the upper arm angle
    rotateVector(lowArm->desiredAngle, lowArm);

    //subtract the position vector by the lower arm vector to get the direction vector of the uppper arm
    for (int i = 0; i < 2; i++)
    {
        upArm->armVector[i] = posVec[i] - lowArm->armVector[i];
    }

    //find the angle of the 2nd arm and offset it by the lower arm angle since the lower arm rotates the reference coordinate
    //system of the upper arm
    upArm->desiredAngle = atan2(upArm->armVector[0], upArm->armVector[1]) - lowArm->desiredAngle;

    //saturate the arm output to meet the RCG of +/- 145 deg of the arm 
    if (upArm->desiredAngle > 145 * PI / 180)
        upArm->desiredAngle = 145 * PI / 180;
    else if (upArm->desiredAngle < -145 * PI / 180)
        upArm->desiredAngle = -145 * PI / 180;

}




//create a data structure holding the PID controller for each arm
typedef struct PID {
    //gains
    float ki;
    float kp;
    float kd;
    float K;

    //sampling
    float currentError;
    float ISRtime;

    //proportional
    float proportionalVal;

    //integral
    float prevError;
    float integralVal;

    //derivative
    float prevMeasure;
    float derivativeVal;
    float pole;

    //output
    float out;
};
typedef struct PID PID;

/*
InitializePID

Inputs:
struct PID* pid
float ISRtime

Purpose:function to initialize the PID values

*/
void InitializePID(PID* pid, float ISRtime) {
    pid->ISRtime = ISRtime;
    pid->integralVal = 0;
    pid->derivativeVal = 0;
    pid->proportionalVal = 0;
    pid->prevError = 0;
    pid->prevMeasure = 0;
}

/*
TunePID

Inputs:
struct PID* pid
float ki
float kd
float kp
float K

Purpose:function to modify the gains of the PID values

*/
void TunePID(PID* pid, float ki, float kd, float kp, float K) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->K = K;
}

/*
CalculatePID
Inputs:
struct PID* pid
float measurement
float desired

Purpose: Update the output of the PID aby multiplying the gain, taking the derivative, and integral
*/
void CalculatePID(PID* pid, float measurement, float desired) {
    //obtain the error of the arm angle
    pid->currentError = desired - measurement;

    //multply the error by the porportional gain
    pid->proportionalVal = pid->kp * pid->currentError;
    //perform and integral by using the trapezoidal rule multiplied by integral gain
    pid->integralVal += pid->ISRtime * (pid->currentError + pid->prevError) / 2 * pid->ki;
    //perform and integral by taking the difference of error over the ISR period and multiply by the derivative gain
    pid->derivativeVal = pid->kd * (measurement - pid->prevMeasure) / pid->ISRtime;

    //sum all the PID terms and multiply by the ultimate gain
    pid->out = (pid->proportionalVal + pid->integralVal + pid->derivativeVal) * pid->K;

    if (pid->out > 24)
        pid->out = 24;
    else if (pid->out < -24)
        pid->out = -24;

    //save the error to be used in the next PID calculation for integral and derivative
    pid->prevError = pid->currentError;
    pid->prevMeasure = measurement;
}


//use a linked list to help with path planning and telling where and when the arm should move
struct PositionList {
    float coordinates[2];
    int time;
    int index;
    struct PositionList* next;
};
typedef struct PositionList PositionList;


/*
InitializePID

Inputs:
struct PID* pid
float ISRtime

Purpose:function to initialize the PID values

*/
void InitializePID(PID* pid, float ISRtime) {
    pid->ISRtime = ISRtime;
    pid->integralVal = 0;
    pid->derivativeVal = 0;
    pid->proportionalVal = 0;
    pid->prevError = 0;
    pid->prevMeasure = 0;
}

/*
TunePID

Inputs:
struct PID* pid
float ki
float kd
float kp
float K

Purpose:function to modify the gains of the PID values

*/
void TunePID(PID* pid, float ki, float kd, float kp, float K) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->K = K;
}

/*
CalculatePID
Inputs:
struct PID* pid
float measurement
float desired

Purpose: Update the output of the PID aby multiplying the gain, taking the derivative, and integral
*/
void CalculatePID(PID* pid, float measurement, float desired) {
    //obtain the error of the arm angle
    pid->currentError = measurement - desired;

    //multply the error by the porportional gain
    pid->proportionalVal = pid->kp * pid->currentError;
    //perform and integral by using the trapezoidal rule multiplied by integral gain
    pid->integralVal += pid->ISRtime * (pid->currentError + pid->prevError) / 2 * pid->ki;
    //perform and integral by taking the difference of error over the ISR period and multiply by the derivative gain
    pid->derivativeVal = pid->kd * (measurement - pid->prevMeasure) / pid->ISRtime;

    //sum all the PID terms and multiply by the ultimate gain
    pid->out = (pid->proportionalVal + pid->integralVal + pid->derivativeVal) * pid->K;

    if (pid->out > 24)
        pid->out = 24;
    else if (pid->out < -24)
        pid->out = -24;

    //save the error to be used in the next PID calculation for integral and derivative
    pid->prevError = pid->currentError;
    pid->prevMeasure = measurement;
}


//motor states
int stateMotor1;
int stateMotor2;

//encoder values
int encoderA1;
int encoderA2;
int encoderB1;
int encoderB2;

//arm structures
SCARAArm upperArm;
SCARAArm lowerArm;

//PID controllers
PID pid1;
PID pid2;

//path planing list
PositionList head;

void setup() {
    Serial.begin(115200);

    //setup timer for 1ms ISR, timer has CF of 16MHz
    noInterrupts();
    TCCR1A = 0;			    //Reset entire TCCR1A register
    TCCR1B = 0;			    //Reset entire TCCR1B register
    TCCR1A |= B00000100;    //Set CS12 to 1 so we get Prescalar = 256
    TCNT1 = 0;			    //Reset Timer 1 value to 0

    //specify whether the pins are output or input
    pinMode(MOTOR1PWM, OUTPUT);
    pinMode(MOTOR2PWM, OUTPUT);
    pinMode(MOTOR1DIRECTION, OUTPUT);
    pinMode(MOTOR2DIRECTION, OUTPUT);
    pinMode(ENCODERA1, INPUT);
    pinmode(ENCODERA2, INPUT);
    pinMode(ENCODERB1, INPUT);
    pinmode(ENCODERB2, INPUT);
    pinMode(HOMING1, INPUT);
    pinmode(HOMING2, INPUT);
    pinmode(LIMIT2LOW, INPUT);

    //create the PID structures
    InitializePID(&pid1, 0.001);
    InitializePID(&pid2, 0.001);

    //tune the PID found from the simulink and simx using heurestic tuning
    tunePID(&pid1, 1, 1, 1, 1);
    tunePID(&pid2, 1, 1, 1, 1);

    //create the arm structures
    InitializeArm(&upperArm, 0.4);
    InitializeArm(&lowerArm, 0.4);
    
    //setup interupts for the encoder
    attachInterrupt(digitalPinToInterrupt(), ChangePosA1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(), ChangePosB1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(), ChangePosA2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(), ChangePosB2, CHANGE);

    //begin homing, start with rotating the arms clockwise at low speed
    digitalWrite(MOTOR1DIRECTION, CW);
    analogWrite(MOTOR1PWM, 4 * 255 / 24);
    //arm2 will reach a lower limit when clockwise
    digitalWrite(MOTOR2DIRECTION, CW);
    analogWrite(MOTOR2PWM, 4 * 255 / 24);

    while (1)
    {
        int motor1Homed = 0;
        int motor2Homed = 0;
        int interuptEnabled = 0;

        //stop the motors once the arm is homed
        if(digitalRead(HOMING1) == 1 && motor1Homed == 0){
            motor1Homed == 1;
            analogWrite(MOTOR1PWM, 0);
        }

        if (digitalRead(HOMING2) == 1 && motor2Homed == 0) {
            motor1Homed == 1;
            analogWrite(MOTOR1PWM, 0);
        }

        //reverse the 2nd motor once it reaches the lower limit 
        if(digitalRead(LIMIT2LOW) == 1)
            digitalWrite(MOTOR2DIRECTION, CCW);

        //break the loop once homed
        if (motor1Homed == 1 && motor2Homed == 1)
            break;
    }

    //read encoder values and initialize the states
    encoderA1 = digitalRead(ENCODERA1);
    encoderA2 = digitalRead(ENCODERA2);
    encoderB1 = digitalRead(ENCODERB1);
    encoderB2 = digitalRead(ENCODERB2);
    InitializeStates(encoderA1, encoderB1, &stateMotor1);
    InitializeStates(encoderA2, encoderB2, &stateMotor2);


}



void loop() {
    
    //run the code indefinetly

}

void InitializeStates(int * state, int encoderA, int encoderB){

    if (encoderA == HIGH & encoderB == HIGH)
        *state = A1B1;
    else if (encoderA == LOW & encoderB == LOW)
        *state = A0B0;
    else if (encoderA == LOW & encoderB == HIGH)
        *state = A0B1;
    else if (encoderA == HIGH & encoderB == LOW)
        *state = A1B0;
}

//ISR functions

//update the PID values and control the motors at 1000Hz (1ms)
ISR(TIMER1_COMPA_vect) {
    //update the PID values
    CalculatePID(&pid1, lowerArm.encoderAngle, lowerArm.desiredAngle);
    CalculatePID(&pid2, upperArm.encoderAngle, upperArm.desiredAngle);

    //drive the lower arm motor using the PID output
    analogWrite(MOTOR1PWM, (int)fabs(pid1.out) * 255 / 24);
    digitalWrite(MOTOR1DIRECTION, (pid1.out >= 0));

    //drive the upper arm motor using the PID output
    analogWrite(MOTOR2PWM, (int) fabs(pid2.out) * 255/24);
    digitalWrite(MOTOR2DIRECTION, (pid2.out >= 0));

}

//Change the position given the encoder reading, A leads B when clockwise and B leads A when counter clockwise
void ChangePosA1() {
    int AValue = digitalRead(ENCODERA1);

    //A falls when B is on, B is leading and position goes down
    if (stateMotor1 == A1B1 && AValue == 0) {
        lowerArm.encoderAngle+= 2*PI/600;
        stateMotor1 = A0B1;
    }

    //A rises when B is off, A is leading and position goes down
    else if (stateMotor1 == A0B1 && AValue == 1) {
        lowerArm.encoderAngle-= 2*PI/600;
        stateMotor1 = A1B1;
    }

    //A falls when B is on, A is leading and position goes down
    else if (stateMotor1 == A1B0 && AValue == 0) {
        lowerArm.encoderAngle-= 2*PI/600;
        stateMotor1 = A0B0;
    }

    //A falls when B is on, B is leading and position goes up
    else if (stateMotor1 == A0B0 && AValue == 1) {
        lowerArm.encoderAngle+= 2*PI/600;
        stateMotor1 = A1B0;
    }

}

void ChangePosB1() {
    int BValue = digitalRead(ENCODERB1);

    //A is on and B falls, B is leading and position goes up
    if (stateMotor1 == A1B1 && BValue == 0) {
        lowerArm.encoderAngle+=2*PI/600;
        stateMotor1 = A1B0;
    }

    //A is on and B rises, A is leading and position goes down
    else if (stateMotor1 == A1B0 && BValue == 1) {
        lowerArm.encoderAngle-= 2*PI/600;
        stateMotor1 = A1B1;
    }

    //A is off and B falls, A is leading and position goes down
    else if (stateMotor1 == A0B1 && BValue == 0) {
        lowerArm.encoderAngle-=2*PI/600;
        stateMotor1 = A0B0;
    }

    //A is off and B rises, B is leading and position goes up
    else if (stateMotor1 == A0B0 && BValue == 1) {
        lowerArm.encoderAngle+=2*PI/600;
        stateMotor1 = A0B1;
    }
}

void ChangePosA2() {
    int AValue = digitalRead(ENCODERA1);

    if (stateMotor2 == A1B1 && AValue == 0) {
        upperArm.encoderAngle+=2*PI/600;
        stateMotor2 = A0B1;
    }

    else if (stateMotor2 == A0B1 && AValue == 1) {
        upperArm.encoderAngle-=2*PI/600;
        stateMotor2 = A1B1;
    }

    else if (stateMotor2 == A1B0 && AValue == 0) {
        upperArm.encoderAngle-=2*PI/600;
        stateMotor2 = A0B0;
    }

    else if (stateMotor2 == A0B0 && AValue == 1) {
        upperArm.encoderAngle+=2*PI/600;
        stateMotor2 = A1B0;
    }
}

void ChangePosB2() {
    int BValue = digitalRead(ENCODERB1);

    if (stateMotor2 == A1B1 && BValue == 0) {
        upperArm.encoderAngle+=2*PI/600;
        stateMotor2 = A1B0;
    }

    else if (stateMotor2 == A1B0 && BValue == 1) {
        upperArm.encoderAngle-=2*PI/600;
        stateMotor2 = A1B1;
    }

    else if (stateMotor1 == A0B1 && BValue == 0) {
        upperArm.encoderAngle-=2*PI/600;
        stateMotor2 = A0B0;
    }

    else if (stateMotor1 == A0B0 && BValue == 1) {
        upperArm.encoderAngle+=2*PI/600;
        stateMotor2 = A0B1;
    }
}
