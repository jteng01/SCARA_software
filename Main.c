#include <math.h>
#include <stdlib.h>
#include <stdio.h>

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

#define A1B1 1
#define A0B0 3
#define A1B0 4
#define A0B1 2

/*
Data structure/classes and functions to manipulate the class/object
*/

//create a data structure holding the position and angle data of each arm
struct SCARAArm {
    float length;
    float armVector[2];
    float desiredAngle;
    float encoderAngle;
};
typedef struct SCARAArm SCARAArm;

//create a data structure holding the PIDCont controller for each arm
struct PIDCont {
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
typedef struct PIDCont PIDCont;

//use a linked list to help with path planning and telling where and when the arm should move
struct PositionNode {
    float xpos;
    float ypos;
    int pauseTime;
    struct PositionNode* next;
};
typedef struct PositionNode PositionNode;


/*
initializeArm

Inputs:
struct SCARAArm* arm
float length

Purpose:function to initialize the arm

*/
void initializeArm(SCARAArm* arm, float length) {
    arm->length = length;
    arm->armVector[0] = length;
    arm->armVector[1] = 0;
    arm->encoderAngle = 0;
    arm->desiredAngle = 0;
}

/*
rotateVector

Inputs:
float theta
float posvec[2]

Purpose: Rotates the arm to a desired angle counter clockwise

*/
void rotateVector(float theta, float posvec[]) {
    float newX = posvec[0] * cos(theta) - posvec[1] * sin(theta);
    float newY = posvec[0] * sin(theta) + posvec[1] * cos(theta);

    posvec[0] = newX;
    posvec[1] = newY;
}

/*
positionToAngle
Inputs:
SCARAArm lowArm
SCARAArm upArm
float xpos
float ypos

purpose: find the desired angles required to move the arms to the desired position,
the c code implements the function PostoAngle.m written in matlab
*/
void positionToAngle(SCARAArm* lowArm, SCARAArm* upArm, float xpos, float ypos) {

    //set l3 as the length of the origin to the desired coordinate and express as a coordinate vector and obtain the angle
    float l3 = sqrt(pow(xpos, 2) + pow(ypos, 2));
    float posangle = atan2(ypos, xpos);
    float posVec[2] = { xpos, ypos };

    //Do not do anything if the coordinate is out of bounds which is the sum of the arm lengths
    if (pow(posVec[0], 2) + pow(posVec[1], 2) > (double)upArm->length + lowArm->length)
        return;

    //find the angle difference of the lower arm using the cosine law
    float thetal1l3 = acos((pow(lowArm->length, 2) + pow(l3, 2) - pow(upArm->length, 2)) / ((double)(2 * lowArm->length * l3)));
    //subtract the position vector by the angle difference
    lowArm->desiredAngle = posangle - thetal1l3;

    //rotate the arm coordinates to be used in finding the upper arm angle
    rotateVector(lowArm->desiredAngle, lowArm->armVector);

    //subtract the position vector by the lower arm vector to get the direction vector of the uppper arm
    for (int i = 0; i < 2; i++)
    {
        upArm->armVector[i] = posVec[i] - lowArm->armVector[i];
    }

    //find the angle of the 2nd arm and offset it by the lower arm angle since the lower arm rotates the reference coordinate
    //system of the upper arm
    upArm->desiredAngle = atan2(upArm->armVector[1], upArm->armVector[0]) - lowArm->desiredAngle;

    //saturate the arm output to meet the RCG of +/- 145 deg of the arm 
    if (upArm->desiredAngle > 145 * PI / 180)
        upArm->desiredAngle = 145 * PI / 180;
    else if (upArm->desiredAngle < -145 * PI / 180)
        upArm->desiredAngle = -145 * PI / 180;
}

/*
angleToPos
Inputs:
SCARAArm lowArm
SCARAArm upArm
float actualPos[2]

purpose: find the position based on the actual angles of the arm
*/
void angleToPos(float actualPos[], SCARAArm* lowArm, SCARAArm* upArm) {

    //create temkporary storage for the xy pos of the arm
    float lowArmVec[2] = { 0, lowArm->length };

    //rotate the arm
    rotateVector(lowArm->encoderAngle, lowArmVec);

    float upArmVec[2] = { 0, upArm->length };

    //Offset the 2nd arm angle by the first arm angle
    rotateVector(upArm->encoderAngle + lowArm->encoderAngle, upArmVec);

    for (int i = 0; i < 2; i++)
    {
        actualPos[i] = lowArmVec[i] + upArmVec[i];
    }
}


/*
initializePIDCont

Inputs:
struct PIDCont* pid
float ISRtime

Purpose:function to initialize the PIDCont values

*/
void initializePIDCont(PIDCont* pid, float ISRtime) {
    pid->ISRtime = ISRtime;
    pid->integralVal = 0;
    pid->derivativeVal = 0;
    pid->proportionalVal = 0;
    pid->prevError = 0;
    pid->prevMeasure = 0;
}

/*
tunePIDCont

Inputs:
struct PIDCont* pid
float ki
float kd
float kp
float K

Purpose:function to modify the gains of the PIDCont values

*/
void tunePIDCont(PIDCont* pid, float Ki, float Kd, float Kp, float Kult) {
    pid->kp = Kp;
    pid->ki = Ki;
    pid->kd = Kd;
    pid->K = Kult;
}

/*
calculatePIDCont
Inputs:
struct PIDCont* pid
float measurement
float desired

Purpose: Update the output of the PIDCont aby multiplying the gain, taking the derivative, and integral
*/
void calculatePIDCont(PIDCont* pid, float measurement, float desired) {
    //obtain the error of the arm angle
    pid->currentError = desired - measurement;

    //multply the error by the porportional gain
    pid->proportionalVal = pid->kp * pid->currentError;
    //perform and integral by using the trapezoidal rule multiplied by integral gain
    pid->integralVal += pid->ISRtime * (pid->currentError + pid->prevError) / 2 * pid->ki;
    //perform and integral by taking the difference of error over the ISR period and multiply by the derivative gain
    pid->derivativeVal = pid->kd * (measurement - pid->prevMeasure) / pid->ISRtime;

    //sum all the PIDCont terms and multiply by the ultimate gain
    pid->out = (pid->proportionalVal + pid->integralVal + pid->derivativeVal) * pid->K;

    if (pid->out > 24)
        pid->out = 24;
    else if (pid->out < -24)
        pid->out = -24;

    //save the error to be used in the next PIDCont calculation for integral and derivative
    pid->prevError = pid->currentError;
    pid->prevMeasure = measurement;
}

void appendPosition(PositionNode** head, float xpos, float ypos, int msPauseTime)
{
    PositionNode* newNode = (PositionNode*)malloc(sizeof(PositionNode));

    newNode->pauseTime = msPauseTime;
    newNode->xpos = xpos;
    newNode->ypos = ypos;
    newNode->next = NULL;

    PositionNode* p = *head;

    if (*head == NULL)
        *head = newNode;

    else {//traverse to the end
        while (p->next != NULL) {
            p = p->next;
        }
        p->next = newNode;
    }
}

void insertPosition(PositionNode** head, float xpos, float ypos, int msPauseTime, int pos) {

    PositionNode* newNode = (PositionNode*)malloc(sizeof(PositionNode));

    newNode->pauseTime = msPauseTime;
    newNode->xpos = xpos;
    newNode->ypos = ypos;
    newNode->next = NULL;

    if (*head == NULL) {
        *head = newNode;
        return;
    }

    PositionNode* p = *head;
    PositionNode* nextHead = p->next;
    int i;

    if (pos == 0) {
        newNode->next = *head;
        *head = newNode;
        return;
    }

    for (i = 1; i < pos; i++) {
        nextHead = nextHead->next;
        p = p->next;

        if (nextHead->next == NULL) {
            nextHead->next = newNode;
            return;
        }

    }

    p->next = newNode;
    newNode->next = nextHead;
}

void deletePosition(PositionNode** head, int pos) {

    if (*head == NULL)
        return;

    PositionNode* p = *head;
    PositionNode* nextHead = p->next;

    if (pos == 0) {
        free(p);
        *head = nextHead;
        return;
    }

    int i;

    for (i = 1; i < pos; i++) {
        nextHead = nextHead->next;
        p = p->next;
        if (nextHead->next == NULL) {
            free(nextHead);
            p->next = NULL;
            return;
        }
    }

    p->next = nextHead->next;
    free(nextHead);

}









/*
Main code that drives the SCARA arm
*/

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

//PIDCont controllers
PIDCont pid1;
PIDCont pid2;

//path planing list
PositionNode* headList = NULL;

void setup() {
    Serial.begin(115200);

    //setup timer for 1ms ISR, timer has CF of 16MHz
    noInterrupts();

    //set timer0 interrupt at 1000Hz
    TCCR0A = 0;
    TCCR0B = 0; //reset timer0 registers
    TCNT0 = 0;//initialize counter value to 0

    OCR0A = 249;// compare to = (116MHz) / (1Hz*prescalar) - 1
    TCCR0A |= (1 << WGM01); //turn on CTC mode, reset when timer reaches OCR0A value
    TCCR0B |= (1 << CS01) | (1 << CS00); //set the prescalar to 64
    TIMSK0 |= (1 << OCIE0A); //enable timer interupt

    //specify whether the pins are output or input
    pinMode(MOTOR1PWM, OUTPUT);
    pinMode(MOTOR2PWM, OUTPUT);
    pinMode(MOTOR1DIRECTION, OUTPUT);
    pinMode(MOTOR2DIRECTION, OUTPUT);
    pinMode(ENCODERA1, INPUT);
    pinMode(ENCODERA2, INPUT);
    pinMode(ENCODERB1, INPUT);
    pinMode(ENCODERB2, INPUT);
    pinMode(HOMING1, INPUT);
    pinMode(HOMING2, INPUT);
    pinMode(LIMIT2LOW, INPUT);

    //create the PIDCont structures
    initializePIDCont(&pid1, 0.001);
    initializePIDCont(&pid2, 0.001);

    //tune the PIDCont found from the simulink and simx using heurestic tuning
    tunePIDCont(&pid1, (float)1, (float)1, (float)1, (float)1);
    tunePIDCont(&pid2, (float)1, (float)1, (float)1, (float)1);

    //create the arm structures
    initializeArm(&upperArm, 0.4);
    initializeArm(&lowerArm, 0.4);

    //setup interupts for the encoder
    attachInterrupt(digitalPinToInterrupt(ENCODERA1), changePosA1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODERA2), changePosB1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODERB1), changePosA2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODERB2), changePosB2, CHANGE);

    //begin homing, start with rotating the arms clockwise at low speed
    digitalWrite(MOTOR1DIRECTION, CW);
    analogWrite(MOTOR1PWM, 255 - (4 * 255 / 24));
    //arm2 will reach a lower limit when clockwise
    digitalWrite(MOTOR2DIRECTION, CW);
    analogWrite(MOTOR2PWM, 255 - (4 * 255 / 24));

    while (1)
    {
        int motor1Homed = 0;
        int motor2Homed = 0;

        //stop the motors once the arm is homed, sensor signal gets turned off
        if (digitalRead(HOMING1) == 0 && motor1Homed == 0) {
            motor1Homed == 0;
            analogWrite(MOTOR1PWM, 0);
        }

        if (digitalRead(HOMING2) == 0 && motor2Homed == 0) {
            motor1Homed == 0;
            analogWrite(MOTOR1PWM, 0);
        }

        //reverse the 2nd motor once it reaches the lower limit 
        if (digitalRead(LIMIT2LOW) == 0)
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
    initializeStates(&stateMotor1, encoderA1, encoderB1);
    initializeStates(&stateMotor2, encoderA2, encoderB2);

    interrupts();

    FILE* fileptr;

    fileptr = fopen("PositionData.txt", "r");

    if (fileptr == NULL) {
        printf("Can not open file.");
    }
    else
    {
        float xAppend;
        float yAppend;
        float deltaTAppend;
        while (fscanf(fileptr, "%f %f %d\n", xAppend, yAppend, deltaTAppend) != EOF)
        {
            appendPosition(&headList, xAppend, yAppend, deltaTAppend);
        }

        fclose(fileptr);
    }


}



void loop() {

    PositionNode* p = headList;
    while (p != NULL) {
        Serial.println();
        positionToAngle(&upperArm, &lowerArm, p->xpos, p->ypos);
        delay(p->pauseTime);
        p = p->next;
    }


}

void initializeStates(int* state, int encoderA, int encoderB) {

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

//update the PIDCont values and control the motors at 1000Hz (1ms)
ISR(TIMER0_COMPA_vect) {
    //update the PIDCont values
    calculatePIDCont(&pid1, lowerArm.encoderAngle, lowerArm.desiredAngle);
    calculatePIDCont(&pid2, upperArm.encoderAngle, upperArm.desiredAngle);

    //drive the lower arm motor using the PIDCont output, the amplifier inverts the PWM signal where 5V 75% PWM input outputs a 24V 25% PWM signal
    //in the amplifier
    analogWrite(MOTOR1PWM, (int)(255 - (fabs(pid1.out) * 255 / 24)));
    digitalWrite(MOTOR1DIRECTION, (pid1.out >= 0));

    //drive the upper arm motor using the PIDCont output
    analogWrite(MOTOR2PWM, (int)(255 - (fabs(pid2.out) * 255 / 24)));
    digitalWrite(MOTOR2DIRECTION, (pid2.out >= 0));

}

//Change the position given the encoder reading, A leads B when clockwise and B leads A when counter clockwise
void changePosA1() {
    int AValue = digitalRead(ENCODERA1);

    //A falls when B is on, B is leading and position goes down
    if (stateMotor1 == A1B1 && AValue == 0) {
        lowerArm.encoderAngle += 2 * PI / 2400;
        stateMotor1 = A0B1;
    }

    //A rises when B is off, A is leading and position goes down
    else if (stateMotor1 == A0B1 && AValue == 1) {
        lowerArm.encoderAngle -= 2 * PI / 2400;
        stateMotor1 = A1B1;
    }

    //A falls when B is on, A is leading and position goes down
    else if (stateMotor1 == A1B0 && AValue == 0) {
        lowerArm.encoderAngle -= 2 * PI / 2400;
        stateMotor1 = A0B0;
    }

    //A falls when B is on, B is leading and position goes up
    else if (stateMotor1 == A0B0 && AValue == 1) {
        lowerArm.encoderAngle += 2 * PI / 2400;
        stateMotor1 = A1B0;
    }

}

void changePosB1() {
    int BValue = digitalRead(ENCODERB1);

    //A is on and B falls, B is leading and position goes up
    if (stateMotor1 == A1B1 && BValue == 0) {
        lowerArm.encoderAngle += 2 * PI / 2400;
        stateMotor1 = A1B0;
    }

    //A is on and B rises, A is leading and position goes down
    else if (stateMotor1 == A1B0 && BValue == 1) {
        lowerArm.encoderAngle -= 2 * PI / 2400;
        stateMotor1 = A1B1;
    }

    //A is off and B falls, A is leading and position goes down
    else if (stateMotor1 == A0B1 && BValue == 0) {
        lowerArm.encoderAngle -= 2 * PI / 2400;
        stateMotor1 = A0B0;
    }

    //A is off and B rises, B is leading and position goes up
    else if (stateMotor1 == A0B0 && BValue == 1) {
        lowerArm.encoderAngle += 2 * PI / 2400;
        stateMotor1 = A0B1;
    }
}

void changePosA2() {
    int AValue = digitalRead(ENCODERA1);

    if (stateMotor2 == A1B1 && AValue == 0) {
        upperArm.encoderAngle += 2 * PI / 2400;
        stateMotor2 = A0B1;
    }

    else if (stateMotor2 == A0B1 && AValue == 1) {
        upperArm.encoderAngle -= 2 * PI / 2400;
        stateMotor2 = A1B1;
    }

    else if (stateMotor2 == A1B0 && AValue == 0) {
        upperArm.encoderAngle -= 2 * PI / 2400;
        stateMotor2 = A0B0;
    }

    else if (stateMotor2 == A0B0 && AValue == 1) {
        upperArm.encoderAngle += 2 * PI / 2400;
        stateMotor2 = A1B0;
    }
}

void changePosB2() {
    int BValue = digitalRead(ENCODERB1);

    if (stateMotor2 == A1B1 && BValue == 0) {
        upperArm.encoderAngle += 2 * PI / 2400;
        stateMotor2 = A1B0;
    }

    else if (stateMotor2 == A1B0 && BValue == 1) {
        upperArm.encoderAngle -= 2 * PI / 2400;
        stateMotor2 = A1B1;
    }

    else if (stateMotor1 == A0B1 && BValue == 0) {
        upperArm.encoderAngle -= 2 * PI / 2400;
        stateMotor2 = A0B0;
    }

    else if (stateMotor1 == A0B0 && BValue == 1) {
        upperArm.encoderAngle += 2 * PI / 2400;
        stateMotor2 = A0B1;
    }
}