#include <math.h>
#include <stdio.h>
#define PI 3.14159265358979323846
#define TRUE 1
#define FALSE 0


struct SCARAArm {
    float length;
    float armVector[2];
    float desiredAngle;
    float encoderAngle;
};
typedef struct SCARAArm SCARAArm;


/*
initializeArm

Inputs:
struct SCARAArm* arm
float length

Purpose:function to modify the gains of the PID values

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
    float newX = posvec[0]*cos(theta) - posvec[1]*sin(theta);
    float newY = posvec[0]*sin(theta) + posvec[1]*cos(theta);

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

int main() {

    SCARAArm lowerArm;
    SCARAArm upperArm;

    initializeArm(&lowerArm, 0.4);
    initializeArm(&upperArm, 0.4);
    positionToAngle(&lowerArm, &upperArm, 0.3, 0.5);

    printf("%f %f\n", lowerArm.desiredAngle, upperArm.desiredAngle);

    system("pause");
    return(1);
}


