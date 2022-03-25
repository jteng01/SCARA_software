#include <math.h>
#define PI 3.14159265358979323846


struct SCARAArm{
    float length;
    float desiredEncoderAngle;
    float armVector[2];
    float encoderAngle;
    float desiredAngle;
};
typedef struct SCARAArm SCARAArm;

void initializeArm(SCARAArm* arm, float length) {
    arm->length = length;
    arm->desiredEncoderAngle = 0;
    arm->armVector[0] = length;
    arm->armVector[1] = 0;
    arm->encoderAngle = 0;
    arm->desiredAngle = 0;
}

void rotateVector(float theta, float posvec[]){
    posvec[0] = cos(theta)*posvec[0] - sin(theta)*posvec[0];
    posvec[1] = sin(theta)*posvec[1] + cos(theta)*posvec[1];
}

positionToAngle(SCARAArm* lowArm, SCARAArm* upArm, float xpos, float ypos) {
    float l3 =  pow(xpos,2) + pow(ypos,2);
    float posangle = atan2(xpos, ypos);
    float posVec[2] = {xpos, ypos};

    float thetal1l3 = acos( (pow(lowArm->length,2) + pow(l3,2) - pow(upArm->length,2)) / (2*lowArm->length*l3) );

    float lowerArmRightDelta = PI - fabs(fabs(posangle + thetal1l3 - lowArm->encoderAngle) - PI);
    float lowerArmLeftDelta = PI - fabs(fabs(posangle - thetal1l3 - lowArm->encoderAngle) - PI);
    
    if (lowerArmRightDelta > lowerArmLeftDelta)
        lowArm->desiredAngle = lowerArmLeftDelta;
    else
        lowArm->desiredAngle = lowerArmRightDelta;
        
    rotateVector(lowArm->desiredAngle, lowArm);

    for (int i = 0; i < 2; i++)
    {
        upArm->armVector[i] = posVec[i] - lowArm->armVector[i];
    }    
    
    upArm->desiredAngle = atan2(upArm->armVector[0], upArm->armVector[1]) - lowArm->desiredAngle;

    if (upArm->desiredAngle > 145*PI/180)
        upArm->desiredAngle = 145*PI/180;
    else if (upArm->desiredAngle < -145*PI/180)
        upArm->desiredAngle = -145*PI/180;

}

