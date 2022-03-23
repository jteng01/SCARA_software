struct UpperArm{
    float length;
    float desiredAngle;
    float targetPosVec[2]; 
};

struct LowerArm{
    float length;
    float desiredAngle;
    float targetPosVec[2];
};

LocateShortestPath(LowerArm *lowArm, UpperArm *upArm, xpos, ypos);

LowerArmRight(LowerArm *lowArm, UpperArm *upArm, float xpos, float ypos);

LowerArmRight(LowerArm *lowArm, UpperArm *upArm, float xpos, float ypos);