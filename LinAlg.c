#include <math.h>

setRotationMatrix(float theta, float posvec[]){
    posVec[1] = cos(theta)*posVec[0] + -sin(theta)*posVec[0];
    posVec[2] = sin(theta)*posVec[1] + cos(theta)*posVec[1];
}