typedef struct PID{
  //gains
  float ki;
  float kp;
  float kd;
  float K;
  
  //sampling
  float currentError;
  float ISRfreq;

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

void PID_init(PID *pid, float ISRfreq) {
  pid->ISRfreq = ISRfreq;
  pid-> integralVal = 0;
  pid-> derivativeVal = 0;
  pid-> proportionalVal = 0;
  pid-> prevError = 0;
  pid-> prevMeasure = 0;

}

void PID_tune(PID *pid, float ki, float kd, float kp, float K){
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->K = K;
}

void PID_calculate(PID *pid, float measurement, float desired){
  pid->currentError = measurement - desired;

  pid->proportionalVal = pid-> kp * pid->currentError;
  pid->integralVal += pid->ISRfreq * (pid->currentError - pid->prevError)/2 * pid->ki;
  pid->derivativeVal = pid-> kd * (measurement - pid->prevMeasure)/pid->ISRfreq; 

  pid->out = pid->proportionalVal + pid->integralVal + pid->derivativeVal;

  pid->prevError = pid->currentError;
  pid->prevMeasure = measurement;
}


