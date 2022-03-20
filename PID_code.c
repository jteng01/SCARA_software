struct PID {
  float ki;
  float kp;
  float kd;
  float K;
  float currentError;
  float sampleTime;

  //proportional
  float proportionalVal;

  //integral
  float prevError;
  float integralVal;

  //derivative
  float prevMeasure;
  float derivativeVal;
  float pole;

  float out;
}

void PID_init(PID *pid) {
  pid-> integralVal = 0;
  pid-> derivativeVal = 0;
  pid-> proportionalVal = 0;
  pid-> prevError = 0;
  pid-> prevMeasure = 0;

}

PID_tune(PID *pid,ki,kd,kp,K){
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->K = K;
}

void PID_calculate(PID *pid, measurement, desired, error){
  pid->currentError = measurement - desired;

  pid->proportionalVal = pid-> kp * currentError;
  pid->integralVal += pid->sampleTime * (currentError - prevError)/2 * pid->ki;
  pid->derivativeVal = pid-> kd * (measurement - prevMeasure)/pid->sampleTime; 

  pid->out = pid->proportionalVal + pid->integralVal + pid->derivativeVal;
}

