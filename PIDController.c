typedef struct PID{
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
void InitializePID(PID *pid, float ISRtime) {
	pid-> ISRtime = ISRtime;
	pid-> integralVal = 0;
	pid-> derivativeVal = 0;
	pid-> proportionalVal = 0;
	pid-> prevError = 0;
	pid-> prevMeasure = 0;
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
void TunePID(PID *pid, float ki, float kd, float kp, float K){
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
void CalculatePID(PID *pid, float measurement, float desired){
	//obtain the error of the arm angle
	pid->currentError = measurement - desired;

	//multply the error by the porportional gain
	pid->proportionalVal = pid-> kp * pid->currentError;
	//perform and integral by using the trapezoidal rule multiplied by integral gain
	pid->integralVal += pid->ISRtime * (pid->currentError + pid->prevError)/2 * pid->ki;
	//perform and integral by taking the difference of error over the ISR period and multiply by the derivative gain
	pid->derivativeVal = pid-> kd * (measurement - pid->prevMeasure)/pid->ISRtime;

	//sum all the PID terms and multiply by the ultimate gain
	pid->out = (pid->proportionalVal + pid->integralVal + pid->derivativeVal)*pid->K;
	//saturate the output voltage
	if (pid->out > 24)
		pid->out = 24;
	else if (pid->out < -24)
		pid->out = -24;
	
	//save the error to be used in the next PID calculation for integral and derivative
	pid->prevError = pid->currentError;
	pid->prevMeasure = measurement;
}


