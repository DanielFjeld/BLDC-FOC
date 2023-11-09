
#ifndef __PID_H_
#define __PID_H_
	#include <main.h>

	#define DIRECT 0
	#define REVERSE 1

	#define MANUAL 0
	#define AUTOMATIC 1

	#define P_ON_M 0
	#define P_ON_E 1

	typedef struct PID_instance{
		float Input, Output, Setpoint;
		float outputSum, lastInput;
		float kp, ki, kd;
		float outMin, outMax;
		uint32_t SampleTime; //1 000 000 = 1 sec
		uint8_t inAuto;
		int controllerDirection;
		uint8_t pOnE, pOnM;
		float pOnEKp, pOnMKp;
	}PID_instance;

	void Compute(PID_instance *val);
	void SetTunings(PID_instance *val, float Kp, float Ki, float Kd, float pOn);
	void SetSampleTime(PID_instance *val, int NewSampleTime);
	void SetOutputLimits(PID_instance *val, float Min, float Max);
	void SetMode(PID_instance *val, int Mode);
	void Initialize(PID_instance *val);
	void SetControllerDirection(PID_instance *val, int Direction);


#endif /* __PID_H_ */
