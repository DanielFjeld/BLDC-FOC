/*working variables*/
#include "main.h"
#include "PID.h"

#include <stdio.h>

void Compute(PID_instance *val)
{
	if(!val->inAuto) return;
	/*Compute all the working error variables*/
	float error = val->Setpoint - val->Input;
	float dInput = (val->Input - val->lastInput);
	val->outputSum+= (val->ki * error);

	/*Add Proportional on Measurement, if P_ON_M is specified*/
	if(val->pOnM) val->outputSum-= val->pOnMKp * dInput;

	if(val->outputSum > val->outMax) val->outputSum= val->outMax;
	else if(val->outputSum < val->outMin) val->outputSum= val->outMin;

	/*Add Proportional on Error, if P_ON_E is specified*/
	if(val->pOnE) val->Output = val->pOnEKp * error;
	else val->Output = 0;

	/*Compute Rest of PID Output*/
	val->Output += val->outputSum - val->kd * dInput;

	if(val->Output > val->outMax) val->Output = val->outMax;
	else if(val->Output < val->outMin) val->Output = val->outMin;

	/*Remember some variables for next time*/
	val->lastInput = val->Input;
}

void SetTunings(PID_instance *val, float Kp, float Ki, float Kd, float pOn)
{
   if (Kp<0 || Ki<0|| Kd<0 || pOn<0 || pOn>1) return;

   val->pOnE = pOn>0; //some p on error is desired;
   val->pOnM = pOn<1; //some p on measurement is desired;

   float SampleTimeInSec = ((float)val->SampleTime)/1000000;
   val->kp = Kp;
   val->ki = Ki * SampleTimeInSec;
   val->kd = Kd / SampleTimeInSec;

  if(val->controllerDirection ==REVERSE)
   {
	  val->kp = (0 - val->kp);
	  val->ki = (0 - val->ki);
	  val->kd = (0 - val->kd);
   }

  val->pOnEKp = pOn * val->kp;
  val->pOnMKp = (1 - pOn) * val->kp;
}

void SetSampleTime(PID_instance *val, int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = (float)NewSampleTime
                      / (float)val->SampleTime;
      val->ki *= ratio;
      val->kd /= ratio;
      val->SampleTime = (unsigned long)NewSampleTime;
   }
}

void SetOutputLimits(PID_instance *val, float Min, float Max)
{
   if(Min > Max) return;
   val->outMin = Min;
   val->outMax = Max;

   if(val->Output > val->outMax) val->Output = val->outMax;
   else if(val->Output < val->outMin) val->Output = val->outMin;

   if(val->outputSum > val->outMax) val->outputSum= val->outMax;
   else if(val->outputSum < val->outMin) val->outputSum= val->outMin;
}

void SetMode(PID_instance *val, int Mode)
{
    uint8_t newAuto = (Mode == AUTOMATIC);
    if(newAuto == !val->inAuto)
    {  /*we just went from manual to auto*/
        Initialize(val);
    }
    val->inAuto = newAuto;
}

void Initialize(PID_instance *val)
{
	val->lastInput = val->Input;
	val->outputSum = val->Output;
   if(val->outputSum > val->outMax) val->outputSum= val->outMax;
   else if(val->outputSum < val->outMin) val->outputSum= val->outMin;
}

void SetControllerDirection(PID_instance *val, int Direction)
{
	val->controllerDirection = Direction;
}
