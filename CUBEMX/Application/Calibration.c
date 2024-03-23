/*
 * Calibration.c
 *
 *  Created on: Dec 10, 2023
 *      Author: Daniel
 */


#include "calibration.h"
//#include "foc.h"
//#include "PreferenceWriter.h"
//#include "user_config.h"
//#include "motor_config.h"
//#include "current_controller_config.h"

#include "CTRL.h"
#include "Encoders_SPI.h"
#include "current_ADC.h"
#include "print_server.h"
#include "math.h"

#define CAL_DUTY 400

//uint32_t find_closest(float arr[], int length, float target);


#define WINDOW_SIZE SIZE


float error_filt[SIZE*NPP] = {0};
float error_filt_temp[SIZE*NPP] = {0};

uint8_t PHASE_ORDER = 0;
float electrical_offset = 0;
float pi = 3.14159265f;

uint32_t motor_lut[LUT_SIZE];

#define MAX_IIR 16
typedef struct IIR{
	uint8_t size;
	float Coef_a[MAX_IIR];
	float Coef_b[MAX_IIR];

	float last_x[MAX_IIR];
	float last_y[MAX_IIR];
}IIR_t;

float IIR(IIR_t *handler, float in){

	//----------SHIFT LAST VALUES
	for(int i = handler->size; i > 0; i--)handler->last_y[i] = handler->last_y[i-1]; //delay input
	for(int i = handler->size; i > 0; i--)handler->last_x[i] = handler->last_x[i-1]; //delay output
	handler->last_x[0] = in;

	//----------CALCULATE NEXT OUTPUT
	float y = 0;
	for(int i = 0; i <= handler->size; i++)y +=  handler->Coef_b[i]*handler->last_x[i];
	for(int i = 1; i <= handler->size; i++)y -=  handler->Coef_a[i]*handler->last_y[i];
	handler->last_y[0] = y;

	//----------RETURN OUTPUT
	return y;
}
IIR_t LPF1 = {
		.size = 3,
		.Coef_a = {
			1.0f,
			-1.99f,
			0.99f
		},
		.Coef_b = {
			0.0015f,
			0.0029f,
			0.0015f
		},
		.last_x = {0},
		.last_y = {0}
};

void order_phases(Encoders *ps, Current *cs){ //, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs

    ///Checks phase order, to ensure that positive Q current produces
    ///torque in the positive direction wrt the position sensor.
	PrintServerPrintf("\n\r Checking phase ordering\n\r");
    float theta_ref = 0;
    float theta_actual = 0;
    int sample_counter = 0;

    float d;
    float q;

    PHASE_ORDER = 0;

    ///Set voltage angle to zero, wait for rotor position to settle
    inverter((int16_t)theta_ref, CAL_DUTY, PHASE_ORDER);
    HAL_Delay(1000);
    //float theta_start = ps->GetMechPositionFixed();                                  //get initial rotor position
    float theta_start;

    //current d and q
    dq0(theta_ref*pi/180, (float)cs->Current_M1/1000, (float)cs->Current_M2/1000, (float)cs->Current_M3/1000, &d, &q);
//    controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);    //Calculate phase currents from ADC readings
//    controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);
//    controller->i_a = -controller->i_b - controller->i_c;
//    dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //dq0 transform on currents
    float current = sqrt((d*d + q*q));
    PrintServerPrintf("\n\rCurrent\n\r");
    PrintServerPrintf("%f %f %f\n\r\n\r", d, q, current);
    /// Rotate voltage angle
    while(theta_ref < 360*2){       //rotate for 2 electrical cycles
    	inverter((int16_t)theta_ref, CAL_DUTY, PHASE_ORDER);
    	HAL_Delay(1);
       theta_actual = (float)ps->Encoder1_pos/1000; //sample position sensor
       if(theta_ref==0){theta_start = theta_actual;}
       if(sample_counter >= 1){
           sample_counter = 0 ;
           PrintServerPrintf("%.4f %.4f\n\r", (float)theta_ref, theta_actual);
        }
        sample_counter++;
       theta_ref += 1;
        }
    float theta_end = (float)ps->Encoder1_pos/1000;
    int direction = (theta_end - theta_start)>0;
    if ((theta_end - theta_start) > 180) direction = 0;
    if ((theta_end - theta_start) < -180) direction = 1;
    PrintServerPrintf("Theta Start:   %f    Theta End:  %f\n\r", theta_start, theta_end);
    PrintServerPrintf("Direction:  %d\n\r", direction);
    if(direction){PrintServerPrintf("Phasing correct\n\r");}
    else if(!direction){PrintServerPrintf("Phasing incorrect.  Swapping phases V and W\n\r");}
    PHASE_ORDER = !direction;
    HAL_Delay(10);
    }

float error_temp = 0.0f;

float aa_test_3 = 0;
float aa_test_4 = 0;

void smoothArray(float *arr, int size) {
    float prev, current;

    if (size < 3) return; // Not enough elements to smooth

    prev = arr[0];
    for (int i = 1; i < size - 1; i++) {
        current = arr[i];
        arr[i] = (prev + arr[i] + arr[i + 1]) / 3;
        prev = current;
    }
}

void calibrate(Encoders *ps, Current *cs){ //, PositionSensor *ps, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs
    /// Measures the electrical angle offset of the position sensor
    /// and (in the future) corrects nonlinearity due to position sensor eccentricity
	PrintServerPrintf("Starting calibration procedure\n\r");

   const int n = SIZE*NPP;                                                      // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
   const int n2 = 40;                                                          // increments between saved samples (for smoothing motion)
   float delta = 360.0f*NPP/(n*n2);                                              // change in angle between samples
   const int  n_lut =  SIZE*NPP;
   const int window = WINDOW_SIZE;
   float cogging_current[WINDOW_SIZE] = {0};


   float theta_ref = 0;
   float theta_ref_last = 0;
   int8_t theta_ref_count = 0;

   float theta_actual = 0;
   float theta_actual_last = 0;
   int8_t theta_actual_count = 0;

   float d;
   float q;


    float error_f[n];
    float error_b[n];
    int lut[n];
    int raw_f[n];
    int raw_b[n];
    float error[n];
//    float error_filt[SIZE*NPP] = {0};

    //ps->WriteLUT(lut);




    ///Set voltage angle to zero, wait for rotor position to settle
    inverter((int16_t)theta_ref, CAL_DUTY, PHASE_ORDER);
    HAL_Delay(1000);

    dq0(theta_ref*pi/180, (float)cs->Current_M1/1000, (float)cs->Current_M2/1000, (float)cs->Current_M3/1000, &d, &q);
    float current = sqrt((d*d + q*q));
    PrintServerPrintf("Current Angle : Rotor Angle : Raw Encoder \n\r\n\r");
    for(int i = 0; i<n; i++){                                                   // rotate forwards
       for(int j = 0; j<n2; j++){
        theta_ref += delta;
        inverter((int16_t)theta_ref, CAL_DUTY, PHASE_ORDER);
        HAL_Delay(1);
       theta_actual = (float)ps->Encoder1_pos/1000; //fixed position
       if(!i)theta_actual_last=theta_actual;

       if(theta_actual-theta_actual_last < -90) theta_actual_count--;
       if(theta_actual-theta_actual_last > 90) theta_actual_count++;
       theta_actual_last = theta_actual;

       //aa_test_3 = theta_ref/NPP;
       aa_test_4 = theta_actual+theta_actual_count*360;
       error_temp = (theta_ref/NPP - theta_actual+theta_actual_count*360);
       error_f[i] = error_temp;
       raw_f[i] = ps->Encoder1_pos_raw; //raw position
       PrintServerPrintf("%.4f %.4f%d\n\r", theta_ref/(NPP), theta_actual, raw_f[i]);
        }
    }

    for(int i = 0; i<n; i++){                                                   // rotate backwards
       for(int j = 0; j<n2; j++){
       theta_ref -= delta;
       inverter((int16_t)theta_ref, CAL_DUTY, PHASE_ORDER);
       HAL_Delay(1);                                                         // sample position sensor
       theta_actual = (float)ps->Encoder1_pos/1000;

       if(theta_actual-theta_actual_last < -90) theta_actual_count--;
	   if(theta_actual-theta_actual_last > 90) theta_actual_count++;
	   theta_actual_last = theta_actual;

	   aa_test_4 = theta_actual+theta_actual_count*360;
	   error_temp = (theta_ref/NPP - theta_actual+theta_actual_count*360);
       error_b[i] = error_temp;

       aa_test_3 = 0.5f*(error_b[i] + error_f[n-i-1]);

       raw_b[i] =  ps->Encoder1_pos_raw;
       PrintServerPrintf("%.4f %.4f %d\n\r", theta_ref/(NPP), theta_actual, raw_b[i]);
       //theta_ref -= delta;
        }
    }

        electrical_offset = 0;
        for(int i = 0; i<n; i++){
        	electrical_offset += (error_f[i] + error_b[n-1-i])/(2.0f*n);                   // calclate average position sensor offset
            }
        electrical_offset = fmod(electrical_offset*NPP, 360);                                        // convert mechanical angle to electrical angle

        for (int i = 0; i<n; i++){
        	error_filt[i] = 0.5f*(error_f[i] + error_b[n-i-1]);
        }

        //smoothArray(error_filt, n);


        PrintServerPrintf("\n\rEncoder Electrical Offset (deg) %f\n\r",  electrical_offset);
    }
