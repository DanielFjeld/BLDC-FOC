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

#define CAL_DUTY 100

//uint32_t find_closest(float arr[], int length, float target);


#define WINDOW_SIZE SIZE


float error_filt[SIZE*NPP] = {0};

uint8_t PHASE_ORDER = 0;
float electrical_offset = 0;
float pi = 3.14159265f;

uint32_t motor_lut[LUT_SIZE];

void order_phases(Encoders *ps, Current *cs){ //, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs

    ///Checks phase order, to ensure that positive Q current produces
    ///torque in the positive direction wrt the position sensor.
	PrintServerPrintf("\n\r Checking phase ordering\n\r");
    float theta_ref = 0;
    float theta_actual = 0;
    int sample_counter = 0;

    float d;
    float q;


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
   float theta_actual = 0;

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
       error_f[i] = theta_ref/NPP - theta_actual;
	   if(error_f[i] < 0)error_f [i] = error_f[i]+ 360.0f;
       raw_f[i] = ps->Encoder1_pos_raw; //raw position
       PrintServerPrintf("%.4f %.4f%d\n\r", theta_ref/(NPP), theta_actual, raw_f[i]);
        }
    }

    for(int i = 0; i<n; i++){                                                   // rotate backwards
       for(int j = 0; j<n2; j++){
       theta_ref -= delta;
       inverter((int16_t)theta_ref, CAL_DUTY, PHASE_ORDER);
       HAL_Delay(1);                                                         // sample position sensor
       theta_actual = (float)ps->Encoder1_pos/1000;                                   // get mechanical position
       error_b[i] = theta_ref/NPP - theta_actual;
       if(error_b[i] < 0)error_b[i] = error_b[i]+ 360.0f;
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

        /// Perform filtering to linearize position sensor eccentricity
        /// FIR n-sample average, where n = number of samples in one electrical cycle
        /// This filter has zero gain at electrical frequency and all integer multiples
        /// So cogging effects should be completely filtered out.


        float mean = 0;
        for (int i = 0; i<n; i++){                                              //Average the forward and back directions
            error[i] = 0.5f*(error_f[i] + error_b[n-i-1]);
            }
        for (int i = 0; i<n; i++){
            for(int j = 0; j<window; j++){
                int ind = -window/2 + j + i;                                    // Indexes from -window/2 to + window/2
                if(ind<0){
                    ind += n;}                                                  // Moving average wraps around
                else if(ind > n-1) {
                    ind -= n;}
                if(error[ind] == NAN)while(1);
                error_filt[i] += error[ind]/(float)window;
                if(error_filt[i] == NAN)while(1);
                }
            if(i<window){
                cogging_current[i] = current*sinf((error[i] - error_filt[i])*NPP);
                }
//            PrintServerPrintf("%.4f   %4f    %.4f   %.4f\n\r", error[i], error_filt[i], error_f[i], error_b[i]);
//            HAL_Delay(10);
            mean += error_filt[i]/n;
            }
        int raw_offset = (raw_f[0] + raw_b[n-1])/2;                             //Insensitive to errors in this direction, so 2 points is plenty


//        PrintServerPrintf("\n\r Encoder non-linearity compensation table\n\r");
//        PrintServerPrintf("Sample Number : Lookup Index : Lookup Value\n\r\n\r");
//        for (int i = 0; i<n_lut; i++){                                          // build lookup table
//            int ind = (raw_offset>>7) + i;
//            if(ind > (n_lut-1)){
//                ind -= n_lut;
//                }
//            lut[ind] = (int) (((error_filt[i] - mean)*(float)(CPR)/(360.0f)));
//            PrintServerPrintf("%d %d %d\n\r", i, ind, lut[ind]);
//            HAL_Delay(10);
//            }

//        ps->WriteLUT(lut);                                                      // write lookup table to position sensor object
        //memcpy(controller->cogging, cogging_current, sizeof(controller->cogging));  //compensation doesn't actually work yet....

//        memcpy(&ENCODER_LUT, lut, 128*4);                                 // copy the lookup table to the flash array
        PrintServerPrintf("\n\rEncoder Electrical Offset (deg) %f\n\r",  electrical_offset);
//
//        float error_test[SIZE*NPP] = {0};
//        for(int i = 0; i < SIZE*NPP; i++){
//        	error_test[i] = error_filt[i] + i * 360.0 / (SIZE*NPP);
//        	if(error_test[i] > 360.0f)error_test[i] -= 360.0f;
//        }
//        for (int i = 0; i<LUT_SIZE; i++){
//        	float wanted_pos = i *360.0f / LUT_SIZE;
//        	motor_lut[i] = find_closest(error_test, SIZE*NPP, wanted_pos);
//        	PrintServerPrintf("%d\n\r", motor_lut[i]);
//        }
    }

// Function to find the closest number in the array
//uint32_t find_closest(float arr[], int length, float target){
//    float smallest_diff = arr[0] - target;
//    if(smallest_diff < 0)smallest_diff = -1*smallest_diff;
//    uint32_t index = 0;
//
//    for (int i = 1; i < length; i++) {
//    	float diff;
//        diff = arr[i] - target;
//        if(diff < 0)diff = -1*diff;
//
//        if (diff < smallest_diff) {
//            smallest_diff = diff;
//            index = i;
//        }
//
//        diff = arr[i] - target - 360.0f;
//        if(diff < 0)diff = -1*diff;
//
//		if (diff < smallest_diff) {
//		smallest_diff = diff;
//		index = i;
//		}
//
//		diff = arr[i] - target + 360.0f;
//		if(diff < 0)diff = -1*diff;
//
//		if (diff < smallest_diff) {
//		  smallest_diff = diff;
//		  index = i;
//		}
//
//    }
//    return index; //(index*360000)/length;
//}
