#ifndef __hil_lib_H
#define __hil_lib_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "usbd_cdc_if.h"
	 
#define NUMPARAMHIL 1	 
#define BUFFOUTSIZE 64
	 
// No arduino
// Maior número double inteiro          4294967040.00
// Maior número double com decimal            
typedef struct PID_DATA{
  double lastProcessValue;
  double sumError;
  double P_Factor;
  double I_Factor;
  double D_Factor;
  double maxError;
  double maxSumError;
} pidData_t;

void breakString();
void doubleToString( uint8_t precisao );
void pid_Init( double p_factor, double i_factor, double d_factor, struct PID_DATA *pid);
double pid_Controller( double dSpeed, double mSpeed, struct PID_DATA *pid_st);

#ifdef __cplusplus
}
#endif
#endif /*__hil_lib_H */
