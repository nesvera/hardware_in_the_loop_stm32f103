#include "hil_lib.h"
#include "usbd_cdc_if.h"
#include "math.h"

#define MAXDOUBLE MAX_DOUBLE

double paramHilIn[ NUMPARAMHIL ];
extern uint8_t strHilIn[ BUFFOUTSIZE ];
extern uint8_t strHilOut[ BUFFOUTSIZE ];

 // Maximum value of variables
 //#define MAX_INT         INT16_MAX                                                            /* Descobrir os limites */
 //#define MAX_LONG        INT32_MAX
 //#define MAX_I_TERM      (MAX_LONG / 2)

double MAX_DOUBLE = 65536;
double MAX_I_TERM = 0;

struct PID_DATA pidData;


/**
*
*
*/
void breakString(){

	//	12.525;12;0;12648.5156;*

	uint8_t indBuf = 0;
	uint8_t indBufTemp = 0;
	uint8_t indParam = 0;
	char tempBuffer[32];

	while(1)
	{
		uint8_t tChar = strHilIn[ indBuf++ ];

		if( tChar != '*' )
		{
			if( tChar != ';' )
			{
				tempBuffer[ indBufTemp++ ] = tChar;
			}
			else
			{
				// String to double
				paramHilIn[ indParam++ ] = atof( tempBuffer );
		
        memset( &tempBuffer, 0, sizeof(tempBuffer) );
        indBufTemp = 0;
			}
		}
		else
		{
			return;
		}
	}
}

/**
*
*
*/
void doubleToString( uint8_t precisao ){
    uint8_t indStr = 0;

    //"12.52;15;1;123456.12;987.654321;*"

		memset( strHilOut, '\0', sizeof(strHilOut) );

    uint8_t indParamIn = 0;
    while( indParamIn < NUMPARAMHIL )
    {
        //double temp = fabs( paramHilIn[indParamIn] );
				double temp = -12.56;
        double tempP = fabs(temp);
			
        uint8_t casaInt = 0;
        while(1)
        {
            if( tempP >= pow( 10, casaInt ) )
            {
                casaInt++;
            }
            else
            {
                break;
            }
        }

        double auxP = pow( 10,precisao );
        double numD = fabs(temp*auxP);
        int numInt = numD;

        uint8_t newDoubleFlag = 0;
				
        int numMod = casaInt + precisao;
        while( numMod > 0 ){
            if( newDoubleFlag == 0 )
            {
                if( temp < 0 )
                {
                    strHilOut[indStr++] = '-';
                    //indStr = indStr + 1;
                }
                newDoubleFlag = 1;
            }
            
            if( numMod == precisao )
            {
							strHilOut[indStr++] = '.';
              //indStr = indStr + 1;
            }

            double powD = pow(10, (--numMod) );
            //numMod = numMod - 1;
            int powI = (int)powD;

            double atCasa = ((int)numInt/powI)%10;
            int atCasaI = atCasa;

            strHilOut[indStr++] = '0' + atCasaI;
            //indStr = indStr + 1;
            
        }

        strHilOut[indStr++] = ';' ;
        //indStr = indStr + 1;

        indParamIn++;
        newDoubleFlag = 0;
    }

    strHilOut[indStr++] = '*';
		
		return;
}

/**
*		Set up PID controller parameters
**/
void pid_Init( double p_factor, double i_factor, double d_factor, struct PID_DATA *pid){
  // Start values for PID controller
  pid->sumError = 0;
  pid->lastProcessValue = 0;

  // Tuning constants for PID loop
  pid->P_Factor = p_factor;
  pid->I_Factor = i_factor;
  pid->D_Factor = d_factor;

  // Limits to avoid overflow
  //pid->maxError = MAX_INT / (pid->P_Factor + 1);
  pid->maxError = MAX_DOUBLE / (pid->P_Factor + 1);
  pid->maxSumError = MAX_I_TERM / (pid->I_Factor + 1);
}

/**
*
**/
double pid_Controller( double dSpeed, double mSpeed, struct PID_DATA *pid_st){

  double error, p_term, i_term, d_term, ret, temp;
 
  //error = setPoint - processValue;
  error = dSpeed - mSpeed;

  // Calculate Pterm and limit error overflow
  if (error > pid_st->maxError){                                                          /* Alterar esse limites */
    //p_term = MAX_INT;
    p_term = MAX_DOUBLE;
  
  }else if (error < -pid_st->maxError){
    //p_term = -MAX_INT;
    p_term = -MAX_DOUBLE;
  
  }else{
    p_term = pid_st->P_Factor * error;
  }

  // Calculate Iterm and limit integral runaway
  temp = pid_st->sumError + error;
  if(temp > pid_st->maxSumError){                                                         /* Alterar esse limites */
    i_term = MAX_I_TERM;
    pid_st->sumError = pid_st->maxSumError;
  
  }else if(temp < -pid_st->maxSumError){
    i_term = -MAX_I_TERM;
    pid_st->sumError = -pid_st->maxSumError;
  
  }else{
    i_term = pid_st->I_Factor * pid_st->sumError;
    pid_st->sumError = temp;
  }
 
  // Calculate Dterm
  //d_term = pid_st->D_Factor * (pid_st->lastProcessValue - processValue);
  d_term = pid_st->D_Factor * (pid_st->lastProcessValue - mSpeed );
 
  //pid_st->lastProcessValue = processValue;
  pid_st->lastProcessValue = mSpeed;

  //ret = (p_term + i_term + d_term) / SCALING_FACTOR;
  ret = (p_term + i_term + d_term);
  //if(ret > MAX_INT){                                                                      /* Alterar esse limites */
  if(ret > MAX_DOUBLE){
    //ret = MAX_INT;
    ret = MAX_DOUBLE;


  //}else if(ret < -MAX_INT){
  }else if(ret < -MAX_DOUBLE){
    //ret = -MAX_INT;
    ret = -MAX_DOUBLE;
  }
 
  return ret;

}



