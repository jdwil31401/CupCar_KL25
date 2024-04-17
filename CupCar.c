//#include "derivative.h" /* include peripheral declarations */
#include "CMSIS\MKL25Z4.h"
#include "TFC\MKL25Z4.h" /* include peripheral declarations */
#include "TFC\TFC.h"
#include "core_cm0plus.h"
#include <math.h>
#include <stdlib.h>

#define THRESHOLD 3500
#define RTHRESH 110
#define LTHRESH 16
#define LEDGENULL -200
#define REDGENULL 260
#define STEERAMOUNT .013
#define EXPOSURE_TIME 16000
#define MOTOR_SPEED_DIV 3
#define SPEED_DRIFT_MULT 1.30
#define LEFT 1u
#define RIGHT 2u
#define STRAIGHT 0u

int   sobel[5] = {1, 2, 0, -2, -1};
int   x1, y1;
int   D1[128];
int8_t Led=0;
int8_t Red=0;
float R_Turn_perc=0;
float L_Turn_perc=0;
float Current_Turn_perc=0;
float temppot0=0;
uint8_t turn=0;

void Set_Motor_Speed1( float turn_speed_perc, uint8_t direction);
void Auto_Drive(void); 

int main(void)
{
        int16_t t, Delta, i=0;
        int16_t  LineScanTemp[128], Ledge, Redge, Center;

        TFC_Init();
        t = 0;
        i = 0;
        TFC_Ticker[3] = 0;

        while (!(TFC_PUSH_BUTTON_0_PRESSED)){


                while (TFC_Ticker[3]<1000) continue;
                TFC_Ticker[3] = 0;
                TFC_BAT_LED0_ON;
                while (TFC_Ticker[3]<1000) continue;
                TFC_Ticker[3] = 0;
                TFC_BAT_LED0_OFF;
        }
        TFC_Ticker[3] = 0;
        TFC_SetLineScanExposureTime(EXPOSURE_TIME);//micoseconds

        while (TFC_Ticker[3]<2000) continue; //wait 2 seconds p button push
        TFC_SetServo(0,0.0);  //center wheels

        for(;;)
        {
                //TFC_Task must be called in your main loop.  This keeps certain processing happy (I.E. Serial port queue check)
                TFC_Task();
                //This Demo program will look at the middle 2 switch to select one of 4 demo modes.
                //Let's look at the middle 2 switches
                switch((TFC_GetDIP_Switch()>>1)&0x03)
                {
                default:
                case 0 :
                        //Demo mode 0 just tests the switches and LED's
                        if(TFC_PUSH_BUTTON_0_PRESSED)
                                TFC_BAT_LED0_ON;
                        else
                                TFC_BAT_LED0_OFF;

                        if(TFC_PUSH_BUTTON_1_PRESSED)
                                TFC_BAT_LED3_ON;
                        else
                                TFC_BAT_LED3_OFF;

                        if(TFC_GetDIP_Switch()&0x01)
                                TFC_BAT_LED1_ON;
                        else
                                TFC_BAT_LED1_OFF;

                        if(TFC_GetDIP_Switch()&0x08)
                                TFC_BAT_LED2_ON;
                        else
                                TFC_BAT_LED2_OFF;
                        break;

                case 1:
                        //Demo mode 1 will just move the servos with the on-board potentiometers
                        if(TFC_Ticker[0]>=20)
                        {
                                TFC_Ticker[0] = 0; //reset the Ticker
                                //Every 20 mSeconds, update the Servos
                                TFC_SetServo(0,TFC_ReadPot(0));
                                TFC_SetServo(1,TFC_ReadPot(1));
                        }
                        //Let's put a pattern on the LEDs
                        if(TFC_Ticker[1] >= 125)
                        {
                                TFC_Ticker[1] = 0;
                                t++;
                                if(t>4)
                                {
                                        t=0;
                                }
                                TFC_SetBatteryLED_Level(t);
                        }


                        TFC_SetMotorPWM(0,0); //Make sure motors are off
                        TFC_HBRIDGE_DISABLE;
                        break;

                case 2 :
                        //Demo Mode 2 will use the Pots to make the motors move
                        TFC_HBRIDGE_ENABLE;
                        TFC_SetMotorPWM(TFC_ReadPot(0),TFC_ReadPot(1));
                        //Let's put a pattern on the LEDs
                        if(TFC_Ticker[1] >= 125)
                                {
                                        TFC_Ticker[1] = 0;
                                                t++;
                                                if(t>4)
                                                {
                                                        t=0;
                                                }
                                        TFC_SetBatteryLED_Level(t);
                                }
                        break;

                case 3 :
                        Auto_Drive();
                }   //end of Case
        }// end of infinite for loop

        return 0;
}  // end of main SHOULD NEVER RETURN

void Auto_Drive(void)
{       
        // Sobel operator: original image is img[ ]
        // D1 is resultant of sobel operation
        Set_Motor_Speed1(Current_Turn_perc,turn);

        if(TFC_Ticker[0]>100 && LineScanImageReady==1)
        {
                TFC_Ticker[0] = 0;
                LineScanImageReady=0;

                //  Here is the line scan processing for black line detection Method #2
                R_Turn_perc=0;
                L_Turn_perc=0;
                //200 for no edge
                Ledge=LEDGENULL;
                Redge=REDGENULL;
                Led=0b0;
                Red=0b0;
                for(i=0;i<128;i++)   //first save the line to the temp file
                {
                        LineScanTemp[i] = LineScanImage0[i];
                }

                memset(D1, 0, sizeof(D1));
                //Sobel Operator to transform Linescan data
                for (x1 = 2; x1 < 125; x1++)
                {
                        for (y1 = 0; y1 < 5; y1++)
                        {
                                D1[x1] += sobel[y1] * LineScanTemp[x1+y1-2];
                        }
                        //TERMINAL_PRINTF("%X,",D1[x1]); // Use this print to anylyze Edge data
                }
                //Find Left Edge Index
                for(i=0; i<100;i++)
                {
                        if(D1[i]<=-THRESHOLD && Led==0)
                        {
                                Ledge=i;
                                Led=0b1;
                                break;
                        }
                }
                // Find Right Edge index // since could expand based upon last i value
                for(i=27; i<127;i++)
                {
                        if(D1[i]>=THRESHOLD&& Red==0 )
                        {
                                Redge=i; 
                                Red=0b1;
                                break;
                        }
                //TERMINAL_PRINTF("%X,",D1[i]); // Use this print to anylyze Edge data
                }
                        //right edge not seen turn right
                        //XNOR IF BOTH ARE NULL NO EDGE FOUND OR IF BOTH EDGES FOUND
                Delta = abs(Redge - Ledge);
                //calculate center
                Center = Ledge + ((Delta)/2);
                //case where redge exists but needs to turn right D[Ledge]>D[Redge]
                if(Delta<17)
                {
                if(-D1[Ledge]>D1[Redge])
                {
                        Redge=REDGENULL;
                        Red=0b0;
                //to turn right
                }
                else
                {
                        Led=0b0;
                //to turn left
                }
                }
                //turn right
        }
        //now make adjustment in steering
        if(((Center >= 60) && (Center <= 68))||Center==30) 
        {
                TFC_SetServo(0,0.0);
                turn=STRAIGHT;//center wheels
                Current_Turn_perc=0;
        }
        else if (Led==0||(Redge < RTHRESH))
        {
                L_Turn_perc = (127-Redge)*STEERAMOUNT;
                turn=LEFT;
                Current_Turn_perc=L_Turn_perc;
                TFC_SetServo(0,-L_Turn_perc);                                                //turn left
        }
        else if (Red==0||(Ledge > LTHRESH)){
                R_Turn_perc= Ledge*STEERAMOUNT;
                turn=RIGHT;
                Current_Turn_perc=R_Turn_perc;
                TFC_SetServo(0,+R_Turn_perc);                                                //turn right
        }
        break;
}
void Set_Motor_Speed1(float turn_speed_perc, uint8_t direction)
{
		switch (direction)
		{
			case 0: //Straight
				TFC_HBRIDGE_ENABLE;
				TFC_SetMotorPWM(1,1);
				TFC_HBRIDGE_DISABLE;
				TFC_SetMotorPWM(1,1);
				break;
			case 1: //Turn Left
				//backright motor faster
				temppot0=TFC_ReadPot(0);
				turn_speed_perc /= MOTOR_SPEED_DIV;
				temppot0 = temppot0 - turn_speed_perc;
				TFC_HBRIDGE_ENABLE;
				TFC_SetMotorPWM(temppot0,temppot0*SPEED_DRIFT_MULT);
				TFC_HBRIDGE_DISABLE;
				TFC_SetMotorPWM(temppot0,temppot0*SPEED_DRIFT_MULT);
				break;
			case 2://Turn right
				//back left motor faster
				temppot0=TFC_ReadPot(0);
				turn_speed_perc /= MOTOR_SPEED_DIV;
				temppot0 = temppot0 - turn_speed_perc;
				TFC_HBRIDGE_ENABLE;
				TFC_SetMotorPWM(temppot0*SPEED_DRIFT_MULT,temppot0);
				TFC_HBRIDGE_DISABLE;
				TFC_SetMotorPWM(temppot0*SPEED_DRIFT_MULT,temppot0);
				break;
		}
}