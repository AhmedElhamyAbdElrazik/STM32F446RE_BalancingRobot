/************************************************************************
 * Author  :Ahmed Elhamy
 * Date    :18/2/2022
 * Version :v_01
 ***********************************************************************/

#include "PID.h"
#include <stdint.h>
#include   <stdio.h>

static float KP=0,KD=0,KI=0;
static int32_t MinOutput,MaxOutput;

void PID_Init(float kp,float kd,float ki,int32_t MinOutputValue,int32_t MaxOutputValue)
{
	KP=kp;
	KD=kd;
	KI=ki;
	MinOutput=MinOutputValue;
	MaxOutput=MaxOutputValue;
}
/********************************************************************************************
*Here the Angle value  is treated as if it is the error since the reference angle is always =0
*the Gyro value for the same axis is treated as if it is the rate of change of the error
	since it is equal to the rate of change of the angle
*Clamping method is used as an anti wind-up method
 ********************************************************************************************/
int32_t PID_Calc(float Angle,float GyroValue,uint32_t SampleTime_msec)
{
	static uint8_t IntegrateFlag=1,ClampingSaturationFlag=0,ClampingSameSignFlag=0;
	static float IntegrationValue=0;

	float Output=0;

	if(IntegrateFlag)
	{
		IntegrationValue+=(SampleTime_msec/1000.0)*Angle;
	}

	Output=KP*Angle+KD*GyroValue+KI*IntegrationValue;

	if(Output>MaxOutput)
	{
		Output=MaxOutput;
		ClampingSaturationFlag=1;
	}
	else if(Output<MinOutput)
	{
		Output=MinOutput;
		ClampingSaturationFlag=1;
	}
	else
	{
		ClampingSaturationFlag=0;
	}
	//output sign is the same as the error sign
	ClampingSameSignFlag=(((Output>0)&&(Angle>0))||((Output<0)&&(Angle<0)))?1:0;

	IntegrateFlag=(ClampingSameSignFlag&&ClampingSaturationFlag)?0:1;

    return (int32_t)Output;

}
