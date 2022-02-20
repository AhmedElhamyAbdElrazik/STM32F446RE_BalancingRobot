#include "MPU6050.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "main.h"
#include <math.h>







#define PI 3.141592654
extern I2C_HandleTypeDef hi2c1;

static float XKalmanFilter(float AccelAngle,float GyroAngleRate);
static float YKalmanFilter(float AccelAngle,float GyroAngleRate);

static uint8_t Buffer[14];
static float GyroXCal=114.6832,GyroYCal=-73.4804,GyroZCal=-17.698;
static float AccelXCal=-1630.43,AccelYCal=378,AccelZCal=-448.446;

enum {
	ACCEL_X_H,
	ACCEL_X_L,
	ACCEL_Y_H,
	ACCEL_Y_L,
	ACCEL_Z_H,
	ACCEL_Z_L,
	TEMP_H,
	TEMP_L,
	GYRO_X_H,
	GYRO_X_L,
	GYRO_Y_H,
	GYRO_Y_L,
	GYRO_Z_H,
	GYRO_Z_L,
};

void MPU_Init()
{   
    //power on delay
	HAL_Delay(1000);
	MX_I2C1_Init();
	uint8_t DLPF=0x00;                             //bandwidth digital low pass filter
	uint8_t SampleRate=0x07;                      //sampling frequency =GyroFreq/(7+1)   =8KHz/8 =1KHz
    uint8_t ClkSource=MPU_CLK_PLL_X_REF_VALUE;   //MPU_CLK_PLL_X_REF_VALUE;
    uint8_t AccelRange=MPU_ACCEL_FS_SEL_2g_VALUE;
	uint8_t GyroRange=MPU_GYRO_FS_SEL_500_VALUE;
	uint8_t EnableInterrupt=0x01;
	
	//set DLPF
	HAL_I2C_Mem_Write(&hi2c1,MPU_WRITE_REG(MPU_ADDRESS),MPU_CONFIG_REG,MPU_REGISTER_SIZE,&DLPF,MPU_REGISTER_SIZE,HAL_MAX_DELAY);
	//set clk source
	HAL_I2C_Mem_Write(&hi2c1,MPU_WRITE_REG(MPU_ADDRESS),MPU_PWR_MGMT_1_REG,MPU_REGISTER_SIZE,&ClkSource,MPU_REGISTER_SIZE,HAL_MAX_DELAY);
	//set sample data rate
	HAL_I2C_Mem_Write(&hi2c1,MPU_WRITE_REG(MPU_ADDRESS),MPU_SMPLRT_DIV_REG,MPU_REGISTER_SIZE,&SampleRate,MPU_REGISTER_SIZE,HAL_MAX_DELAY);
	//set gyro full scale
	HAL_I2C_Mem_Write(&hi2c1,MPU_WRITE_REG(MPU_ADDRESS),MPU_GYROCONFIG_REG,MPU_REGISTER_SIZE,&GyroRange,MPU_REGISTER_SIZE,HAL_MAX_DELAY);
	//set accel full scale
	HAL_I2C_Mem_Write(&hi2c1,MPU_WRITE_REG(MPU_ADDRESS),MPU_ACCELCONFIG_REG,MPU_REGISTER_SIZE,&AccelRange,MPU_REGISTER_SIZE,HAL_MAX_DELAY);
	//set interrupt
	HAL_I2C_Mem_Write(&hi2c1,MPU_WRITE_REG(MPU_ADDRESS),MPU_INT_ENABLE_REG,MPU_REGISTER_SIZE,&EnableInterrupt,MPU_REGISTER_SIZE,HAL_MAX_DELAY);

}

void MPU_Read()
{

	uint8_t InterruptStatus=0;
	while(InterruptStatus!=1)
	{
		HAL_I2C_Mem_Read(&hi2c1,MPU_READ_REG(MPU_ADDRESS),MPU_INT_STATUS_REG,MPU_REGISTER_SIZE,&InterruptStatus,1,HAL_MAX_DELAY);
	}
	HAL_I2C_Mem_Read(&hi2c1,MPU_READ_REG(MPU_ADDRESS),MPU_ACCEL_XOUT_H_REG,MPU_REGISTER_SIZE,(uint8_t*)&Buffer,sizeof(Buffer),HAL_MAX_DELAY);
}

void MPU_Calibrate()
{
	printf("Calibrating...\r\n");

	uint16_t i;
	int16_t GX,GY,GZ,AX,AY,AZ;
	GyroXCal=0,GyroYCal=0,GyroZCal=0;
	AccelXCal=0,AccelYCal=0,AccelZCal=0;

	for(i=0;i<2000;i++)
	{
		MPU_Read();
		GX=(int16_t)(((uint16_t)Buffer[GYRO_X_H ]<<8)|Buffer[GYRO_X_L ]);
		GY=(int16_t)(((uint16_t)Buffer[GYRO_Y_H ]<<8)|Buffer[GYRO_Y_L ]);
		GZ=(int16_t)(((uint16_t)Buffer[GYRO_Z_H ]<<8)|Buffer[GYRO_Z_L ]);
		AX=(int16_t)(((uint16_t)Buffer[ACCEL_X_H]<<8)|Buffer[ACCEL_X_L]);
		AY=(int16_t)(((uint16_t)Buffer[ACCEL_Y_H]<<8)|Buffer[ACCEL_Y_L]);
		AZ=(int16_t)(((uint16_t)Buffer[ACCEL_Z_H]<<8)|Buffer[ACCEL_Z_L]);

		GyroXCal+=GX/2000.0;
		GyroYCal+=GY/2000.0;
		GyroZCal+=GZ/2000.0;
		AccelXCal+=AX/2000.0;
		AccelYCal+=AY/2000.0;
		AccelZCal+=((int32_t)16384-AZ)/2000.0;
	}
	printf("%f,%f,%f,%f,%f,%f\r\n",GyroXCal,GyroYCal,GyroZCal,AccelXCal,AccelYCal,AccelZCal);
}
 
void MPU_GetGYROValue(float* Gyro,MPU_Axis_t Axis)
{
	switch(Axis)
	{
		case MPU_X:
			*Gyro=((int16_t)(((uint16_t)Buffer[GYRO_X_H]<<8)|Buffer[GYRO_X_L])-GyroXCal)/MPU_GYRO_FS_500_SENSITIVITY_VALUE;
		break;
		case MPU_Y:
			*Gyro=((int16_t)(((uint16_t)Buffer[GYRO_Y_H]<<8)|Buffer[GYRO_Y_L])-GyroYCal)/MPU_GYRO_FS_500_SENSITIVITY_VALUE;
		break;
		case MPU_Z:
			*Gyro=((int16_t)(((uint16_t)Buffer[GYRO_Z_H]<<8)|Buffer[GYRO_Z_L])-GyroZCal)/MPU_GYRO_FS_500_SENSITIVITY_VALUE;
		break;
	}

}

void MPU_GetACCELValue(float* Accel,MPU_Axis_t Axis)
{
	switch(Axis)
	{
		case MPU_X:
			*Accel=((int16_t)(((uint16_t)Buffer[ACCEL_X_H]<<8)|Buffer[ACCEL_X_L])-AccelXCal)/MPU_ACCEL_FS_2g_SENSITIVITY_VALUE;
		break;
		case MPU_Y:
			*Accel=((int16_t)(((uint16_t)Buffer[ACCEL_Y_H]<<8)|Buffer[ACCEL_Y_L])-AccelYCal)/MPU_ACCEL_FS_2g_SENSITIVITY_VALUE;
		break;
		case MPU_Z:
			*Accel=((int16_t)(((uint16_t)Buffer[ACCEL_Z_H]<<8)|Buffer[ACCEL_Z_L])-AccelZCal)/MPU_ACCEL_FS_2g_SENSITIVITY_VALUE;
		break;
	}

}

void MPU_GetAngle(float* Angle,MPU_Axis_t Axis)
{
	float GyroX=0,GyroY=0;
	float AccelX=0,AccelY=0,AccelZ=0;
	float RollAngle=0,PitchAngle=0;

	switch(Axis)
	{
		case MPU_X:
			 MPU_GetGYROValue (&GyroX, Axis);
			 MPU_GetACCELValue(&AccelY,MPU_Y);
			 MPU_GetACCELValue(&AccelZ,MPU_Z);
			 RollAngle=atan2(AccelY,AccelZ)*180.0/PI;
			 *Angle=XKalmanFilter(RollAngle, GyroX);
		break;
		case MPU_Y:
			 MPU_GetGYROValue (&GyroY, Axis);
			 MPU_GetACCELValue(&AccelX,MPU_X);
			 MPU_GetACCELValue(&AccelY,MPU_Y);
			 MPU_GetACCELValue(&AccelZ,MPU_Z);
			 PitchAngle=atan2(-AccelX,sqrt(AccelZ*AccelZ+AccelY*AccelY))*180.0/PI;
			 *Angle=YKalmanFilter(PitchAngle,GyroY);
		break;
		case MPU_Z:
			*Angle=0;
		break;
	}

}


static float XKalmanFilter(float AccelAngle,float GyroAngleRate)
{
	static float  PredictedAngle=0,
			      MeasuredAngle=0,
			      PreditedBias=0,
				  P[2][2]={{0,0},{0,0}},
				  QAngle=0.001,
				  QBias=0.01,
				  R_K=0.09,
				  K[2][1]={{0},{0}},
				  H[2]={1,0},
				  dt_K=-1;
	//not enter in the first time just update dt_K
	if(dt_K!=-1)
	{
	  dt_K=(HAL_GetTick()-(uint32_t)dt_K)/1000.0;
	  //prediction
	  PredictedAngle+=dt_K*(GyroAngleRate-PreditedBias);

	  P[0][0]=P[0][0]+dt_K*(dt_K*P[1][1]-P[1][0]-P[0][1]+QAngle);
	  P[0][1]=P[0][1]-dt_K*P[1][1];
	  P[1][0]=P[1][0]-dt_K*P[1][1];
	  P[1][1]=P[1][1]+dt_K*QBias;

	 //measured
	  MeasuredAngle=H[0]*AccelAngle;

	  //kalman filter gain
	  K[0][0]=P[0][0]/(P[0][0]+R_K);
	  K[1][0]=P[1][0]/(P[0][0]+R_K);

	  //estimated result
	  PredictedAngle=PredictedAngle+K[0][0]*(MeasuredAngle-PredictedAngle);
	  PreditedBias=PreditedBias+K[1][0]*(MeasuredAngle-PredictedAngle);

	  double PrevP00=P[0][0],PrevP01=P[0][1];
	  P[0][0]=P[0][0]*(1-K[0][0]);
	  P[0][1]=P[0][1]*(1-K[0][0]);
	  P[1][0]=P[1][0]-K[1][0]*PrevP00;
	  P[1][1]=P[1][1]-K[1][0]*PrevP01;
	}
    dt_K=HAL_GetTick();
    return PredictedAngle;
}


static float YKalmanFilter(float AccelAngle,float GyroAngleRate)
{

	static float  PredictedAngle=0,
			      MeasuredAngle=0,
			      PreditedBias=0,
				  P[2][2]={{0,0},{0,0}},
				  QAngle=0.001,
				  QBias=0.01,
				  R_K=0.09,
				  K[2][1]={{0},{0}},
				  H[2]={1,0},
				  dt_K=-1;
	//not enter in the first time just update dt_K
	if(dt_K!=-1)
	{
	  dt_K=(HAL_GetTick()-(uint32_t)dt_K)/1000.0;
	  //prediction
	  PredictedAngle+=dt_K*(GyroAngleRate-PreditedBias);

	  P[0][0]=P[0][0]+dt_K*(dt_K*P[1][1]-P[1][0]-P[0][1]+QAngle);
	  P[0][1]=P[0][1]-dt_K*P[1][1];
	  P[1][0]=P[1][0]-dt_K*P[1][1];
	  P[1][1]=P[1][1]+dt_K*QBias;

	 //measured
	  MeasuredAngle=H[0]*AccelAngle;

	  //kalman filter gain
	  K[0][0]=P[0][0]/(P[0][0]+R_K);
	  K[1][0]=P[1][0]/(P[0][0]+R_K);

	  //estimated result
	  PredictedAngle=PredictedAngle+K[0][0]*(MeasuredAngle-PredictedAngle);
	  PreditedBias=PreditedBias+K[1][0]*(MeasuredAngle-PredictedAngle);

	  double PrevP00=P[0][0],PrevP01=P[0][1];
	  P[0][0]=P[0][0]*(1-K[0][0]);
	  P[0][1]=P[0][1]*(1-K[0][0]);
	  P[1][0]=P[1][0]-K[1][0]*PrevP00;
	  P[1][1]=P[1][1]-K[1][0]*PrevP01;
	}
  dt_K=HAL_GetTick();
  return PredictedAngle;

}
