/************************************************************************
 * Author  :Ahmed Elhamy
 * Date    :18/2/2022
 * Version :v_01
 ***********************************************************************/

/**********************************************************************************
 * this PID control used with MPU sensor only since the derivative value
 * for the angle is the GYRO value so we don't need to differentiate the angle value
 * i make it as simple as possible so i assumed that the REFEERENCE value is always =0
 ************************************************************************************/
#ifndef INC_PID_H_
#define INC_PID_H_
#include <stdint.h>


void PID_Init(float Kp,float kd,float ki,int32_t MinOutputValue,int32_t MaxOutputValue);

int32_t PID_Calc(float Angle,float GyroValue,uint32_t SampleTime_msec);

#endif /* INC_PID_H_ */
