/************************************************************************
 * Author  :Ahmed Elhamy
 * Date    :18/2/2022
 * Version :v_01
 ***********************************************************************/
#ifndef MPU_6050_H_
#define MPU_6050_H_
#include <stdint.h>

#define MPU_REGISTER_ADDRESS_SIZE_VALUE      (7)                 //all registers addresses are 7 bits
#define MPU_REGISTER_SIZE                    (1)                 //all registers are 1 byte size 
#define MPU_ADDRESS                          (0x68)              //You can find it From who am I register  0b110100 0(selected by hardware) 
#define MPU_SMPLRT_DIV_REG                   (0x19)
#define MPU_CONFIG_REG                       (0x1A)
#define MPU_GYROCONFIG_REG                   (0x1B)
#define MPU_ACCELCONFIG_REG                  (0x1C)
#define MPU_FIFO_EN_REG                      (0x23)
#define MPU_USER_CTRL_REG                    (0x6A)

#define MPU_FIFO_EN_VALUE                    (0x40)                 //used with USER_CTRL_REG
#define MPU_FIFO_DISABLE_VALUE               (0x00)
#define MPU_FIFO_RESET_VALUE                 (0x04)
#define MPU_FIFO_EN_GYRO_ACCEL_VALUE         (0x78)                 //FIFO will contain ACCEL_X Y Z_H L GYRO_X Y Z_H L in the same order

#define MPU_GYRO_FS_SEL_250_VALUE            (0x00)                 //with self test disabled 
#define MPU_GYRO_FS_SEL_500_VALUE            (0x08)                       
#define MPU_GYRO_FS_SEL_1000_VALUE           (0x10)
#define MPU_GYRO_FS_SEL_2000_VALUE           (0x18)

#define MPU_ACCEL_FS_SEL_2g_VALUE            (0x00)                 //with self test disabled 
#define MPU_ACCEL_FS_SEL_4g_VALUE            (0x08)                       
#define MPU_ACCEL_FS_SEL_8g_VALUE            (0x10)
#define MPU_ACCEL_FS_SEL_16g_VALUE           (0x18)

#define MPU_ACCEL_FS_2g_SENSITIVITY_VALUE    (16384.0)                 //to get the acceleration value with respect to gravity then divide the accel value by this value  
#define MPU_ACCEL_FS_4g_SENSITIVITY_VALUE    (8192.0)                       
#define MPU_ACCEL_FS_8g_SENSITIVITY_VALUE    (4069.0)
#define MPU_ACCEL_FS_16g_SENSITIVITY_VALUE   (2048.0)

#define MPU_GYRO_FS_250_SENSITIVITY_VALUE    (131.0)                   //to get the gyro value within the range that you choose it to be then divide the gyro value by this value 
#define MPU_GYRO_FS_500_SENSITIVITY_VALUE    (65.5)                       
#define MPU_GYRO_FS_1000_SENSITIVITY_VALUE   (32.8)
#define MPU_GYRO_FS_2000_SENSITIVITY_VALUE   (16.4)

#define MPU_INT_ENABLE_REG                   (0x38)
#define MPU_INT_STATUS_REG                   (0x3A)
#define MPU_ACCEL_XOUT_H_REG                 (0x3B)
#define MPU_ACCEL_XOUT_L_REG                 (0x3C)
#define MPU_ACCEL_YOUT_H_REG                 (0x3D)
#define MPU_ACCEL_YOUT_L_REG                 (0x3E)
#define MPU_ACCEL_ZOUT_H_REG                 (0x3F)
#define MPU_ACCEL_ZOUT_L_REG                 (0x40)
                        
#define MPU_GYRO_XOUT_H_REG                  (0x43)
#define MPU_GYRO_XOUT_L_REG                  (0x44)
#define MPU_GYRO_YOUT_H_REG                  (0x45)
#define MPU_GYRO_YOUT_L_REG                  (0x46)
#define MPU_GYRO_ZOUT_H_REG                  (0x47)
#define MPU_GYRO_ZOUT_L_REG                  (0x48)

#define MPU_CLK_INTERNAL_8_MHZ_VALUE         (0x0)
#define MPU_CLK_PLL_X_REF_VALUE              (0x1)
#define MPU_CLK_PLL_Y_REF_VALUE              (0x2)
#define MPU_CLK_PLL_Z_REF_VALUE              (0x3)                                         
#define MPU_PWR_MGMT_1_REG                   (0x6B)
#define MPU_PWR_MGMT_2_REG                   (0x6C)
#define MPU_FIFO_COUNT_H_REG                 (0x72)
#define MPU_FIFO_COUNT_L_REG                 (0x73)
#define MPU_FIFO_RW_REG                      (0x74)
#define MPU_WHO_AM_I_REG                     (0x75)

#define MPU_WRITE_REG(REG)                   ((REG<<1)|0x00)
#define MPU_READ_REG(REG)                   ((REG<<1)|0x01)



typedef enum{
	MPU_X,
	MPU_Y,
	MPU_Z
}MPU_Axis_t;



/**************************************************************************************
  This function must be called once and before any dealing with the sensor
***************************************************************************************/
void MPU_Init();
/**************************************************************************************
  This Function must be called before getting any specific sensor value
  each call will update the all the MPU readings
***************************************************************************************/
void MPU_Read();
/**************************************************************************************
 Used once, the MPU6050 must be stable during calibration
***************************************************************************************/
void MPU_Calibrate();

void MPU_GetGYROValue(float* Gyro,MPU_Axis_t Axis);

void MPU_GetACCELValue(float* Accel,MPU_Axis_t Axis);
/***************************************************************************************
 Kalman filter is used to get right accurate values
***************************************************************************************/
void MPU_GetAngle(float* Angle,MPU_Axis_t Axis);
















#endif
