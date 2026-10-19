#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f1xx_hal.h"
#include "main.h"
#include "i2c.h"
#include "math.h"

#define MPU6050_ADDR 0xD0//从机地址
#define SMPLRT_DIV_REG 0x19//采样率
#define CONFIG 0x1A
#define GYRO_CONFIG_REG 0x1B//陀螺仪
#define ACCEL_CONFIG_REG 0x1C//加速度
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define PWR_MGMT_2_REG 0x6C
#define WHO_AM_I_REG 0x75

typedef struct {
	float Ax,Ay,Az; //加速度
	float Gx,Gy,Gz;  //角速度
	float svm;   //合加速度
	float gyr;   //合角速度
}MPU6050_Data_t;

extern MPU6050_Data_t mpu_data;

void MPU6050_Init(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
void MPU6050_Read_Result(void);//读roll, pitch, yaw
void MPU6050_EnterLowPowerMode(void);
void MPU6050_ExitLowPowerMode(void);

//姿态角自适应标定与查询接口（任意佩戴方向，上电站立标定一次）
void Tilt_AccumulateBase(void);            //累加当前帧到基准和
void Tilt_FinishBase(void);                //用累加平均值写入站立基准（姿态角自动清零）
uint8_t Tilt_IsCalibrated(void);           //是否已标定
float MPU6050_Get_TiltAngle(void);         //姿态角：相对站立基准，站立≈0°


//导出，导出后数据才能被其他程序应用
//extern float Ax, Ay, Az, Gx, Gy, Gz,roll, pitch, yaw;//A代表加速度，G代表角速度



#endif
