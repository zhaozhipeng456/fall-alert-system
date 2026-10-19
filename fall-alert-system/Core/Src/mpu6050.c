#include "mpu6050.h"
#include "math.h"

 int16_t Accel_X_RAW = 0;
 int16_t Accel_Y_RAW = 0;
 int16_t Accel_Z_RAW = 0;
 int16_t Gyro_X_RAW = 0;
 int16_t Gyro_Y_RAW = 0;
 int16_t Gyro_Z_RAW = 0;

//加速度计标定参数，硬件不同，值也不同
int16_t AccX_Off = 0;
int16_t AccY_Off = 0;
int16_t AccZ_Off = 0;
float AccX_Scl = 1.0;
float AccY_Scl = 1.0;
float AccZ_Scl = 1.0;

//陀螺仪零点偏移（陀螺仪只需要Offset，不存在增益，一般不校增益）
float GyroX_Off = 1.22;
float GyroY_Off = 0.93;
float GyroZ_Off = 0.43;

//使用加速度计算《加速度欧拉角》，先定义几个变量。_a的意思就是用的加速度
float roll_a, pitch_a;

//使用角速度计算《角速度欧拉角》，先定义几个变量。_g的意思就是用的角速度
float roll_g, pitch_g, yaw_g;


//欧拉角
float roll, pitch, yaw;

//定义结构体变量
MPU6050_Data_t mpu_data;

//以下是函数
void MPU6050_Init(void)//初始化
{
	uint8_t check;
	uint8_t Data;

	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
	
	if (check == 0x70)  //不同芯片ID号可能不同，我的是旧版MPU6050
	{
		//电源管理1
		Data = 0x01;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

		//电源管理2
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_2_REG, 1, &Data, 1, 1000);

		//滤波
		Data = 0x04;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, CONFIG, 1, &Data, 1, 1000);
		Data = 0x03;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x1D, 1, &Data, 1, 1000);


		//采样频率分频器寄存器
		Data = 0x09;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		//加速度计配置，+16
		Data = 0x18;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		//陀螺仪配置 +16
		Data = 0x18;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
}

void MPU6050_Read_Accel(void)//读取加速度 （单位：g,量程：+-16g）
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	
	//=====标定校正核心=====
	float rawX = (Accel_X_RAW - AccX_Off) * AccX_Scl;
	float rawY = (Accel_Y_RAW - AccY_Off) * AccY_Scl;
	float rawZ = (Accel_Z_RAW - AccZ_Off) * AccZ_Scl;
	
	mpu_data.Ax = rawX  / 2048.0 ;
	mpu_data.Ay = rawY / 2048.0 ;
	mpu_data.Az = rawZ  / 2048.0 ;
	
	//计算合加速度
	mpu_data.svm = sqrt(mpu_data.Ax *mpu_data.Ax + mpu_data.Ay *mpu_data.Ay + mpu_data.Az *mpu_data.Az);
	if(mpu_data.svm > 5)
	{
		MPU6050_Init();
	}
}

void MPU6050_Read_Gyro(void)//读取角速度 （单位：/s,量程：+-250/s）
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	mpu_data.Gx = Gyro_X_RAW / 16.4 -GyroX_Off;
	mpu_data.Gy = Gyro_Y_RAW / 16.4 - GyroY_Off;
	mpu_data.Gz = Gyro_Z_RAW / 16.4 + GyroZ_Off;
	
	//计算合角速度
	mpu_data.gyr = sqrt(mpu_data.Gx *mpu_data.Gx + mpu_data.Gy *mpu_data.Gy + mpu_data.Gz *mpu_data.Gz);

}


//进入低功耗并且开启中断,我都硬件有问题，后续修改代码
void MPU6050_EnterLowPowerMode(void)
{
	uint8_t Data;

		//进入Cycle模式，Cycle=1，SLeep=0；
		Data = 0x20;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
		HAL_Delay(10);
	
		//配置低功耗采样率（0.5Hz，省电）
		Data = 0x02;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_2_REG, 1, &Data, 1, 1000);


		//设置运动检测阈值
		Data = 40;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x1F, 1, &Data, 1, 1000);

		//设置运动检测持续时间（约250ms）
		Data = 0x40;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x20, 1, &Data, 1, 1000);

		//配置中断引脚,推挽输出,高电平有效
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x37, 1, &Data, 1, 1000);
		
		//使能运动中断
		Data = 0x40;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x38, 1, &Data, 1, 1000);
//		HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x38, 1, &Data, 1, 1000);
//		printf("0x%02X\r\n",Data);

		//使能运动检测
		Data = 0x01;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x69, 1, &Data, 1, 1000);

		uint8_t Int;
		HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x3A, 1, &Int, 1, 1000);
}

//退出低功耗并且关闭中断
void MPU6050_ExitLowPowerMode(void)
{
	uint8_t Data = 0x01;

		//唤醒MPU6050
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
		HAL_Delay(50);//等待上电稳定
	
		//唤醒MPU6050
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_2_REG, 1, &Data, 1, 1000);
		HAL_Delay(50);
	
		//关闭中断
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x38, 1, &Data, 1, 1000);

		uint8_t Int;
		HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x3A, 1, &Int, 1, 1000);
		printf("0x%02X\r\n",Int);
	
		//重新初始化MPU6050
		MPU6050_Init();
}

/* ========== 姿态角自适应标定（任意佩戴方向） ========== */
static float G_base[3] = {0.0f, 0.0f, 1.0f};   //站立基准向量
static uint8_t g_calibrated = 0;               //是否已标定
static float   g_sumX = 0, g_sumY = 0, g_sumZ = 0;
static uint16_t g_sumCnt = 0;

//累加当前帧到基准和（标定采样阶段每帧调用）
void Tilt_AccumulateBase(void)
{
    g_sumX += mpu_data.Ax;
    g_sumY += mpu_data.Ay;
    g_sumZ += mpu_data.Az;
    g_sumCnt++;
}

//用累加平均值写入站立基准（姿态角自动清零），标定完成
void Tilt_FinishBase(void)
{
    if(g_sumCnt == 0) return;
    float nx = g_sumX / g_sumCnt;
    float ny = g_sumY / g_sumCnt;
    float nz = g_sumZ / g_sumCnt;
    float n  = sqrtf(nx*nx + ny*ny + nz*nz);
    if(n < 0.1f) return;
    G_base[0] = nx/n;
    G_base[1] = ny/n;
    G_base[2] = nz/n;
    g_calibrated = 1;
    g_sumCnt = 0;
}

//查询是否已标定
uint8_t Tilt_IsCalibrated(void)
{
    return g_calibrated;
}

//姿态角：相对站立基准的夹角 0~180°，站立≈0°、蹲/坐≈0°、躺倒≈90°
float MPU6050_Get_TiltAngle(void)
{
    if(!g_calibrated) return 0.0f;   //未标定返回0，不会误触发
    float n = sqrtf(mpu_data.Ax*mpu_data.Ax + mpu_data.Ay*mpu_data.Ay + mpu_data.Az*mpu_data.Az);
    if(n < 0.5f) return 0.0f;        //运动/冲击瞬间合加速度≠1g，夹角不可靠，跳过
    float cos_a = (mpu_data.Ax*G_base[0] + mpu_data.Ay*G_base[1] + mpu_data.Az*G_base[2]) / n;
    if(cos_a > 1.0f)  cos_a = 1.0f;
    if(cos_a < -1.0f) cos_a = -1.0f;
    return acosf(cos_a) / 3.14159f * 180.0f;
}









