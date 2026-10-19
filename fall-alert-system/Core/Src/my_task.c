#include "my_task.h"
#include "fall_detect.h"
#include "OLED.h"
#include "mpu6050.h"


volatile float g_display_svm = 0.0f;
volatile float g_display_gyr = 0.0f;
volatile float g_display_angle = 0.0f; //姿态角(°)，用于OLED显示与调试
volatile uint8_t g_display_fall_status = 0; //0=正常，1=摔倒
volatile uint8_t g_display_system_redy = 0; //


//传感器数据采集任务,采集频率50Hz
void Sensor_task(void)
{
	uint32_t idle_counter = 0;
	static uint8_t  calib_phase = 0;      //0=等待静止,1=采样中,2=完成
	static uint32_t calib_timer = 0;
	const uint32_t IDLE_THRESHOLD = 1500;//30秒(20ms * 1500 = 30)

	SensorData_t queue_data;
	
	TickType_t last_wake_time = xTaskGetTickCount();
	MPU6050_Init();
	for(;;)
	{ 
		MPU6050_Read_Accel();
		MPU6050_Read_Gyro();
		//=====姿态角自适应标定：等待静止→采样平均→写入基准=====
		if(calib_phase != 2)
		{
			if(calib_phase == 0)
			{
				if(mpu_data.svm > 0.95f && mpu_data.svm < 1.05f)
				{
					if(calib_timer == 0) calib_timer = xTaskGetTickCount() * portTICK_PERIOD_MS;
					if(xTaskGetTickCount() * portTICK_PERIOD_MS - calib_timer >= 2000)
					{
						calib_phase = 1;
						calib_timer = xTaskGetTickCount() * portTICK_PERIOD_MS;
					}
				}
				else calib_timer = 0;
			}
			else if(calib_phase == 1)
			{
				Tilt_AccumulateBase();
				if(xTaskGetTickCount() * portTICK_PERIOD_MS - calib_timer >= 2000)
				{
					Tilt_FinishBase();
					calib_phase = 2;
					printf("calib ok\r\n");
				}
			}
		}

		queue_data.svm = mpu_data.svm;
		queue_data.gyr = mpu_data.gyr;
		queue_data.angle = MPU6050_Get_TiltAngle();
		if(xQueueSend(sensorQueueHandle,&queue_data,pdMS_TO_TICKS(10)) != pdPASS)
		{
			printf("发送队列失败\r\n");
		}
		if(mpu_data.svm > 0.97 && mpu_data.svm < 1.03)
		{
			idle_counter++;
			if(idle_counter > IDLE_THRESHOLD)
			{
				idle_counter = 0;
				printf("进入低功耗\r\n");
				System_EnterStopMode();
			}
		}
		else
		{
			idle_counter = 0;
		}
		vTaskDelayUntil(&last_wake_time,pdMS_TO_TICKS(20));
	}
}

//摔倒检测核心算法
void Detect_task(void)
{
	SensorData_t data;    				//读取队列数据结构体
	FallDetect_Init();
	for(;;)
	{
		if(xQueueReceive(sensorQueueHandle,&data,portMAX_DELAY) != pdPASS)
		{
			printf("读取队列失败\r\n");
		}
		g_display_svm = data.svm;
		g_display_gyr = data.gyr;
		  
		g_display_fall_status = FallDetect_Updata(data.svm, data.gyr, data.angle);
		g_display_angle = data.angle;
		if(g_display_fall_status  == 1)
		{
			//报警
			if(CommTaskHandle != NULL)
			{
				xTaskNotifyGive(CommTaskHandle);
			}
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

//通讯任务（需要唤醒）
void Comm_task(void)
{
	char * fall_cmd = "FALL\r\n";
	for(;;)
	{
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		HAL_UART_Transmit(&huart1,(uint8_t *)fall_cmd,strlen(fall_cmd),100);
		vTaskDelay(1000);	
	}
}
//显示任务
void OLED_task(void)
{
	uint8_t ID;
	OLED_Init();
  	OLED_ShowString(1, 1, "ID:");
	OLED_ShowString(4,1,"STATUS:");
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &ID, 1, 1000);
	OLED_ShowHexNum(1, 4, ID, 2);

	for(;;)
	{    
		if(OLED_CheckAlive() == 1)
		{
			OLED_Init();
			OLED_ShowString(1, 1, "ID:");
			OLED_ShowString(4,1,"STATUS:");
			HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &ID, 1, 1000);
			OLED_ShowHexNum(1, 4, ID, 2);
			printf("OLED设备已重新连接\r\n");
		}
		OLED_ShowFloatNum(2,1,g_display_svm,2,2);
		OLED_ShowFloatNum(2,8,g_display_gyr,3,1);
		OLED_ShowNum(3,1,(uint32_t)g_display_angle,3);

		if(!Tilt_IsCalibrated())
		{
			OLED_ShowString(4,1,"STAND!  ");
			OLED_ShowString(4,8,"       ");
		}
		else
		{
			OLED_ShowString(4,1,"STATUS:");
			if(g_display_fall_status == 1)
			{
				OLED_ShowString(4,8,"SFALL!");
			}
			else
			{
				OLED_ShowString(4,8,"OK    ");
			}
		}
	vTaskDelay(100);
  }
}

