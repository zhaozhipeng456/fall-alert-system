#ifndef __MY_TASK__H_
#define __MY_TASK__H_

#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

void Sensor_task(void);
void Detect_task(void);
void Comm_task(void);
void OLED_task(void);

//队列传输结构体
typedef struct {
	float svm;					//合加速度
	float gyr;					//合角速度
	float angle;					//姿态角(°)
}SensorData_t;

extern TaskHandle_t SensorTaskHandle;
extern TaskHandle_t DetectTaskHandle;
extern TaskHandle_t CommTaskHandle;
extern TaskHandle_t OLEDTaskHandle;
extern QueueHandle_t sensorQueueHandle;


#endif
