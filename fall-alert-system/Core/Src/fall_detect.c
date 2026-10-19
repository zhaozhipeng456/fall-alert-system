#include "fall_detect.h"

/* ========== 状态枚举 ========== */
typedef enum{
	STATE_IDIE,						//空闲检监测
	STATE_SHOCK,					//检测到冲击
	STATE_CONFIRM,					//判断是否躺平
	STATE_FALL						//确定摔倒
}FallState_t;

/* ========== 阈值配置 ========== */
#define SVM_THRESHILD    1.5f						//冲击触发阈值2.8
#define GYR_THRESHILD    80.0						//旋转触发阈值100
#define SVM_STILL_LOW    0.95						//躺平时SVM下限
#define SVM_STILL_HIGH   1.05						//躺平时SVM上限
#define CONFIRM_MS       5000						//确认时间5秒
#define ALARM_MS         5000						//报警时间5秒
#define FALL_ANGLE_THRESHOLD 60.0f					//姿态角阈值：≥60°视为躺倒

/* ========== 私有变量 ========== */
static 	FallState_t state = STATE_IDIE;
static  uint32_t state_start_time = 0;
static  uint8_t fall_flag = 0;

/* ========== 实现 ========== */
//初始化函数
void FallDetect_Init(void)
{
	state = STATE_IDIE;		//初始状态
	fall_flag = 0;
}

//核心算法函数
uint8_t FallDetect_Updata(float svm, float gyr, float angle)
{	  
	  switch(state)
		  {
			  //状态 IDIE：检测中
			  case STATE_IDIE:
				  if((svm >= SVM_THRESHILD) && (gyr >= GYR_THRESHILD))
				  {
					  state = STATE_SHOCK;
					  state_start_time =xTaskGetTickCount() * portTICK_PERIOD_MS;
					  printf("冲击触发\r\n");
				  }
				  break;
			  //状态 SHOCK：冲击触发，等待确认期
			  case STATE_SHOCK:
				  if(xTaskGetTickCount() * portTICK_PERIOD_MS - state_start_time >= CONFIRM_MS)
				  {
					  state = STATE_CONFIRM;
					  state_start_time =xTaskGetTickCount() * portTICK_PERIOD_MS;
				  }
				  else if(svm >= SVM_THRESHILD)
				  {
					  state_start_time =xTaskGetTickCount() * portTICK_PERIOD_MS;
				  }
				  break;
			  //状态 CONFIRM：判断是否真的摔倒
			  case STATE_CONFIRM:
				  if((svm >= SVM_STILL_LOW) && (svm <=SVM_STILL_HIGH) && (angle >= FALL_ANGLE_THRESHOLD))
				  {
					  state = STATE_FALL;
					  state_start_time =xTaskGetTickCount() * portTICK_PERIOD_MS;
					  fall_flag = 1;
					  printf("确认摔倒,正在报警...\r\n");
				  }
				  else
				  {
					  state = STATE_IDIE;
					  printf("没有摔倒,\r\n");
				  }
				  break;
			  //状态 FALL: 确认摔倒，报警
			  case STATE_FALL:
				  if(xTaskGetTickCount() * portTICK_PERIOD_MS - state_start_time >= ALARM_MS)
				  {
					  printf("已报警\r\n");
					  fall_flag = 0;
					  state = STATE_IDIE;
				  }
				  break;
		}
	  return fall_flag;
  }

  //状态获取函数
uint8_t FallDetect_GetState(void)
{
	return state;
}
