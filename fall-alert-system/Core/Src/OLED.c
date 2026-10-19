#include "gpio.h"
#include "i2c.h"
#include "OLED_Font.h"

/* 硬件I2C句柄，CubeMX生成 hi2c1 */
#define OLED_HI2C        hi2c1
#define OLED_ADDR_7BIT   0x3C    //7位I2C地址

/**
  * @brief  OLED写命令
  * @param  Command 要写入的命令
  * @retval HAL状态，0成功，非0通信错误
  */
HAL_StatusTypeDef OLED_WriteCommand(uint8_t Command)
{
    uint8_t buf[2];
    buf[0] = 0x00;   //00=命令
    buf[1] = Command;
    return HAL_I2C_Mem_Write(&OLED_HI2C, OLED_ADDR_7BIT<<1, 0x00, I2C_MEMADD_SIZE_8BIT, buf+1, 1, 10);
}

/**
  * @brief  OLED写数据
  * @param  Data 要写入的数据
  * @retval HAL状态
  */
HAL_StatusTypeDef OLED_WriteData(uint8_t Data)
{
    uint8_t buf[2];
    buf[0] = 0x40;   //0x40=显存数据
    buf[1] = Data;
    return HAL_I2C_Mem_Write(&OLED_HI2C, OLED_ADDR_7BIT<<1, 0x40, I2C_MEMADD_SIZE_8BIT, buf+1, 1, 10);
}

/**
  * @brief  OLED设置光标位置
  * @param  Y 页地址0~7
  * @param  X X坐标0~127
  * @retval 无
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));
    OLED_WriteCommand(0x00 | (X & 0x0F));
}

/**
  * @brief  OLED清屏
  * @param  无
  * @retval 无
  */
void OLED_Clear(void)
{
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		for(i = 0; i < 128; i++)
		{
			OLED_WriteData(0x00);
		}
	}
}

/**
  * @brief  OLED显示一个字符 8x16字体
  * @param  Line 行位置1~4
  * @param  Column 列位置1~16
  * @param  Char ASCII字符
  * @retval 无
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{      	
	uint8_t i;
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i]);
	}
	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
	}
}

void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}

uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}

void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

/**
  * @brief  OLED显示浮点数(带小数，四舍五入)
  * @param  Line		行号 1~4
  * @param  Column	列号 1~16
  * @param  Number	要显示的浮点数
  * @param  IntLength	整数位数
  * @param  FraLength	小数位数
  * @retval 无
  */
void OLED_ShowFloatNum(uint8_t Line, uint8_t Column, double Number, uint8_t IntLength, uint8_t FraLength)
{
	uint32_t PowNum, IntNum, FraNum;
	
	if (Number >= 0)						
	{
		OLED_ShowChar(Line, Column, '+');	//显示+号
	}
	else									
	{
		OLED_ShowChar(Line, Column, '-');	//显示‑号
		Number = -Number;					
	}

	/*提取整数、小数，四舍五入处理*/
	IntNum = (uint32_t)Number;						
	Number -= IntNum;						
	PowNum = OLED_Pow(10, FraLength);		
	FraNum = round(Number * PowNum);		
	IntNum += FraNum / PowNum;				//处理小数进位

	/*整数部分*/
	OLED_ShowNum(Line , Column + 1, IntNum, IntLength);
	
	/*小数点*/
	OLED_ShowChar(Line , Column + 1 + IntLength, '.');
	
	/*小数部分*/
	OLED_ShowNum(Line , Column + 1 + IntLength + 1, FraNum, FraLength);
}


/**
  * @brief  OLED硬件I2C初始化
  * @param  无
  * @retval 无
  */
void OLED_Init(void)
{
	HAL_Delay(20);
	
	OLED_WriteCommand(0xAE);	//关闭显示
	
	OLED_WriteCommand(0xD5);
	OLED_WriteCommand(0x80);
	
	OLED_WriteCommand(0xA8);
	OLED_WriteCommand(0x3F);
	
	OLED_WriteCommand(0xD3);
	OLED_WriteCommand(0x00);
	
	OLED_WriteCommand(0x40);
	
	OLED_WriteCommand(0xA1);
	
	OLED_WriteCommand(0xC8);
	OLED_WriteCommand(0xDA);
	OLED_WriteCommand(0x12);
	
	OLED_WriteCommand(0x81);
	OLED_WriteCommand(0xCF);
	OLED_WriteCommand(0xD9);
	OLED_WriteCommand(0xF1);
	OLED_WriteCommand(0xDB);
	OLED_WriteCommand(0x30);
	OLED_WriteCommand(0xA4);
	OLED_WriteCommand(0xA6);
	OLED_WriteCommand(0x8D);
	OLED_WriteCommand(0x14);
	OLED_WriteCommand(0xAF);
		
	OLED_Clear();
}

/**
 * @brief 探测OLED是否应答（硬件I2C版本）
 * @retval 0:在线；1:无应答
 */
uint8_t OLED_CheckAlive(void)
{
    if(HAL_I2C_IsDeviceReady(&OLED_HI2C, OLED_ADDR_7BIT<<1, 1, 10) == HAL_OK)
    {
        return 0;
    }
    return 1;
}
