#include "gpio.h"
#include "bsp_delay.h"


void MyI2C_W_SCL(GPIO_TypeDef * GPIO_Port, uint16_t GPIO_Pin, uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, (GPIO_PinState)BitValue);		
  delay_us(1);														

}
void MyI2C_W_SDA(GPIO_TypeDef * GPIO_Port, uint16_t GPIO_Pin, uint8_t BitValue)
{
	HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, (GPIO_PinState)BitValue);		
  delay_us(1);														

}
uint8_t MyI2C_R_SDA(GPIO_TypeDef * GPIO_Port, uint16_t GPIO_Pin)
{
	uint8_t BitValue;
	BitValue = HAL_GPIO_ReadPin(GPIO_Port, GPIO_Pin);		//读取SDA电平
	return BitValue;											//返回SDA电平
}


void MyI2C_Start(GPIO_TypeDef * SCL_GPIO_Port, uint16_t SCL_GPIO_Pin, GPIO_TypeDef * SDA_GPIO_Port, uint16_t SDA_GPIO_Pin)
{
	MyI2C_W_SDA(SDA_GPIO_Port, SDA_GPIO_Pin, 1);							
	MyI2C_W_SCL(SCL_GPIO_Port, SCL_GPIO_Pin, 1);							
	MyI2C_W_SDA(SDA_GPIO_Port, SDA_GPIO_Pin, 0);							
	MyI2C_W_SCL(SCL_GPIO_Port, SCL_GPIO_Pin, 0);							
}


void MyI2C_Stop(GPIO_TypeDef * SCL_GPIO_Port, uint16_t SCL_GPIO_Pin, GPIO_TypeDef * SDA_GPIO_Port, uint16_t SDA_GPIO_Pin)
{
	MyI2C_W_SDA(SDA_GPIO_Port, SDA_GPIO_Pin, 0);						
	MyI2C_W_SCL(SCL_GPIO_Port, SCL_GPIO_Pin, 1);						
	MyI2C_W_SDA(SDA_GPIO_Port, SDA_GPIO_Pin, 1);						
}


void MyI2C_SendByte(GPIO_TypeDef * SCL_GPIO_Port, uint16_t SCL_GPIO_Pin, GPIO_TypeDef * SDA_GPIO_Port, uint16_t SDA_GPIO_Pin, uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)			
	{

		MyI2C_W_SDA(SDA_GPIO_Port, SDA_GPIO_Pin, !!(Byte & (0x80 >> i)));
		MyI2C_W_SCL(SCL_GPIO_Port, SCL_GPIO_Pin, 1);					
		MyI2C_W_SCL(SCL_GPIO_Port, SCL_GPIO_Pin, 0);					
	}
}


uint8_t MyI2C_ReceiveByte(GPIO_TypeDef * SCL_GPIO_Port, uint16_t SCL_GPIO_Pin, GPIO_TypeDef * SDA_GPIO_Port, uint16_t SDA_GPIO_Pin)
{
	uint8_t i, Byte = 0x00;					
	MyI2C_W_SDA(SDA_GPIO_Port, SDA_GPIO_Pin, 1);							
	for (i = 0; i < 8; i ++)				
	{
		MyI2C_W_SCL(SCL_GPIO_Port, SCL_GPIO_Pin, 1);					
		if (MyI2C_R_SDA(SDA_GPIO_Port, SDA_GPIO_Pin)){Byte |= (0x80 >> i);}
												
		MyI2C_W_SCL(SCL_GPIO_Port, SCL_GPIO_Pin, 0);					
	}
	return Byte;							//返回接收到的一个字节数据
}


void MyI2C_SendAck(GPIO_TypeDef * SCL_GPIO_Port, uint16_t SCL_GPIO_Pin, GPIO_TypeDef * SDA_GPIO_Port, uint16_t SDA_GPIO_Pin, uint8_t AckBit)
{
	MyI2C_W_SDA(SDA_GPIO_Port, SDA_GPIO_Pin, AckBit);					
	MyI2C_W_SCL(SCL_GPIO_Port, SCL_GPIO_Pin, 1);							
	MyI2C_W_SCL(SCL_GPIO_Port, SCL_GPIO_Pin, 0);							
}


uint8_t MyI2C_ReceiveAck(GPIO_TypeDef * SCL_GPIO_Port, uint16_t SCL_GPIO_Pin, GPIO_TypeDef * SDA_GPIO_Port, uint16_t SDA_GPIO_Pin)
{
	uint8_t AckBit;			
	MyI2C_W_SDA(SDA_GPIO_Port, SDA_GPIO_Pin, 1);							
	MyI2C_W_SCL(SCL_GPIO_Port, SCL_GPIO_Pin, 1);							
	AckBit = MyI2C_R_SDA(SDA_GPIO_Port, SDA_GPIO_Pin);				
	MyI2C_W_SCL(SCL_GPIO_Port, SCL_GPIO_Pin, 0);							
	return AckBit;					
}