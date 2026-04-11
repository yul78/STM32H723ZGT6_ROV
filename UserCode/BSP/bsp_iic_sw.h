#ifndef __BSP_IIC_SW_H
#define __BSP_IIC_SW_H

uint8_t MyI2C_ReceiveAck(GPIO_TypeDef * SCL_GPIO_Port, uint16_t SCL_GPIO_Pin, GPIO_TypeDef * SDA_GPIO_Port, uint16_t SDA_GPIO_Pin);
void MyI2C_SendAck(GPIO_TypeDef * SCL_GPIO_Port, uint16_t SCL_GPIO_Pin, GPIO_TypeDef * SDA_GPIO_Port, uint16_t SDA_GPIO_Pin, uint8_t AckBit);
uint8_t MyI2C_ReceiveByte(GPIO_TypeDef * SCL_GPIO_Port, uint16_t SCL_GPIO_Pin, GPIO_TypeDef * SDA_GPIO_Port, uint16_t SDA_GPIO_Pin);
void MyI2C_SendByte(GPIO_TypeDef * SCL_GPIO_Port, uint16_t SCL_GPIO_Pin, GPIO_TypeDef * SDA_GPIO_Port, uint16_t SDA_GPIO_Pin, uint8_t Byte);
void MyI2C_Stop(GPIO_TypeDef * SCL_GPIO_Port, uint16_t SCL_GPIO_Pin, GPIO_TypeDef * SDA_GPIO_Port, uint16_t SDA_GPIO_Pin);
void MyI2C_Start(GPIO_TypeDef * SCL_GPIO_Port, uint16_t SCL_GPIO_Pin, GPIO_TypeDef * SDA_GPIO_Port, uint16_t SDA_GPIO_Pin);
void MyI2C_W_SCL(GPIO_TypeDef * GPIO_Port, uint16_t GPIO_Pin, uint8_t BitValue);
void MyI2C_W_SDA(GPIO_TypeDef * GPIO_Port, uint16_t GPIO_Pin, uint8_t BitValue);
uint8_t MyI2C_R_SDA(GPIO_TypeDef * GPIO_Port, uint16_t GPIO_Pin);

#endif