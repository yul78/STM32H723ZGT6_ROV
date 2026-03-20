#ifndef __GAMEPAD_H
#define __GAMEPAD_H

#include "main.h"

#define gamepad_huart   huart3

// 协议定义
#define FRAME_HEAD1     0xAA
#define FRAME_HEAD2     0x55
#define FRAME_TAIL      0x0D
#define FRAME_LENGTH    14      // 整包长度

// 手柄数据结构体
typedef struct {
    uint8_t  leftX;         // 左摇杆X  (0~255, 128居中)
    uint8_t  leftY;         // 左摇杆Y  (0~255, 128居中)
    uint8_t  rightX;        // 右摇杆X  (0~255, 128居中)
    uint8_t  rightY;        // 右摇杆Y  (0~255, 128居中)
    uint16_t buttons;       // 按键 (1=A, 2=B, 4=X, 8=Y, 16=开关左边按键, 64=开关右边按键, 128=左摇杆按下, 256=右摇杆按下, 512=LB(按键有点问题), 1024=RB)
    uint8_t  hatX;          // 十字键X  (0=左, 1=中, 2=右)
    uint8_t  hatY;          // 十字键Y  (0=上, 1=中, 2=下)
    uint8_t  lt;            // 左扳机
    uint8_t  rt;            // 右扳机
    uint8_t  isUpdated;     // 数据更新标志
} GamepadData_t;

typedef struct {
    int16_t leftX;  // 左摇杆X  (-4000~4000)
    int16_t leftY;  // 左摇杆Y  (-4000~4000)
} Analysis_GamepadData_t;

extern uint8_t gamepad_rxByte;

// 函数声明
void Gamepad_Init(UART_HandleTypeDef *huart);
void Gamepad_RxCallback(uint8_t data);
GamepadData_t* Gamepad_GetData(void);
uint8_t Gamepad_IsButtonPressed(uint16_t btn);
void GamepadData_Analysis(GamepadData_t *raw, Analysis_GamepadData_t *data);

#endif
