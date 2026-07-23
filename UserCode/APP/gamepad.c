#include "gamepad.h"
#include <string.h>

/*
字节:  0     1     2    3     4    5     6      7       8       9     10    11    12    13
内容: 0xAA  0x55   LX   LY    RX   RY   BTN_H  BTN_L   HAT_X  HAT_Y   LT    RT    CHK   0x0D
      帧头  帧头  左X   左Y   右X   右Y  按键高  按键低  方向X   方向Y  左扳机 右扳机 校验   帧尾

      Y(8)
X(4) BTN_L B(2)
      A(1)
      
      上(0)
L(0) HAT_X R(2)
      下(2)
      
      上(0)
L(0) HAT_Y R(2)
      下(2)
*/

// 接收状态机
typedef enum {
    STATE_WAIT_HEAD1 = 0,
    STATE_WAIT_HEAD2,
    STATE_RECV_DATA,
} RxState_t;

// 私有变量
static GamepadData_t gamepadData = {128, 128, 128, 128, 0, 1, 1, 0};
static uint8_t rxBuffer[FRAME_LENGTH];
static uint8_t rxIndex = 0;
static RxState_t rxState = STATE_WAIT_HEAD1;
static UART_HandleTypeDef *gamepadUart;
uint8_t gamepad_rxByte;


/**
 * @brief 初始化手柄接收
 */
void Gamepad_Init(UART_HandleTypeDef *huart)
{
    gamepadUart = huart;
    // 开启串口中断接收（每次接收1个字节）
    HAL_UART_Receive_IT(gamepadUart, &gamepad_rxByte, 1);
}

/**
 * @brief 解析一帧完整数据
 */
static void Gamepad_ParseFrame(uint8_t *frame)
{
    // 校验和验证
    uint8_t checksum = 0;
    for (int i = 2; i <= 11; i++) {
        checksum += frame[i];
    }
    checksum &= 0xFF;

    if (checksum != frame[12]) {
        return;  // 校验失败，丢弃
    }

    // 解析数据
    gamepadData.leftX   = frame[2];
    gamepadData.leftY   = frame[3];
    gamepadData.rightX  = frame[4];
    gamepadData.rightY  = frame[5];
    gamepadData.buttons = (frame[6] << 8) | frame[7];
    gamepadData.hatX    = frame[8];
    gamepadData.hatY    = frame[9];
    gamepadData.lt      = frame[10];
    gamepadData.rt      = frame[11];
    gamepadData.isUpdated = 1;  // 设置更新标志
}

/**
 * @brief 串口接收回调
 */
void Gamepad_RxCallback(uint8_t data)
{

    switch (rxState) {
        case STATE_WAIT_HEAD1:
            if (data == FRAME_HEAD1) {
                rxBuffer[0] = data;
                rxIndex = 1;
                rxState = STATE_WAIT_HEAD2;
            }
            break;
        case STATE_WAIT_HEAD2:
            if (data == FRAME_HEAD2) {
                rxBuffer[1] = data;
                rxIndex = 2;
                rxState = STATE_RECV_DATA;
            } else {
                rxState = STATE_WAIT_HEAD1;  // 重新等待
            }
            break;
        case STATE_RECV_DATA:
            rxBuffer[rxIndex++] = data;
            if (rxIndex >= FRAME_LENGTH) {
                // 检查帧尾
                if (rxBuffer[FRAME_LENGTH - 1] == FRAME_TAIL) {
                    Gamepad_ParseFrame(rxBuffer);
                }
                rxState = STATE_WAIT_HEAD1;  // 重置状态机
                rxIndex = 0;
            }
            break;
        default:
            rxState = STATE_WAIT_HEAD1;
            break;
    }
}

/**
 * @brief 获取手柄数据指针
 */
GamepadData_t* Gamepad_GetData(void)
{
    return &gamepadData;
}

/**
 * @brief 检查某个按键是否按下
 */
uint8_t Gamepad_IsButtonPressed(uint16_t btn)
{
    return (gamepadData.buttons & btn) ? 1 : 0;
}

/**
 * @brief 解析原始数据
 */
void GamepadData_Analysis(GamepadData_t *raw, Analysis_GamepadData_t *data)
{
    #define JOY_DEADBAND 8   // 原始值死区，对应约190RPM

    int16_t lx = (int16_t)raw->leftX  - 128;
    int16_t ry = (int16_t)raw->rightY - 128;

    // 死区处理：中点附近输出0，保证过零时能触发停机
    if (lx > -JOY_DEADBAND && lx < JOY_DEADBAND) lx = 0;
    if (ry > -JOY_DEADBAND && ry < JOY_DEADBAND) ry = 0;

    data->leftX  = lx * 3000 / 127;
    data->rightY = ry * 3000 / 127;
    // rightX leftY 同理处理
    data->rightX = ((int16_t)raw->rightX - 128) * 3000 / 127;
    data->leftY  = ((int16_t)raw->leftY  - 128) * 3000 / 127;
}