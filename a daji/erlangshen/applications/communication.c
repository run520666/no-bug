#include "usart.h"

#define FRAME_HEAD 0xAA
#define FRAME_TAIL 0x55
#define DATA_LENGTH 3

uint8_t rxBuffer[5];  // 帧头+长度+data+校验+帧尾 = 5字节
uint8_t rxIndex = 0;
uint8_t rxByte;
volatile uint8_t frameReceived = 0;
uint8_t recivedata;

// 发送三个数据到Arduino
//初始值都发0，改变状态时发1

void UART_SendData(UART_HandleTypeDef *huart,uint8_t dataone, uint8_t datatwo, uint8_t datathree) {
    uint8_t frame[7];
    frame[0] = FRAME_HEAD;
    frame[1] = DATA_LENGTH;              // 数据长度=3
    frame[2] = dataone;                    // 第一个数据,左边检测到障碍物
    frame[3] = datatwo;                    // 第二个数据,右边检测到障碍物
	frame[4] = datathree;                    // 第三个数据,小车停稳的信号
    frame[5] = FRAME_HEAD + DATA_LENGTH + dataone + datatwo + datathree;  // 校验
    frame[6] = FRAME_TAIL;
    
    HAL_UART_Transmit(huart, frame, 7, 100);
}

// 处理接收到的数据帧
void processReceivedData(void) {
    if (frameReceived) {
        // 校验帧头和帧尾
        if (rxBuffer[0] == FRAME_HEAD && rxBuffer[4] == FRAME_TAIL) {
            uint8_t length = rxBuffer[1];
            uint8_t data = rxBuffer[2];
            uint8_t checksum = rxBuffer[3];
            
              // 计算校验和
            uint8_t calc_checksum = FRAME_HEAD + length + data ;
            
            // 校验数据
            if (checksum == calc_checksum && length == DATA_LENGTH) {
                // 收到Arduino的数据data
				recivedata = data ;	
            }
        }
        
        
        // 重置接收状态
        rxIndex = 0;
        frameReceived = 0;
    }
}
