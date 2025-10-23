#include "hwt101.h"
#include "mecanum_control.h"
#include "usart.h"
static uint8_t RxBuffer[11]; /* 接收缓冲区 */
static volatile uint8_t RxState = 0; /*接收状态标志位*/
static uint8_t RxIndex = 0; /*接受数组索引*/
float yaw; //偏航角度

void hwt101_ReceiveData(uint8_t RxData)
{ 
    uint8_t i,sum=0;
    
    if (RxState == 0)	//等待包头
    {
        if (RxData == 0x55)	//收到包头
        {
            RxBuffer[RxIndex] = RxData;
            RxState = 1;
            RxIndex = 1; //进入下一状态
        }
    }
    
    else if (RxState == 1)
    {
        if (RxData == 0x53)	/*判断数据内容，修改这里可以改变要读的数据内容，0x53为角度输出*/
        {
            RxBuffer[RxIndex] = RxData;
            RxState = 2;
            RxIndex = 2; //进入下一状态
        }
    }
    
    else if (RxState == 2)	//接收数据
    {
        RxBuffer[RxIndex++] = RxData;
        if(RxIndex == 11)	//接收完成
        {
            for(i=0;i<10;i++)
            {
                sum = sum + RxBuffer[i]; //计算校验和
            }
            if(sum == RxBuffer[10])		//校验成功
            {
                /*计算数据，根据数据内容选择对应的计算公式*/
 //              g_roll_jy61 = ((uint16_t) ((uint16_t) RxBuffer[3] << 8 | (uint16_t) RxBuffer[2])) / 32768.0f * 180.0f; //hwt001无
 //              g_pitch_jy61 = ((uint16_t) ((uint16_t) RxBuffer[5] << 8 | (uint16_t) RxBuffer[4])) / 32768.0f * 180.0f; //hwt001无
                yaw= ((uint16_t) ((uint16_t) RxBuffer[7] << 8 | (uint16_t) RxBuffer[6])) / 32768.0f * 180.0f;  //套公式，算出来的值是0~360
                set_yaw(yaw); //转换yaw为-180~180,并且传入麦轮结构体
            }
            RxState = 0;
            RxIndex = 0; //读取完成，回到最初状态，等待包头
        }
    }

}
 //转换yaw为-180~180,并且传入麦轮结构体 正值逆时针 负值顺时针
void set_yaw(float yaw)
{

	if (yaw>=180.0f)
	{
		yaw = yaw-360.0f;
	}
	else
	{
		yaw = yaw;
	}
    mecanum.current_pos.yaw = yaw;
}

void hwt101_restart(UART_HandleTypeDef *huart)
{
    uint8_t unlock_cmd[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};  // 解锁命令（解锁key寄存器）
    uint8_t restart_cmd[5] = {0xFF, 0xAA, 0x00, 0xFF, 0x00}; // 重启命令
    
    // 发送解锁命令
    HAL_UART_Transmit(huart, unlock_cmd, 5, 100);
    HAL_Delay(50);  // 延时处理
    
    // 发送重启命令（不需要保存）
    HAL_UART_Transmit(huart, restart_cmd, 5, 100);
    HAL_Delay(1000); // 重启后等待设备完成重启过程
}

