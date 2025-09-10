#include "hwt101.h"
#include "mecanum_control.h"
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
                yaw= ((uint16_t) ((uint16_t) RxBuffer[7] << 8 | (uint16_t) RxBuffer[6])) / 32768.0f * 180.0f;
                set_yaw(yaw);
            }
            RxState = 0;
            RxIndex = 0; //读取完成，回到最初状态，等待包头
        }
    }

}

//设置yaw为0~180 和 -180~0,并且传入麦轮结构体
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

