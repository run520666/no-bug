#include "stp_23l.h"
#include "mecanum_control.h"

extern mecanum_control_t mecanum;

uint16_t stp23l_distance = 0;  /* 全局距离变量 */

static uint8_t rx_state = 0;
static uint8_t rx_crc = 0;
static uint8_t rx_cnt = 0;
static uint8_t pack_flag = 0;
static uint16_t data_len = 0;
static uint8_t state_flag = 1;
static LidarPointTypedef pack_data[12];

// 初始化STP23L
void STP23L_Init(UART_HandleTypeDef *huart)
{
    rx_state = 0;
    rx_crc = 0;
    rx_cnt = 0;
    stp23l_distance = 0;
}

static void data_process(void)
{
    uint8_t i;
    uint16_t count = 0;
    uint32_t sum = 0;
    
    /* 12个点取平均 */
    for(i = 0; i < 12; i++)
    {
        if(pack_data[i].distance != 0)  /* 去除0的点 */
        {
            count++;
            sum += pack_data[i].distance;
        }
    }
    
    if(count != 0)
    {
        stp23l_distance = sum / count;
        /* 更新到 mecanum 结构体 */
        mecanum.stp_distance_l = stp23l_distance;
    }
}

void STP23L_RxCallback(uint8_t temp_data)
{
    if(rx_state < 4)  /* 起始符验证 */
    {
        if(temp_data == HEADER) rx_state++;
        else rx_state = 0;
    }
    else if(rx_state < 10 && rx_state > 3)
    {
        switch(rx_state)
        {
            case 4:   
                if(temp_data == DEVICE_ADDRESS)
                {
                    rx_state++;
                    rx_crc = temp_data;
                }
                else rx_state = 0, rx_crc = 0;
                break;
                
            case 5:   
                if(temp_data == PACK_GET_DISTANCE)
                {
                    pack_flag = PACK_GET_DISTANCE;
                    rx_state++;
                    rx_crc += temp_data;
                }
                else rx_state = 0, rx_crc = 0;
                break;
                
            case 6:
            case 7:
                if(temp_data == CHUNK_OFFSET)
                {
                    rx_state++;
                    rx_crc += temp_data;
                }
                else rx_state = 0, rx_crc = 0;
                break;
                
            case 8:
                data_len = (uint16_t)temp_data;
                rx_state++;
                rx_crc += temp_data;
                break;
                
            case 9:
                data_len = data_len + ((uint16_t)temp_data << 8);
                rx_state++;
                rx_crc += temp_data;
                break;
                
            default: break;
        }
    }
    else if(rx_state == 10) state_flag = 0;
    
    if(pack_flag == PACK_GET_DISTANCE && state_flag == 0)
    {
        if(rx_state > 9)
        {
            if(rx_state < 190)
            {
                uint8_t state_num = (rx_state - 10) % 15;
                
                switch(state_num)
                {
                    case 0:
                        pack_data[rx_cnt].distance = (uint16_t)temp_data;
                        rx_crc += temp_data;
                        rx_state++;
                        break;
                    case 1:
                        pack_data[rx_cnt].distance = ((uint16_t)temp_data << 8) + pack_data[rx_cnt].distance;
                        rx_crc += temp_data;
                        rx_state++;
                        break;
                    case 2:
                        pack_data[rx_cnt].noise = (uint16_t)temp_data;
                        rx_crc += temp_data;
                        rx_state++;
                        break;
                    case 3:
                        pack_data[rx_cnt].noise = ((uint16_t)temp_data << 8) + pack_data[rx_cnt].noise;
                        rx_crc += temp_data;
                        rx_state++;
                        break;
                    case 4:
                        pack_data[rx_cnt].peak = (uint32_t)temp_data;
                        rx_crc += temp_data;
                        rx_state++;
                        break;
                    case 5:
                    case 6:
                    case 7:
                        pack_data[rx_cnt].peak = ((uint32_t)temp_data << (8 * (state_num - 4))) + pack_data[rx_cnt].peak;
                        rx_crc += temp_data;
                        rx_state++;
                        break;
                    case 8:
                        pack_data[rx_cnt].confidence = temp_data;
                        rx_crc += temp_data;
                        rx_state++;
                        break;
                    case 9:
                    case 10:
                    case 11:
                    case 12:
                        pack_data[rx_cnt].intg = ((uint32_t)temp_data << (8 * (state_num - 9))) + pack_data[rx_cnt].intg;
                        rx_crc += temp_data;
                        rx_state++;
                        break;
                    case 13:
                        pack_data[rx_cnt].reftof = (int16_t)temp_data;
                        rx_crc += temp_data;
                        rx_state++;
                        break;
                    case 14:
                        pack_data[rx_cnt].reftof = ((int16_t)temp_data << 8) + pack_data[rx_cnt].reftof;
                        rx_crc += temp_data;
                        rx_state++;
                        rx_cnt++;
                        break;
                    default: break;
                }
            }
            
            /* 跳过时间戳 191-194 */
            if(rx_state >= 191 && rx_state <= 194)
            {
                rx_crc += temp_data;
                rx_state++;
            }
            else if(rx_state == 195)
            {
                if(temp_data == rx_crc)  /* 校验成功 */
                {
                    data_process();  /* 处理数据 */
                }
                
                /* 复位状态 */
                rx_crc = 0;
                rx_state = 0;
                state_flag = 1;
                rx_cnt = 0;
            }
            
            if(rx_state == 190) rx_state++;
        }
    }
}



