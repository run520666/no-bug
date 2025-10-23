#include "stp_23l.h"
#include "mecanum_control.h"

extern mecanum_control_t mecanum;

// 左激光全局变量
uint16_t stp23l_distance_left = 0;
static uint8_t rx_state_left = 0;
static uint8_t rx_crc_left = 0;
static uint8_t rx_cnt_left = 0;
static uint8_t pack_flag_left = 0;
static uint16_t data_len_left = 0;
static uint8_t state_flag_left = 1;
static LidarPointTypedef pack_data_left[12];

// 右激光全局变量
uint16_t stp23l_distance_right = 0;
static uint8_t rx_state_right = 0;
static uint8_t rx_crc_right = 0;
static uint8_t rx_cnt_right = 0;
static uint8_t pack_flag_right = 0;
static uint16_t data_len_right = 0;
static uint8_t state_flag_right = 1;
static LidarPointTypedef pack_data_right[12];

// 统一初始化函数
void STP23L_Init(UART_HandleTypeDef *huart, uint8_t lr)
{
    if(lr == 0) // 左激光
    {
        rx_state_left = 0;
        rx_crc_left = 0;
        rx_cnt_left = 0;
        stp23l_distance_left = 0;
    }
    else // 右激光
    {
        rx_state_right = 0;
        rx_crc_right = 0;
        rx_cnt_right = 0;
        stp23l_distance_right = 0;
    }
}

// 左激光数据处理
static void data_process_left(void)
{
    uint8_t i;
    uint16_t count = 0;
    uint32_t sum = 0;
    
    for(i = 0; i < 12; i++)
    {
        if(pack_data_left[i].distance != 0)
        {
            count++;
            sum += pack_data_left[i].distance;
        }
    }
    
    if(count != 0)
    {
        stp23l_distance_left = sum / count;
        mecanum.stp_distance_l = stp23l_distance_left;
    }
}

// 右激光数据处理
static void data_process_right(void)
{
    uint8_t i;
    uint16_t count = 0;
    uint32_t sum = 0;
    
    for(i = 0; i < 12; i++)
    {
        if(pack_data_right[i].distance != 0)
        {
            count++;
            sum += pack_data_right[i].distance;
        }
    }
    
    if(count != 0)
    {
        stp23l_distance_right = sum / count;
        mecanum.stp_distance_r = stp23l_distance_right;
    }
}

// 统一的接收回调函数
void STP23L_RxCallback(uint8_t temp_data, uint8_t lr)
{
    // 根据 lr 参数选择使用左侧还是右侧的变量
    uint8_t *rx_state = (lr == 0) ? &rx_state_left : &rx_state_right;
    uint8_t *rx_crc = (lr == 0) ? &rx_crc_left : &rx_crc_right;
    uint8_t *rx_cnt = (lr == 0) ? &rx_cnt_left : &rx_cnt_right;
    uint8_t *pack_flag = (lr == 0) ? &pack_flag_left : &pack_flag_right;
    uint16_t *data_len = (lr == 0) ? &data_len_left : &data_len_right;
    uint8_t *state_flag = (lr == 0) ? &state_flag_left : &state_flag_right;
    LidarPointTypedef *pack_data = (lr == 0) ? pack_data_left : pack_data_right;
    
    if(*rx_state < 4)
    {
        if(temp_data == HEADER) (*rx_state)++;
        else *rx_state = 0;
    }
    else if(*rx_state < 10 && *rx_state > 3)
    {
        switch(*rx_state)
        {
            case 4:   
                if(temp_data == DEVICE_ADDRESS)
                {
                    (*rx_state)++;
                    *rx_crc = temp_data;
                }
                else *rx_state = 0, *rx_crc = 0;
                break;
                
            case 5:   
                if(temp_data == PACK_GET_DISTANCE)
                {
                    *pack_flag = PACK_GET_DISTANCE;
                    (*rx_state)++;
                    *rx_crc += temp_data;
                }
                else *rx_state = 0, *rx_crc = 0;
                break;
                
            case 6:
            case 7:
                if(temp_data == CHUNK_OFFSET)
                {
                    (*rx_state)++;
                    *rx_crc += temp_data;
                }
                else *rx_state = 0, *rx_crc = 0;
                break;
                
            case 8:
                *data_len = (uint16_t)temp_data;
                (*rx_state)++;
                *rx_crc += temp_data;
                break;
                
            case 9:
                *data_len = *data_len + ((uint16_t)temp_data << 8);
                (*rx_state)++;
                *rx_crc += temp_data;
                break;
                
            default: break;
        }
    }
    else if(*rx_state == 10) *state_flag = 0;
    
    if(*pack_flag == PACK_GET_DISTANCE && *state_flag == 0)
    {
        if(*rx_state > 9)
        {
            if(*rx_state < 190)
            {
                uint8_t state_num = (*rx_state - 10) % 15;
                
                switch(state_num)
                {
                    case 0:
                        pack_data[*rx_cnt].distance = (uint16_t)temp_data;
                        *rx_crc += temp_data;
                        (*rx_state)++;
                        break;
                    case 1:
                        pack_data[*rx_cnt].distance = ((uint16_t)temp_data << 8) + pack_data[*rx_cnt].distance;
                        *rx_crc += temp_data;
                        (*rx_state)++;
                        break;
                    case 2:
                        pack_data[*rx_cnt].noise = (uint16_t)temp_data;
                        *rx_crc += temp_data;
                        (*rx_state)++;
                        break;
                    case 3:
                        pack_data[*rx_cnt].noise = ((uint16_t)temp_data << 8) + pack_data[*rx_cnt].noise;
                        *rx_crc += temp_data;
                        (*rx_state)++;
                        break;
                    case 4:
                        pack_data[*rx_cnt].peak = (uint32_t)temp_data;
                        *rx_crc += temp_data;
                        (*rx_state)++;
                        break;
                    case 5:
                    case 6:
                    case 7:
                        pack_data[*rx_cnt].peak = ((uint32_t)temp_data << (8 * (state_num - 4))) + pack_data[*rx_cnt].peak;
                        *rx_crc += temp_data;
                        (*rx_state)++;
                        break;
                    case 8:
                        pack_data[*rx_cnt].confidence = temp_data;
                        *rx_crc += temp_data;
                        (*rx_state)++;
                        break;
                    case 9:
                    case 10:
                    case 11:
                    case 12:
                        pack_data[*rx_cnt].intg = ((uint32_t)temp_data << (8 * (state_num - 9))) + pack_data[*rx_cnt].intg;
                        *rx_crc += temp_data;
                        (*rx_state)++;
                        break;
                    case 13:
                        pack_data[*rx_cnt].reftof = (int16_t)temp_data;
                        *rx_crc += temp_data;
                        (*rx_state)++;
                        break;
                    case 14:
                        pack_data[*rx_cnt].reftof = ((int16_t)temp_data << 8) + pack_data[*rx_cnt].reftof;
                        *rx_crc += temp_data;
                        (*rx_state)++;
                        (*rx_cnt)++;
                        break;
                    default: break;
                }
            }
            
            if(*rx_state >= 191 && *rx_state <= 194)
            {
                *rx_crc += temp_data;
                (*rx_state)++;
            }
            else if(*rx_state == 195)
            {
                if(temp_data == *rx_crc)
                {
                    // 根据 lr 参数调用对应的处理函数
                    if(lr == 0)
                        data_process_left();
                    else
                        data_process_right();
                }
                
                *rx_crc = 0;
                *rx_state = 0;
                *state_flag = 1;
                *rx_cnt = 0;
            }
            
            if(*rx_state == 190) (*rx_state)++;
        }
    }
}
