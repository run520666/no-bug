/**
  ****************************(C) COPYRIGHT 2023 YOUR_NAME****************************
  * @file       command_parser.c/h
  * @brief      串口命令解析模块，用于接收和解析坐标导航命令
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-07-21      YOUR_NAME       1. 创建
  *
  @verbatim
  ==============================================================================
  支持的命令格式：
  1. "goto x y" - 导航到指定坐标，例如 "goto 1000 500"
  2. "stop" - 停止导航
  3. "reset" - 重置当前位置为原点
  4. "status" - 查询当前状态
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 YOUR_NAME****************************
  */

#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include "struct_typedef.h"
#include "mecanum_control.h"
#include "encoder_position.h"

#define CMD_BUFFER_SIZE 128

// 命令解析结构体
typedef struct
{
    char buffer[CMD_BUFFER_SIZE];  // 命令缓冲区
    uint16_t index;                // 当前索引
    uint8_t command_ready;         // 命令就绪标志
    
    mecanum_control_t *mecanum;    // 麦轮控制指针
    encoder_position_t *encoder;   // 编码器位置指针
    
} command_parser_t;

// 命令解析器初始化
extern void command_parser_init(command_parser_t *parser, mecanum_control_t *mecanum, encoder_position_t *encoder);

// 处理接收到的字符
extern void command_parser_input_char(command_parser_t *parser, char ch);

// 处理命令（需要周期调用）
extern void command_parser_process(command_parser_t *parser);

#endif // COMMAND_PARSER_H
