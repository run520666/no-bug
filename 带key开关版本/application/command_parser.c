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

#include "command_parser.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/**
  * @brief          命令解析器初始化
  * @param[out]     parser: 命令解析器结构体指针
  * @param[in]      mecanum: 麦轮控制结构体指针
  * @param[in]      encoder: 编码器位置结构体指针
  * @retval         none
  */
void command_parser_init(command_parser_t *parser, mecanum_control_t *mecanum, encoder_position_t *encoder)
{
    if (parser == NULL)
    {
        return;
    }
    
    memset(parser->buffer, 0, CMD_BUFFER_SIZE);
    parser->index = 0;
    parser->command_ready = 0;
    parser->mecanum = mecanum;
    parser->encoder = encoder;
}

/**
  * @brief          处理接收到的字符
  * @param[in,out]  parser: 命令解析器结构体指针
  * @param[in]      ch: 接收到的字符
  * @retval         none
  */
void command_parser_input_char(command_parser_t *parser, char ch)
{
    if (parser == NULL)
    {
        return;
    }
    
    // 处理回车换行
    if (ch == '\r' || ch == '\n')
    {
        if (parser->index > 0)
        {
            parser->buffer[parser->index] = '\0';
            parser->command_ready = 1;
        }
        return;
    }
    
    // 处理退格
    if (ch == '\b' || ch == 127)
    {
        if (parser->index > 0)
        {
            parser->index--;
            parser->buffer[parser->index] = '\0';
        }
        return;
    }
    
    // 添加字符到缓冲区
    if (parser->index < CMD_BUFFER_SIZE - 1 && ch >= 32 && ch <= 126)
    {
        parser->buffer[parser->index] = ch;
        parser->index++;
    }
}

/**
  * @brief          解析goto命令
  * @param[in]      parser: 命令解析器结构体指针
  * @param[in]      args: 命令参数字符串
  * @retval         none
  */
static void parse_goto_command(command_parser_t *parser, const char *args)
{
    float x, y;
    
    if (sscanf(args, "%f %f", &x, &y) == 2)
    {
        mecanum_nav_set_target(parser->mecanum, x, y);
        mecanum_nav_enable(parser->mecanum, 1);
        printf("Navigation target set to (%.1f, %.1f)\n", x, y);
    }
    else
    {
        printf("Error: Invalid goto command format. Use: goto x y\n");
    }
}

/**
  * @brief          处理status命令
  * @param[in]      parser: 命令解析器结构体指针
  * @retval         none
  */
static void handle_status_command(command_parser_t *parser)
{
    float current_x, current_y, current_angle;
    encoder_position_get(parser->encoder, &current_x, &current_y, &current_angle);
    
    nav_state_t nav_state = mecanum_nav_get_state(parser->mecanum);
    float distance = mecanum_nav_get_distance_to_target(parser->mecanum);
    
    printf("=== Robot Status ===\n");
    printf("Current Position: X=%.1f mm, Y=%.1f mm, Angle=%.2f deg\n", 
           current_x, current_y, current_angle * 180.0f / 3.14159f);
    
    const char* state_names[] = {"IDLE", "MOVING", "ARRIVED", "ERROR"};
    printf("Navigation State: %s\n", state_names[nav_state]);
    printf("Distance to target: %.1f mm\n", distance);
    
    printf("Wheel speeds: [%.0f, %.0f, %.0f, %.0f] rpm\n", 
           parser->mecanum->wheel_speed[0], parser->mecanum->wheel_speed[1], 
           parser->mecanum->wheel_speed[2], parser->mecanum->wheel_speed[3]);
    printf("==================\n");
}

/**
  * @brief          处理命令（需要周期调用）
  * @param[in,out]  parser: 命令解析器结构体指针
  * @retval         none
  */
void command_parser_process(command_parser_t *parser)
{
    if (parser == NULL || !parser->command_ready)
    {
        return;
    }
    
    // 转换为小写
    for (uint16_t i = 0; i < parser->index; i++)
    {
        if (parser->buffer[i] >= 'A' && parser->buffer[i] <= 'Z')
        {
            parser->buffer[i] += 32;
        }
    }
    
    // 解析命令
    if (strncmp(parser->buffer, "goto ", 5) == 0)
    {
        parse_goto_command(parser, parser->buffer + 5);
    }
    else if (strcmp(parser->buffer, "stop") == 0)
    {
        mecanum_nav_enable(parser->mecanum, 0);
        printf("Navigation stopped\n");
    }
    else if (strcmp(parser->buffer, "reset") == 0)
    {
        encoder_position_reset(parser->encoder);
        printf("Position reset to origin\n");
    }
    else if (strcmp(parser->buffer, "status") == 0)
    {
        handle_status_command(parser);
    }
    else if (strcmp(parser->buffer, "help") == 0)
    {
        printf("Available commands:\n");
        printf("  goto x y  - Navigate to position (x, y) in mm\n");
        printf("  stop      - Stop navigation\n");
        printf("  reset     - Reset position to origin\n");
        printf("  status    - Show current status\n");
        printf("  help      - Show this help\n");
    }
    else if (strlen(parser->buffer) > 0)
    {
        printf("Unknown command: %s\n", parser->buffer);
        printf("Type 'help' for available commands\n");
    }
    
    // 清空缓冲区
    memset(parser->buffer, 0, CMD_BUFFER_SIZE);
    parser->index = 0;
    parser->command_ready = 0;
}
