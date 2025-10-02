//题目1：如何读取麦克纳姆轮底盘累计行驶距离？
//请写出如何在主循环中读取 mecanum_control_t 结构体的累计行驶距离（假设你已经在周期性调用里程解算函数）。
mecanum_xy_distance(&mecanum);//周期调用
float distance = mecanum.current_pos.distance;

//题目2：如何获取某个电机的实时编码器值和速度？
//请写出如何通过 get_chassis_motor_measure_point(i) 获取第 i 个电机的编码器值和速度，并打印出来。
for(int i = 0; i < 4; i++){
    const motor_measure_t *motor_data = get_chassis_motor_measure_point(i);
    //const为只读类型不会改变 motor_measure_t结构体 内的值

    if(motor_data){
      uint16_t ecd = motor_data->ecd;           // 编码器值
      int16_t speed = motor_data->speed_rpm;    // 速度（rpm）
    }
}

//题目3：如何向麦克纳姆轮控制结构体写入目标速度？
//请写一个函数，将四个目标速度写入 mecanum_control_t 结构体的 wheel_speed[4] 数组。
void mecanum_set_target(mecanum_control_t *mecanum_control)
{
    fp32 set_speed[4];
    for (int i = 0; i < 4; i++)
    {
       set_speed[i]=1000.0f;
    }
    //1
   memcpy(mecanum_control->target_speed, set_speed, sizeof(set_speed));
   
   //2
   mecanum_control->target_speed[0] = set_speed[0];
   mecanum_control->target_speed[1] = set_speed[1];
   mecanum_control->target_speed[2] = set_speed[2];
   mecanum_control->target_speed[3] = set_speed[3];

   //3
    for (int i = 0; i < 4; i++)
    {
       mecanum_control->target_speed[i] = set_speed[i];
    }
}





