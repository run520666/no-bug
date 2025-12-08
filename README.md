graph TB
    %% 定义样式类，模仿图片中的黑白灰/方框风格
    classDef container fill:#e0e0e0,stroke:#000,stroke-width:2px;
    classDef module fill:#fff,stroke:#000,stroke-width:1px;
    classDef title fill:none,stroke:none,font-size:14px,font-weight:bold;

    %% --- 左侧：感知与识别 (上位机) ---
    subgraph Upper_Computer [感知与识别 (上位机)]
        direction TB
        Arduino[Arduino 控制器]
        
        subgraph Sensors [传感器组]
            direction TB
            GM65[GM65 二维码模块<br/>(识别药品种类)]
            Color[感为颜色传感器<br/>(RGB药品识别)]
            Sonar[HC-SR04 光电/超声波<br/>(近距离避障)]
        end
        
        %% 内部连接
        Sensors -->|TTL/I2C/GPIO| Arduino
    end

    %% --- 中间：主从机控制核心 ---
    subgraph Core_Control [主从机控制系统]
        direction TB
        
        %% 上位机逻辑映射
        ArduinoLogic[数据打包与决策]
        
        %% 通信接口
        Comm[UART 串口通信<br/>(指令交互)]
        
        %% 下位机 (大疆板)
        subgraph Lower_Computer [运动控制 (下位机)]
            DJI_Board[大疆开发板 A 型]
            Motion_Algo[麦卡纳姆轮<br/>逆运动学解算]
            PID_Control[电机速度/位置<br/>PID 闭环控制]
        end
        
        %% 连接关系
        ArduinoLogic --> Comm
        Comm <--> DJI_Board
        DJI_Board --> Motion_Algo
        Motion_Algo --> PID_Control
    end

    %% --- 右侧/下方：底层驱动与执行 ---
    subgraph Hardware [底层硬件执行机构]
        Motor_Driver[M2006 电机驱动器<br/>(FOC/电调)]
        Motors[4x M2006 无刷电机]
        Wheels[麦卡纳姆轮组<br/>(全向移动)]
    end

    %% --- 系统级连接 ---
    Arduino -.->|提供感知数据| ArduinoLogic
    PID_Control ==>|CAN总线/PWM| Motor_Driver
    Motor_Driver ==> Motors
    Motors ==> Wheels

    %% 样式应用
    class Upper_Computer,Core_Control,Hardware container;
    class GM65,Color,Sonar,Arduino,ArduinoLogic,Comm,DJI_Board,Motion_Algo,PID_Control,Motor_Driver,Motors,Wheels module;

    %% 布局调整连接 (隐形线辅助布局)
    Upper_Computer ~~~ Core_Control
    Core_Control ~~~ Hardware
