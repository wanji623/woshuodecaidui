
# 我说的才队小车开发日志  
## 概况  

**开始时间**2025.9.26  
**预计结束时间**2025.10.26  
**团队成员**  
-电控组：蔡佳妤·郭佳琦  
-硬件组：饶予天  
-机械组：于子洋·邓雨秋  
**功能板块思路及说明按照小组内分工按组进行描述（在下文机械，电控，硬件）  
## 开发进程  
### 9.26  
确定最终团队成员，完成初步分工，**第一次队内会议**集体研读比赛手册，熟悉赛制赛程，互相交换对整车设计方案的看法并确认大致方向。  

### 9.29  
细化分工，确定以乒乓球收集器为原型设计小球收集装置，采购基本物资，安排国庆学习任务。  

### 10.1~10.8  
成员按分组参加培训，掌握基本技能.  

### 10.9~10.13  
进一步了解比赛场地和规则，确定两辆小车分工及一车细节  

### 10.13~10.15  
基本完成接线前的准备工作，控制代码初步完善，PCB板设计完成等待下单，一车所需3D建模完成，用于连接各部分的材料配置齐全，等待打印。万事具备只欠东风。  

### 10.16  
和学长交流后发现我们的车有翻车和过桥困难的问题，尺寸也要有较大调整，有可能导致之前的设计要推翻，我们也做好了小车需要改三四版的准备。  
### 10.18  
仔细研读参赛手册，发现缺少强制断电环节，加购两个继电器，开ps2调试口。  
###10.23
电路板即将到货，着手研究实际操作，过程中发现诸多问题，如接线方面，一个降压模块如何连接多个原件，最终询问学长解决接线问题。


*各组工作细节*
## 机械组：（机械小车的造型设计）

![小车外观设计的问题及解决方案完整版（截至10.16）]("C:\Users\郭佳琦\Desktop\37a803bcd35989c85430a4c7edc1f7b9.jpg" "小车外观设计的问题及解决方案完整版（截至10.16）")

9.29在网上收集乒乓球收集器的信息，了解其工作原理，并依据实际情况大致设计自己队小车的滚筒结构。

10.9上午根据机械组培训录播开始学习fushion的使用方法，包括一些常规的建构，草图创建立方体等，当天下午建好底盘模型，当晚建好一车的底盘支架模型。

10.10上午开始进行滚轮的建模，优点：支撑性好；缺点：无凹槽不便于绑弹力带。经讨论后，给滚轮的圆环边缘添加凹槽，将其建成类似齿轮形状，同时增加滚轮圆环外缘和它中心处连接三处的宽度，将其设计成曲线，增加整个面的受力面积。
![滚轮]("C:\Users\郭佳琦\Desktop\9d43db3e84e97d391408acb260e76b84.jpg")

10.11下午完善支架和螺丝孔细节。
![初版一车造型]("C:\Users\郭佳琦\Desktop\25d540a8fde91e8dfab5de5b882c04c0.jpg" "初版一车造型")

10.12经过上午培训课，被学长纠正了认为固定支架的杆可以打印的错误认知，并修改相应孔的尺寸，为避免打印误差，依据教学将要打印的零件单独拆出，用step形式导入3d打印软件。
![各板块建模]("C:\Users\郭佳琦\Desktop\096f03e0eba41ec818fc8253a401aa21.jpg" "各板块建模")

10.13晚完成二车大致建模，类似叉车前端的造型，确认了辆车的功能，一车主收集，二车不涉及收集功能，负责聚拢小球方便一车收集，起到阻挡对手小车走位的作用，主机动灵活。
![二车前部示意]("C:\Users\郭佳琦\Desktop\1f4aa1a9cf7a1cf32e2dff76e4bad1f5.jpg" "二车前部示意")

10.15发现底盘装上麦轮和马达后可能超过规定宽度，看过群消息之后考虑更换底板。  

10.16下午和机械学长交流后发现未考虑一车上桥车身过窄的影响过桥问题，上网搜索后得知使用原方案可能降低行车效率但不会造成不能过桥的结果，经讨论后最后决定还是沿用麦轮进行一次下坡，只有最开始的收集在桥上进行。  当晚发现新底盘过长可能留给一车滚轮的收集装置空间过小，二车前端叉子过短，故想要沿用旧底板，但需要打磨，同时为避免小车下坡时重心不稳小车前翻，降低了一车上方的框高。  

10.19因新底板长度过长，预留给滚轮的空间太小，且材料过硬不便于打磨，决定打磨原底板，与硬件和电控研究后预留安装元件的空间，重新给零件的尺寸建模。  

10.23去虎溪打印了滚筒部分的材料，可是滚轮和车子连接处的支架打印未成功，有一处连接不上，决定买热熔胶进行连接后续再想办法再加固。
![打印出问题的支架部分]("C:\Users\郭佳琦\Desktop\4329b2477d27857a64e1a031de7c0685.jpg" "打印出问题的支架部分")
 
 ![PCB板]("C:\Users\郭佳琦\Desktop\5c7b082ddaf25b665560cf5100673cab.png" "PCB板")
**尾声时期遇到的困难及解决措施：

原计划改用新底板，后因小车的构造和新版底板的金属材料还需布置绝缘处理，最后使用原计划。  

继电器问题是后期才发现的，原来的电路中未预留继电器的位置，经搜索各类资料并参考官方物资后发现继电器有单独的遥控器，无需在代码上改动，我们现在需要把继电器接在电源和降压模块之间，将其连接在电路的干路上，做到强制断电。  

调试口被占用，在电路板接好后直接接上马达进行调试。


## 硬件组：（为减少飞线过多造成的线路杂乱，掉线问题，本组决定设计PCB电路板）  

**初期**

1. 反复观看硬件组PCB培训视频，在每次的温习中获取新知，解答自己的各种新疑惑，查缺补漏。

2. 学习使用嘉立创进行PCB练习板绘制，研究原理及硬件作用。

3. 根据小组汽车的需求进行队伍PCB正式板绘制。发现电源模块以及降压模块的接线仍存在不懂的地方，通过询问学长，运用AI软件以及搜索引擎逐渐解决自己的疑惑。

**后期**  

设计出电路板
![PCB板]("C:\Users\郭佳琦\Desktop\5c7b082ddaf25b665560cf5100673cab.png" "PCB板")

学习期间的疑虑和解惑过程：  
1. 电池盒上只有一根正极线，但需要为两个电机驱动模块供电，所以在电源连接降压模块后通过电路板分别给两个电机驱动模块供电。  
2. 设计过程中引脚布置不合理导致导致PCB布线困难，我们队里的电控和硬件互相提建议共同完成开引脚和布线。
3. 焊笔架不稳固，手动加固，焊台不平整，用纸箱扣洞保证焊接过程中的平稳
![电路板实物图]("C:\Users\郭佳琦\Desktop\IMG_20251024_145405.jpg" "电路板实物图")
![焊接过程]("C:\Users\郭佳琦\Desktop\IMG_20251024_145752.jpg" "焊接过程")




## 电控组：（按照进程来编写，共有四个小板块，内涵部分代码，占行略多，但代码内有注释各部分的功能和设计原因）

### 环境搭建：  
- STM32CubeMX安装配置
- 开发环境搭建（Keil MDK）
- 创建基础工作框架

！*问题*：环境配置不成功，无法正常使用各个软件推进进度。

！*解决办法*：大量搜集网络资料，逐一排除操作过程中的错误，成功安装。


### 电机驱动模块：  

|1.了解L298N工作原理 |
| :------------ |
| 2. 配置PWM定时器（TIM1，2）；GPIO方向控制 |
| 3.编写电机基础控制函数  |

**以下为控制电机的代码**  

```c++
#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"

// 电机编号定义
typedef enum {
    MOTOR_FRONT_LEFT = 0,    // 左前电机
    MOTOR_REAR_LEFT,         // 左后电机
    MOTOR_FRONT_RIGHT,       // 右前电机
    MOTOR_REAR_RIGHT,        // 右后电机
    MOTOR_COUNT              // 电机总数
} Motor_ID;

// 电机方向定义
typedef enum {
    MOTOR_STOP = 0,          // 停止
    MOTOR_FORWARD,           // 正转
    MOTOR_BACKWARD,          // 反转
    MOTOR_BRAKE              // 刹车
} Motor_Direction;

// 电机配置结构体
typedef struct {
    TIM_HandleTypeDef* pwm_timer;    // PWM定时器
    uint32_t pwm_channel;            // PWM通道
    GPIO_TypeDef* in1_port;          // IN1端口
    uint16_t in1_pin;                // IN1引脚
    GPIO_TypeDef* in2_port;          // IN2端口
    uint16_t in2_pin;                // IN2引脚
} Motor_Config;

// 函数声明
void Motor_Init(void);
void Motor_Control(Motor_ID motor, Motor_Direction dir, uint16_t speed);
void Motor_Set_Speed(Motor_ID motor, uint16_t speed);
void Motor_Stop_All(void);
void Motor_Brake_All(void);

// 简化控制函数
void Motor_Forward(Motor_ID motor, uint16_t speed);
void Motor_Backward(Motor_ID motor, uint16_t speed);

#endif /* __MOTOR_H */```
```c++
#include "motor.h"

// 电机配置数组
static Motor_Config motors[MOTOR_COUNT];

// 电机初始化
void Motor_Init(void)
{
    /* L298N #1 配置 - 控制左轮 */
    // 左前电机
    motors[MOTOR_FRONT_LEFT].pwm_timer = &htim1;
    motors[MOTOR_FRONT_LEFT].pwm_channel = TIM_CHANNEL_1;  // ENA
    motors[MOTOR_FRONT_LEFT].in1_port = IN1_GPIO_Port;
    motors[MOTOR_FRONT_LEFT].in1_pin = IN1_Pin;           // IN1
    motors[MOTOR_FRONT_LEFT].in2_port = IN2_GPIO_Port;
    motors[MOTOR_FRONT_LEFT].in2_pin = IN2_Pin;           // IN2
    
    // 左后电机
    motors[MOTOR_REAR_LEFT].pwm_timer = &htim2;
    motors[MOTOR_REAR_LEFT].pwm_channel = TIM_CHANNEL_1;   // ENA
    motors[MOTOR_REAR_LEFT].in1_port = IN1_GPIO_Port;
    motors[MOTOR_REAR_LEFT].in1_pin = IN1_Pin;            // IN1
    motors[MOTOR_REAR_LEFT].in2_port = IN2_GPIO_Port;
    motors[MOTOR_REAR_LEFT].in2_pin = IN2_Pin;            // IN2
    
    /* L298N #2 配置 - 控制右轮 */
    // 右前电机
    motors[MOTOR_FRONT_RIGHT].pwm_timer = &htim1;
    motors[MOTOR_FRONT_RIGHT].pwm_channel = TIM_CHANNEL_2; // ENB
    motors[MOTOR_FRONT_RIGHT].in1_port = IN3_GPIO_Port;
    motors[MOTOR_FRONT_RIGHT].in1_pin = IN3_Pin;          // IN3
    motors[MOTOR_FRONT_RIGHT].in2_port = IN4_GPIO_Port;
    motors[MOTOR_FRONT_RIGHT].in2_pin = IN4_Pin;          // IN4
    
    // 右后电机
    motors[MOTOR_REAR_RIGHT].pwm_timer = &htim2;
    motors[MOTOR_REAR_RIGHT].pwm_channel = TIM_CHANNEL_2;  // ENB
    motors[MOTOR_REAR_RIGHT].in1_port = IN3_GPIO_Port;
    motors[MOTOR_REAR_RIGHT].in1_pin = IN3_Pin;           // IN3
    motors[MOTOR_REAR_RIGHT].in2_port = IN4_GPIO_Port;
    motors[MOTOR_REAR_RIGHT].in2_pin = IN4_Pin;           // IN4
    
    // 启动所有PWM通道
    for(int i = 0; i < MOTOR_COUNT; i++) {
        HAL_TIM_PWM_Start(motors[i].pwm_timer, motors[i].pwm_channel);
    }
    
    // 初始状态：停止所有电机
    Motor_Stop_All();
    
    printf("电机系统初始化完成\r\n");
}

// 控制单个电机
void Motor_Control(Motor_ID motor, Motor_Direction dir, uint16_t speed)
{
    if(motor >= MOTOR_COUNT) return;
    
    Motor_Config* m = &motors[motor];
    
    // 设置方向
    switch(dir) {
        case MOTOR_STOP:
            HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
            break;
            
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
            break;
            
        case MOTOR_BACKWARD:
            HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
            break;
            
        case MOTOR_BRAKE:
            HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
            break;
    }
    
    // 设置速度（PWM占空比）
    __HAL_TIM_SET_COMPARE(m->pwm_timer, m->pwm_channel,speed);
}

// 仅设置电机速度（不改变方向）
void Motor_Set_Speed(Motor_ID motor, uint16_t speed)
{
    if(motor >= MOTOR_COUNT) return;
    Motor_Config* m = &motors[motor];
    __HAL_TIM_SET_COMPARE(m->pwm_timer, m->pwm_channel, speed);
}

// 停止所有电机
void Motor_Stop_All(void)
{
    for(int i = 0; i < MOTOR_COUNT; i++) {
        Motor_Control(i, MOTOR_STOP, 0);
    }
}

// 刹车所有电机
void Motor_Brake_All(void)
{
    for(int i = 0; i < MOTOR_COUNT; i++) {
        Motor_Control(i, MOTOR_BRAKE, 0);
    }
}

// 简化函数 - 电机正转
void Motor_Forward(Motor_ID motor, uint16_t speed)
{
    Motor_Control(motor, MOTOR_FORWARD, speed);
}

// 简化函数 - 电机反转
void Motor_Backward(Motor_ID motor, uint16_t speed)
{
    Motor_Control(motor, MOTOR_BACKWARD, speed);
}
```



！*问题：*无法驱动电机

！*解决办法：*改变使能引脚为高电平



#### PS2手柄集成：  
- 理解PS2通信协议
- GPIO模拟PS2时序
- 应用学长给出的配置PS2头文件源文件进行手柄数据解析

！*问题*：PS2通信始终链接不上，耗时过就，严重拖延进度。

！*解决办法*：逐行检查错误之处，一一对应代码上与实际开启引脚的差别，矫正引脚A和B的输入错误，合理布局引脚。



### 小车运动控制：  
|  学习麦轮在小车各向运动中的工作原理根据我们自己的小车情况合理布置引脚，规划不同运动状态时小车速度 |
| :------------ |
|编写控制不同运动方向的电机转发及转动功率|
|用PS2手柄上的遥杆及键位，通过摇杆推动的幅度配置车速|

**以下为小车不同运动状态时的控制代码**  

```c++
#include "mecanum_wheel.h"
#include "motor_control.h"
#include "math.h"
// 运动模式枚举
typedef enum {
    STOP = 0,
    FORWARD,
    BACKWARD, 
    LEFT,
    RIGHT,
    ROTATE_CW,    // 顺时针旋转
    ROTATE_CCW    // 逆时针旋转
} Motion_Mode;

/**
  * @brief  麦克纳姆轮运动学计算
  * @param  vx: X方向速度(-16000到16000)
  * @param  vy: Y方向速度(-16000到16000) 
  * @param  omega: 旋转速度(-12000到12000)
  * @retval 无
  */
void Mecanum_Kinematics(int32_t vx, int32_t vy, int32_t omega)
{
    // 四个轮子的速度计算
    
    
    int32_t wheel1_speed = vy + vx + omega;
    int32_t wheel2_speed = vy - vx - omega;
    int32_t wheel3_speed = vy - vx + omega; 
    int32_t wheel4_speed = vy + vx - omega;
    
    // 设置四个电机速度
    Set_Motor_Speed(0, wheel1_speed); // 左前轮
    Set_Motor_Speed(1, wheel2_speed); // 右前轮
    Set_Motor_Speed(2, wheel3_speed); // 左后轮  
    Set_Motor_Speed(3, wheel4_speed); // 右后轮
}

/**
  * @brief  PS2手柄控制小车运动
  * @param  joystick: 手柄数据结构体指针
  * @retval 无
  */
void PS2_Control_Car(JOYSTICK_TypeDef *joystick)
{
    int32_t vx = 0, vy = 0, omega = 0;
    
    // 读取左摇杆值并转换为速度(-500到500)
    // 左摇杆上下控制前后(Y方向)
    int32_t left_y = (int32_t)joystick->LJoy_UD - 128;
    // 左摇杆左右控制左右(X方向)  
    int32_t left_x = (int32_t)joystick->LJoy_LR - 128;
    
    // 读取右摇杆左右值控制旋转
    int32_t right_x = (int32_t)joystick->RJoy_LR - 128;
    
    // 设置死区，避免摇杆微小晃动
    if(abs(left_y) < 20) left_y = 0;
    if(abs(left_x) < 20) left_x = 0; 
    if(abs(right_x) < 20) right_x = 0;
    
    // 缩放速度值
    vy = left_y * 40;    // 前后速度
    vx = left_x * 40;    // 左右平移速度  
    omega = right_x * 40; //旋转速度
    
    // 调用运动学计算
    Mecanum_Kinematics(vx, vy, omega);
}```
```c++
#include "carcontral.h"

// 默认速度配置
static Speed_Config speed_config = {
    .normal_speed = 600,    // 60%速度
    .turn_speed = 400,      // 40%速度
    .spin_speed = 500       // 50%速度
};

static Car_State current_state = CAR_STOP;

// 小车控制初始化
void Car_Control_Init(void)
{
    current_state = CAR_STOP;
    Car_Stop();
    printf("小车控制系统初始化完成\r\n");
}

// 设置速度配置
void Car_Control_Set_Speed_Config(uint16_t normal, uint16_t turn, uint16_t spin)
{
    speed_config.normal_speed = normal;
    speed_config.turn_speed = turn;
    speed_config.spin_speed = spin;
}

// 小车前进
void Car_Forward(void)
{
    Motor_Forward(MOTOR_FRONT_LEFT, speed_config.normal_speed);
    Motor_Forward(MOTOR_REAR_LEFT, speed_config.normal_speed);
    Motor_Forward(MOTOR_FRONT_RIGHT, speed_config.normal_speed);
    Motor_Forward(MOTOR_REAR_RIGHT, speed_config.normal_speed);
    current_state = CAR_FORWARD;
}

// 小车后退
void Car_Backward(void)
{
    Motor_Backward(MOTOR_FRONT_LEFT, speed_config.normal_speed);
    Motor_Backward(MOTOR_REAR_LEFT, speed_config.normal_speed);
    Motor_Backward(MOTOR_FRONT_RIGHT, speed_config.normal_speed);
    Motor_Backward(MOTOR_REAR_RIGHT, speed_config.normal_speed);
    current_state = CAR_BACKWARD;
}

// 小车左转（差速转弯）
void Car_TurnLeft(void)
{
    // 左轮慢速反转，右轮快速正转
    Motor_Backward(MOTOR_FRONT_LEFT, speed_config.turn_speed / 2);
    Motor_Backward(MOTOR_REAR_LEFT, speed_config.turn_speed / 2);
    Motor_Forward(MOTOR_FRONT_RIGHT, speed_config.turn_speed);
    Motor_Forward(MOTOR_REAR_RIGHT, speed_config.turn_speed);
    current_state = CAR_TURN_LEFT;
}

// 小车右转（差速转弯）
void Car_TurnRight(void)
{
    // 左轮快速正转，右轮慢速反转
    Motor_Forward(MOTOR_FRONT_LEFT, speed_config.turn_speed);
    Motor_Forward(MOTOR_REAR_LEFT, speed_config.turn_speed);
    Motor_Backward(MOTOR_FRONT_RIGHT, speed_config.turn_speed / 2);
    Motor_Backward(MOTOR_REAR_RIGHT, speed_config.turn_speed / 2);
    current_state = CAR_TURN_RIGHT;
}

// 小车原地左转
void Car_SpinLeft(void)
{
    // 左轮反转，右轮正转
    Motor_Backward(MOTOR_FRONT_LEFT, speed_config.spin_speed);
    Motor_Backward(MOTOR_REAR_LEFT, speed_config.spin_speed);
    Motor_Forward(MOTOR_FRONT_RIGHT, speed_config.spin_speed);
    Motor_Forward(MOTOR_REAR_RIGHT, speed_config.spin_speed);
    current_state = CAR_SPIN_LEFT;
}

// 小车原地右转
void Car_SpinRight(void)
{
    // 左轮正转，右轮反转
    Motor_Forward(MOTOR_FRONT_LEFT, speed_config.spin_speed);
    Motor_Forward(MOTOR_REAR_LEFT, speed_config.spin_speed);
    Motor_Backward(MOTOR_FRONT_RIGHT, speed_config.spin_speed);
    Motor_Backward(MOTOR_REAR_RIGHT, speed_config.spin_speed);
    current_state = CAR_SPIN_RIGHT;
}

// 小车停止
void Car_Stop(void)
{
    Motor_Stop_All();
    current_state = CAR_STOP;
}

// 小车刹车
void Car_Brake(void)
{
    Motor_Brake_All();
    current_state = CAR_STOP;
}

// 带速度控制的运动函数
void Car_Move_With_Speed(Car_State state, uint16_t speed)
{
    switch(state) {
        case CAR_FORWARD:
            Motor_Forward(MOTOR_FRONT_LEFT, speed);
            Motor_Forward(MOTOR_REAR_LEFT, speed);
            Motor_Forward(MOTOR_FRONT_RIGHT, speed);
            Motor_Forward(MOTOR_REAR_RIGHT, speed);
            break;
            
        case CAR_BACKWARD:
            Motor_Backward(MOTOR_FRONT_LEFT, speed);
            Motor_Backward(MOTOR_REAR_LEFT, speed);
            Motor_Backward(MOTOR_FRONT_RIGHT, speed);
            Motor_Backward(MOTOR_REAR_RIGHT, speed);
            break;
            
        default:
            break;
    }
    current_state = state;
}

// 差速转向（高级控制）
void Car_Differential_Turn(int16_t left_speed, int16_t right_speed)
{
    // 左轮控制
    if(left_speed >= 0) {
        Motor_Forward(MOTOR_FRONT_LEFT, left_speed);
        Motor_Forward(MOTOR_REAR_LEFT, left_speed);
    } else {
        Motor_Backward(MOTOR_FRONT_LEFT, -left_speed);
        Motor_Backward(MOTOR_REAR_LEFT, -left_speed);
    }
    
    // 右轮控制
    if(right_speed >= 0) {
        Motor_Forward(MOTOR_FRONT_RIGHT, right_speed);
        Motor_Forward(MOTOR_REAR_RIGHT, right_speed);
    } else {
        Motor_Backward(MOTOR_FRONT_RIGHT, -right_speed);
        Motor_Backward(MOTOR_REAR_RIGHT, -right_speed);
    }
    
    current_state = CAR_TURN_LEFT; // 简化状态
}

// 获取当前状态
Car_State Car_Get_Current_State(void)
{
    return current_state;
} ```  



~~这里只引用了几个代码，全部写上太长~~  

