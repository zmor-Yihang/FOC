#include "test_pid.h"
#include <math.h>

/* 模拟电机参数 */
#define MOTOR_INERTIA 0.001f    /* 转动惯量 kg·m² */
#define MOTOR_RESISTANCE 1.5f   /* 相电阻 Ω */
#define MOTOR_KT 0.05f          /* 转矩常数 Nm/A */
#define MOTOR_KE 0.05f          /* 反电动势常数 V/(rad/s) */
#define MOTOR_FRICTION 0.001f   /* 摩擦系数 */

/* 模拟控制参数 */
#define SIM_DT 0.001f           /* 仿真步长 1ms */
#define SIM_TIME 2.0f           /* 仿真总时间 2秒 */

/* 全局模拟状态变量 */
static float sim_speed = 0.0f;      /* 当前转速 rad/s */
static float sim_current_q = 0.0f;  /* Q轴电流 A */
static float sim_current_d = 0.0f;  /* D轴电流 A */
static float sim_position = 0.0f;   /* 位置 rad */

/**
 * @brief 模拟Q轴电流响应（一阶惯性环节）
 * @param target_iq 目标Q轴电流
 * @param voltage_q Q轴电压
 * @param dt 时间步长
 * @return 实际Q轴电流
 */
static float simulate_current_q(float target_iq, float voltage_q, float dt)
{
    /* 简化的电流环模型：一阶惯性 */
    float tau_current = 0.005f; /* 电流环时间常数 5ms */
    float alpha = dt / (tau_current + dt);
    
    /* 考虑电压限制的目标电流 */
    float iq_from_voltage = voltage_q / MOTOR_RESISTANCE;
    
    /* 电流响应 */
    sim_current_q += alpha * (iq_from_voltage - sim_current_q);
    
    return sim_current_q;
}

/**
 * @brief 模拟D轴电流响应（通常为0）
 * @param target_id 目标D轴电流
 * @param voltage_d D轴电压
 * @param dt 时间步长
 * @return 实际D轴电流
 */
static float simulate_current_d(float target_id, float voltage_d, float dt)
{
    /* 简化的电流环模型 */
    float tau_current = 0.005f;
    float alpha = dt / (tau_current + dt);
    
    float id_from_voltage = voltage_d / MOTOR_RESISTANCE;
    sim_current_d += alpha * (id_from_voltage - sim_current_d);
    
    return sim_current_d;
}

/**
 * @brief 模拟电机机械响应
 * @param iq Q轴电流（产生转矩）
 * @param load_torque 负载转矩
 * @param dt 时间步长
 * @return 实际转速 rad/s
 */
static float simulate_motor_mechanics(float iq, float load_torque, float dt)
{
    /* 电机转矩 = Kt * Iq */
    float motor_torque = MOTOR_KT * iq;
    
    /* 摩擦转矩 */
    float friction_torque = MOTOR_FRICTION * sim_speed;
    
    /* 净转矩 */
    float net_torque = motor_torque - load_torque - friction_torque;
    
    /* 角加速度 = 转矩 / 转动惯量 */
    float angular_accel = net_torque / MOTOR_INERTIA;
    
    /* 更新速度 */
    sim_speed += angular_accel * dt;
    
    /* 更新位置 */
    sim_position += sim_speed * dt;
    
    return sim_speed;
}

/**
 * @brief 测试FOC三闭环控制（速度环 -> Q轴电流环 -> D轴电流环）
 */
void test_foc_triple_loop(void)
{
    printf("\r\n========== FOC三闭环PID测试 ==========\r\n");
    
    /* 初始化三个PID控制器 */
    pid_controller_t pid_speed; /* 速度环PID */
    pid_controller_t pid_iq;    /* Q轴电流环PID */
    pid_controller_t pid_id;    /* D轴电流环PID */
    
    /* 速度环参数（外环，较慢） */
    pid_init(&pid_speed, 
             0.5f,   /* Kp */
             0.1f,   /* Ki */
             0.0f,   /* Kd */
             10.0f,  /* 输出上限(目标Iq) */
             -10.0f  /* 输出下限 */
    );
    
    /* Q轴电流环参数（内环，较快） */
    pid_init(&pid_iq,
             2.0f,   /* Kp */
             50.0f,  /* Ki */
             0.0f,   /* Kd */
             12.0f,  /* 输出上限(Vq电压) */
             -12.0f  /* 输出下限 */
    );
    
    /* D轴电流环参数（通常Id=0） */
    pid_init(&pid_id,
             2.0f,   /* Kp */
             50.0f,  /* Ki */
             0.0f,   /* Kd */
             12.0f,  /* 输出上限(Vd电压) */
             -12.0f  /* 输出下限 */
    );
    
    /* 设置目标 */
    float target_speed = 100.0f; /* 目标转速 100 rad/s */
    float target_id = 0.0f;      /* D轴电流目标为0 */
    float load_torque = 0.02f;   /* 负载转矩 0.02 Nm */
    
    /* 复位仿真状态 */
    sim_speed = 0.0f;
    sim_current_q = 0.0f;
    sim_current_d = 0.0f;
    sim_position = 0.0f;
    
    printf("目标转速: %.2f rad/s\r\n", target_speed);
    printf("负载转矩: %.3f Nm\r\n", load_torque);
    printf("仿真步长: %.3f ms\r\n", SIM_DT * 1000.0f);
    printf("\r\n时间(s)\t速度(rad/s)\tIq(A)\tId(A)\t速度误差\r\n");
    printf("----------------------------------------------------------\r\n");
    
    /* 仿真循环 */
    int step_count = (int)(SIM_TIME / SIM_DT);
    int print_interval = 100; /* 每100步打印一次 (100ms) */
    
    for (int i = 0; i < step_count; i++)
    {
        float time = i * SIM_DT;
        
        /* === 第1层：速度环（最外环）=== */
        float target_iq = pid_calculate(&pid_speed, target_speed, sim_speed);
        
        /* === 第2层：Q轴电流环 === */
        float voltage_q = pid_calculate(&pid_iq, target_iq, sim_current_q);
        
        /* === 第3层：D轴电流环 === */
        float voltage_d = pid_calculate(&pid_id, target_id, sim_current_d);
        
        /* === 模拟被控对象 === */
        /* 电流响应 */
        float actual_iq = simulate_current_q(target_iq, voltage_q, SIM_DT);
        float actual_id = simulate_current_d(target_id, voltage_d, SIM_DT);
        
        /* 机械响应 */
        float actual_speed = simulate_motor_mechanics(actual_iq, load_torque, SIM_DT);
        
        /* 定期打印结果 */
        if (i % print_interval == 0)
        {
            float speed_error = target_speed - actual_speed;
            printf("%.3f\t\t%.2f\t\t%.2f\t%.2f\t%.2f\r\n", 
                   time, actual_speed, actual_iq, actual_id, speed_error);
        }
        
        /* 在1秒时增加负载 */
        if (time > 1.0f && time < 1.001f)
        {
            load_torque = 0.05f;
            printf("\r\n>>> 负载变化: 0.05 Nm <<<\r\n\r\n");
        }
    }
    
    printf("\r\n========== 测试完成 ==========\r\n");
    printf("最终转速: %.2f rad/s\r\n", sim_speed);
    printf("速度误差: %.2f rad/s (%.1f%%)\r\n", 
           target_speed - sim_speed,
           fabs(target_speed - sim_speed) / target_speed * 100.0f);
}

/**
 * @brief 测试单个PID控制器（阶跃响应）
 */
void test_single_pid(void)
{
    printf("\r\n========== 单个PID控制器测试 ==========\r\n");
    
    pid_controller_t pid;
    
    /* 初始化PID参数 */
    pid_init(&pid, 
             1.0f,   /* Kp */
             0.5f,   /* Ki */
             0.1f,   /* Kd */
             100.0f, /* out_max */
             -100.0f /* out_min */
    );
    
    float setpoint = 50.0f;  /* 目标值 */
    float feedback = 0.0f;   /* 当前值（被控对象输出） */
    float plant_output = 0.0f;
    
    printf("目标值: %.2f\r\n", setpoint);
    printf("\r\n步数\t反馈值\tPID输出\t误差\r\n");
    printf("----------------------------------------\r\n");
    
    /* 模拟控制过程 */
    for (int i = 0; i < 100; i++)
    {
        /* PID计算 */
        float control = pid_calculate(&pid, setpoint, feedback);
        
        /* 简化的被控对象（一阶惯性） */
        float tau = 10.0f; /* 时间常数 */
        plant_output += (control - plant_output) / tau;
        feedback = plant_output;
        
        /* 每10步打印一次 */
        if (i % 10 == 0)
        {
            printf("%d\t%.2f\t%.2f\t%.2f\r\n", 
                   i, feedback, control, setpoint - feedback);
        }
    }
    
    printf("\r\n========== 测试完成 ==========\r\n");
    printf("最终值: %.2f\r\n", feedback);
    printf("稳态误差: %.2f\r\n", setpoint - feedback);
}

/**
 * @brief 通用电机响应模拟函数（供外部调用）
 * @param voltage 输入电压
 * @param load_torque 负载转矩
 * @param dt 时间步长
 * @return 转速
 */
float simulate_motor_response(float voltage, float load_torque, float dt)
{
    /* 电流 = 电压 / 电阻 */
    float current = voltage / MOTOR_RESISTANCE;
    
    return simulate_motor_mechanics(current, load_torque, dt);
}

/**
 * @brief 测试FOC三闭环控制，并发送数据到VOFA+上位机
 * @param huart 串口句柄（如&huart1）
 * 
 * VOFA+配置：
 * 1. 协议选择：RawData
 * 2. 通道数：5（目标速度、实际速度、Iq、Id、速度误差）
 * 3. 波特率：115200
 */
void test_foc_triple_loop_vofa(UART_HandleTypeDef *huart)
{
    /* 初始化三个PID控制器 */
    pid_controller_t pid_speed; /* 速度环PID */
    pid_controller_t pid_iq;    /* Q轴电流环PID */
    pid_controller_t pid_id;    /* D轴电流环PID */
    
    /* 速度环参数（外环，较慢） */
    pid_init(&pid_speed, 
             0.5f,   /* Kp */
             0.1f,   /* Ki */
             0.0f,   /* Kd */
             10.0f,  /* 输出上限(目标Iq) */
             -10.0f  /* 输出下限 */
    );
    
    /* Q轴电流环参数（内环，较快） */
    pid_init(&pid_iq,
             2.0f,   /* Kp */
             50.0f,  /* Ki */
             0.0f,   /* Kd */
             12.0f,  /* 输出上限(Vq电压) */
             -12.0f  /* 输出下限 */
    );
    
    /* D轴电流环参数（通常Id=0） */
    pid_init(&pid_id,
             2.0f,   /* Kp */
             50.0f,  /* Ki */
             0.0f,   /* Kd */
             12.0f,  /* 输出上限(Vd电压) */
             -12.0f  /* 输出下限 */
    );
    
    /* 设置目标 */
    float target_speed = 100.0f; /* 目标转速 100 rad/s */
    float target_id = 0.0f;      /* D轴电流目标为0 */
    float load_torque = 0.02f;   /* 负载转矩 0.02 Nm */
    
    /* 复位仿真状态 */
    sim_speed = 0.0f;
    sim_current_q = 0.0f;
    sim_current_d = 0.0f;
    sim_position = 0.0f;
    
    
    /* 仿真循环 */
    int step_count = (int)(SIM_TIME / SIM_DT);
    int send_interval = 10; /* 每10步发送一次 (10ms) 降低串口负载 */
    
    for (int i = 0; i < step_count; i++)
    {
        float time = i * SIM_DT;
        
        /* === 第1层：速度环（最外环）=== */
        float target_iq = pid_calculate(&pid_speed, target_speed, sim_speed);
        
        /* === 第2层：Q轴电流环 === */
        float voltage_q = pid_calculate(&pid_iq, target_iq, sim_current_q);
        
        /* === 第3层：D轴电流环 === */
        float voltage_d = pid_calculate(&pid_id, target_id, sim_current_d);
        
        /* === 模拟被控对象 === */
        /* 电流响应 */
        float actual_iq = simulate_current_q(target_iq, voltage_q, SIM_DT);
        float actual_id = simulate_current_d(target_id, voltage_d, SIM_DT);
        
        /* 机械响应 */
        float actual_speed = simulate_motor_mechanics(actual_iq, load_torque, SIM_DT);
        
        /* 定期发送数据到VOFA+ */
        if (i % send_interval == 0)
        {
            float speed_error = target_speed - actual_speed;
            
            /* VOFA+ RawData格式：CSV格式，逗号分隔，换行结尾 */
            /* 通道1: 目标速度, 通道2: 实际速度, 通道3: Iq电流, 通道4: Id电流, 通道5: 速度误差 */
            vofa_print(huart, "%.2f,%.2f,%.2f,%.2f,%.2f\n", 
                       target_speed, actual_speed, actual_iq, actual_id, speed_error);
        }
        
        /* 在1秒时增加负载 */
        if (time > 1.0f && time < 1.001f)
        {
            load_torque = 0.05f;
        }
    }
    
    /* 发送测试结束标志 */
    HAL_Delay(100);
    vofa_print(huart, "FOC Triple Loop Test Complete\n");
}
