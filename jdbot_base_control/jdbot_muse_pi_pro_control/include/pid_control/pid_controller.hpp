#ifndef MYPID_H 
#define MYPID_H

#include <math.h>

typedef struct
{
    /* 参数 */
    float kp;
    float ki;
    float kd;

    /* 运行状态 */
    float setpoint;     // 目标值
    float feedback;     // 测量值
    float err;
    float last_err;
    float integral;

    /* 输出 */
    float p_out;
    float i_out;
    float d_out;
    float output;

    /* 限制 */
    float i_limit;
    float out_limit;

    /* 时间 */
    float dt;

} PID_Controller;

void PID_Controller_Init(PID_Controller *pid,
              float kp, float ki, float kd,
              float dt,
              float i_limit,
              float out_limit);

// 普通PID
float PID_Update(PID_Controller *pid, float feedback);

// 前馈控制
float PID_FF_Update(PID_Controller *pid, float feedback, float pwm_ff);

// BangBang PID
float PID_BangBang_Update(PID_Controller *pid, float feedback);

#endif
