
#include "pid_controller.hpp"

void PID_Controller_Init(PID_Controller *pid,
              float kp, float ki, float kd,
              float dt,
              float i_limit,
              float out_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->dt = dt;
    pid->i_limit = i_limit;
    pid->out_limit = out_limit;

    pid->setpoint = 0;
    pid->feedback = 0;
    pid->err = 0;
    pid->last_err = 0;
    pid->integral = 0;
    pid->output = 0;
}

// 普通PID
float PID_Update(PID_Controller *pid, float feedback)
{
    pid->feedback = feedback;
    pid->err = pid->setpoint - feedback;

    /* P */
    pid->p_out = pid->kp * pid->err;

    /* I（积分分离 + 限幅） */
    pid->integral += pid->err * pid->dt;
    if(pid->integral > pid->i_limit)  pid->integral = pid->i_limit;
    if(pid->integral < -pid->i_limit) pid->integral = -pid->i_limit;
    pid->i_out = pid->ki * pid->integral;

    /* D */
    pid->d_out = pid->kd * (pid->err - pid->last_err) / pid->dt;

    pid->output = pid->p_out + pid->i_out + pid->d_out;

    /* 输出限幅 */
    if(pid->output > pid->out_limit)  pid->output = pid->out_limit;
    if(pid->output < -pid->out_limit) pid->output = pid->out_limit;

    pid->last_err = pid->err;
	
	if (pid->output < 0) pid->output = 0;

    return pid->output;
}

// 前馈 + PID
float PID_FF_Update(PID_Controller *pid, float feedback, float pwm_ff)
{
    pid->feedback = feedback;
    pid->err = pid->setpoint - feedback;

    /* ---------- PID 计算 ---------- */
    /* P */
    pid->p_out = pid->kp * pid->err;

    /* I（积分分离 + 限幅） */
    pid->integral += pid->err * pid->dt;
    if(pid->integral > pid->i_limit)  pid->integral = pid->i_limit;
    if(pid->integral < -pid->i_limit) pid->integral = -pid->i_limit;
    pid->i_out = pid->ki * pid->integral;

    /* D */
    pid->d_out = pid->kd * (pid->err - pid->last_err) / pid->dt;

    pid->output = pid->p_out + pid->i_out + pid->d_out;

    pid->last_err = pid->err;

    /* ---------- 合成 ---------- */
    pid->output = pwm_ff + pid->output;

    /* -------- 输出限幅 --------- */
    if(pid->output > pid->out_limit)  pid->output = pid->out_limit;
	if (pid->output < 0) pid->output = 0;

    return pid->output;
}


// 极限变化
float PID_BangBang_Update(PID_Controller *pid, float feedback)
{
    pid->feedback = feedback;
    pid->err = pid->setpoint - feedback;

    if (fabsf(pid->err) > 0.5) {
        if(pid->err > 0) {
            pid->output = pid->out_limit;
        }
        else{
            pid->output = 1;
        }
        return pid->output;
    } 

    /* P */
    pid->p_out = pid->kp * pid->err;

    /* I（积分分离 + 限幅） */
    pid->integral += pid->err * pid->dt;
    if(pid->integral > pid->i_limit)  pid->integral = pid->i_limit;
    if(pid->integral < -pid->i_limit) pid->integral = -pid->i_limit;
    pid->i_out = pid->ki * pid->integral;

    /* D */
    pid->d_out = pid->kd * (pid->err - pid->last_err) / pid->dt;

    pid->output = pid->p_out + pid->i_out + pid->d_out;

    /* 输出限幅 */
    if(pid->output > pid->out_limit)  pid->output = pid->out_limit;
    if(pid->output < -pid->out_limit) pid->output = pid->out_limit;

    pid->last_err = pid->err;
	
	if (pid->output < 0) pid->output = 0;

    return pid->output;
}
