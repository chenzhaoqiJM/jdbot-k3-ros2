#pragma once
#include <lgpio.h>

class MotorDirectionCtrl
{
public:
    MotorDirectionCtrl();
    ~MotorDirectionCtrl();

    // 0: stop, 1: forward, 2: backward
    void motor1_direction(int direction);
    void motor2_direction(int direction);

private:
    int chip_;

    void claim_output(int gpio);
    void set_pair(int in1, int in2, int direction);
};
