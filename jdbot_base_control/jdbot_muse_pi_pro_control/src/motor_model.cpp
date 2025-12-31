#include "jdbot_muse_pi_pro_control/motor_model.hpp"

namespace motor_model
{
    double motor1_model(int dir, double speed)
    {
        if (dir == 0) return 0.0;

        if (dir == 1)  // forward
        {
            constexpr double k = 0.2781;
            constexpr double b = 0.0233;
            return k * speed + b;
        }
        else           // backward
        {
            constexpr double k = 0.2549;
            constexpr double b = 0.0306;
            return k * speed + b;
        }
    }

    double motor2_model(int dir, double speed)
    {
        if (dir == 0) return 0.0;

        if (dir == 1)  // forward
        {
            constexpr double k = 0.2542;
            constexpr double b = 0.0612;
            return k * speed + b;
        }
        else           // backward
        {
            constexpr double k = 0.2829;
            constexpr double b = 0.0359;
            return k * speed + b;
        }
    }
}
