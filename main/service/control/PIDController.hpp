#pragma once

namespace minicar::service {

class PIDController {
public:
    PIDController(float kp, float ki, float kd);

    float update(float setpoint, float measurement, float dt_s);
    void reset();

    void setOutputLimit(float min_output, float max_output);
    void setIntegralLimit(float min_integral, float max_integral);

private:
    float kp_{0.0f};
    float ki_{0.0f};
    float kd_{0.0f};

    float min_output_{-1.0f};
    float max_output_{1.0f};
    float min_integral_{-1.0f};
    float max_integral_{1.0f};

    float integral_{0.0f};
    float previous_error_{0.0f};
    bool has_previous_error_{false};
};

}  // namespace minicar::service
