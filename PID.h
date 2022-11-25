#ifndef _PID_H_
#define _PID_H_

// 定义PID结构体
class PID {
private:
    float error = 0;
    float lastError = 0;
    float integral = 0;
    float derivative = 0;
    float output = 0;

public:
    static constexpr float UNLIMITED = 1e6;
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float target = 0;
    // 积分分离
    float integralLimit = UNLIMITED;
    // 积分限幅
    float integralMax = UNLIMITED;
    // 输出限幅
    float outputMax = UNLIMITED;
    // 构造函数
    PID(float kp, float ki, float kd, float target, float outputMax = UNLIMITED,
        float integralLimit = UNLIMITED, float integralMax = UNLIMITED)
        : kp(kp),
          ki(ki),
          kd(kd),
          target(target),
          outputMax(outputMax),
          integralLimit(integralLimit),
          integralMax(integralMax) {}
    // PID重置函数
    void reset() {
        error = 0;
        lastError = 0;
        integral = 0;
        derivative = 0;
        output = 0;
    }
    // PID计算函数
    float calculate(float input) {
        error = target - input;
        integral += error;
        // 积分分离
        if (abs(error) > integralLimit) {
            integral = 0;
        }
        // 积分限幅
        if (abs(integral) > integralMax) {
            integral = integralMax * (integral > 0 ? 1 : -1);
        }
        derivative = error - lastError;
        output = kp * error + ki * integral + kd * derivative;
        // 输出限幅
        if (abs(output) > outputMax) {
            output = outputMax * (output > 0 ? 1 : -1);
        }
        lastError = error;
        return output;
    }
};

#endif  // !_PID_H_