#ifndef _LIGHTSENSOR_H_
#define _LIGHTSENSOR_H_

// 定义光照传感器结构体
class LightSensor {
private:
    // 光照传感器值
    float value = 0;

public:
    // 光照传感器引脚
    uint8_t pin;
    // 光照传修正系数
    float k;
    // 光照传感器偏移量
    float b;
    LightSensor(uint8_t pin, float k = 1, float b = 0) : pin(pin), k(k), b(b) {}
    // 光照传感器初始化函数
    void init() { pinMode(pin, INPUT); }
    // 光照传感器读取函数
    float read() {
        value = analogRead(pin) * k + b;
        return value;
    }
};

#endif  // !_LIGHTSENSOR_H_