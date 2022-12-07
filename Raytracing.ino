#include <Servo.h>

#include "ByteRead.h"
#include "LightSensor.h"
#include "PID.h"

// 宏函数，用于将布尔值转换为字符串
#define BOOL2STR(x) ((x) ? "true" : "false")

constexpr int flagAddr = 0;
constexpr int paramsAddr = 1;

bool debug = false;
bool verticalControl = true;
bool horizontalControl = true;
long lightThreshold = 600;

// 定义横向纵向两个舵机，舵机的引脚分别为9和10，型号为SG90 180°舵机
Servo servoX;
Servo servoY;
int angleX = 90;
int angleY = 90;
// 定义上下左右四个光照传感器
LightSensor lightSensorUp(A0, 1, 0);
LightSensor lightSensorDown(A1, 1.19, 0);
LightSensor lightSensorLeft(A2, 1, 0);
LightSensor lightSensorRight(A3, 2.31, -500);
// 定义PID结构体变量
PID pidX(0.010000, 0.000200, 0.010000, 0, 5, 250, 5);
PID pidY(0.030000, 0.000500, 0.050000, 0, 5, 250, 5);

void setup() {
    // 初始化串口
    Serial.begin(115200);
    Serial.setTimeout(2);
    // 初始化两个舵机，脉冲宽度范围为500-2500us
    servoX.attach(9, 500, 2500);
    servoY.attach(10, 500, 2500);
    // 设置舵机的初始状态为90°
    servoX.write(angleX);
    servoY.write(angleY);
    // 初始化光照传感器
    lightSensorUp.init();
    lightSensorDown.init();
    lightSensorLeft.init();
    lightSensorRight.init();
    if (EEPROM.read(flagAddr) == 'T') {
        // 如果EEPROM中有数据则从中读取参数
        Serial.println("{\"msg\":\"Read parameters from EEPROM\"}");
        readParams();
    } else {
        // 否则将参数写入EEPROM，并将标志设置为'T'
        Serial.println("{\"msg\":\"Write parameters to EEPROM\"}");
        writeParams();
        EEPROM.write(flagAddr, 'T');
    }
    // 输出当前光照传感器的参数
    printParams();
}

void loop() {
    readCmd();
    raytracing();
}

void raytracing() {
    // 读取光照传感器的值
    float lightUp = lightSensorUp.read();
    float lightDown = lightSensorDown.read();
    float lightLeft = lightSensorLeft.read();
    float lightRight = lightSensorRight.read();
    // 如果四个光照传感器的值都大于阈值，则停止舵机
    if (lightUp > lightThreshold && lightDown > lightThreshold &&
        lightLeft > lightThreshold && lightRight > lightThreshold) {
        delay(50);
        return;
    }
    // 计算光照传感器的偏差
    float errorX = lightLeft - lightRight;
    float errorY = lightUp - lightDown;
    // 计算舵机的转速
    int speedX = pidX.calculate(errorX);
    int speedY = pidY.calculate(errorY);
    // 转动舵机
    angleX += speedX;
    if (angleX < 0) angleX = 0;
    if (angleX > 180) angleX = 180;
    angleY += speedY;
    if (angleY < 0) angleY = 0;
    if (angleY > 180) angleY = 180;
    if (horizontalControl) servoX.write(angleX);
    if (verticalControl) servoY.write(angleY);

    if (debug) {
        static int cnt = 0;
        if (++cnt >= 50) {
            cnt = 0;
            char str[30];
            sprintf(str, "{\"errorX\":%d,\"errorY\":%d,", (int)(errorX),
                    (int)(errorY));
            Serial.print(str);
            sprintf(str,
                    "\"Up\":%ld,\"Down\":%ld,\"Left\":%ld,\"Right\":%ld}\n",
                    (long)lightUp, (long)lightDown, (long)lightLeft,
                    (long)lightRight);
            Serial.print(str);
        }
    }
    delay(20);
}

void readCmd() {
    if (!Serial.available()) {
        return false;
    }
    char cmd = Serial.read();
    switch (cmd) {
        case '-':  // Write X Angle
            angleX = Serial.parseInt();
            servoX.write(angleX);
            Serial.println("{\"angleX\":" + String(angleX) + "}");
            break;
        case '|':  // Write Y Angle
            angleY = Serial.parseInt();
            servoY.write(angleY);
            Serial.println("{\"angleY\":" + String(angleY) + "}");
            break;
        case '#':  // Read State
            Serial.println("{\"angleX\":" + String(angleX) +
                           ",\"angleY\":" + String(angleY) +
                           ",\"Horizontal\":" + BOOL2STR(horizontalControl) +
                           ",\"Vertical\":" + BOOL2STR(verticalControl) +
                           ",\"Debug\":" + BOOL2STR(debug) + "}");
            break;
        case 'H':  // Horizontal
            horizontalControl = !horizontalControl;
            Serial.println(String("{\"Horizontal\":") + BOOL2STR(horizontalControl) +
                           "}");
            break;
        case 'V':  // Vertical
            verticalControl = !verticalControl;
            Serial.println(String("{\"Vertical\":") + BOOL2STR(verticalControl) + "}");
            break;
        case '!':  // Debug
            debug = !debug;
            Serial.println(String("{\"Debug\":") + BOOL2STR(debug) + "}");
            break;
        case 'D':  // Down
            lightSensorDown.k = Serial.parseFloat();
            lightSensorDown.b = Serial.parseFloat();
            Serial.println("{\"k\":" + String(lightSensorDown.k) +
                           ",\"b\":" + String(lightSensorDown.b) + "}");
            break;
        case 'R':  // Rright
            lightSensorRight.k = Serial.parseFloat();
            lightSensorRight.b = Serial.parseFloat();
            Serial.println("{\"k\":" + String(lightSensorRight.k) +
                           ",\"b\":" + String(lightSensorRight.b) + "}");
            break;
        case 'X':  // pidX
            pidX.kp = Serial.parseFloat();
            pidX.ki = Serial.parseFloat();
            pidX.kd = Serial.parseFloat();
            Serial.println("{\"kp\":" + String(pidX.kp, 6) +
                           ",\"ki\":" + String(pidX.ki, 6) +
                           ",\"kd\":" + String(pidX.kd, 6) + "}");
            break;
        case 'Y':  // pidX
            pidY.kp = Serial.parseFloat();
            pidY.ki = Serial.parseFloat();
            pidY.kd = Serial.parseFloat();
            Serial.println("{\"kp\":" + String(pidY.kp, 6) +
                           ",\"ki\":" + String(pidY.ki, 6) +
                           ",\"kd\":" + String(pidY.kd, 6) + "}");
            break;
        case 'T':  // Threshold
            lightThreshold = Serial.parseInt();
            Serial.println("{\"Threshold\":" + String(lightThreshold) + "}");
            break;
        case 'P':  // Print
            printParams();
            break;
        case 'S':  // Save
            writeParams();
            Serial.println("{\"msg\":\"Saved\"}");
            break;
        case 'C':  // Clear
            EEPROM.write(flagAddr, 0);
            Serial.println("{\"msg\":\"Cleared, restarting...\"}");
            delay(1000);
            // 调用0地址的函数，重启
            ((void (*)())0)();
            break;
    }
}

void readParams() {
#define READ_PARAM(param)                          \
    do {                                           \
        (param) = ByteRead<decltype(param)>(addr); \
        addr += sizeof(param);                     \
    } while (0)
    int addr = paramsAddr;
    // 读取光照传感器的参数
    READ_PARAM(lightSensorDown.k);
    READ_PARAM(lightSensorDown.b);
    READ_PARAM(lightSensorRight.k);
    READ_PARAM(lightSensorRight.b);
    // 读取PID的参数
    READ_PARAM(pidX.kp);
    READ_PARAM(pidX.ki);
    READ_PARAM(pidX.kd);

    READ_PARAM(pidY.kp);
    READ_PARAM(pidY.ki);
    READ_PARAM(pidY.kd);
    // 读取阈值
    READ_PARAM(lightThreshold);
#undef READ_PARAM
}

void writeParams() {
#define WRITE_PARAM(param)                         \
    do {                                           \
        ByteWrite<decltype(param)>(addr, (param)); \
        addr += sizeof(param);                     \
    } while (0)
    int addr = paramsAddr;
    // 写入光照传感器的参数
    WRITE_PARAM(lightSensorDown.k);
    WRITE_PARAM(lightSensorDown.b);
    WRITE_PARAM(lightSensorRight.k);
    WRITE_PARAM(lightSensorRight.b);
    // 写入PID的参数
    WRITE_PARAM(pidX.kp);
    WRITE_PARAM(pidX.ki);
    WRITE_PARAM(pidX.kd);

    WRITE_PARAM(pidY.kp);
    WRITE_PARAM(pidY.ki);
    WRITE_PARAM(pidY.kd);
    // 写入阈值
    WRITE_PARAM(lightThreshold);
#undef WRITE_PARAM
}

void printParams() {
    // 输出当前光照传感器的参数
    Serial.print("{\"Down\":{\"k\":" + String(lightSensorDown.k) +
                 ",\"b\":" + String(lightSensorDown.b) + "},");
    Serial.print("\"Right\":{\"k\":" + String(lightSensorRight.k) +
                 ",\"b\":" + String(lightSensorRight.b) + "},");
    // 输出当前PID的参数
    Serial.print("\"pidX\":{\"kp\":" + String(pidX.kp, 6) + ",\"ki\":" +
                 String(pidX.ki, 6) + ",\"kd\":" + String(pidX.kd, 6) + "},");
    Serial.print("\"pidY\":{\"kp\":" + String(pidY.kp, 6) + ",\"ki\":" +
                 String(pidY.ki, 6) + ",\"kd\":" + String(pidY.kd, 6) + "},");
    // 输出当前阈值
    Serial.println("\"Threshold\":" + String(lightThreshold) + "}");
}