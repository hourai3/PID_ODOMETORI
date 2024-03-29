#include "mbed.h"
#include "PID.hpp"

int suuti = 0;
int gohan = 0;
int sokudo = 0;
int mokuhyou = 5000;

BufferedSerial pc(USBTX, USBRX, 115200);
CAN can(PA_11, PA_12, (int)1e6);
int16_t pwm[4] = {0, 0, 0, 0}; // モタドラ
uint8_t DATA[8] = {};

// PID controller parameters
const float kp = 0.01;
const float ki = 0.016;
const float kd = 0.001;
const float sample_time = 0.02; // 20ms sample time

// Create PID controller
PID pid_controller(kp, ki, kd, sample_time);

//////////////オドメトリ//////////////////////////

// ロリコンのピン
InterruptIn rorikon1_A(PA_0);
DigitalIn rorikon1_B(PA_1);
InterruptIn rorikon2_A(PA_2);
DigitalIn rorikon2_B(PA_3);

// ロボットのパラメータ
const float wheel_diameter = 0.1; // オムニホイールの直径（単位はメートル）
const float wheel_distance = 0.2; // オムニホイールホイールの間隔（単位はメートル）

// ロリコンの状態
volatile int count1 = 0;
volatile int count2 = 0;

// ロリコンの割り込み処理
void rorikon1_isr()
{
    if (rorikon1_A == rorikon1_B)
    {
        count1++;
    }
    else
    {
        count1--;
    }
}

void rorikon2_isr()
{
    if (rorikon2_A == rorikon2_B)
    {
        count2++;
    }
    else
    {
        count2--;
    }
}

int main()
{
    // 割り込みの設定
    rorikon1_A.rise(&rorikon1_isr);
    rorikon1_A.fall(&rorikon1_isr);
    rorikon2_A.rise(&rorikon2_isr);
    rorikon2_A.fall(&rorikon2_isr);

    // オドメトリの変数
    float x = 0.0; // X座標
    float y = 0.0; // Y座標
    float θ = 0.0; // 角度

    // ロリコンのカウント値
    int prev_count1 = count1;
    int prev_count2 = count2;

    while (1)
    {
        CANMessage msg;
        if (pc.readable())
        {
            char buf;
            pc.read(&buf, sizeof(buf));
            if (buf == 'w')
            {
                mokuhyou = 5000;
            }
            else if (buf == 's')
            {
                mokuhyou = 1000;
            }
            else if (buf == 'z')
            {
                mokuhyou = 0;
            }
        }

        // Calculate PID output
        float output = pid_controller.calculate(mokuhyou, sokudo);

        int8_t output_int8 = static_cast<int8_t>(output);
        pwm[0] = output_int8;

        CANMessage msg0(1, (const uint8_t *)pwm, 8);
        can.write(msg0);
        if (can.read(msg))
        {
            sokudo = (msg.data[8]);
            printf("sokudo : %d\n", sokudo);
        }

        // 変化量
        int delta_count1 = count1 - prev_count1;
        int delta_count2 = count2 - prev_count2;

        // 移動距離の計算
        float distance1 = delta_count1 * (3.14159 * wheel_diameter) / 2048.0; // 変化量 × (3.14159 × オムニホイールの直径) ÷ 2048.0
        float distance2 = delta_count2 * (3.14159 * wheel_diameter) / 2048.0;
        float distance = (distance1 + distance2) / 2.0;

        // 位置と角度の更新
        x += distance * cos(θ);
        y += distance * sin(θ);
        θ += (distance1 - distance2) / wheel_distance;

        // カウント値の更新
        prev_count1 = count1;
        prev_count2 = count2;

        // ちょっとだけ待つ
        ThisThread::sleep_for(10ms);
    }
}
