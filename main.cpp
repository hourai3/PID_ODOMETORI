
#include "mbed.h"
#include "PID.hpp"

// グローバル変数
int8_t pwm[4] = {0, 0, 0, 0};
uint8_t DATA[8] = {};
int count_hozon[4] = {};                // ロリコンのカウント合計値
int count_hozon_pre[4] = {};            // ロリコンの前回のカウント合計値
const int encoder_resolution = 256 * 4; // 255パルス

// PIDパラメーター
const float kp = 0.01;
const float ki = 0.016;
const float kd = 0.001;
const float sample_time = 0.02; // 2000ms sample time

// PIDコントローラー
PID pid_controller(kp, ki, kd, sample_time);

// ロボットのパラメータ
const float wheel_diameter = 0.8; // オムニホイールの直径

// オドメトリ関数
void Odometry(float &x_mtere, float &y_mtere, float &theta)
{
    float dx = 0;
    float dy = 0;
    float dtheta_rad = 0;
    constexpr float M_4 = M_PI / 4;
    // x,y,θの変位を求める
    for (int i = 0; i < 4; ++i)
    {
        dx += (count_hozon[i] - count_hozon_pre[i]) * cos((2 * i) * M_4 + theta);
        dy += (count_hozon[i] - count_hozon_pre[i]) * sin((2 * i) * M_4 + theta);
        dtheta_rad += count_hozon[i] - count_hozon_pre[i];
    }
    // 現在の位置を更新
    x_mtere += dx * wheel_diameter / encoder_resolution;
    y_mtere += dy * wheel_diameter / encoder_resolution;
    theta += 2 * M_PI / -30000 * dtheta_rad;
    // printf("output : x = % 4.2f, y = %4.2f , theta = %4.2f\n", x_mtere, y_mtere, theta);

    // 現在の回転数を更新する
    for (int i = 0; i < 4; ++i)
    {
        count_hozon_pre[i] = count_hozon[i];
    }
}

// オムニホイールのPWM出力を計算する関数
void Omni(float Omni_x_speed, float Omni_y_speed, float Omni_theta_speed, int8_t pwm[4])
{
    float r = hypot(Omni_x_speed, Omni_y_speed);
    float theta_2 = atan2(Omni_y_speed, Omni_x_speed);
    //                           |< + piteh
    // pwm[3] = r * cos(theta_2 + 45) + Omni_theta_speed;
    // pwm[2] = r * cos(theta_2 - 45) + Omni_theta_speed;
    // pwm[1] = r * cos(theta_2 - 125) + Omni_theta_speed;
    // pwm[0] = r * cos(theta_2 - 225) + Omni_theta_speed;

    // printf("pwm: pwm[0] = %d , pwm [1] = %d , pwm[2] = %d , pwm[3] = %d\n", pwm[0], pwm[1], pwm[2], pwm[3]);
}

int main()
{
    // 初期化
    BufferedSerial pc(USBTX, USBRX, 115200);
    CAN can(PA_11, PA_12, (int)1e6);
    CANMessage msg;
    float x_mtere = 0.0;    // X座標
    float y_mtere = 0.0;    // Y座標
    float theta = 0.0;      // θ座標
    float target_x = 3.0;   // 目標のx座標
    float target_y = 3.0;   // 目標のy座標
    float target_theta = 0; // 目標のθ座標

    while (1)
    {
        auto now = HighResClock::now();
        static auto pre = now;

        if (can.read(msg) && msg.id == 48)
            for (int i = 0; i < 4; i += 1)
            {
                signed short data_value = (msg.data[2 * i + 1] << 8) | msg.data[2 * i];
                count_hozon[i] += data_value;
            }

        // オドメトリ計算
        Odometry(x_mtere, y_mtere, theta);

        // PID
        float Omni_x_speed = pid_controller.calculate(target_x, x_mtere); // オムニの速度を出す
        float Omni_y_speed = pid_controller.calculate(target_y, y_mtere);
        float Omni_theta_speed = pid_controller.calculate(target_theta, theta);
        // printf("output : x = % 4.2f, y = %4.2f , theta = %4.2f\n", Omni_x_speed, Omni_y_speed, Omni_theta_speed);

        // オムニ
        Omni(Omni_x_speed, Omni_y_speed, Omni_theta_speed, pwm);
        CANMessage msg(1, (const uint8_t *)pwm, 8);
        can.write(msg);

        if (now - pre > 500ms)
        {
            printf("output : x = % 4.2f, y = %4.2f , theta = %4.2f\n", x_mtere, y_mtere, theta);
            printf("pwm: pwm[0] = % 4.2d , pwm [1] = % 4.2d , pwm[2] = % 4.2d , pwm[3] = % 4.2d\n", pwm[0], pwm[1], pwm[2], pwm[3]);
            printf("output : x = % 4.2f, y = %4.2f , theta = %4.2f\n", Omni_x_speed, Omni_y_speed, Omni_theta_speed);
            pre = now;
        }
    }
}
// #include "mbed.h"

// BufferedSerial pc(USBTX, USBRX, 115200); // パソコンとのシリアル通信
// CANMessage msg;

// CAN can1(PA_11, PA_12, (int)1e6);

// int16_t pwm0[4] = {0, 0, 0, 0}; // モタドラ1

// int main()
// {

//     while (1) 
//     {

//         if (pc.readable())
//         {
//             char buf;
//             pc.read(&buf, sizeof(buf));
//             if (buf == '1') // 左
//             {
//                 pwm[0] = 0;
//                 pwm[1] = 0;
//                 pwm[2] = 0;
//                 pwm[3] = 0;
//             }
//             CANMessage msg0(1, (const uint8_t *)pwm0, 8);
//             can1.write(msg0);
//         }
//     }
// }
