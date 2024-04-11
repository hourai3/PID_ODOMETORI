
#include "mbed.h"
#include "PID.hpp"

// グローバル変数
int8_t pwm[4] = {0, 0, 0, 0};
uint8_t DATA[8] = {};
int count_hozon[4] = {};     // ロリコンのカウント合計値
int count_hozon_pre[4] = {}; // ロリコンの前回のカウント合計値

// PIDパラメーター
const float kp = 0.01;
const float ki = 0.016;
const float kd = 0.001;
const float sample_time = 0.02; // 20ms sample time

// PIDコントローラー
PID pid_controller(kp, ki, kd, sample_time);

// ロボットのパラメータ
const float wheel_diameter = 0.8;       // オムニホイールの直径
const int encoder_resolution = 256 * 4; // 256パルス * 4つ

// PID制御で使用する変数
float Omni_x_PID = 0;
float Omni_y_PID = 0;
float Omni_theta_PID = 0;

// オドメトリ関数
void Odometry(float &x_mtere, float &y_mtere, float &theta)
{
    float dx = 0;
    float dy = 0;
    float dtheta_rad = 0 * M_PI / 180;
    constexpr float M_4 = M_PI / 4; // エンコーダが4つあるので4で割る
    // x,y,θの変位を求める
    for (int i = 0; i < 4; ++i)
    {
        dx += (count_hozon[i] - count_hozon_pre[i]) * cos((2 * i) * M_4 + theta); //(2 * i)に +1
        dy += (count_hozon[i] - count_hozon_pre[i]) * sin((2 * i) * M_4 + theta); //(2 * i)に +1
        dtheta_rad += count_hozon[i] - count_hozon_pre[i];
    }
    // 現在の位置を更新
    x_mtere += dx * wheel_diameter / encoder_resolution;
    y_mtere += dy * wheel_diameter / encoder_resolution;
    theta += 2 * M_PI / -30000 * dtheta_rad;

    // 現在の回転数を更新する
    for (int i = 0; i < 4; ++i)
    {
        count_hozon_pre[i] = count_hozon[i];
    }
}

// PIDの計算
void PID_calc(float target_x, float x_mtere, float target_y, float y_mtere, float target_theta, float theta)
{
    float Omni_x_PID = pid_controller.calculate(target_x, x_mtere); // オムニの速度を出す
    float Omni_y_PID = pid_controller.calculate(target_y, y_mtere);
    float Omni_theta_PID = pid_controller.calculate(target_theta, theta);
    // printf("output : x = % 4.2f, y = %4.2f , theta = %4.2f\n", Omni_x_PID, Omni_y_PID, Omni_theta_PID);
}

// オムニホイールのPWM出力を計算する関数
void Omni(float Omni_x_PID, float Omni_y_PID, float Omni_theta_PID, int8_t pwm[4])
{
    float r = hypot(Omni_x_PID, Omni_y_PID);
    float theta_2 = atan2(Omni_y_PID, Omni_x_PID);
    //                           |< + piteh(姿勢角)
    pwm[3] = r * cos(theta_2 + (45 * M_PI / 180)) + Omni_theta_PID;
    pwm[2] = r * cos(theta_2 - (45 * M_PI / 180)) + Omni_theta_PID;
    pwm[1] = r * cos(theta_2 - (125 * M_PI / 180)) + Omni_theta_PID;
    pwm[0] = r * cos(theta_2 - (225 * M_PI / 180)) + Omni_theta_PID;
}

int main()
{
    // 初期化
    BufferedSerial pc(USBTX, USBRX, 115200);
    CAN can(PA_11, PA_12, (int)1e6);
    CANMessage msg;
    float x_mtere = 0.0;                 // X座標
    float y_mtere = 0.0;                 // Y座標
    float theta = 0 * M_PI / 180;        // θ座標
    float target_x = 3.0;                // 目標のx座標
    float target_y = 3.0;                // 目標のy座標
    float target_theta = 0 * M_PI / 180; // 目標のθ座標

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
        PID_calc(target_x, x_mtere, target_y, y_mtere, target_theta, theta);

        // オムニ
        Omni(Omni_x_PID, Omni_y_PID, Omni_theta_PID, pwm);

        // CANMessage msg(1, (const uint8_t *)pwm, 8);
        // can.write(msg);

        if (now - pre > 50ms)
        {
            // printf("output : x = % 4.2f, y = %4.2f , theta = %4.2f\n", x_mtere, y_mtere, theta);//現在地
            // printf("output : x = % 4.2f, y = %4.2f , theta = %4.2f\n", Omni_x_PID, Omni_y_PID, Omni_theta_PID);//PID計算結果

            printf("pwm: pwm[0] = % 4.2d , pwm [1] = % 4.2d , pwm[2] = % 4.2d , pwm[3] = % 4.2d\n", pwm[0], pwm[1], pwm[2], pwm[3]); // pwmの値
            pre = now;
        }
    }
}
