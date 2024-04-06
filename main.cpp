#include "mbed.h"
#include "PID.hpp"

// グローバル変数
int16_t pwm[4] = {0, 0, 0, 0};
uint8_t DATA[8] = {};
volatile int count_hozon[4] = {};   // ロリコンのカウント合計値
volatile int rorikon_total_pre = 0; // ロリコンの前回のカウント合計値
int encoder_resolution = 1020;      // 1020パルス

// PIDパラメーター
const float kp = 0.01;
const float ki = 0.016;
const float kd = 0.001;
const float sample_time = 2.00; // 2000ms sample time

// PIDコントローラー
PID pid_controller(kp, ki, kd, sample_time);

// ロボットのパラメータ
const float wheel_diameter = 0.05;  // オムニホイールの直径
const float wheel_distance = 0.420; // オムニホイールホイールの間隔

// オドメトリ関数
void Odometry(float &x, float &y, float &theta)
{
    // ロリコンのカウント合計
    int rorikon_total_now = count_hozon[0] + count_hozon[1] + count_hozon[2] + count_hozon[3];

    // ロリコンのカウントの変化分
    int rorikon_delta = rorikon_total_now - rorikon_total_pre;
    printf("count_hozon[i]: %d\n", rorikon_delta);

    // ロリコンのカウント合計値をラジアンに変換
    float theta_delta = (2 * M_PI * rorikon_delta) / encoder_resolution;
    rorikon_total_pre = rorikon_delta;

    // θの変化分を現在のθに追加
    theta += theta_delta;

    // xとyの変化分を計算
    float delta_x = rorikon_delta * wheel_diameter / (4 * M_PI); // 四輪オムニの場合、4を車輪の数で置き換える
    float delta_y = 0.0;                                         // この値はロボットの運動によって異なります

    // 現在の位置を更新
    x += delta_x * cos(theta) - delta_y * sin(theta);
    y += delta_x * sin(theta) + delta_y * cos(theta);
}

int main()
{
    // 初期化
    BufferedSerial pc(USBTX, USBRX, 250000);
    CAN can(PA_11, PA_12, (int)1e6);
    CANMessage msg;

    float x = 0.0;          // X座標
    float y = 0.0;          // Y座標
    float theta = 0.0;      // θ座標
    float target_x = 0.3;   // 目標のx座標
    float target_y = 0.3;   // 目標のy座標
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
                // printf("count_hozon[i]: %d\n", count_hozon[i]);
            }

        // オドメトリ計算
        Odometry(x, y, theta);

        // PID出力の計算
        float error_x = target_x - x; // 目標値との差
        float error_y = target_y - y;
        float error_theta = target_theta - theta;

        // X軸とY軸方向のPID計算
        float output_x = pid_controller.calculate(0, error_x);
        float output_y = pid_controller.calculate(0, error_y);
        float output_theta = pid_controller.calculate(0, error_theta);

        // オムニ
        //  PWM出力を設定

        // デバッグ情報の出力
        if (now - pre > 50ms)
        {
            // printf("Current Position: x = %.2f, y = %.2f\n", x, y);
            pre = now;
        }

        // 目標位置に到達したらループを抜ける
        if (fabs(target_x - x) < 0.01 && fabs(target_y - y) < 0.01 && fabs(target_theta - theta) < 0.01)
        {
            break;
        }
        CANMessage msg(1, (const uint8_t *)pwm, 8);
        can.write(msg);
    }
}
