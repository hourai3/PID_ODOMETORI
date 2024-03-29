#include <iostream>
#include <cmath> // math.h の代わりに cmath を使用します
#include <mbed.h>

#define M_PI 3.141592

using namespace std;

typedef struct Pose_2D
{
    float X;
    float Y;
    float Yaw;
} Pose_2D;

void getOdometryDisplacement(const float r_left, const float r_right, Pose_2D &delta_odometry)
{
    const float d_width = 0.05;
    float rho_rotationCenter;
    float delta_l;

    if (r_left == r_right)
    {
        // output
        delta_odometry.X = r_left;
        delta_odometry.Y = 0.;
        delta_odometry.Yaw = 0.;
        return;
    }

    delta_odometry.Yaw = 0.5 * (r_right - r_left) / d_width;

    rho_rotationCenter = (r_left + r_right) / (r_right - r_left) * d_width;

    delta_l = 2. * rho_rotationCenter * sin(0.5 * delta_odometry.Yaw);

    // output
    delta_odometry.X = delta_l * cos(0.5 * delta_odometry.Yaw);
    delta_odometry.Y = delta_l * sin(0.5 * delta_odometry.Yaw);
    return;
}

void getPoseAfterOdometry(const Pose_2D delta_odometry, const Pose_2D pose_before_w, Pose_2D &pose_w)
{
    pose_w.X = pose_before_w.X + cos(pose_before_w.Yaw) * delta_odometry.X - sin(pose_before_w.Yaw) * delta_odometry.Y;
    pose_w.Y = pose_before_w.Y + sin(pose_before_w.Yaw) * delta_odometry.X + cos(pose_before_w.Yaw) * delta_odometry.Y;
    pose_w.Yaw = pose_before_w.Yaw + delta_odometry.Yaw;
    return;
}

int main()
{
    // variable
    float r_left, r_right;
    r_left = 0.4;
    r_right = 0.45;
    Pose_2D pose_before_w;
    pose_before_w.X = 1.;
    pose_before_w.Y = 0.;
    pose_before_w.Yaw = 30. * M_PI / 180.;

    // result variable
    Pose_2D pose_w;

    cout << "Pose: (X, Y, Yaw) = (";
    cout << pose_before_w.X << " ,";
    cout << pose_before_w.Y << ", ";
    cout << pose_before_w.Yaw << ")" << endl;
    cout << endl;

    cout << "Input(Odometory): (r_l, r_r) = (";
    cout << r_left << ", ";
    cout << r_right << ")" << endl;
    cout << endl;

    cout << "start calculation" << endl;
    cout << endl;
    Pose_2D delta_odometry;
    getOdometryDisplacement(r_left, r_right, delta_odometry);
    getPoseAfterOdometry(delta_odometry, pose_before_w, pose_w);

    cout << "Displacement(Odometory): (X, Y, Yaw) = (";
    cout << delta_odometry.X << " ,";
    cout << delta_odometry.Y << ", ";
    cout << delta_odometry.Yaw << ")" << endl;
    cout << endl;

    cout << "Pose: (X, Y, Yaw) = (";
    cout << pose_w.X << " ,";
    cout << pose_w.Y << ", ";
    cout << pose_w.Yaw << ")" << endl;
    cout << endl;

    return 0;
}
