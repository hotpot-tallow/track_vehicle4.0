//
// Created by xyl on 24-4-3.
//

#ifndef SRC_D_LG_EKF_H
#define SRC_D_LG_EKF_H

#define Deg2Rad M_PI / 180.0

#include "Eigen/Dense"
#include <vector>


//四元数 --> 欧拉角(Z-Y-X，即RPY)（确保pitch的范围[-pi/2, pi/2]）
inline Eigen::Vector3d Quaternion2EulerAngles(Eigen::Quaterniond q) {
    Eigen::Vector3d angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles(2) = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

inline Eigen::Vector3d Rotation2EulerAngles(Eigen::Matrix3d R) {
//旋转矩阵 --> 欧拉角(Z-Y-X，即RPY)（确保pitch的范围[-pi/2, pi/2]）
    Eigen::Vector3d eulerAngle_mine;
    Eigen::Matrix3d rot = std::move(R);
    eulerAngle_mine(0) = std::atan2(rot(2, 1), rot(2, 2));
    eulerAngle_mine(1) = std::atan2(-rot(2, 0), std::sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2)));
    eulerAngle_mine(2) = std::atan2(rot(1, 0), rot(0, 0));

    return eulerAngle_mine;
}

inline Eigen::Matrix3d BuildSkewSymmetricMatrix3(const Eigen::Vector3d &vec) {
    Eigen::Matrix3d matrix;
    matrix << 0.0, -vec(2), vec(1),
            vec(2), 0.0, -vec(0),
            -vec(1), vec(0), 0.0;

    return matrix;
}

inline Eigen::Matrix3d deltaSO3Exp(const Eigen::Vector3d &v) {
    Eigen::Matrix3d R;
    double theta = v.norm();
    Eigen::Vector3d v_normalized = v.normalized();
    R = cos(theta) * Eigen::Matrix3d::Identity()
        + (1.0 - cos(theta)) * v_normalized * v_normalized.transpose()
        + std::sin(theta) * BuildSkewSymmetricMatrix3(v_normalized);
    return R;
}
inline Eigen::Matrix3d Rotation_X(const double _angle)
{
    Eigen::Matrix3d R;
    double sin_a = sin(_angle);
    double cos_a = cos(_angle);
    R << 1, 0, 0,
        0, cos_a, -sin_a,
        0, sin_a, cos_a;
    return R;
}
inline Eigen::Matrix3d Rotation_Y(const double _angle)
{
    Eigen::Matrix3d R;
    double sin_a = sin(_angle);
    double cos_a = cos(_angle);
    R << cos_a, 0, sin_a,
        0, 1, 0,
        -sin_a, 0, cos_a;
    return R;
}
inline Eigen::Matrix3d Rotation_Z(const double _angle)
{
    Eigen::Matrix3d R;
    double sin_a = sin(_angle);
    double cos_a = cos(_angle);
    R << cos_a, -sin_a, 0,
        sin_a, cos_a, 0,
        0, 0, 1;
    return R;
}
class ESKF{
private:
    std::vector<Eigen::Vector3d> imu_gyro_buffer;

    typedef Eigen::Matrix<double,18,18> Matrix18d;
    typedef Eigen::Matrix<double,6,6> Matrix6d;
    typedef Eigen::Matrix<double,6,18> Matrix6_18d;
    typedef Eigen::Matrix<double,18,6> Matrix18_6d;
    typedef Eigen::Matrix<double,18,1> Vector18d;
    typedef Eigen::Matrix<double,6,1> Vector6d;


    double T;//sample time

    Matrix18d F_x;//状态转移矩阵
    Matrix18d P_k;//
    Matrix6_18d H_k;//观测矩阵
    Matrix18_6d K_k;//卡尔曼增益

    Matrix6d R;//观测噪声协方差矩阵
    Matrix18d Q;//过程噪声协方差矩阵
    Eigen::Vector3d offset_camera2imu;

    Vector18d error_x;//状态误差
    Vector6d error_z;//观测误差

    Eigen::Vector3d normal_x_pos;//位置向量
    Eigen::Matrix3d normal_x_Rota;//旋转向量
    Eigen::Vector3d normal_x_vec;//速度向量
    Eigen::Vector3d normal_x_ba;//加速度计偏置
    Eigen::Vector3d normal_x_bg;//陀螺仪偏置
    Eigen::Vector3d normal_x_gvt;//重力向量

    Eigen::Vector3d last_imu_acc;//上一次IMU加速度
    Eigen::Vector3d last_imu_gyro;//上一次IMU陀螺仪读数

public:
    ESKF(double sample_time);
    ~ESKF() = default;
    void initESKF();
    void setInitState(Eigen::Vector3d &_pos, Eigen::Quaterniond &_q);
    void setCameraOffset();
    void predictUpdate(Eigen::Vector3d &_accel, Eigen::Vector3d &_gyro);
    void measureCorrect(Eigen::Vector3d &_pos, Eigen::Quaterniond &_q);
    void eliminateError();

    void getPosition(Eigen::Vector3d &_pos);
    void getRotation(Eigen::Matrix3d &_rata);
    void getCameOffset(Eigen::Vector3d &_pos);

    void setMatrixR();
    void setMatrixQ();
    bool addImuData(Eigen::Vector3d &_accel, Eigen::Vector3d &_gyro);
};


#endif //SRC_D_LG_EKF_H
