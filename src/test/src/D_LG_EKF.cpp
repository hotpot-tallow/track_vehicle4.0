//
// Created by xyl on 24-4-3.
//

#include "../include/D_LG_EKF.h"
#include "ros/ros.h"

//初始化ESKF，对滤波器赋初值
void ESKF::initESKF() {
    error_x = Vector18d::Zero();
    error_z = Vector6d::Zero();

    normal_x_pos = Eigen::Vector3d::Zero();
    normal_x_Rota = Eigen::Matrix3d::Zero();
    normal_x_vec = Eigen::Vector3d::Zero();
    normal_x_ba = Eigen::Vector3d::Zero();
    normal_x_bg = Eigen::Vector3d::Zero();
    normal_x_gvt  << 0.0 ,0.0 ,-9.8;

    last_imu_acc = Eigen::Vector3d::Zero();
    last_imu_gyro = Eigen::Vector3d::Zero();



    F_x = Matrix18d::Identity();
    P_k = Matrix18d::Zero();
    //position std conv 1m
    P_k.block<3,3>(0,0) = 1 * Eigen::Matrix3d::Identity();
    //velocity std conv 1m/s
    P_k.block<3,3>(3,3) = 1 * Eigen::Matrix3d::Identity();
    // roll pitch std conv 5 degree.
    P_k.block<2,2>(6,6) = pow(1 * Deg2Rad,2) * Eigen::Matrix2d::Identity();
    // yaw std conv 10 degree.
    P_k(8,8) = pow(2 * Deg2Rad,2);
    // acc bias std conv 0.05
    P_k.block<3,3>(9,9) = 0.0025 * Eigen::Matrix3d::Identity();
    // gyro bias std conv 0.05
    P_k.block<3,3>(12,12) = 0.0025 * Eigen::Matrix3d::Identity();
    // gravty bias std conv 0.0001
    P_k.block<3,3>(15,15) = 0.00000001 * Eigen::Matrix3d::Identity();

    H_k = Matrix6_18d::Zero();
    H_k.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    H_k.block<3,3>(3,6) = Eigen::Matrix3d::Identity();

    K_k = Matrix18_6d::Zero();


    setMatrixQ();
    setMatrixR();
    setCameraOffset();

}

//更新预测函数
void ESKF::predictUpdate(Eigen::Vector3d &_imu_acc, Eigen::Vector3d &_imu_gyro)
{

    Eigen::Vector3d acc_unbias = 0.5 * (_imu_acc + last_imu_acc) - normal_x_ba;
    Eigen::Vector3d gyro_unbias = 0.5 * (_imu_gyro + last_imu_gyro) - normal_x_bg;
    Eigen::Vector3d delta_angle_axis = gyro_unbias * T;
    last_imu_acc = _imu_acc;
    last_imu_gyro = _imu_gyro;

    //imu预积分
    Eigen::Vector3d last_pos = normal_x_pos;
    Eigen::Vector3d last_vec = normal_x_vec;
    Eigen::Matrix3d  last_R = normal_x_Rota;

    Eigen::Vector3d acc_word = last_R * acc_unbias;

    normal_x_pos = last_pos + last_vec * T +
                   0.5 * (acc_word + normal_x_gvt) * T * T;
    normal_x_vec = last_vec + (acc_word + normal_x_gvt) * T;
    if (delta_angle_axis.norm() > 1e-12)
    {
        normal_x_Rota = last_R * deltaSO3Exp(delta_angle_axis);
    }

    //计算F阵
    F_x.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * T;
    F_x.block<3,3>(3,6) = -normal_x_Rota * BuildSkewSymmetricMatrix3(acc_unbias) * T;
    F_x.block<3,3>(3,9) = -normal_x_Rota * T;
    F_x.block<3,3>(3,15) = Eigen::Matrix3d::Identity() * T;
    if (delta_angle_axis.norm() > 1e-12)
    {
        F_x.block<3,3>(6,6) = deltaSO3Exp(-delta_angle_axis);
    } else{
        F_x.block<3,3>(6,6).setIdentity();
    }
    F_x.block<3,3>(6,12) = - Eigen::Matrix3d::Identity() * T;

//    std::cout << "F:" << std::endl << F_x << std::endl;
    P_k = F_x * P_k * F_x.transpose() + Q;
}

//测量修正函数
void ESKF::measureCorrect(Eigen::Vector3d &_pos, Eigen::Quaterniond &_q)
{
    //测量计算测量误差
    error_z.block<3,1>(0,0) = _pos - (normal_x_pos + normal_x_Rota * offset_camera2imu);

    Eigen::Quaterniond q_normal(normal_x_Rota);
    Eigen::Quaterniond delta_q = q_normal.inverse() * _q;
    Eigen::Vector3d temp;
    temp[0] = -delta_q.x();
    temp[1] = -delta_q.y();
    temp[2] = -delta_q.z();
    error_z.block<3,1>(3,0) = 2 * temp;

    //更新H
    H_k.block<3,3>(0,6) = -normal_x_Rota * BuildSkewSymmetricMatrix3(offset_camera2imu);

    K_k = P_k * H_k.transpose() * (H_k * P_k * H_k.transpose() + R).inverse();
    error_x = K_k * error_z;

    eliminateError();

    P_k = (Matrix18d::Identity() - K_k * H_k) * P_k;
}

//消除误差函数
void ESKF::eliminateError()
{
    normal_x_pos = normal_x_pos + error_x.block<3,1>(0,0);
    normal_x_vec = normal_x_vec + error_x.block<3,1>(3,0);
    normal_x_ba = normal_x_ba + error_x.block<3,1>(9,0);
    normal_x_bg = normal_x_bg + error_x.block<3,1>(12,0);
    normal_x_gvt = normal_x_gvt + error_x.block<3,1>(15,0);

    if (error_x.block<3,1>(6,0).norm() > 1e-12)
    {
        normal_x_Rota = normal_x_Rota * deltaSO3Exp(error_x.block<3,1>(6,0));
    }
}

//设置过程噪声协方差矩阵
void ESKF::setMatrixQ()
{
    double Q_ng,Q_na,Q_nba,Q_nbg;
    double T2 = T * T;
    ros::param::get("Q_ng",Q_ng);
    ros::param::get("Q_na",Q_na);
    ros::param::get("Q_nba",Q_nba);
    ros::param::get("Q_nbg",Q_nbg);

    Q =  Matrix18d::Zero();
    Q.block<3,3>(3,3) = T2 * Q_na * Q_na * Eigen::Matrix3d::Identity();
    Q.block<3,3>(6,6) = T2 * Q_ng * Q_ng * Eigen::Matrix3d::Identity();
    Q.block<3,3>(9,9) = T * Q_nba * Q_nba * Eigen::Matrix3d::Identity();
    Q.block<3,3>(12,12) = T * Q_nbg * Q_nba * Eigen::Matrix3d::Identity();
}

//设置观测噪声协方差矩阵
void ESKF::setMatrixR()
{
    double R_pos,R_rota;
    ros::param::get("R_pos",R_pos);
    ros::param::get("R_rota",R_rota);
    R_pos *= R_pos;
    R_rota *= R_rota;

    R = Matrix6d::Zero();
    R.diagonal() << R_pos, R_pos, R_pos, R_rota, R_rota, R_rota;
}

//构造函数
ESKF::ESKF(double sample_time) : T(sample_time)
{
    initESKF();
}

void ESKF::setInitState(Eigen::Vector3d &_pos, Eigen::Quaterniond &_q) {
    normal_x_Rota = _q.toRotationMatrix();
    normal_x_pos = _pos;

    Eigen::Vector3d sum_gyro(0.0, 0.0, 0.0);
    for(const auto& _gyro_data : imu_gyro_buffer)
    {
        sum_gyro += _gyro_data;
    }
    imu_gyro_buffer.clear();
}

//设置相机偏移
void ESKF::setCameraOffset() {

    double offset_camera2imu_x,offset_camera2imu_y,offset_camera2imu_z;
    ros::param::get("offset_camera2imu_x",offset_camera2imu_x);
    ros::param::get("offset_camera2imu_y",offset_camera2imu_y);
    ros::param::get("offset_camera2imu_z",offset_camera2imu_z);

    offset_camera2imu[0] = offset_camera2imu_x;
    offset_camera2imu[1] = offset_camera2imu_y;
    offset_camera2imu[2] = offset_camera2imu_z;
}

void ESKF::getPosition(Eigen::Vector3d &_pos) {
    _pos = normal_x_pos;
}

//添加IMU数据
bool ESKF::addImuData(Eigen::Vector3d &_accel, Eigen::Vector3d &_gyro) {
    if(imu_gyro_buffer.size() > 150)
    {
        return true;
    }
    else
    {
        imu_gyro_buffer.push_back(_gyro);
        return false;
    }
}

//获取旋转矩阵
void ESKF::getRotation(Eigen::Matrix3d &_rata) {
    _rata = normal_x_Rota;
}

//获取相机偏移
void ESKF::getCameOffset(Eigen::Vector3d &_pos) {
    _pos = offset_camera2imu;
}

