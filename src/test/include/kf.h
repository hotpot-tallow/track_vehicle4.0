#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <Eigen/Dense>


//卡尔曼预测参数
struct kf_state
{
    ros::Time time_stamp;
    Eigen::VectorXd x; // 状态估计
    Eigen::MatrixXd P; // 估计误差协方差
};

//传感器测量参数
struct sensor_measurement
{
   ros::Time time_stamp;
   Eigen::VectorXd z;
};

//卡尔曼滤波器
class KFTracker
{
private:
   ros::NodeHandle nh_;
   ros::NodeHandle nh_private_;

   kf_state kf_state_pred_; //先验卡尔曼滤波参数
   Eigen::VectorXd x_pred_; //先验状态估计
   Eigen::MatrixXd P_pred_; //先验误差协方差
   sensor_measurement z_meas_; //测量参数
   sensor_measurement z_last_meas_; //最近一次测量参数
   Eigen::MatrixXd F_; //状态转移矩阵
   Eigen::MatrixXd H_; //观测矩阵
   Eigen::MatrixXd Q_; //过程噪声斜方差
   Eigen::MatrixXd R_; //观测噪声斜方差
   double q_; //过程噪声标准差
   double r_; //观测噪声标准差
   double dt_pred_; //卡尔曼时间步长
   bool is_state_initialzed_; //状态是否已经初始化
   std::vector<kf_state> state_buffer_; //存储过去预测的状态和协方差
   unsigned int state_buffer_size_; //状态缓存区长度
   std::string tracking_frame_; 
   bool do_update_step_; //是否进行测量更新
   ros::Time last_measurement_time_; //上一次收到观测值时间
   double measurement_off_time_; //最长允许未收到观测值时间

   bool debug_; //是否调试

   /**
    * @brief Computes estimate covariance Q based on sampling time dt_pred_ and  process noise q_.
    */
   void setQ(void);

   /**
    * @brief Computes observation covariance R based on  observation noise r_.
    */
   void setR(void);

   /**
    * @brief Computes discrete transition matrix F_ based on sampling time dt_pred_.
    */
   void setF(void);

   /**
    * @brief Computes discrete observation matrix H_. Observations are positions x,y,z.
    */
   void setH(void);

   //初始一个误差协方差矩阵
   void initP(void);

   //初始化滤波器所有参数
   void initKF(void);

   //更新卡尔曼状态缓存区
   void updateStateBuffer(void);

   //执行预测步骤
   bool predict(void);

   //执行校正步骤
   void update(void);

   //卡尔曼迭代循环
   void filterLoop(const ros::TimerEvent& event);

   //获取测量数据
   void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

   //发布卡尔曼滤波数据
   void publishState(void);

   ros::Publisher state_pub_;
   ros::Subscriber pose_sub_; /**< Subscriber to measurments. */
   ros::Timer kf_loop_timer_;

public:
   KFTracker(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
   ~KFTracker();
};