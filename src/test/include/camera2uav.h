#pragma once

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include <array>
#include <geometry_msgs/PoseStamped.h>


class Camera2UAV{
    private:
            //欧拉角中间四元数
            tf2::Quaternion quat;
            //Apriltag相对于无人机的坐标
            geometry_msgs::QuaternionStamped quat_base;
            //创建一个tf广播器
            tf2_ros::StaticTransformBroadcaster c2u;//广播器
            geometry_msgs::TransformStamped ts;//转换四元数
            tf2::Quaternion qtn; // tf2的四元数类
            //创建一个tf监听器
            tf2_ros::Buffer buffer;//监听数据缓存区
            tf2_ros::TransformListener listener;
            geometry_msgs::QuaternionStamped quat_camera;//目标相对与相机的旋转
    public:
            double roll, pitch, yaw;
            Camera2UAV();
            void updateQuaternion(double x, double y, double z, double w);
            void sendTransform();
            //相机坐标转换到机体坐标
            void camera_to_uav();
            //获得机体旋转的欧拉角
            std::array<double, 3> get_euler();
            void ts_set(const geometry_msgs::PoseStamped& uav_pose);
};

