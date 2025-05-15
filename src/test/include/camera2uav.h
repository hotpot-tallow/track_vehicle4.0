#pragma once

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include <array>


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
            Camera2UAV():listener(buffer){
                //tf监听器设置
                quat_camera.header.frame_id = "iris_0::usb_cam::link";
                //tf广播器设置
                ts.header.frame_id = "iris_0::iris::base_link";
                ts.child_frame_id = "iris_0::usb_cam::link";
                ts.transform.translation.x = 0.0;
                ts.transform.translation.y = 0.0;
                ts.transform.translation.z = 0.0;
                qtn.setRPY(0, 3.14159, 1.5708); // 设置欧拉角，旋转顺序ZYX
                ts.transform.rotation.x = qtn.getX();
                ts.transform.rotation.y = qtn.getY();
                ts.transform.rotation.z = qtn.getZ();
                ts.transform.rotation.w = qtn.getW();
            }
            void updateQuaternion(double x, double y, double z, double w) {
                quat_camera.quaternion.x = x;
                quat_camera.quaternion.y = y;
                quat_camera.quaternion.z = z;
                quat_camera.quaternion.w = w;
                quat_camera.header.stamp = ros::Time::now();
            }
            void sendTransform(){
                c2u.sendTransform(ts);
                ts.header.stamp = ros::Time::now();
            }
            //相机坐标转换到机体坐标
            void camera_to_uav(){
                try{
                    quat_base = buffer.transform(quat_camera,"iris_0::iris::base_link");
                    //ROS_INFO("目标的w:%.3f,目标的x:%.3f,目标的y:%.3f,目标的z:%.3f",quat_base.quaternion.w,quat_base.quaternion.x,quat_base.quaternion.y,quat_base.quaternion.z);
                }
                catch (const std::exception &e){
                    //ROS_ERROR("%s", e.what());
                }
            }
            //获得机体旋转的欧拉角
            std::array<double, 3> get_euler(){
                quat.setW(quat_base.quaternion.w);
                quat.setX(quat_base.quaternion.x);
                quat.setY(quat_base.quaternion.y);
                quat.setZ(quat_base.quaternion.z);
                tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
                return {roll,pitch,yaw};
            }
};

