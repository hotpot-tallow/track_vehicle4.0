//检测apriltag相对于无人机的位置和姿态
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

bool takeoff = false;
double curH;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
Eigen::Quaterniond attitude;
Eigen::Quaterniond attitude1;
ros::Publisher local_vec_pub;
ros::Publisher yaw_pub;

void yaw_adjustment(mavros_msgs::PositionTarget& yaw_set,const double& yaw_angle){
    yaw_set.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    yaw_set.type_mask = 
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_PX |
                        mavros_msgs::PositionTarget::IGNORE_PY |
                        mavros_msgs::PositionTarget::IGNORE_PZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW;
    yaw_set.velocity.x = 0;
    yaw_set.velocity.y = 0;
    yaw_set.velocity.z = 0;
    if(yaw_angle >= 90){
        yaw_set.yaw_rate = 0.168;
    }     
    if(yaw_angle < 90){
        yaw_set.yaw_rate = -0.168;
    }
}

void apriltag_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg){
    static double x0, y0, z0, x1, y1, z1;
    double yaw, pitch, roll, yaw_angle, yaw1, yaw_angle1;
    geometry_msgs::Twist speed_set;
    mavros_msgs::PositionTarget yaw_set;
    yaw_set.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    if(!msg->detections.empty()){

        std::vector<apriltag_ros::AprilTagDetection> sorted_detections = msg->detections;
        // 按照 detection.id[0] 排序（注意：id 是 vector<int>，这里假设每个检测只有一个 ID）
        std::sort(sorted_detections.begin(), sorted_detections.end(),
                  [](const apriltag_ros::AprilTagDetection& a, const apriltag_ros::AprilTagDetection& b)
                  {
                      return a.id[0] > b.id[0];
                  });
        ROS_INFO_STREAM("原版ID: " << msg->detections[0].id[0] << "修改后ID: " << sorted_detections[0].id[0]);      
        
        x0 = sorted_detections[0].pose.pose.pose.position.x;
        y0 = sorted_detections[0].pose.pose.pose.position.y;
        z0 = sorted_detections[0].pose.pose.pose.position.z;
        attitude.x() = (sorted_detections[0].pose.pose.pose.orientation.x);
        attitude.y() = (sorted_detections[0].pose.pose.pose.orientation.y);
        attitude.z() = (sorted_detections[0].pose.pose.pose.orientation.z);
        attitude.w() = (sorted_detections[0].pose.pose.pose.orientation.w);
        // x1 = msg->detections[1].pose.pose.pose.position.x;
        // y1 = msg->detections[1].pose.pose.pose.position.y;
        // z1 = msg->detections[1].pose.pose.pose.position.z;
        Eigen::Matrix3d rotation_matrix = attitude.toRotationMatrix();
        Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);  // ZYX 顺序，即 yaw-pitch-roll
        yaw = euler_angles[0];
        pitch = euler_angles[1];
        roll = euler_angles[2];
        yaw_angle = yaw * (180 / M_PI);
        // if (abs(yaw_angle) > 8){
        // yaw_adjustment(yaw_set,yaw_angle); 
        // yaw_pub.publish(yaw_set);
        // }        
        // speed_set.angular.z = 0.1;
        // speed_set.linear.x = 0;
        // speed_set.linear.y = 0;
        // speed_set.linear.z = 0;   
        ROS_INFO("目标7的yaw:%.3f",yaw_angle);
        // ROS_INFO("目标的x0坐标:%.3f,目标的y0坐标:%.3f,目标的z0坐标:%.3f",x0,y0,z0);
        // ROS_INFO("目标的x1坐标:%.3f,目标的y1坐标:%.3f,目标的z1坐标:%.3f",x1,y1,z1);

    }
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void do_H(const mavros_msgs::Altitude::ConstPtr& msg){
    curH = msg->local;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

int main(int argc, char **argv){
    //定义一些变量
    mavros_msgs::SetMode offb_set_mode;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 10;
    geometry_msgs::Twist speed;
    speed.linear.x = 0;
    speed.linear.y = 0;
    speed.linear.z = 0;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::init(argc, argv, "dected_april");
    ros::NodeHandle nh;
    setlocale(LC_ALL,"");
    ros::Time last_request = ros::Time::now();
    ros::Subscriber apriltag_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("tag_detections", 5,apriltag_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",10,state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10,pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    local_vec_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    yaw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);
    ros::Subscriber height_sub = nh.subscribe<mavros_msgs::Altitude>("mavros/altitude", 10, do_H);

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    tf2_ros::StaticTransformBroadcaster c2u;
    geometry_msgs::TransformStamped ts;

    ros::Rate rate(20.0);
    while(ros::ok()){
            //tf广播器
            ts.header.stamp = ros::Time::now();
            ts.header.frame_id = "iris_0::iris::base_link";
            ts.child_frame_id = "iris_0::usb_cam::link";
            ts.transform.translation.x = 0.0;
            ts.transform.translation.y = 0.0;
            ts.transform.translation.z = 0.0;
            tf2::Quaternion qtn; // tf2的四元数类
            qtn.setRPY(0, 3.14159, 1.5708); // 设置欧拉角
            // 获取旋转的四元数值
            ts.transform.rotation.x = qtn.getX();
            ts.transform.rotation.y = qtn.getY();
            ts.transform.rotation.z = qtn.getZ();
            ts.transform.rotation.w = qtn.getW();
            c2u.sendTransform(ts);

            //tf监听器
            geometry_msgs::QuaternionStamped quat_camera;
            quat_camera.header.frame_id = "iris_0::usb_cam::link";
            quat_camera.header.stamp = ros::Time::now();
            quat_camera.quaternion.x = attitude.x();
            quat_camera.quaternion.y = attitude.y();
            quat_camera.quaternion.z = attitude.z();
            quat_camera.quaternion.w = attitude.w();

            try{
                geometry_msgs::QuaternionStamped quat_base;
                quat_base = buffer.transform(quat_camera,"iris_0::iris::base_link");
                //ROS_INFO("目标的w:%.3f,目标的x:%.3f,目标的y:%.3f,目标的z:%.3f",quat_base.quaternion.w,quat_base.quaternion.x,quat_base.quaternion.y,quat_base.quaternion.z);

            }
            catch (const std::exception &e){
                ROS_ERROR("%s", e.what());
            }
        if(!takeoff)
        {
            if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0))){
                if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                    ROS_INFO("已设置为OFFBOARD模式");
                }
                last_request = ros::Time::now();
            }
            if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(2.0))){
                if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("已解锁");
                    takeoff = true;
                }
                last_request = ros::Time::now();
            }            
        }
        if(curH < 4){
            local_pos_pub.publish(pose);
        }
        else
        {
            local_vec_pub.publish(speed);
        }        
        ros::spinOnce();
        rate.sleep();
    }
}