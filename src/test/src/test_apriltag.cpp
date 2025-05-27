#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <Eigen/Core>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core.hpp>


#include "apriltag_ros/AprilTagDetectionArray.h"
#include "PID_controller.h"
#include "D_LG_EKF.h"
#include "ESKF_param.h"
#include "camera2uav.h"



// 一些全局变量

// 悬空高度（追踪小车的高度）
const double h = 8;
// 调整高度的速度（上升或下降）
const double hv = 0.1;
// 无人机当前的高度
double curH;
double curX;
double curY;
//小车速度
double uav_last_velx,uav_last_vely;
// 无人机是否已经稳定在空中的标志
bool start = false;
bool tag_detected = false;
//程序是否结束标志
bool circle = true;
bool land_detected = false;
geometry_msgs::PoseStamped tag_pose;
// 控制无人机的速度
geometry_msgs::Twist velocity;
// 控制无人机姿态
geometry_msgs::PoseStamped pose;
// 无人机目前状态
mavros_msgs::State current_mode;
// 创建PID控制器指针
PID_Controller *pos_controll;
//创建TF转换的指针
Camera2UAV *camera2uav;
//速度发布者
ros::Publisher local_vec_pub;
//位置发布者
ros::Publisher local_pos_pub;
//偏航角发布者
ros::Publisher yaw_pub;
//设置降落模式
mavros_msgs::SetMode land_set_mode;
//模式请求服务
ros::ServiceClient set_mode_client;
//Apriltag相对于相机姿态
Eigen::Quaterniond attitude;
//相机相关参数
const int IMG_WIDTH = 640;
const int IMG_HEIGHT = 480;
int mutiple = 4;
int center_x = IMG_WIDTH / 2;
int center_y = IMG_HEIGHT / 2;
image_geometry::PinholeCameraModel cam_model;
cv::Point3d point3D;
cv::Point2d pixel;

// 状态机定义
enum State { CRUISE, ROTATE, DESCEND, MDA, LAND};
State current_state = CRUISE;

double vehicle_velocity(const double& dx,const double& dy){
    double vehicle_speed = sqrt(dx*dx+dy*dy);
    return vehicle_speed;
} 

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
        yaw_set.yaw_rate = 0.315;
    }     
    if(yaw_angle < 90){
        yaw_set.yaw_rate = -0.315;
    }
}

void uav_control(mavros_msgs::PositionTarget& yaw_set){
    yaw_set.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    yaw_set.type_mask = 
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_PX |
            mavros_msgs::PositionTarget::IGNORE_PY |
            mavros_msgs::PositionTarget::IGNORE_PZ |
            mavros_msgs::PositionTarget::IGNORE_YAW;
    yaw_pub.publish(yaw_set);
}
//apriltag码订阅回调函数
void tag_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) 
{
    static double x, y, z, dx0, dx1, dy0, dy1, dx, dy;
    tf2::Quaternion quat;
    static mavros_msgs::PositionTarget yaw_set;
    static geometry_msgs::Twist speed_set;
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";
    geometry_msgs::PoseStamped target_pose;
    static ros::Time last_find_time; 
    static double z_descend = -0.3;
    static int number = 0,dot = 0;
    static double error_x, error_y, error_z,error_yaw;
    static double roll, pitch, yaw;
    static double x_last, y_last, z_last;
    static bool run = false;
    double yaw_angle,roll_angle,pitch_angle;

    if(!msg->detections.empty()) 
    {
        x_last = curX;
        y_last = curY;
        z_last = curH;
        tag_detected = true;

        last_find_time = ros::Time::now();
        std::vector<apriltag_ros::AprilTagDetection> sorted_detections = msg->detections;

        // 按照 detection.id[0] 升序排序（注意：id 是 vector<int>）
        std::sort(sorted_detections.begin(), sorted_detections.end(),
                  [](const apriltag_ros::AprilTagDetection& a, const apriltag_ros::AprilTagDetection& b)
                  {
                      return a.id[0] < b.id[0];
                  });
        ROS_INFO_THROTTLE(5, "原版ID:%d,修改后ID:%d", msg->detections[0].id[0],sorted_detections[0].id[0]);

        switch(current_state)
        {
            case CRUISE:
            {
                apriltag_ros::AprilTagDetection target;
                if (sorted_detections.size() >= 2) {
                    if (sorted_detections[1].id[0] == 7) {
                        target = sorted_detections[1];
                    } else {
                        target = sorted_detections[0];
                    }
                } 
                else {
                    // fallback: 如果只有一个目标，就默认取第一个
                    target = sorted_detections[0];
                }

                y = -(target.pose.pose.pose.position.x);//相机坐标系与机体坐标系不同
                x = -(target.pose.pose.pose.position.y);
                z = -(target.pose.pose.pose.position.z);
                attitude.x() = (target.pose.pose.pose.orientation.x);
                attitude.y() = (target.pose.pose.pose.orientation.y);
                attitude.z() = (target.pose.pose.pose.orientation.z);
                attitude.w() = (target.pose.pose.pose.orientation.w);
                ROS_INFO("发现大目标,目标的x坐标:%.3f,目标的y坐标:%.3f,目标的z坐标:%.3f",x,y,z);
                error_x = -x;
                error_y = -y;
                point3D.x = target.pose.pose.pose.position.x;
                point3D.y = target.pose.pose.pose.position.y;
                point3D.z = target.pose.pose.pose.position.z;
                pixel = cam_model.project3dToPixel(point3D);
                // std::array<double, 3>euler = camera2uav->get_euler();转换到机体坐标系，求对应欧拉角
                // roll = euler[0];
                // pitch = euler[1];
                // yaw = euler[2];
                // roll_angle = roll * (180 / M_PI);
                // pitch_angle = pitch * (180 / M_PI);
                // yaw_angle = yaw * (180 / M_PI) + 90;
                Eigen::Matrix3d rotation_matrix = attitude.toRotationMatrix();
                Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);  // ZYX 顺序，即 yaw-pitch-roll
                yaw = euler_angles[0];
                pitch = euler_angles[1];
                roll = euler_angles[2];
                yaw_angle = yaw * (180 / M_PI);
                //ROS_INFO("目标的yaw_angle偏航:%.3f",yaw_angle);
                if (fabs(yaw_angle) > 8 || fabs(yaw_angle) < 172){
                yaw_adjustment(yaw_set,yaw_angle) ; 
                }
                yaw_set = pos_controll->control(error_x,error_y,0); 

                ROS_INFO("CRUISE无人机速度,x = %.3f,y = %.3f,z = %.3f",yaw_set.velocity.x,yaw_set.velocity.y,yaw_set.velocity.z);
                uav_control(yaw_set);
                //local_vec_pub.publish(speed_set);
                last_find_time = ros::Time::now();
                if(sqrt(x*x + y*y) < 2 && yaw_set.velocity.x >= 2){
                    number += 1;
                    if(number > 20){
                       current_state = DESCEND; 
                       number = 0;
                    }
                }
                break; 
            }
        }

    } 
    else if(tag_detected && circle) 
    {
        target_pose.pose.position.x = curX + x;
        target_pose.pose.position.y = curY + y;
        target_pose.pose.position.z = curH;
        local_pos_pub.publish(target_pose);
    }
}



void state_cb(const mavros_msgs::State::ConstPtr& msg){//回调函数，将msg中的状态储存在current_state中
    current_mode = *msg;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
    attitude_FCU = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
}

void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    curX = msg->pose.position.x;
    curY = msg->pose.position.y;
    curH = msg->pose.position.z;
}

// void ugv_vel_cb(const geometry_msgs::Twist::ConstPtr& msg){
//     ugv_vel = msg->linear.x;
//     ROS_INFO_THROTTLE(1.0, "小车速度: %.3f", ugv_vel);
// }


void image_cb(const sensor_msgs::Image::ConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat& image = cv_ptr->image;

    //显示侦测到的Apriltag码
    cv::circle(image, pixel, 5, cv::Scalar(0, 0, 255), -1);
    // 显示图像
    cv::imshow("Tracking Boundary", image);
    cv::waitKey(1);

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");//初始化一个名为offb_node的ROS节点
    setlocale(LC_ALL, "");//确保程序能够正确处理本地字符集。例如，如果程序中需要打印中文日志信息，或者处理中文路径
    ros::NodeHandle nh;
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    local_vec_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped",10);
    yaw_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local",10);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber tag_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>//订阅apriltag消息
            ("tag_detections", 10, tag_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data",10,imu_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pose_cb);
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>
            ("/tag_detections_image", 1, image_cb);
    // ros::Subscriber ugv_sub = nh.subscribe<geometry_msgs::Twist>
    //         ("/ugv_0/cmd_vel",10,ugv_vel_cb);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);//设置循环频率，每秒循环10次
    pos_controll = new PID_Controller();//初始化PID控制器
    camera2uav = new Camera2UAV();//初始化tf转换器


    // 构造相机内参
    sensor_msgs::CameraInfo cam_info;
    // 分辨率
    cam_info.width = 640;
    cam_info.height = 480;
    // K - 相机矩阵
    cam_info.K = {
        277.191356, 0.0, 320.5,
        0.0, 277.191356, 240.5,
        0.0, 0.0, 1.0
    };
    // D - 畸变
    cam_info.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    cam_info.distortion_model = "plumb_bob";
    // R - 校正矩阵
    cam_info.R = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    // P - 投影矩阵
    cam_info.P = {
        277.191356, 0.0, 320.5, 0.0,
        0.0, 277.191356, 240.5, 0.0,
        0.0, 0.0, 1.0, 0.0
    };
    cam_model.fromCameraInfo(cam_info);

    // wait for FCU connection
    while(ros::ok() && !current_mode.connected)
    {
        ros::spinOnce();//处理所有回调函数
        rate.sleep();//按照设定的频率暂定循环
    }
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = h;

    velocity.linear.x = 0;
    velocity.linear.y = 0;
    velocity.linear.z = 0;

    mavros_msgs::SetMode offb_set_mode;//mavros_msgs::SetMode：ROS消息类型，用于切换无人机的飞行模式
    offb_set_mode.request.custom_mode = "OFFBOARD";//将offb_set_mode.request.custom_mode设置为OFFBOARD，表示切换至飞行模式

    mavros_msgs::CommandBool arm_cmd;//mavros_msgs::CommandBool：ROS消息类型，用于解锁或锁定无人机
    arm_cmd.request.value = true;//将arm_cmd.request.value设置为true，表示解锁无人机


    ros::Time last_request = ros::Time::now();//更新请求时间

    bool takeoff = false;

    while(ros::ok() && circle)
    {
        camera2uav->updateQuaternion(attitude.x(), attitude.y(), attitude.z(), attitude.w());//更新转换的初始坐标
        camera2uav->sendTransform();//发布转换关系
        camera2uav->camera_to_uav();//将相机坐标系转换到机体坐标系
        if(!takeoff) 
        {
            if( current_mode.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(2.0))){//内层条件：检查是否为起飞模式并尝试切换
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }

            if( !current_mode.armed && (ros::Time::now() - last_request > ros::Duration(2.0)))//内层条件：检查是否解锁无人机并尝试解锁
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }

            if( current_mode.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) //内层条件：如果无人机是否已经解锁，并且距离上次请求时间是否大于5秒
            {
                takeoff = true;//判断是否已经起飞
                ROS_INFO("Vehicle stabled");
                start = true;//判断是否可以开始任务
                ROS_INFO("开始追踪...");
                last_request = ros::Time::now();//更新请求时间
            }

            local_pos_pub.publish(pose);//用于发布目标位置

        }
        // if (current_state ==ROTATE){
        //     yaw_pub.publish(yaw_set);
        // } 
        // else 
        // {
        //     local_vec_pub.publish(velocity);//用于发布无人机速度
        // }

        ros::spinOnce();//处理所有待处理回调函数
        rate.sleep();//按照设定的频率暂定循环
    }
    delete camera2uav;
    delete pos_controll;
    return 0;
}
