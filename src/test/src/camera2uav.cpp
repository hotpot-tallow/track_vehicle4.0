#include "camera2uav.h"

Camera2UAV::Camera2UAV():listener(buffer){
    //tf监听器设置
    quat_camera.header.frame_id = "base_link";
    //tf广播器设置
    ts.header.frame_id = "map";
    ts.child_frame_id = "base_link";
    ts.transform.translation.x = 0.0;
    ts.transform.translation.y = 0.0;
    ts.transform.translation.z = 0.0;
    qtn.setRPY(0, 0, 0); // 设置欧拉角，旋转顺序ZYX
    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();
}
void Camera2UAV::updateQuaternion(double x, double y, double z, double w) {
    quat_camera.quaternion.x = x;
    quat_camera.quaternion.y = y;
    quat_camera.quaternion.z = z;
    quat_camera.quaternion.w = w;
    quat_camera.header.stamp = ros::Time::now();
}
void Camera2UAV::sendTransform(){
    ts.header.stamp = ros::Time::now();
    c2u.sendTransform(ts);
}
//相机坐标转换到机体坐标
void Camera2UAV::camera_to_uav(){
    try{
        buffer.canTransform("base_link","usb_cam",ros::Time(0), ros::Duration(1.0));
        quat_base = buffer.transform(quat_camera,"base_link");
        //ROS_INFO("目标的w:%.3f,目标的x:%.3f,目标的y:%.3f,目标的z:%.3f",quat_base.quaternion.w,quat_base.quaternion.x,quat_base.quaternion.y,quat_base.quaternion.z);
    }
    catch (const std::exception &e){
        //ROS_ERROR("%s", e.what());
    }
}
//获得机体旋转的欧拉角
std::array<double, 3> Camera2UAV::get_euler(){
    quat.setW(quat_base.quaternion.w);
    quat.setX(quat_base.quaternion.x);
    quat.setY(quat_base.quaternion.y);
    quat.setZ(quat_base.quaternion.z);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return {roll,pitch,yaw};
}
//设置地图与无人机之间的tf转换
void Camera2UAV::ts_set(const geometry_msgs::PoseStamped& uav_pose){
    this->ts.transform.translation.x = uav_pose.pose.position.x;
    this->ts.transform.translation.y = uav_pose.pose.position.y;
    this->ts.transform.translation.z = uav_pose.pose.position.z;
    this->ts.transform.rotation = uav_pose.pose.orientation;
}