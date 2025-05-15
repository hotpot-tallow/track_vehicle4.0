//
// Created by xyl on 24-5-15.
//
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include<arpa/inet.h>
#include<sys/socket.h>
#include<unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

sockaddr_in serverAddr;
int sock;

void image_cb(const sensor_msgs::Image::ConstPtr& msg)
{
    static int cnt = 0;
    if(cnt > 2)
    {
        cv::Mat image;
        try
        {
            image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }

        // 将图像转换为字节流
        std::vector<uchar> data;
        cv::imencode(".jpg", image, data);
        sendto(sock, data.data(), data.size(), 0, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
        cnt = 0;
    }
    cnt++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_uav0_node");
    ros::NodeHandle nh;
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Error: Creating socket" << std::endl;
    }

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(9000); // 端口号
    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");


    ros::Subscriber vision_sub = nh.subscribe<sensor_msgs::Image>
            ("tag_detections_image", 1, image_cb);

    ros::spin();

}