#pragma once

#include <Eigen/Dense>
#include <vector>
#include <stdint.h>
#include <D_LG_EKF.h>

Eigen::Vector3d pos_word2camera;
Eigen::Quaterniond attitude_FCU;
ESKF* eskf_fusion;
bool attitude_update = false;
bool vision_update = false;
uint8_t eskf_step = 0;
double pitch,roll,yaw;