// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "loam_velodyne/ScanRegistration.h"
#include "math_utils.h"

#include <tf/transform_datatypes.h>

namespace loam {



bool ScanRegistration::parseParams(const ros::NodeHandle& nh, RegistrationParams& config_out) 
{
  bool success = true;
  int iParam = 0;
  float fParam = 0;

  if (nh.getParam("scanPeriod", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid scanPeriod parameter: %f (expected > 0)", fParam);
      success = false;
    } else {
        config_out.scanPeriod = fParam;
      ROS_INFO("Set scanPeriod: %g", fParam);
    }
  }

  if (nh.getParam("imuHistorySize", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid imuHistorySize parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
        config_out.imuHistorySize = iParam;
      ROS_INFO("Set imuHistorySize: %d", iParam);
    }
  }

  if (nh.getParam("featureRegions", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid featureRegions parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
        config_out.nFeatureRegions = iParam;
      ROS_INFO("Set nFeatureRegions: %d", iParam);
    }
  }

  if (nh.getParam("curvatureRegion", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid curvatureRegion parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
        config_out.curvatureRegion = iParam;
      ROS_INFO("Set curvatureRegion: +/- %d", iParam);
    }
  }

  if (nh.getParam("maxCornerSharp", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid maxCornerSharp parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
        config_out.maxCornerSharp = iParam;
        config_out.maxCornerLessSharp = 10 * iParam;
      ROS_INFO("Set maxCornerSharp / less sharp: %d / %d", iParam, config_out.maxCornerLessSharp);
    }
  }

  if (nh.getParam("maxCornerLessSharp", iParam)) {
    if (iParam < config_out.maxCornerSharp) {
      ROS_ERROR("Invalid maxCornerLessSharp parameter: %d (expected >= %d)", iParam, config_out.maxCornerSharp);
      success = false;
    } else {
        config_out.maxCornerLessSharp = iParam;
      ROS_INFO("Set maxCornerLessSharp: %d", iParam);
    }
  }

  if (nh.getParam("maxSurfaceFlat", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid maxSurfaceFlat parameter: %d (expected >= 1)", iParam);
      success = false;
    } else {
        config_out.maxSurfaceFlat = iParam;
      ROS_INFO("Set maxSurfaceFlat: %d", iParam);
    }
  }

  if (nh.getParam("surfaceCurvatureThreshold", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("Invalid surfaceCurvatureThreshold parameter: %f (expected >= 0.001)", fParam);
      success = false;
    } else {
        config_out.surfaceCurvatureThreshold = fParam;
      ROS_INFO("Set surfaceCurvatureThreshold: %g", fParam);
    }
  }

  if (nh.getParam("lessFlatFilterSize", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("Invalid lessFlatFilterSize parameter: %f (expected >= 0.001)", fParam);
      success = false;
    } else {
        config_out.lessFlatFilterSize = fParam;
      ROS_INFO("Set lessFlatFilterSize: %g", fParam);
    }
  }

  return success;
}

bool ScanRegistration::setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode, RegistrationParams& config_out)
{
  if (!parseParams(privateNode, config_out))
    return false;

  // subscribe to IMU topic
  // _subImu = node.subscribe<sensor_msgs::Imu>("/imu/data", 50, &ScanRegistration::handleIMUMessage, this);
  //_subImu = node.subscribe<sensor_msgs::Imu>("/zed2/zed_node/imu/data", 50, &ScanRegistration::handleIMUMessage, this);

  // advertise scan registration topics
  // _pubPixelCloud            = node.advertise<sensor_msgs::PointCloud2>("/pixel_cloud", 2);
  // ////////////////////////////
  _pubLaserCloud            = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 2);
  _pubCornerPointsSharp     = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);
  _pubCornerPointsLessSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);
  _pubSurfPointsFlat        = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);
  _pubSurfPointsLessFlat    = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);
  _pubImuTrans              = node.advertise<sensor_msgs::PointCloud2>("/imu_trans", 5);

  _pubRightRectified        = node.advertise<sensor_msgs::Image>("/zed2/zed_node/right/image_rect_color", 10);
  _pubLeftRectified         = node.advertise<sensor_msgs::Image>("/zed2/zed_node/left/image_rect_color", 10);

  return true;
}


void ScanRegistration::handleIMUMessage(const sensor_msgs::Imu::ConstPtr& imuIn) // IMU 센서로 부터의 메세지를 처리하기 위한 핸들러. imuIn가 새로운 IMU 메세지.
{
  ///*
  // ROS_INFO( "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
  //             imuIn->linear_acceleration.x, imuIn->linear_acceleration.y, imuIn->linear_acceleration.z,
  //             imuIn->angular_velocity.x, imuIn->angular_velocity.y, imuIn->angular_velocity.z,
  //             imuIn->orientation.x, imuIn->orientation.y, imuIn->orientation.z, imuIn->orientation.w);
  //*/

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  double roll, pitch, yaw;
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  Vector3 acc;
  acc.x() = float(imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81);
  acc.y() = float(imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81);
  acc.z() = float(imuIn->linear_acceleration.x + sin(pitch)             * 9.81);

  IMUState newState;
  newState.stamp = fromROSTime( imuIn->header.stamp);
  newState.roll = roll;
  newState.pitch = pitch;
  newState.yaw = yaw;
  newState.acceleration = acc;

  //*
  //std::cout << "newState parameters - roll, pitch, yaw, acc " << roll << " " << pitch << " " << yaw << " " << acc << " " << std::endl;
  //*

  updateIMUData(acc, newState);
}


void ScanRegistration::publishResult()
{
  auto sweepStartTime = toROSTime(sweepStart());
  // publish full resolution and feature point clouds
  
  publishCloudMsg(_pubLaserCloud, laserCloud(), sweepStartTime, "/camera");
  publishCloudMsg(_pubCornerPointsSharp, cornerPointsSharp(), sweepStartTime, "/camera");
  publishCloudMsg(_pubCornerPointsLessSharp, cornerPointsLessSharp(), sweepStartTime, "/camera");
  publishCloudMsg(_pubSurfPointsFlat, surfacePointsFlat(), sweepStartTime, "/camera");
  publishCloudMsg(_pubSurfPointsLessFlat, surfacePointsLessFlat(), sweepStartTime, "/camera");

  // publish corresponding IMU transformation information
  publishCloudMsg(_pubImuTrans, imuTransform(), sweepStartTime, "/camera");


  // publishCloudMsg(_pubPixelCloud, pixelCloud(), sweepStartTime, "/camera");   // 새로 추가.

  //* 좌 우의 이미지를 publish해 주는 부분 *//
  // sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _mat_right).toImageMsg();
  // sensor_msgs::ImagePtr msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _mat_left).toImageMsg();
  // _pubRightRectified.publish(msg_right);
  // _pubLeftRectified.publish(msg_left);
}

} // end namespace loam
