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

#ifndef LOAM_LASERMAPPING_H
#define LOAM_LASERMAPPING_H


#include "BasicLaserMapping.h"
#include "common.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

////////////////////////////////////////
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>

#include <thread>
using std::thread;
////////////////////////////////////////

namespace loam
{
/** \brief LOAM ????????? ?????? ?????? ?????? ??????. Implementation of the LOAM laser mapping component.
 *
 */
class LaserMapping : public BasicLaserMapping
{
public:
   explicit LaserMapping(const float& scanPeriod = 0.1, const size_t& maxIterations = 10);

   /** \brief ?????? ????????? ?????? ???????????? ?????? / Setup component in active mode.
    *
    * @param node the ROS node handle
    * @param privateNode the private ROS node handle
    */
   virtual bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);


   /** \brief ??? ????????? ?????? ??????????????? ?????? handler ?????????./ Handler method for a new last corner cloud.
    *
    * @param cornerPointsLastMsg ????????? last corner cloud ????????? / the new last corner cloud message
    */
   void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLastMsg);

   /** \brief ??? ????????? ?????? ??????????????? ?????? handler ?????????. / Handler method for a new last surface cloud.
    *
    * @param surfacePointsLastMsg ????????? last surface cloud ????????? / the new last surface cloud message
    */
   void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& surfacePointsLastMsg);

   /** \brief ??? ?????? ????????? ??????????????? ?????? handler ?????????. / Handler method for a new full resolution cloud.
    *
    * @param laserCloudFullResMsg ????????? full resolution cloud ????????? / the new full resolution cloud message
    */
   void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg);

   /** \brief ????????? laser odometry??? ?????? handler ????????? / Handler method for a new laser odometry.
    *
    * @param laserOdometry ????????? laser odometry ????????? / the new laser odometry message
    */
   void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);

   /** \brief IMU ???????????? ?????? handler method. / Handler method for IMU messages.
    *
    * @param imuIn ????????? IMU ????????? / the new IMU message
    */
   void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn);


   // ////////////////////////////////////////////////////////////////
   void zedPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& msg);
   void imageLeftRectifiedHandler(const sensor_msgs::Image::ConstPtr& msg);
   void depthHandler(const sensor_msgs::Image::ConstPtr& msg);
   void leftcamInfoHandler(const sensor_msgs::CameraInfo::ConstPtr& msg);

   void pclviewerprocess();
   void saveDataSet();
   void loadDataSet();
   void saveDepthSet();
   // ////////////////////////////////////////////////////////////////


   /** \brief ????????? ????????? ???????????? ?????? ????????? ??????(?????? ???????????? ??????) / Process incoming messages in a loop until shutdown (used in active mode). */
   void spin();


   /** \brief ????????? ????????? ???????????? ???????????? ?????????. / Try to process buffered data. */
   void process();


protected:
   /** \brief flags ?????? / Reset flags, etc. */
   void reset();

   /** \brief ??? ?????? ????????? ????????? ?????? ????????? ????????? ??? ????????? ??????. / Check if all required information for a new processing step is available. */
   bool hasNewData();

   /** \brief ??? ????????? ?????? ?????? ????????? publis / Publish the current result via the respective topics. */
   void publishResult();

private:
   ros::Time _timeLaserCloudCornerLast;   ///< ?????? last corner cloud??? ?????? / time of current last corner cloud
   ros::Time _timeLaserCloudSurfLast;     ///< ?????? last surface cloud??? ?????? / time of current last surface cloud
   ros::Time _timeLaserCloudFullRes;      ///< ?????? full resolution cloud??? ?????? / time of current full resolution cloud
   ros::Time _timeLaserOdometry;          ///< ?????? laser odometry??? ?????? / time of current laser odometry
   ros::Time _timeZed;//sue 
   bool _newLaserCloudCornerLast;  ///< ????????? last corner cloud??? ???????????? true / flag if a new last corner cloud has been received
   bool _newLaserCloudSurfLast;    ///< ????????? last surface cloud??? ???????????? true / flag if a new last surface cloud has been received
   bool _newLaserCloudFullRes;     ///< ????????? full resolution cloud??? ???????????? true / flag if a new full resolution cloud has been received
   bool _newLaserOdometry;         ///< ????????? laser odometry??? ???????????? true / flag if a new laser odometry has been received


   nav_msgs::Odometry _odomAftMapped;      ///< ?????? odometry ????????? / mapping odometry message
   tf::StampedTransform _aftMappedTrans;   ///< ?????? odometry ?????? / mapping odometry transformation

   ros::Publisher _pubLaserCloudSurround;    ///< map cloud ????????? ????????? / map cloud message publisher

   ros::Publisher _pubLaserCloudFullRes;     ///< ?????? full resolution cloud ????????? ????????? / current full resolution cloud message publisher
   ros::Publisher _pubOdomAftMapped;         ///< ?????? odometry ????????? / mapping odometry publisher
   tf::TransformBroadcaster _tfBroadcaster;  ///< ?????? odometry ??????(??????) broadcaster / mapping odometry transform broadcaster

   ros::Subscriber _subLaserCloudCornerLast;   ///< last ????????? cloud ????????? ????????? / last corner cloud message subscriber
   ros::Subscriber _subLaserCloudSurfLast;     ///< last ?????? cloud ????????? ????????? / last surface cloud message subscriber
   ros::Subscriber _subLaserCloudFullRes;      ///< full resolution cloud ????????? ????????? / full resolution cloud message subscriber
   ros::Subscriber _subLaserOdometry;          ///< laser odometry ????????? ????????? / laser odometry message subscriber
   ros::Subscriber _subImu;                    ///< IMU ????????? ????????? / IMU message subscriber

   //////////////////////////////////////////
   ros::Subscriber _subZedTrans;
   ros::Subscriber _subLeftRectified;
   ros::Subscriber _subDepthRectified;
   ros::Subscriber _subLeftcamInfo;
   bool _newLeftcamInfo = false;   // ??? ????????? ?????? ??????????????? ??????????????? ???????????? flag.

   bool _newLeftImg = false;
   bool _newDepthImg = false;

   bool _newZedPose = false;
   bool _newDataSet = false;

   double loaded_zedWorldTrans[6];
   double loaded_transformSum[6];

   cv::Mat loaded_mat_left;
   cv::Mat loaded_mat_depth;
   double loaded_K[9];

   float loaded_depths[1280*720];

   bool dataloaded = false;

   bool saveKvalue = false;
   //////////////////////////////////////////
};

} // end namespace loam

#endif //LOAM_LASERMAPPING_H
