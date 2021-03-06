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

#include "loam_velodyne/TransformMaintenance.h"


namespace loam {

using std::sin;
using std::cos;
using std::asin;
using std::atan2;


TransformMaintenance::TransformMaintenance()
{
  // initialize odometry and odometry tf messages
  _laserOdometry2.header.frame_id = "/camera_init";
  _laserOdometry2.child_frame_id = "/camera";

  _laserOdometryTrans2.frame_id_ = "/camera_init";
  _laserOdometryTrans2.child_frame_id_ = "/camera";


  for (int i = 0; i < 6; i++) {
    _transformSum[i] = 0;
    _transformIncre[i] = 0;
    _transformMapped[i] = 0;
    _transformBefMapped[i] = 0;
    _transformAftMapped[i] = 0;
  }  
}

TransformMaintenance::~TransformMaintenance()
{
  fclose(fp);
}


bool TransformMaintenance::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode)
{
  // advertise integrated laser odometry topic
  _pubLaserOdometry2 = node.advertise<nav_msgs::Odometry> ("/integrated_to_init", 5);

  // subscribe to laser odometry and mapping odometry topics
  _subLaserOdometry = node.subscribe<nav_msgs::Odometry>
      ("/laser_odom_to_init", 5, &TransformMaintenance::laserOdometryHandler, this);

  _subOdomAftMapped = node.subscribe<nav_msgs::Odometry>
      ("/aft_mapped_to_init", 5, &TransformMaintenance::odomAftMappedHandler, this);
  
  std::string odom_file; //pose file with KITTI calibration tf_cal 
  odom_file = privateNode.param<std::string>("odom_file", "/home/whu/data/ndt_odom_KITTI/KITTI_0X_odom.txt"); 
  fp = fopen(odom_file.c_str(),"w");    
  tf_velo2cam.setBasis(tf::Matrix3x3(   
   4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03,
  -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, 
   9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03));
  tf_velo2cam.setOrigin(tf::Vector3(-1.198459927713e-02,-5.403984729748e-02,-2.921968648686e-01)); 
  tf_zj2velo.setBasis(tf::Matrix3x3(0,0,1,1,0,0,0,1,0));
  tf_zj2velo.setOrigin(tf::Vector3(0,0,0)); 

  //初值
  tf::Matrix3x3 R(1,0,0,0,1,0,0,0,1);
  //R=tf_velo2cam.getBasis()*R;
  tf::Vector3 T(0,0,0);
  fprintf(fp,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	   R[0][0],R[0][1],R[0][2],T[0],
	   R[1][0],R[1][1],R[1][2],T[1],
	   R[2][0],R[2][1],R[2][2],T[2]);

  last_t = ros::WallTime::now();
  
  return true;
}



void TransformMaintenance::transformAssociateToMap()
{
  float x1 = cos(_transformSum[1]) * (_transformBefMapped[3] - _transformSum[3])
             - sin(_transformSum[1]) * (_transformBefMapped[5] - _transformSum[5]);
  float y1 = _transformBefMapped[4] - _transformSum[4];
  float z1 = sin(_transformSum[1]) * (_transformBefMapped[3] - _transformSum[3])
             + cos(_transformSum[1]) * (_transformBefMapped[5] - _transformSum[5]);

  float x2 = x1;
  float y2 = cos(_transformSum[0]) * y1 + sin(_transformSum[0]) * z1;
  float z2 = -sin(_transformSum[0]) * y1 + cos(_transformSum[0]) * z1;

  _transformIncre[3] = cos(_transformSum[2]) * x2 + sin(_transformSum[2]) * y2;
  _transformIncre[4] = -sin(_transformSum[2]) * x2 + cos(_transformSum[2]) * y2;
  _transformIncre[5] = z2;

  float sbcx = sin(_transformSum[0]);
  float cbcx = cos(_transformSum[0]);
  float sbcy = sin(_transformSum[1]);
  float cbcy = cos(_transformSum[1]);
  float sbcz = sin(_transformSum[2]);
  float cbcz = cos(_transformSum[2]);

  float sblx = sin(_transformBefMapped[0]);
  float cblx = cos(_transformBefMapped[0]);
  float sbly = sin(_transformBefMapped[1]);
  float cbly = cos(_transformBefMapped[1]);
  float sblz = sin(_transformBefMapped[2]);
  float cblz = cos(_transformBefMapped[2]);

  float salx = sin(_transformAftMapped[0]);
  float calx = cos(_transformAftMapped[0]);
  float saly = sin(_transformAftMapped[1]);
  float caly = cos(_transformAftMapped[1]);
  float salz = sin(_transformAftMapped[2]);
  float calz = cos(_transformAftMapped[2]);

  float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
              - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                           - calx*salz*(cbly*cblz + sblx*sbly*sblz)
                           + cblx*salx*sbly)
              - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                           - calx*calz*(sbly*sblz + cbly*cblz*sblx)
                           + cblx*cbly*salx);
  _transformMapped[0] = -asin(srx);

  float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                       - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
                 - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                              + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx)
                              - calx*cblx*cbly*saly)
                 + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                              + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly)
                              + calx*cblx*saly*sbly);
  float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                       - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
                 + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                              + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz)
                              + calx*caly*cblx*cbly)
                 - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                              + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz)
                              - calx*caly*cblx*sbly);
  _transformMapped[1] = atan2(srycrx / cos(_transformMapped[0]),
                             crycrx / cos(_transformMapped[0]));

  float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                                               - calx*calz*(sbly*sblz + cbly*cblz*sblx)
                                               + cblx*cbly*salx)
                 - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                                                 - calx*salz*(cbly*cblz + sblx*sbly*sblz)
                                                 + cblx*salx*sbly)
                 + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
  float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                                               - calx*salz*(cbly*cblz + sblx*sbly*sblz)
                                               + cblx*salx*sbly)
                 - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                                                 - calx*calz*(sbly*sblz + cbly*cblz*sblx)
                                                 + cblx*cbly*salx)
                 + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
  _transformMapped[2] = atan2(srzcrx / cos(_transformMapped[0]),
                             crzcrx / cos(_transformMapped[0]));

  x1 = cos(_transformMapped[2]) * _transformIncre[3] - sin(_transformMapped[2]) * _transformIncre[4];
  y1 = sin(_transformMapped[2]) * _transformIncre[3] + cos(_transformMapped[2]) * _transformIncre[4];
  z1 = _transformIncre[5];

  x2 = x1;
  y2 = cos(_transformMapped[0]) * y1 - sin(_transformMapped[0]) * z1;
  z2 = sin(_transformMapped[0]) * y1 + cos(_transformMapped[0]) * z1;

  _transformMapped[3] = _transformAftMapped[3]
                       - (cos(_transformMapped[1]) * x2 + sin(_transformMapped[1]) * z2);
  _transformMapped[4] = _transformAftMapped[4] - y2;
  _transformMapped[5] = _transformAftMapped[5]
                       - (-sin(_transformMapped[1]) * x2 + cos(_transformMapped[1]) * z2);
}



void TransformMaintenance::laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  _transformSum[0] = -pitch;
  _transformSum[1] = -yaw;
  _transformSum[2] = roll;

  _transformSum[3] = laserOdometry->pose.pose.position.x;
  _transformSum[4] = laserOdometry->pose.pose.position.y;
  _transformSum[5] = laserOdometry->pose.pose.position.z;

  transformAssociateToMap();

  geoQuat = tf::createQuaternionMsgFromRollPitchYaw
      (_transformMapped[2], -_transformMapped[0], -_transformMapped[1]);

  _laserOdometry2.header.stamp = laserOdometry->header.stamp;
  _laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
  _laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
  _laserOdometry2.pose.pose.orientation.z = geoQuat.x;
  _laserOdometry2.pose.pose.orientation.w = geoQuat.w;
  _laserOdometry2.pose.pose.position.x = _transformMapped[3];
  _laserOdometry2.pose.pose.position.y = _transformMapped[4];
  _laserOdometry2.pose.pose.position.z = _transformMapped[5];
  _pubLaserOdometry2.publish(_laserOdometry2);

  _laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
  _laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  _laserOdometryTrans2.setOrigin(tf::Vector3(_transformMapped[3], _transformMapped[4], _transformMapped[5]));
  _tfBroadcaster2.sendTransform(_laserOdometryTrans2);
  
  //pose file with KITTI calibration tf_cal
  WriteOdom();

}



void TransformMaintenance::odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  _transformAftMapped[0] = -pitch;
  _transformAftMapped[1] = -yaw;
  _transformAftMapped[2] = roll;

  _transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
  _transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
  _transformAftMapped[5] = odomAftMapped->pose.pose.position.z;

  _transformBefMapped[0] = odomAftMapped->twist.twist.angular.x;
  _transformBefMapped[1] = odomAftMapped->twist.twist.angular.y;
  _transformBefMapped[2] = odomAftMapped->twist.twist.angular.z;

  _transformBefMapped[3] = odomAftMapped->twist.twist.linear.x;
  _transformBefMapped[4] = odomAftMapped->twist.twist.linear.y;
  _transformBefMapped[5] = odomAftMapped->twist.twist.linear.z;
}


/**
* @brief  write odom file,
* @return void
*/
void TransformMaintenance::WriteOdom()
{
  //laserOdometryTrans2 is in the zj frame
  tf::Transform odom_velo= tf_zj2velo*_laserOdometryTrans2*tf_zj2velo.inverse();
  tf::Transform odom_cam=tf_velo2cam*odom_velo*tf_velo2cam.inverse();
  tf::Matrix3x3 R= odom_cam.getBasis();
  tf::Vector3 T=odom_cam.getOrigin();
  fprintf(fp,"%le %le %le %le %le %le %le %le %le %le %le %le\n",
	   R[0][0],R[0][1],R[0][2],T[0],
	   R[1][0],R[1][1],R[1][2],T[1],
	   R[2][0],R[2][1],R[2][2],T[2]);
  
  //screet print 
  ros::WallTime t2 = ros::WallTime::now();
  ros::Time stamp=_laserOdometryTrans2.stamp_;
  std::cout <<" "<< stamp<< "--t: " << (t2 - last_t).toSec() <<std::endl;
  last_t=t2;
    
}

} // end namespace loam
