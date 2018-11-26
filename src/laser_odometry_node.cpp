#include <ros/ros.h>
#include "loam_velodyne/LaserOdometry.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  //loam::LaserOdometry laserOdom(0.1); 
  //第二个参数输入输出比
  loam::LaserOdometry laserOdom(0.1,1);

  if (laserOdom.setup(node, privateNode)) {
    // initialization successful
    laserOdom.spin();
  }

  return 0;
}
