#include <ros/ros.h>
#include "loam_velodyne/MultiScanRegistration.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "scanRegistration");
  // ros系统对话，发布订阅话题消息
  ros::NodeHandle node;
  //明确私有命名空间的节点句柄，配置参数
  //节点拥有相对名字<node_namespace>/my_namespace,而不仅仅是<node_namespace>
  ros::NodeHandle privateNode("~");

  loam::MultiScanRegistration multiScan;

  if (multiScan.setup(node, privateNode)) {
    // initialization successful
    // 回调函数旋转
    ros::spin();
  }

  return 0;
}
