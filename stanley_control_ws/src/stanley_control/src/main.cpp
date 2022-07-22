/**
 * @Author: YunKai Xia
 * @Date:   2022-06-05 22:27:44
 * @Last Modified by:   YunKai Xia
 * @Last Modified time: 2022-06-09 22:26:58
 */
#include <iostream>

#include "stanley_control_node.h"
using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "stanley_control");
  StanleyControlNode control_node;
  if (!control_node.init()) {
    std::cout << "fail to init stanley_control_node" << std::endl;
    return -1;
  }
  ros::spin();
  return 0;
}
