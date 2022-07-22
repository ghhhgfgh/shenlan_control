/**
 * @Author: YunKai Xia
 * @Date:   2022-06-06 21:43:24
 * @Last Modified by:   YunKai Xia
 * @Last Modified time: 2022-06-09 22:08:53
 */

#ifndef __STANLEY_CONTROL_NODE_H__
#define __STANLEY_CONTROL_NODE_H__
#include <fstream>
#include <iostream>
#include <memory>

#include "pid_controller.h"
#include "ros_viz_tools/ros_viz_tools.h"
#include "stanley_control.h"
using namespace shenlan::control;
using namespace ros_viz_tools;
class StanleyControlNode {
 public:
  StanleyControlNode();
  ~StanleyControlNode();
  bool init();

 private:
  //可视化路径的线程函数
  void visTimerLoop(const ros::TimerEvent &);
  //控制线程
  void controlTimerLoop(const ros::TimerEvent &);
  //计算两点之间的距离
  double pointDistance(const TrajectoryPoint &point, const double x,
                       const double y);
  double pointDistance(const double x1, const double y1, const double x,
                       const double y);
  //调用pid控制
  double pid_control();
  //接受定位的回调函数，并转换到车辆前轮坐标
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  //加载路网地图，并设置轨迹的速度信息
  bool loadRoadmap(const std::string &roadmap_path, const double target_speed);
  //将路网转换成可视化的marker数据
  void addRoadmapMarker(const std::vector<TrajectoryPoint> &path,
                        const std::string &frame_id);

 private:
  ros::NodeHandle nh_;                             //句柄
  ros::NodeHandle pnh_;                            //读取配置参数句柄
  ros::Timer visTimer_;                            //可视化的线程
  ros::Timer controlTimer_;                        //控制的线程
  ros::Subscriber VehiclePoseSub_;                 //订阅车辆的定位信息
  ros::Publisher controlPub_;                      //发布控制指令
  std::shared_ptr<RosVizTools> roadmapMarkerPtr_;  //发布可视化路网
  std::shared_ptr<PIDController> speedPidControllerPtr_;  // pid速度控制器
  std::unique_ptr<StanleyController> stanleyController_;  // stanley横向控制器
  VehicleState vehicleState_;                             //全局车辆状态
  TrajectoryData planningPublishedTrajectory_;            //跟踪的轨迹
  bool firstRecord_ = false;       //记录是否有定位
  double targetSpeed_ = 5.0;       //目标速度
  double wheelbase_ = 2.852;       //轴距
  double controlFrequency_ = 100;  //控制频率
  TrajectoryPoint goalPoint_;      //终点
  double goalTolerance_ = 0.5;     //到终点的容忍距离
};

#endif /* __STANLEY_CONTROL_NODE_H__ */
