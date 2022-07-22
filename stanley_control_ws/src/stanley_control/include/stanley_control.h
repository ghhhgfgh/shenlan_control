/**
 * @Author: YunKai Xia
 * @Date:   2022-06-05 22:28:26
 * @Last Modified by:   YunKai Xia
 * @Last Modified time: 2022-06-09 22:23:18
 */
#pragma once
#include <math.h>

#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <iostream>

#include "Eigen/Core"
#include "common.h"

namespace shenlan {
namespace control {

using Matrix = Eigen::MatrixXd;

class StanleyController {
 public:
  StanleyController(){};
  ~StanleyController(){};

  void LoadControlConf(const double k_y);
  //计算前轮转角
  void ComputeControlCmd(const VehicleState &vehicle_state,
                         const TrajectoryData &planning_published_trajectory,
                         ControlCmd &cmd);
  //根据车辆当前状态，计算与参考轨迹的误差
  void ComputeLateralErrors(const double x, const double y, const double theta,
                            double &e_y, double &e_theta);
  //根据当前位置，计算轨迹上最近的点
  TrajectoryPoint QueryNearestPointByPosition(const double x, const double y);

 protected:
  std::vector<TrajectoryPoint> trajectory_points_;
  double k_y_ = 0.0;  // stanley控制参数，即公式中k*e(t)中的k
};

}  // namespace control
}  // namespace shenlan
