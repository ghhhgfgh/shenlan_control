/**
 * @Author: YunKai Xia
 * @Date:   2022-06-05 22:28:06
 * @Last Modified by:   YunKai Xia
 * @Last Modified time: 2022-06-09 22:44:10
 */
#include "stanley_control.h"

#include <math.h>

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"

using namespace std;

namespace shenlan {
namespace control {

bool first_time = true;

double atan2_to_PI(const double atan2) { return atan2 * M_PI / 180; }

double PointDistanceSquare(const TrajectoryPoint &point, const double x,
                           const double y) {
  const double dx = point.x - x;
  const double dy = point.y - y;
  return dx * dx + dy * dy;
}

double plus_minus_2pi(double angle)
{
    double control_angle = angle;
    if(angle < -M_PI)
    {
      control_angle = (angle + 2 * M_PI);
    }
    if(angle > M_PI)
    {
      control_angle = (angle - 2 * M_PI);
    }

    return control_angle;
}

void StanleyController::LoadControlConf(const double k_y) { k_y_ = k_y; }

// /** to-do **/ 计算需要的控制命令, 实现对应的stanley模型,并将获得的控制命令传递给汽车
// 提示，在该函数中你需要调用计算误差
void StanleyController::ComputeControlCmd(
    const VehicleState &vehicle_state,
    const TrajectoryData &planning_published_trajectory, ControlCmd &cmd) 
{
    if(first_time){
      trajectory_points_ = planning_published_trajectory.trajectory_points;
      first_time = false;
    }
    double e_th = 0.0;
    double e_cross = 0.0;
    ComputeLateralErrors(vehicle_state.x,vehicle_state.y,vehicle_state.heading,e_cross,e_th);
    double e_cross_theta = atan2(k_y_ * e_cross , (vehicle_state.velocity + 0.01));

    std::cout<<"e_th: "<<e_th<<std::endl;
    std::cout<<"e_cross_theta: "<<e_cross_theta<<std::endl;

    double final_angle = e_cross_theta + e_th;

    if(final_angle < -M_PI /3.0)
    {
        final_angle = -M_PI /2.0;
    }

    if(final_angle > M_PI /3.0)
    {
        final_angle = M_PI/2.0;
    }
    
    cmd.steer_target = -final_angle;

    //TrajectoryPoint thenearestpath = QueryNearestPointByPosition(vehicle_state.x,vehicle_state.y);


}
// /** to-do **/ 计算需要的误差，包括横向误差，纵向误差
void StanleyController::ComputeLateralErrors(const double x, const double y,
                                             const double theta, double &e_y,
                                             double &e_theta) 
{
    TrajectoryPoint TheNearestPathPoint = QueryNearestPointByPosition(x,y);
    std::cout<<"TheNearestPathPoint.heading: "<<TheNearestPathPoint.heading<<std::endl;
    std::cout<<"ODOM theta: "<<theta<<std::endl;
    e_theta = plus_minus_2pi(TheNearestPathPoint.heading - theta);
    e_y = std::sqrt(PointDistanceSquare(TheNearestPathPoint,x,y));

    double dx = x - TheNearestPathPoint.x;
    double dy = y - TheNearestPathPoint.y;

    double fx_check = dx * sin(theta) - dy * cos (theta);
    if(fx_check >= 0){
      e_y *= 1.0;
    }else{
      e_y *= -1.0;
    }
    std::cout<<"e_y: "<<e_y<<std::endl;
}

TrajectoryPoint StanleyController::QueryNearestPointByPosition(const double x,
                                                               const double y) {
  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;

  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  return trajectory_points_[index_min];
}

}  // namespace control
}  // namespace shenlan
