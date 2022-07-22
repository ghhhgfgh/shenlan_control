/**
 * @Author: YunKai Xia
 * @Date:   2022-06-06 21:44:42
 * @Last Modified by:   YunKai Xia
 * @Last Modified time: 2022-06-09 22:37:04
 */
#include "stanley_control_node.h"

#include <iostream>
using namespace std;

StanleyControlNode::StanleyControlNode() : pnh_("~") {}
StanleyControlNode::~StanleyControlNode() {}

bool StanleyControlNode::init() {
  std::string vehicle_odom_topic;
  std::string vehicle_cmd_topic;
  std::string roadmap_path;
  std::string path_vis_topic;
  std::string frame_id;
  double speed_P, speed_I, speed_D, target_speed, k_y, vis_frequency;
  pnh_.getParam("vehicle_odom_topic",
                vehicle_odom_topic);  //读取车辆定位的topic名
  pnh_.getParam("vehicle_cmd_topic",
                vehicle_cmd_topic);             //读取车辆控制的topic名
  pnh_.getParam("roadmap_path", roadmap_path);  //读取路网文件名
  pnh_.getParam("path_vis_topic", path_vis_topic);  //读取可视化路网名
  pnh_.getParam("target_speed", target_speed);      //读取目标速度
  pnh_.getParam("goal_tolerance", goalTolerance_);  //读取目标速度
  pnh_.getParam("speed_P", speed_P);                //读取PID参数
  pnh_.getParam("speed_I", speed_I);
  pnh_.getParam("speed_D", speed_D);
  pnh_.getParam("control_frequency", controlFrequency_);  //读取控制的频率
  pnh_.getParam("vis_frequency", vis_frequency);  //读取路网显示的频率
  pnh_.getParam("k_y", k_y);                      //读取stanley控制参数
  pnh_.getParam("frame_id", frame_id);            //读取全局坐标系名
  //加载路网文件
  if (!loadRoadmap(roadmap_path, target_speed)) return false;

  targetSpeed_ = target_speed;  //设定目标速度

  roadmapMarkerPtr_ =
      std::shared_ptr<RosVizTools>(new RosVizTools(nh_, path_vis_topic));

  stanleyController_ =
      std::make_unique<StanleyController>();  //初始化stanley控制器
  speedPidControllerPtr_ = std::shared_ptr<PIDController>(
      new PIDController(speed_P, speed_I, speed_D));  //初始化PID速度控制器

  VehiclePoseSub_ = nh_.subscribe(vehicle_odom_topic, 10,
                                  &StanleyControlNode::odomCallback, this);
  controlPub_ =
      nh_.advertise<lgsvl_msgs::VehicleControlData>(vehicle_cmd_topic, 1000);

  visTimer_ = nh_.createTimer(ros::Duration(1 / vis_frequency),
                              &StanleyControlNode::visTimerLoop,
                              this);  //注册可视化线程
  controlTimer_ = nh_.createTimer(ros::Duration(1 / controlFrequency_),
                                  &StanleyControlNode::controlTimerLoop,
                                  this);     //这侧控制线程
  stanleyController_->LoadControlConf(k_y);  //设置stanley调整参数

  addRoadmapMarker(planningPublishedTrajectory_.trajectory_points, frame_id);
  goalPoint_ =
      planningPublishedTrajectory_.trajectory_points.back();  //确定目标点
  ROS_INFO("stanley_control_node init finish!");
  return true;
}
double StanleyControlNode::pointDistance(const TrajectoryPoint& point,
                                         const double x, const double y) {
  double dx = point.x - x;
  double dy = point.y - y;
  return sqrt(dx * dx + dy * dy);
}
double StanleyControlNode::pointDistance(const double x1, const double y1,
                                         const double x, const double y) {
  double dx = x1 - x;
  double dy = y1 - y;
  return sqrt(dx * dx + dy * dy);
}
double StanleyControlNode::pid_control() {
  double ego_speed = std::sqrt(vehicleState_.vx * vehicleState_.vx +
                               vehicleState_.vy * vehicleState_.vy);
  return speedPidControllerPtr_->Control(targetSpeed_ - ego_speed,
                                         1 / controlFrequency_);
}
void StanleyControlNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  if (!firstRecord_) firstRecord_ = true;
  // ROS_ERROR("I heard: [%f]", msg->pose.pose.position.x);
  vehicleState_.vx = msg->twist.twist.linear.x;
  vehicleState_.vy = msg->twist.twist.linear.y;

  // 将orientation(四元数)转换为欧拉角(roll, pitch, yaw)
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(vehicleState_.roll, vehicleState_.pitch,
                          vehicleState_.yaw);

  vehicleState_.heading = vehicleState_.yaw;  // pose.orientation是四元数
  // cout << "vehicle_state_.heading: " << vehicle_state_.heading << endl;

  // 将位置转移到前车轮的中心点
  vehicleState_.x = msg->pose.pose.position.x +
                    std::cos(vehicleState_.heading) * 0.5 * wheelbase_;
  vehicleState_.y = msg->pose.pose.position.y +
                    std::sin(vehicleState_.heading) * 0.5 * wheelbase_;

  // cout << "vehicle_state_.heading: " << vehicle_state_.heading << endl;
  vehicleState_.velocity =
      std::sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                msg->twist.twist.linear.y * msg->twist.twist.linear.y);
  vehicleState_.angular_velocity =
      std::sqrt(msg->twist.twist.angular.x * msg->twist.twist.angular.x +
                msg->twist.twist.angular.y * msg->twist.twist.angular.y);
  vehicleState_.acceleration = 0.0;
}

bool StanleyControlNode::loadRoadmap(const std::string& roadmap_path,
                                     const double target_speed) {
  // 读取参考线路径
  std::ifstream infile;
  infile.open(roadmap_path);  //将文件流对象与文件连接起来
  if (!infile.is_open()) {
    return false;
  }
  std::vector<std::pair<double, double>> xy_points;
  std::string s, x, y;
  while (getline(infile, s)) {
    std::stringstream word(s);
    word >> x;
    word >> y;
    double pt_x = std::atof(x.c_str());
    double pt_y = std::atof(y.c_str());
    xy_points.push_back(std::make_pair(pt_x, pt_y));
  }
  infile.close();
  // Construct the reference_line path profile
  using namespace shenlan::control;
  std::vector<double> headings, accumulated_s, kappas, dkappas;
  //根据离散的点组成的路径，生成路网航向角,累计距离，曲率，曲率的导数
  std::unique_ptr<ReferenceLine> reference_line =
      std::make_unique<ReferenceLine>(xy_points);
  reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas,
                                     &dkappas);

  for (size_t i = 0; i < headings.size(); i++) {
    TrajectoryPoint trajectory_pt;
    trajectory_pt.x = xy_points[i].first;
    trajectory_pt.y = xy_points[i].second;
    trajectory_pt.v = target_speed;
    trajectory_pt.a = 0.0;
    trajectory_pt.heading = headings[i];
    trajectory_pt.kappa = kappas[i];
    planningPublishedTrajectory_.trajectory_points.push_back(trajectory_pt);
  }
  return true;
}
void StanleyControlNode::addRoadmapMarker(
    const std::vector<TrajectoryPoint>& path, const std::string& frame_id) {
  roadmapMarkerPtr_->clear();
  std::string ns = "reference_path";
  visualization_msgs::Marker marker_linestrip = RosVizTools::newLineStrip(
      0.02, ns, 0, ros_viz_tools::LIGHT_BLUE, frame_id);
  for (auto path_point : path) {
    geometry_msgs::Point p;
    p.x = path_point.x;
    p.y = path_point.y;
    p.z = 0;
    marker_linestrip.points.push_back(p);
  }
  std::cout << "path size is " << marker_linestrip.points.size() << std::endl;
  roadmapMarkerPtr_->append(marker_linestrip);
  return;
}
void StanleyControlNode::visTimerLoop(const ros::TimerEvent&) {
  // std::cout << "publish path vis " << std::endl;
  roadmapMarkerPtr_->publish();
}

void StanleyControlNode::controlTimerLoop(const ros::TimerEvent&) {
  ControlCmd cmd;
  if (firstRecord_) {  //有定位数据开始控制
    stanleyController_->ComputeControlCmd(vehicleState_,
                                          planningPublishedTrajectory_, cmd);

    //小于容忍距离，车辆速度设置为0
    if (pointDistance(goalPoint_, vehicleState_.x, vehicleState_.y) <
        goalTolerance_) {
      targetSpeed_ = 0;
    }
    lgsvl_msgs::VehicleControlData control_cmd;
    control_cmd.header.stamp = ros::Time::now();
    double acc_cmd = pid_control();
    control_cmd.acceleration_pct = acc_cmd;
    control_cmd.target_gear = lgsvl_msgs::VehicleControlData::GEAR_DRIVE;
    control_cmd.target_wheel_angle = cmd.steer_target;
    controlPub_.publish(control_cmd);
  }
}
