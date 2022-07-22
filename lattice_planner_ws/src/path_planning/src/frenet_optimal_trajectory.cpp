/**
 * @Author: YunKai Xia
 * @Date:   2022-07-10 22:28:51
 * @Last Modified by:   YunKai Xia
 * @Last Modified time: 2022-07-18 22:26:04
 */

// ***Description***:
// Many thanks to the author of the Frenet algorithm here, this paper may be
// very helpful to you, "Optimal Trajectory Generation for Dynamic Street
// Scenarios in a Frenet Frame"
// https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame
// Thanks to open source codes, python robotics, this website can help you
// quickly verify some algorithms, which is very useful for beginners.
// https://github.com/AtsushiSakai/PythonRobotics

#include "frenet_optimal_trajectory.h"
#include <algorithm>
#include "ros/ros.h"

namespace shenlan {
#define MAX_SPEED 50.0 / 3.6     // maximum speed [m/s]
#define MAX_ACCEL 3.0            // maximum acceleration [m/ss]
#define MAX_CURVATURE 3.0        // maximum curvature [1/m]
#define MAX_ROAD_WIDTH 2.0       // maximum road width [m]
#define D_ROAD_W 0.5            // road width sampling length [m]
#define DT 0.2                   // time tick [s]
#define MAXT 5.0                 // max prediction time [m]
#define MINT 4.0                 // min prediction time [m]
#define TARGET_SPEED 30.0 / 3.6  // target speed [m/s]
#define D_T_S 5.0 / 3.6          // target speed sampling length [m/s]
#define N_S_SAMPLE 1             // sampling number of target speed
#define ROBOT_RADIUS 1.5         // robot radius [m]

#define KJ 0.1
#define KT 0.1
#define KD 1.0
#define KLAT 1.0
#define KLON 1.0

struct compare
{
    float key;
    compare(float const &i): key(i) {}
 
    bool operator()(float const &i) {
        return (std::abs(i) > key);
    }
};
 
FrenetOptimalTrajectory::FrenetOptimalTrajectory() {}
FrenetOptimalTrajectory::~FrenetOptimalTrajectory() {}

double FrenetOptimalTrajectory::NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

float FrenetOptimalTrajectory::sum_of_power(std::vector<float> value_list) {
  float sum = 0;
  for (float item : value_list) {
    sum += item * item;
  }
  return sum;
};

// 01 获取采样轨迹
Vec_Path FrenetOptimalTrajectory::calc_frenet_paths(float c_speed, float c_d,
                                                    float c_d_d, float c_d_dd,
                                                    float s0) 
{
  std::vector<FrenetPath> fp_list;
  for(float wi = -MAX_ROAD_WIDTH ; wi <= MAX_ROAD_WIDTH ; wi += D_ROAD_W)
  {
    for(float ti = MINT; ti <= MAXT ; ti += DT)
    {
      QuinticPolynomial lat_qp (c_d,c_d_d,c_d_dd,wi,0.0,0.0,ti);
      Vec_f time;
      Vec_f d0_;
      Vec_f d1_;
      Vec_f d2_;
      Vec_f d3_;

      for(float dt = 0.0; dt <= ti; dt += DT)
      {
        time.push_back(dt);
        d0_.push_back(lat_qp.calc_point(dt));
        d1_.push_back(lat_qp.calc_first_derivative(dt));
        d2_.push_back(lat_qp.calc_second_derivative(dt));
        d3_.push_back(lat_qp.calc_third_derivative(dt));
      }

      for(float vi = TARGET_SPEED - N_S_SAMPLE * D_T_S ; vi <= TARGET_SPEED + N_S_SAMPLE * D_T_S ; vi += D_T_S)
      {
        QuarticPolynomial lon_qp (s0,c_speed,0.0,vi,0.0,ti);
        FrenetPath one_trajectory;
        Vec_f s0_;
        Vec_f s1_;
        Vec_f s2_;
        Vec_f s3_;

        for(float dt = 0.0; dt <= ti; dt += DT)
        {
          s0_.push_back(lon_qp.calc_point(dt));
          s1_.push_back(lon_qp.calc_first_derivative(dt));
          s2_.push_back(lon_qp.calc_second_derivative(dt));
          s3_.push_back(lon_qp.calc_third_derivative(dt));
        }        

        float js = 0.0;
        float jd = 0.0;
        float j_la = 0.0;
        for(auto & elem : s3_)
        {js += elem * elem;}        

        for(auto & elem : d3_)
        {jd += elem * elem;}    

        for(auto & elem : d0_)
        {j_la += elem * elem;}    

        float diff_v = TARGET_SPEED - s1_.back();
        float diff_l = 0 - d0_.back();

        one_trajectory.t = time;
        one_trajectory.d = d0_;
        one_trajectory.d_d = d1_;
        one_trajectory.d_dd = d2_;
        one_trajectory.d_ddd = d3_;
        one_trajectory.s = s0_;
        one_trajectory.s_d = s1_;
        one_trajectory.s_dd = s2_;
        one_trajectory.s_ddd = s3_;
        float cd_ = KJ * jd + KT * ti + KD * diff_l * diff_l + 10.0 * j_la;
        float cv_ = KJ * js + KT * ti + KD * diff_v * diff_v;
        one_trajectory.cd = cd_;
        one_trajectory.cv = cv_;
        one_trajectory.cf = KLAT * cd_ + KLON * cv_;

        fp_list.push_back(one_trajectory);
      }
    }
  }
  //完成轨迹采样
  //
  cout<<"totalsize: "<<fp_list.size() << std::endl;
  return fp_list;
};

// 02
// 根据参考轨迹与采样的轨迹数组，计算frenet中的其他曲线参数，如航向角，曲率，ds等参数
void FrenetOptimalTrajectory::calc_global_paths(Vec_Path& path_list,
                                                Spline2D csp)
{
    //计算采样轨迹的其他参数
  for(int num = 0 ; num < (int)path_list.size() ; num ++)
  {
    //std::cout<< "num now: "<<num<<std::endl;
    for(int s_num = 0 ; s_num < (int)(path_list[num].s).size(); s_num ++)
    {
      //std::cout<< "s_num now: "<<s_num<<std::endl;
      float sr = path_list[num].s[s_num];
      //std::cout<< "sr now: "<<sr<<std::endl;      
      Poi_f position = csp.calc_postion(sr);
      float xr = position[0];
      float yr = position[1];
      float theta_r = csp.calc_yaw(sr);
      float l_ = path_list[num].d[s_num];
      //std::cout<< "xr now: "<<xr<<std::endl; 
      //std::cout<< "yr now: "<<yr<<std::endl; 
      //std::cout<< "theta_r now: "<<theta_r<<std::endl; 
      //std::cout<< "l_ now: "<<l_<<std::endl; 
      path_list[num].x.push_back(xr - l_ * std::sin(theta_r));
      path_list[num].y.push_back(yr + l_ * std::cos(theta_r));
      float kappa_r = csp.calc_curvature(sr);
      float ll_ = path_list[num].d_d[s_num];

      //std::cout<< "path_list[num].x[s_num]: "<<path_list[num].x[s_num]<<std::endl; 
      //std::cout<< "path_list[num].y[s_num]: "<<path_list[num].y[s_num]<<std::endl; 
      //std::cout<< "kappa_r now: "<<kappa_r<<std::endl; 
      //std::cout<< "ll_ now: "<<ll_<<std::endl;

      //path_list[num].yaw.push_back(NormalizeAngle(std::atan2(ll_,1.0 - kappa_r * l_) + theta_r));
      float ssr = path_list[num].s_d[s_num];
      path_list[num].v.push_back(std::pow(std::pow((ssr * (1.0 - kappa_r * l_)),2) + std::pow(ll_,2),0.5));
    }

    for(int cu_num = 0;cu_num < (int)(path_list[num].s).size();cu_num ++)
    {
      if(cu_num == (int)(path_list[num].s.size()-1))
      {
        path_list[num].ds.push_back(path_list[num].ds[cu_num -1]);
        path_list[num].yaw.push_back(path_list[num].yaw[cu_num -1]);
      }
      else
      {
        float delta_x = path_list[num].x[cu_num+1] - path_list[num].x[cu_num];
        float delta_y = path_list[num].y[cu_num+1] - path_list[num].y[cu_num];
        path_list[num].ds.push_back(std::sqrt(delta_x*delta_x + delta_y*delta_y));
        path_list[num].yaw.push_back(std::atan2(delta_y,delta_x));
      }
    }

    for(int cu_num = 0;cu_num < (int)(path_list[num].s).size();cu_num ++)
    {
      if(cu_num == (int)(path_list[num].s.size()-1))
      {
        path_list[num].c.push_back(path_list[num].c[cu_num -1]);
      }
      else
      {
        path_list[num].c.push_back((path_list[num].yaw[cu_num + 1] - path_list[num].yaw[cu_num]) / path_list[num].ds[cu_num]);
      }
    }
  }
};

bool FrenetOptimalTrajectory::check_collision(FrenetPath path,
                                              const Vec_Poi ob) {
  for (auto point : ob) {
    for (unsigned int i = 0; i < path.x.size(); i++) {
      float dist = std::pow((path.x[i] - point[0]), 2) +
                   std::pow((path.y[i] - point[1]), 2);
      if (dist <= ROBOT_RADIUS * ROBOT_RADIUS) {
        return false;
      }
    }
  }
  return true;
};
// 03
// 检查路径，通过限制做大速度，最大加速度，最大曲率与避障，选取可使用的轨迹数组
Vec_Path FrenetOptimalTrajectory::check_paths(Vec_Path path_list,
                                              const Vec_Poi ob) 
{
  Vec_Path output_fp_list;
  std::cout << "path_list number 1: " << path_list.size()<<std::endl;
  //补全代码
  for(int i = 0;i < (int)path_list.size() ; i++)
  {
    if (std::any_of(path_list[i].s_d.begin(), path_list[i].s_d.end(), compare(MAX_SPEED))){
      continue;
    }   

    if (std::any_of(path_list[i].s_dd.begin(), path_list[i].s_dd.end(), compare(MAX_ACCEL))){
      continue;
    }  

    if (std::any_of(path_list[i].c.begin(), path_list[i].c.end(), compare(MAX_CURVATURE))){
      continue;
    }      

    if(check_collision(path_list[i],ob) == false){
      continue;
    }

    output_fp_list.push_back(path_list[i]);
  }
  std::cout << "path_list number 2: " << output_fp_list.size()<<std::endl;
  return output_fp_list;
};


// to-do step 1 finish frenet_optimal_planning
FrenetPath FrenetOptimalTrajectory::frenet_optimal_planning(
    Spline2D csp, float s0, float c_speed, float c_d, float c_d_d, float c_d_dd,
    Vec_Poi ob) {
  // 01 获取采样轨迹数组
  Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
  //std::cout<<"GOOD step1" <<std::endl;
  // 02
  // 根据参考轨迹与采样的轨迹数组，计算frenet中的其他曲线参数，如航向角，曲率，ds等参数
  calc_global_paths(fp_list, csp);

  // 03
  // 检查路径，通过限制做大速度，最大加速度，最大曲率与避障，选取可使用的轨迹数组
  Vec_Path save_paths = check_paths(fp_list, ob);

  float min_cost = std::numeric_limits<float>::max();
  FrenetPath final_path;
  for (auto path : save_paths) {
    if (min_cost >= path.cf) {
      min_cost = path.cf;
      final_path = path;
    }
  }

  return final_path;
};

FrenetPath FrenetOptimalTrajectory::frenet_optimal_planning(
    Spline2D csp, const FrenetInitialConditions& frenet_init_conditions,
    Vec_Poi ob) {
  float c_speed = frenet_init_conditions.c_speed;
  float c_d = frenet_init_conditions.c_d;
  float c_d_d = frenet_init_conditions.c_d_d;
  float c_d_dd = frenet_init_conditions.c_d_dd;
  float s0 = frenet_init_conditions.s0;

  Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
  calc_global_paths(fp_list, csp);
  Vec_Path save_paths = check_paths(fp_list, ob);

  float min_cost = std::numeric_limits<float>::max();
  FrenetPath final_path;
  for (auto path : save_paths) {
    if (min_cost >= path.cf) {
      min_cost = path.cf;
      final_path = path;
    }
  }
  return final_path;
}

}  // namespace shenlan
