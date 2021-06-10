/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <base_local_planner/trajectory_planner.h>
#include <costmap_2d/footprint.h>
#include <string>
#include <sstream>
#include <math.h>
#include <angles/angles.h>



#include <boost/algorithm/string.hpp>

#include <ros/console.h>

//for computing path distance
#include <queue>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>

using namespace std;
using namespace costmap_2d;

namespace base_local_planner{

  void TrajectoryPlanner::reconfigure(BaseLocalPlannerConfig &cfg)
  {
      BaseLocalPlannerConfig config(cfg);

      boost::mutex::scoped_lock l(configuration_mutex_);

      acc_lim_x_ = config.acc_lim_x;
      acc_lim_y_ = config.acc_lim_y;
      acc_lim_theta_ = config.acc_lim_theta;

      max_vel_x_ = config.max_vel_x;
      min_vel_x_ = config.min_vel_x;
      
      max_vel_th_ = config.max_vel_theta;
      min_vel_th_ = config.min_vel_theta;
      min_in_place_vel_th_ = config.min_in_place_vel_theta;

      sim_time_ = config.sim_time;
      sim_granularity_ = config.sim_granularity;
      angular_sim_granularity_ = config.angular_sim_granularity;

      path_distance_bias_ = config.path_distance_bias;
      goal_distance_bias_ = config.goal_distance_bias;
      occdist_scale_ = config.occdist_scale;

      if (meter_scoring_) {
        //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
        double resolution = costmap_.getResolution();
        goal_distance_bias_ *= resolution;
        path_distance_bias_ *= resolution;
      }

      oscillation_reset_dist_ = config.oscillation_reset_dist;
      escape_reset_dist_ = config.escape_reset_dist;
      escape_reset_theta_ = config.escape_reset_theta;

      vx_samples_ = config.vx_samples;
      vtheta_samples_ = config.vtheta_samples;

      if (vx_samples_ <= 0) {
          config.vx_samples = 1;
          vx_samples_ = config.vx_samples;
          ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
      }
      if(vtheta_samples_ <= 0) {
          config.vtheta_samples = 1;
          vtheta_samples_ = config.vtheta_samples;
          ROS_WARN("You've specified that you don't want any samples in the theta dimension. We'll at least assume that you want to sample one value... so we're going to set vtheta_samples to 1 instead");
      }

      heading_lookahead_ = config.heading_lookahead;

      holonomic_robot_ = config.holonomic_robot;
      
      backup_vel_ = config.escape_vel;

      dwa_ = config.dwa;

      heading_scoring_ = config.heading_scoring;
      heading_scoring_timestep_ = config.heading_scoring_timestep;

      simple_attractor_ = config.simple_attractor;

      //y-vels
      string y_string = config.y_vels;
      vector<string> y_strs;
      boost::split(y_strs, y_string, boost::is_any_of(", "), boost::token_compress_on);

      vector<double>y_vels;
      for(vector<string>::iterator it=y_strs.begin(); it != y_strs.end(); ++it) {
          istringstream iss(*it);
          double temp;
          iss >> temp;
          y_vels.push_back(temp);
          //ROS_INFO("Adding y_vel: %e", temp);
      }

      y_vels_ = y_vels;
      
  }

  TrajectoryPlanner::TrajectoryPlanner(WorldModel& world_model,
      const Costmap2D& costmap,
      std::vector<geometry_msgs::Point> footprint_spec,
      double acc_lim_x, double acc_lim_y, double acc_lim_theta,
      double sim_time, double sim_granularity,
      int vx_samples, int vtheta_samples,
      double path_distance_bias, double goal_distance_bias, double occdist_scale,
      double heading_lookahead, double oscillation_reset_dist,
      double escape_reset_dist, double escape_reset_theta,
      bool holonomic_robot,
      double max_vel_x, double min_vel_x,
      double max_vel_th, double min_vel_th, double min_in_place_vel_th,
      double backup_vel,
      bool dwa, bool heading_scoring, double heading_scoring_timestep, bool meter_scoring, bool simple_attractor,
      vector<double> y_vels, double stop_time_buffer, double sim_period, double angular_sim_granularity)
    : path_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()),
      goal_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()),
      costmap_(costmap),
    world_model_(world_model), footprint_spec_(footprint_spec),
    sim_time_(sim_time), sim_granularity_(sim_granularity), angular_sim_granularity_(angular_sim_granularity),
    vx_samples_(vx_samples), vtheta_samples_(vtheta_samples),
    path_distance_bias_(path_distance_bias), goal_distance_bias_(goal_distance_bias), occdist_scale_(occdist_scale),
    acc_lim_x_(acc_lim_x), acc_lim_y_(acc_lim_y), acc_lim_theta_(acc_lim_theta),
    prev_x_(0), prev_y_(0), escape_x_(0), escape_y_(0), escape_theta_(0), heading_lookahead_(heading_lookahead),
    oscillation_reset_dist_(oscillation_reset_dist), escape_reset_dist_(escape_reset_dist),
    escape_reset_theta_(escape_reset_theta), holonomic_robot_(holonomic_robot),
    max_vel_x_(max_vel_x), min_vel_x_(min_vel_x),
    max_vel_th_(max_vel_th), min_vel_th_(min_vel_th), min_in_place_vel_th_(min_in_place_vel_th),
    backup_vel_(backup_vel),
    dwa_(dwa), heading_scoring_(heading_scoring), heading_scoring_timestep_(heading_scoring_timestep),
    simple_attractor_(simple_attractor), y_vels_(y_vels), stop_time_buffer_(stop_time_buffer), sim_period_(sim_period)
  {
    //the robot is not stuck to begin with
    stuck_left = false;
    stuck_right = false;
    stuck_left_strafe = false;
    stuck_right_strafe = false;
    rotating_left = false;
    rotating_right = false;
    strafe_left = false;
    strafe_right = false;

    escaping_ = false;
    final_goal_position_valid_ = false;


    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, inscribed_radius_, circumscribed_radius_);
  }

  TrajectoryPlanner::~TrajectoryPlanner(){}

  bool TrajectoryPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {
    MapCell cell = path_map_(cx, cy);
    MapCell goal_cell = goal_map_(cx, cy);
    if (cell.within_robot) {
        return false;
    }
    occ_cost = costmap_.getCost(cx, cy);
    if (cell.target_dist == path_map_.obstacleCosts() ||
        cell.target_dist == path_map_.unreachableCellCosts() ||
        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return false;
    }
    path_cost = cell.target_dist;
    goal_cost = goal_cell.target_dist;
    total_cost = path_distance_bias_ * path_cost + goal_distance_bias_ * goal_cost + occdist_scale_ * occ_cost;
    return true;
  }

  //根据机器人当前的速度位置,采样速度，加速度等信息，产生轨迹
  void TrajectoryPlanner::generateTrajectory(
      double x, double y, double theta,
      double vx, double vy, double vtheta,
      double vx_samp, double vy_samp, double vtheta_samp,
      double acc_x, double acc_y, double acc_theta,
      double impossible_cost,
      Trajectory& traj) {

    // 确保运行一半的时候参数不会改变
    boost::mutex::scoped_lock l(configuration_mutex_);
    //记录初始时刻的位姿、速度、角速度
    double x_i = x;
    double y_i = y;
    double theta_i = theta;

    double vx_i, vy_i, vtheta_i;

    vx_i = vx;
    vy_i = vy;
    vtheta_i = vtheta;

    //计算速度的模，非全向机器人即线速度
    double vmag = hypot(vx_samp, vy_samp);

    //计算仿真步数和每一步对应的时间，朝向打分与否对应的步数计算方法略有不同。
    int num_steps;
    if(!heading_scoring_) {
      //sim_granularity_：仿真点之间的距离间隔
      //步数 = max(速度模×总仿真时间/距离间隔，角速度/角速度间隔)，四舍五入
      num_steps = int(max((vmag * sim_time_) / sim_granularity_, fabs(vtheta_samp) / angular_sim_granularity_) + 0.5);
    } else {
      //步数 = 总仿真时间/距离间隔，四舍五入
      num_steps = int(sim_time_ / sim_granularity_ + 0.5);
    }

    //我们希望至少有一步，即使一步都没有，我们也希望能为当前位置打分
    if(num_steps == 0) {
      num_steps = 1;
    }

    double dt = sim_time_ / num_steps;
    double time = 0.0;

    //创造一个潜在的轨迹，代价值暂时默认为-1
    traj.resetPoints();
    traj.xv_ = vx_samp;
    traj.yv_ = vy_samp;
    traj.thetav_ = vtheta_samp;
    traj.cost_ = -1.0;

    //初始化这个轨迹的cost
    double path_dist = 0.0;//路径距离
    double goal_dist = 0.0;//目标距离
    double occ_cost = 0.0;//障碍物代价
    double heading_diff = 0.0;//航向角
    //接下来循环生成轨迹，并计算轨迹对应的代价值，先将当前点从global系转换到地图系，若无法转换到地图上，直接结束轨迹生成，并返回代价-1.0。
    for(int i = 0; i < num_steps; ++i)
    {
      //get map coordinates of a point
      unsigned int cell_x, cell_y;

      //把当前位置(x_i,y_i)转换到地图上，如果无法转换，说明该路径点不在地图上，将其代价设置为-1.0，并return
      if(!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)){
        traj.cost_ = -1.0;
        return;
      }

      //检查当前点路径的合法性
      //转换到地图后，开始考虑机器人的大小，把该点扩张到机器人在该点的足迹范围，并调用footprintCost函数，获得机器人在该点时它的足迹所对应的代价，
      //若返回-1，说明足迹遇障，直接返回-1。这个函数在WorldModel类中声明，在它的派生类CostmapModel中定义，
      //将机器人所在的点扩张到该点足迹，并结合局部规划器的costmap的值返回足迹对应的代价，具体内容下一篇记录。
      double footprint_cost = footprintCost(x_i, y_i, theta_i);

      //如果遇到了障碍物，那么会返回一个负数
      if(footprint_cost < 0){
        traj.cost_ = -1.0;
        return;
      }
      //更新occ_cost：max(max(occ_cost，路径足迹代价)，当前位置的代价)
      //也就是把所有路径点的最大障碍物代价设置为路径的occ_cost
      occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_.getCost(cell_x, cell_y)));

      //接下来考虑与全局路径及目标点的距离：如果simple_attractor_开启，采用比较简单的追踪策略，只考虑与目标点之间的直线距离^2
      //如果只是想简简单单的跟随终点的话，这里调整为true即可
      //经过迭代，goal_dist和path_dist储存的都是路径上最后一个点的对应代价，也就是用这种方法评价一条路径时，若路径有效，【全局路径及目标点的距离】只与路径末点有关。
      //如果开启heading_scoring_即为朝向打分，只有当从开始生成该条路径计时，到达特定时刻时，才会进行唯一一次打分，headingDiff函数的具体过程是：
      //从全局路径终点（目标点）开始，当前点与全局路径上的各点依次连线获得cost，cost为正（无障碍）则立即计算：当前点与迭代到的点间的连线方向与当前点的姿态之差，返回给heading_diff并停止继续连线；
      //若所有连线cost都为负，返回极大值。
      //为朝向打分的轮次将不更新goal_dist和path_dist。
      if (simple_attractor_)
      {
        goal_dist = (x_i - global_plan_[global_plan_.size() -1].pose.position.x) *
          (x_i - global_plan_[global_plan_.size() -1].pose.position.x) +
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y) *
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y);
      }
      else
      {

        bool update_path_and_goal_distances = true;

        //如果有heading scoring，我们要求出航向角，并且为路径某点的路径距离和目标距离打分
        if (heading_scoring_) {
          /**********这里求出本次时间是否是要给航向打分的时间**********/
          //heading_scoring_timestep_是给朝向打分时在时间上要看多远
          //也就是在路径上走过一个特定时刻后，才为朝向打分一次
          if (time >= heading_scoring_timestep_ && time < heading_scoring_timestep_ + dt) {
            //求出航向角
            heading_diff = headingDiff(cell_x, cell_y, x_i, y_i, theta_i);
          } else {
            update_path_and_goal_distances = false;//不更新路径和目标距离
          }
        }
        //// 是否为朝向打分？
        if (update_path_and_goal_distances) {
          //更新路径与目标的距离
          path_dist = path_map_(cell_x, cell_y).target_dist;
          goal_dist = goal_map_(cell_x, cell_y).target_dist;

          //如果一个路径上的点没法明确到达在终点，他就是无效的
          //如果目标距离或路径距离≥impossible_cost（地图尺寸），代价设置为-2.0
          if(impossible_cost <= goal_dist || impossible_cost <= path_dist){
//            ROS_DEBUG("No path to goal with goal distance = %f, path_distance = %f and max cost = %f",
//                goal_dist, path_dist, impossible_cost);
            traj.cost_ = -2.0;
            return;
          }
        }
      }


      //这个点有效，加入轨迹
      traj.addPoint(x_i, y_i, theta_i);

      //迭代计算下一个模拟点速度
      //computeNewVelocity函数作用：不管速度大于还是小于vx_samp，都让其以加速度acc_x接近vx_samp后稳定在vx_samp
      vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
      vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
      vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

      //通过计算出的速度计算下一个位置、姿态
      x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
      y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
      theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

      //增加时间
      time += dt;
    } // end for i < numsteps

    //计算生成路径的代价
    double cost = -1.0;
    if (!heading_scoring_) {
      //代价=路径距离+目标距离+障碍物代价（乘以各自的比例）
      cost = path_distance_bias_ * path_dist + goal_dist * goal_distance_bias_ + occdist_scale_ * occ_cost;
    } else {
      //代价=路径距离+目标距离+障碍物代价+航向角（如果有航偏则会增大代价）
      cost = occdist_scale_ * occ_cost + path_distance_bias_ * path_dist + 0.3 * heading_diff + goal_dist * goal_distance_bias_;
    }
    traj.cost_ = cost;
  }

  double TrajectoryPlanner::headingDiff(int cell_x, int cell_y, double x, double y, double heading){
    unsigned int goal_cell_x, goal_cell_y;

    // find a clear line of sight from the robot's cell to a farthest point on the path
    for (int i = global_plan_.size() - 1; i >=0; --i) {
      if (costmap_.worldToMap(global_plan_[i].pose.position.x, global_plan_[i].pose.position.y, goal_cell_x, goal_cell_y)) {
        if (lineCost(cell_x, goal_cell_x, cell_y, goal_cell_y) >= 0) {
          double gx, gy;
          costmap_.mapToWorld(goal_cell_x, goal_cell_y, gx, gy);
          return fabs(angles::shortest_angular_distance(heading, atan2(gy - y, gx - x)));
        }
      }
    }
    return DBL_MAX;
  }

  //calculate the cost of a ray-traced line
  double TrajectoryPlanner::lineCost(int x0, int x1,
      int y0, int y1){
    //Bresenham Ray-Tracing
    int deltax = abs(x1 - x0);        // The difference between the x's
    int deltay = abs(y1 - y0);        // The difference between the y's
    int x = x0;                       // Start x off at the first pixel
    int y = y0;                       // Start y off at the first pixel

    int xinc1, xinc2, yinc1, yinc2;
    int den, num, numadd, numpixels;

    double line_cost = 0.0;
    double point_cost = -1.0;

    if (x1 >= x0)                 // The x-values are increasing
    {
      xinc1 = 1;
      xinc2 = 1;
    }
    else                          // The x-values are decreasing
    {
      xinc1 = -1;
      xinc2 = -1;
    }

    if (y1 >= y0)                 // The y-values are increasing
    {
      yinc1 = 1;
      yinc2 = 1;
    }
    else                          // The y-values are decreasing
    {
      yinc1 = -1;
      yinc2 = -1;
    }

    if (deltax >= deltay)         // There is at least one x-value for every y-value
    {
      xinc1 = 0;                  // Don't change the x when numerator >= denominator
      yinc2 = 0;                  // Don't change the y for every iteration
      den = deltax;
      num = deltax / 2;
      numadd = deltay;
      numpixels = deltax;         // There are more x-values than y-values
    } else {                      // There is at least one y-value for every x-value
      xinc2 = 0;                  // Don't change the x for every iteration
      yinc1 = 0;                  // Don't change the y when numerator >= denominator
      den = deltay;
      num = deltay / 2;
      numadd = deltax;
      numpixels = deltay;         // There are more y-values than x-values
    }

    for (int curpixel = 0; curpixel <= numpixels; curpixel++) {
      point_cost = pointCost(x, y); //Score the current point

      if (point_cost < 0) {
        return -1;
      }

      if (line_cost < point_cost) {
        line_cost = point_cost;
      }

      num += numadd;              // Increase the numerator by the top of the fraction
      if (num >= den) {           // Check if numerator >= denominator
        num -= den;               // Calculate the new numerator value
        x += xinc1;               // Change the x as appropriate
        y += yinc1;               // Change the y as appropriate
      }
      x += xinc2;                 // Change the x as appropriate
      y += yinc2;                 // Change the y as appropriate
    }

    return line_cost;
  }

  double TrajectoryPlanner::pointCost(int x, int y){
    unsigned char cost = costmap_.getCost(x, y);
    //if the cell is in an obstacle the path is invalid
    if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION){
      return -1;
    }

    return cost;
  }
  //Movebase调用全局规划器生成全局路径后，传入TrajectoryPlannerROS封装类，再通过这个函数传入真正的局部规划器TrajectoryPlanner类中，并且将全局路径的最终点最为目标点final_goal。
  void TrajectoryPlanner::updatePlan(const vector<geometry_msgs::PoseStamped>& new_plan, bool compute_dists)
  {
    global_plan_.resize(new_plan.size());
    for(unsigned int i = 0; i < new_plan.size(); ++i){
      global_plan_[i] = new_plan[i];
    }

    if( global_plan_.size() > 0 ){
      geometry_msgs::PoseStamped& final_goal_pose = global_plan_[ global_plan_.size() - 1 ];
      final_goal_x_ = final_goal_pose.pose.position.x;
      final_goal_y_ = final_goal_pose.pose.position.y;
      final_goal_position_valid_ = true;
    } else {
      final_goal_position_valid_ = false;
    }
    //compute_dists默认为false，即本地规划器在更新全局plan时，不重新计算path_map_和goal_map_。
    if (compute_dists) {
      //reset the map for new operations
      path_map_.resetPathDist();
      goal_map_.resetPathDist();

      //make sure that we update our path based on the global plan and compute costs
      path_map_.setTargetCells(costmap_, global_plan_);
      goal_map_.setLocalGoal(costmap_, global_plan_);
      ROS_DEBUG("Path/Goal distance computed");
    }
  }
  //checkTrajectory调用scoreTrajectory，scoreTrajectory调用generateTrajectory，生成单条路径并返回代价。
  //它们是在足够接近目标时，局部规划器产生降速和自转时生成的对应速度的路径。正常情况下这两个函数并未进行实际工作
  bool TrajectoryPlanner::checkTrajectory(double x, double y, double theta, double vx, double vy,
      double vtheta, double vx_samp, double vy_samp, double vtheta_samp){
    Trajectory t;

    double cost = scoreTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp);

    //if the trajectory is a legal one... the check passes
    if(cost >= 0) {
      return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vx_samp, vy_samp, vtheta_samp, cost);

    //otherwise the check fails
    return false;
  }

  double TrajectoryPlanner::scoreTrajectory(double x, double y, double theta, double vx, double vy,
      double vtheta, double vx_samp, double vy_samp, double vtheta_samp) {
    Trajectory t;
    double impossible_cost = path_map_.obstacleCosts();
    generateTrajectory(x, y, theta,
                       vx, vy, vtheta,
                       vx_samp, vy_samp, vtheta_samp,
                       acc_lim_x_, acc_lim_y_, acc_lim_theta_,
                       impossible_cost, t);

    // return the cost.
    return double( t.cost_ );
  }

  //生成可行的轨迹，并进行打分
  //首先，计算可行的线速度和角速度范围，这里先对最大线速度进行一个限制，即保证速度既不超过预设的最大速度限制，也不超过“起点与目标直线距离/总仿真时间”。
  Trajectory TrajectoryPlanner::createTrajectories(double x, double y, double theta,
      double vx, double vy, double vtheta,
      double acc_x, double acc_y, double acc_theta) {
    //声明最大/小线速度，最大/小角速度
    double max_vel_x = max_vel_x_, max_vel_theta;
    double min_vel_x, min_vel_theta;
    //接近目标点时，降速
    if( final_goal_position_valid_ )
    {
      double final_goal_dist = hypot( final_goal_x_ - x, final_goal_y_ - y );
      max_vel_x = min( max_vel_x, final_goal_dist / sim_time_ );
    }
    //继续计算线速度与角速度的上下限，使用的限制是由当前速度在一段时间内，由最大加减速度达到的速度范围，这里进行了一个判断，即是否使用dwa法：
    //是的话则用的是轨迹前向模拟的周期sim_period_（专用于dwa法计算速度的一个时间间隔，基本是一个控制周期）；
    //不使用dwa法，则用的是整段仿真时间sim_time_,相对较长，可能是1s之类的
    //在生成范围时注意用预先给定的速度和角速度范围参数进行保护。
    if (dwa_) {
      max_vel_x = max(min(max_vel_x, vx + acc_x * sim_period_), min_vel_x_);
      min_vel_x = max(min_vel_x_, vx - acc_x * sim_period_);

      max_vel_theta = min(max_vel_th_, vtheta + acc_theta * sim_period_);
      min_vel_theta = max(min_vel_th_, vtheta - acc_theta * sim_period_);
    } else {
      max_vel_x = max(min(max_vel_x, vx + acc_x * sim_time_), min_vel_x_);
      min_vel_x = max(min_vel_x_, vx - acc_x * sim_time_);

      max_vel_theta = min(max_vel_th_, vtheta + acc_theta * sim_time_);
      min_vel_theta = max(min_vel_th_, vtheta - acc_theta * sim_time_);
    }


    //接下来根据预设的线速度与角速度的采样数，和上面计算得到的范围，分别计算出采样间隔，并把范围内最小的线速度和角速度作为初始采样速度。
    //不考虑全向机器人的情况，即不能y向移动，故暂不考虑vy上采样。
    double dvx = (max_vel_x - min_vel_x) / (vx_samples_ - 1);//计算速度采样间隔
    double dvtheta = (max_vel_theta - min_vel_theta) / (vtheta_samples_ - 1);//计算角速度采样间隔

    double vx_samp = min_vel_x;
    double vtheta_samp = min_vel_theta;
    double vy_samp = 0.0;

    //为了迭代比较不同采样速度生成的不同路径的代价，这里声明best_traj和comp_traj并都将其代价初始化为-1
    Trajectory* best_traj = &traj_one;
    best_traj->cost_ = -1.0;

    Trajectory* comp_traj = &traj_two;
    comp_traj->cost_ = -1.0;

    Trajectory* swap = NULL;

    //设置一个不可能的代价值，目前取地图尺寸作为这个不可能值
    double impossible_cost = path_map_.obstacleCosts();

    //在机器人没有处于逃逸状态时，开始遍历所有线速度和角速度，调用类内generateTrajectory函数用它们生成轨迹。
    //二重迭代时，外层遍历线速度（正值），内层遍历角速度。在遍历时，单独拎出角速度=0，即直线前进的情况，避免由于采样间隔的设置而跃过了这种特殊情况。
    //边迭代生成边比较，获得代价最小的路径，存放在best_traj。
    //执行完成后，也跳出了“未处于逃逸状态”这个判断结构。
    if (!escaping_) {
      //loop through all x velocities
      for(int i = 0; i < vx_samples_; ++i)
      {
        vtheta_samp = 0;
        //首先采样的是直线运动，防止按角度采样时漏掉
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
            acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

        //对比生成路径和当前最优路径的分数，如果生成的路径分数更小，就把当前路径和最优路径交换
        //这里会将第一次生成路径的代价赋给best_traj
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }

        vtheta_samp = min_vel_theta;
        //接下来迭代循环生成所有角速度的路径、打分
        for(int j = 0; j < vtheta_samples_ - 1; ++j)
        {
          generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
              acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

          //同样地，如果新路径代价更小，和best_traj作交换
          if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
            swap = best_traj;
            best_traj = comp_traj;
            comp_traj = swap;
          }
          //遍历角速度
          vtheta_samp += dvtheta;
        }
        //遍历线速度
        vx_samp += dvx;
      }

      //非全向机器人不考虑y向线速度，会跳过这个判断结构。
      if (holonomic_robot_) {
        //explore trajectories that move forward but also strafe slightly
        vx_samp = 0.1;
        vy_samp = 0.1;
        vtheta_samp = 0.0;
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
            acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

        //if the new trajectory is better... let's take it
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }

        vx_samp = 0.1;
        vy_samp = -0.1;
        vtheta_samp = 0.0;
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
            acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

        //if the new trajectory is better... let's take it
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }
      }
    } // end if not escaping

    //接下来，继续考虑原地旋转的情况，来考虑一下这种情况什么时候会发生：
    //1.快要到达目标附近时的原地旋转，根本不会进入到这个函数的这部分来处理。
    //2.由于局部规划器的路径打分机制是：“与目标点的距离”和“与全局路径的偏离”这两项打分都只考虑路径终点的cell，
    //  而不是考虑路径上所有cell的综合效果，机器人运动到一个cell上，哪怕有任何一条能向目标再前进的无障碍路径，它的最终得分一定是要比原地旋转的路径得分来得高的。
    //所以，这里的原地自转，是行进过程中的、未达目标附近时的原地自转，并且，是机器人行进过程中遇到障碍、前方无路可走只好原地自转，
    //或是连原地自转都不能满足，要由逃逸状态后退一段距离，再原地自转调整方向，准备接下来的行动。一种可能情况是机器人行进前方遇到了突然出现而不在地图上的障碍。
    vtheta_samp = min_vel_theta;
    vx_samp = 0.0;
    vy_samp = 0.0;

    //let's try to rotate toward open space
    double heading_dist = DBL_MAX;
   //循环所有角速度
    for(int i = 0; i < vtheta_samples_; ++i)
    {
      //强制最小原地旋转速度,防止太小的速度无法执行
      double vtheta_samp_limited = vtheta_samp > 0 ? max(vtheta_samp, min_in_place_vel_th_)
        : min(vtheta_samp, -1.0 * min_in_place_vel_th_);
      //产生遍历角速度的路径
      generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp_limited,
          acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

      //这里排除y方向的速度不为0的情况是为了排除全向移动机器人的情况，后面单独处理
      if(comp_traj->cost_ >= 0
          && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0 || best_traj->yv_ != 0.0)
          && (vtheta_samp > dvtheta || vtheta_samp < -1 * dvtheta)){
        //获取旋转瞄准的目标点
        double x_r, y_r, th_r;
        comp_traj->getEndpoint(x_r, y_r, th_r);//获取到的是最新加入的点
        x_r += heading_lookahead_ * cos(th_r);
        y_r += heading_lookahead_ * sin(th_r);
        unsigned int cell_x, cell_y;

        //保证瞄准的是一个有效的网格，然后保证瞄准的点是代价值最小的，这保证了原地旋转后能更快到达目标点
        if (costmap_.worldToMap(x_r, y_r, cell_x, cell_y)) {
          double ahead_gdist = goal_map_(cell_x, cell_y).target_dist;
          if (ahead_gdist < heading_dist) {
            //震荡的情况被滤掉了，也就是计算结果是角速度小于0，则上一次的旋转方向标志位就不能是stuck_left
            //震荡抑制能够避免机器人在一个小范围内左右来回乱转。
            if (vtheta_samp < 0 && !stuck_left) {
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
              heading_dist = ahead_gdist;
            }
            //if we haven't already tried rotating right since we've moved forward
            else if(vtheta_samp > 0 && !stuck_right) {
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
              heading_dist = ahead_gdist;
            }
          }
        }
      }

      vtheta_samp += dvtheta;
    }
    //轨迹生成已完成，接下来的工作可以概括为：
    //1.如果轨迹cost非负，即找到有效轨迹，则将其返回；
    //2.若找不到有效轨迹，进入逃逸状态，后退、原地自转，若找不到下一步的有效路径则再后退、自转，直到后退的距离或转过的角度达到一定标准，才会退出逃逸状态，重新规划前向轨迹。
    //其中再加上震荡抑制。
    if (best_traj->cost_ >= 0)
    {
      //嵌套的几个if结构作用是为“震荡”和“逃逸”做记录，不影响轨迹的发布，只要轨迹有效，都会执行到return *best_traj，返回轨迹。
      //抑制震荡影响：当机器人在某方向移动时，对下一个周期的与其相反方向标记为无效，直到机器人从标记震荡的位置处离开一定距离，返回最佳轨迹。
      //由于线速度采样范围是正的，当出现非正采样速度时，只可能是上一次的traj无效，发布了后退命令，进入逃逸模式，然后重新循环，跳过前向轨迹规划，进入原地自转模式，导致采样速度=0
      if ( ! (best_traj->xv_ > 0))
      {
        //若轨迹的角速度<0，标记rotating_right为真，若角速度>0，标记rotating_left为真
        if (best_traj->thetav_ < 0)
        {
          if (rotating_right)
          {
            stuck_right = true;
          }
          rotating_right = true;
        }
        else if (best_traj->thetav_ > 0)
        {
          if (rotating_left)
          {
            stuck_left = true;
          }
          rotating_left = true;
        }
        else if(best_traj->yv_ > 0)
        {
          if (strafe_right)
          {
            stuck_right_strafe = true;
          }
          strafe_right = true;
        }
        else if(best_traj->yv_ < 0)
        {
          if (strafe_left)
          {
            stuck_left_strafe = true;
          }
          strafe_left = true;
        }

        //记录当前自转的位置
        prev_x_ = x;
        prev_y_ = y;
      }
      //直到机器人离开记录的位置一段距离后，以上标志位才会恢复false。
      double dist = hypot(x - prev_x_, y - prev_y_);
      if (dist > oscillation_reset_dist_) {
        rotating_left = false;
        rotating_right = false;
        strafe_left = false;
        strafe_right = false;
        stuck_left = false;
        stuck_right = false;
        stuck_left_strafe = false;
        stuck_right_strafe = false;
      }

      dist = hypot(x - escape_x_, y - escape_y_);
      if(dist > escape_reset_dist_ ||
          fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_){
        escaping_ = false;
      }

      return *best_traj;
    }

    //全向轮机器人采样，暂不考虑
    if (holonomic_robot_) {
      //if we can't rotate in place or move forward... maybe we can move sideways and rotate
      vtheta_samp = min_vel_theta;
      vx_samp = 0.0;

      //loop through all y velocities
      for(unsigned int i = 0; i < y_vels_.size(); ++i){
        vtheta_samp = 0;
        vy_samp = y_vels_[i];
        //sample completely horizontal trajectories
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
            acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

        //if the new trajectory is better... let's take it
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0)){
          double x_r, y_r, th_r;
          comp_traj->getEndpoint(x_r, y_r, th_r);
          x_r += heading_lookahead_ * cos(th_r);
          y_r += heading_lookahead_ * sin(th_r);
          unsigned int cell_x, cell_y;

          //make sure that we'll be looking at a legal cell
          if(costmap_.worldToMap(x_r, y_r, cell_x, cell_y)) {
            double ahead_gdist = goal_map_(cell_x, cell_y).target_dist;
            if (ahead_gdist < heading_dist) {
              //if we haven't already tried strafing left since we've moved forward
              if (vy_samp > 0 && !stuck_left_strafe) {
                swap = best_traj;
                best_traj = comp_traj;
                comp_traj = swap;
                heading_dist = ahead_gdist;
              }
              //if we haven't already tried rotating right since we've moved forward
              else if(vy_samp < 0 && !stuck_right_strafe) {
                swap = best_traj;
                best_traj = comp_traj;
                comp_traj = swap;
                heading_dist = ahead_gdist;
              }
            }
          }
        }
      }
    }

    //do we have a legal trajectory
    if (best_traj->cost_ >= 0) {
      if (!(best_traj->xv_ > 0)) {
        if (best_traj->thetav_ < 0) {
          if (rotating_right){
            stuck_right = true;
          }
          rotating_left = true;
        } else if(best_traj->thetav_ > 0) {
          if(rotating_left){
            stuck_left = true;
          }
          rotating_right = true;
        } else if(best_traj->yv_ > 0) {
          if(strafe_right){
            stuck_right_strafe = true;
          }
          strafe_left = true;
        } else if(best_traj->yv_ < 0) {
          if(strafe_left){
            stuck_left_strafe = true;
          }
          strafe_right = true;
        }

        //set the position we must move a certain distance away from
        prev_x_ = x;
        prev_y_ = y;

      }

      double dist = hypot(x - prev_x_, y - prev_y_);
      if(dist > oscillation_reset_dist_) {
        rotating_left = false;
        rotating_right = false;
        strafe_left = false;
        strafe_right = false;
        stuck_left = false;
        stuck_right = false;
        stuck_left_strafe = false;
        stuck_right_strafe = false;
      }

      dist = hypot(x - escape_x_, y - escape_y_);
      if(dist > escape_reset_dist_ || fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_) {
        escaping_ = false;
      }

      return *best_traj;
    }

    //轨迹有效的部分结束，当轨迹cost为负即无效时，执行接下来的部分，设置一个负向速度，产生让机器人缓慢退后的轨迹。此处也还是判断一下震荡距离。
    vtheta_samp = 0.0;
    vx_samp = backup_vel_;//后退速度
    vy_samp = 0.0;
    generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
        acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);
    //we'll allow moving backwards slowly even when the static map shows it as blocked
    swap = best_traj;
    best_traj = comp_traj;
    comp_traj = swap;

    double dist = hypot(x - prev_x_, y - prev_y_);
    if (dist > oscillation_reset_dist_) {
      rotating_left = false;
      rotating_right = false;
      strafe_left = false;
      strafe_right = false;
      stuck_left = false;
      stuck_right = false;
      stuck_left_strafe = false;
      stuck_right_strafe = false;
    }

    //若后退速度生成的轨迹的终点有效（> -2.0），进入逃逸状态，循环后退、自转，并且记录下的逃逸位置和姿态，只有当离开逃逸位置一定距离或转过一定角度，才能退出逃逸状态，再次规划前向速度。
    //逃逸判断和震荡判断一样，也已在上面多次执行，只要发布best_traj前就执行一次判断。
    //仅当存在有效终点时才进入逃逸模式,若终点无效，返回负cost，本次局部规划失败。
    if (!escaping_ && best_traj->cost_ > -2.0) {
      escape_x_ = x;
      escape_y_ = y;
      escape_theta_ = theta;
      escaping_ = true;
    }

    dist = hypot(x - escape_x_, y - escape_y_);

    if (dist > escape_reset_dist_ ||
        fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_) {
      escaping_ = false;
    }


    //若后退轨迹遇障，还是继续后退，因为后退一点后立刻就会进入原地自转模式。
    if(best_traj->cost_ == -1.0)
      best_traj->cost_ = 1.0;

    return *best_traj;

  }

  //计算应该跟随什么轨迹，给定当前机器人位置和朝向，计算机器人应该跟随的好轨迹。
  //局部规划的整个流程体现在findBestPath函数中，它能够在范围内生成下一步的可能路线，选择出最优路径，并返回该路径对应的下一步的速度。
  Trajectory TrajectoryPlanner::findBestPath(const geometry_msgs::PoseStamped& global_pose,
      geometry_msgs::PoseStamped& global_vel, geometry_msgs::PoseStamped& drive_velocities) {
    //将当前机器人位置和方向转变成float形式的向量
    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));

    //重置地图，清除所有障碍物信息以及地图内容
    path_map_.resetPathDist();
    goal_map_.resetPathDist();

    //暂时移走机器人footprint上的障碍物
    std::vector<base_local_planner::Position2DInt> footprint_list =
        footprint_helper_.getFootprintCells(
            pos,
            footprint_spec_,
            costmap_,
            true);

    //将初始footprint内的所有cell标记 “within_robot = true”，表示在机器人足迹内
    for (unsigned int i = 0; i < footprint_list.size(); ++i) {
      path_map_(footprint_list[i].x, footprint_list[i].y).within_robot = true;
    }

    //确保我们根据全局计划更新map并且计算代价
    //这两个地图是局部规划器中专用的“地图”，即MapGrid类，和costmap的组织形式一样，都以cell为单位，
    //path_map_记录各cell与全局规划路径上的cell之间的距离，goal_map_记录各cell与目标cell之间的距离，
    //再最终计算代价时，将这两个因素纳入考虑，以保证局部规划的结果既贴合全局规划路径、又不致偏离目标。
    path_map_.setTargetCells(costmap_, global_plan_);
    goal_map_.setLocalGoal(costmap_, global_plan_);
    ROS_DEBUG("Path/Goal distance computed");

    //找到代价最低的轨迹。输入分别是目前机器人位置，速度以及加速度限制，生成诸多可能轨迹，选取其中打分最高的。这里也就是最关键的一步。
    Trajectory best = createTrajectories(pos[0], pos[1], pos[2],
        vel[0], vel[1], vel[2],
        acc_lim_x_, acc_lim_y_, acc_lim_theta_);
    ROS_DEBUG("Trajectories created");
    // 如果最后打分如果小于0，说明所有的路径都不可用
    if(best.cost_ < 0){
      drive_velocities.pose.position.x = 0;
      drive_velocities.pose.position.y = 0;
      drive_velocities.pose.position.z = 0;
      drive_velocities.pose.orientation.w = 1;
      drive_velocities.pose.orientation.x = 0;
      drive_velocities.pose.orientation.y = 0;
      drive_velocities.pose.orientation.z = 0;
    }
    else{
      drive_velocities.pose.position.x = best.xv_;
      drive_velocities.pose.position.y = best.yv_;
      drive_velocities.pose.position.z = 0;
      tf2::Quaternion q;
      q.setRPY(0, 0, best.thetav_);
      tf2::convert(q, drive_velocities.pose.orientation);
    }

    return best;
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double TrajectoryPlanner::footprintCost(double x_i, double y_i, double theta_i){
    //check if the footprint is legal
    return world_model_.footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_);
  }


  void TrajectoryPlanner::getLocalGoal(double& x, double& y){
    x = path_map_.goal_x_;
    y = path_map_.goal_y_;
  }

};


