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
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base {

  MoveBase::MoveBase(tf2_ros::Buffer& tf) :
    tf_(tf),
    as_(NULL),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"), 
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false)
  {
    //创建move_base action,绑定回调函数。定义一个名为move_base的SimpleActionServer。该服务器的Callback为moveBase::executeCb
    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R;

    //加载参数，从参数服务器获取一些参数，包括两个规划器名称、代价地图坐标系、规划频率、控制周期等
    std::string global_planner, local_planner;
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);
    private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    // parameters of make_plan service
    private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
    private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

    //为三种规划器（planner_plan_保存最新规划的路径，传递给latest_plan_，
    //然后latest_plan_通过executeCycle中传给controller_plan_）设置内存缓冲区
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //设置规划器线程，planner_thread_是boost::thread*类型的指针
    //其中，MoveBase::planThread()函数是planner线程的入口。这个函数需要等待actionlib服务器的
    //cbMoveBase::executeCb来唤醒启动。主要作用是调用全局路径规划获取路径，
    //同时保证规划的周期性以及规划超时清除goal
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

    //创建发布者，话题名一个是cmd_vel，一个是cunrrent_goal，一个是goal
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

    ros::NodeHandle action_nh("move_base");
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);

    //提供消息类型为geometry_msgs::PoseStamped的发送goals的接口，
    //比如cb为MoveBase::goalCB，在rviz中输入的目标点就是通过这个函数来响应的
    ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

    //设置costmap参数，技巧是把膨胀层设置为大于机器人的半径
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

    //设置全局路径规划器，planner_costmap_ros_是costmap_2d::Costmap2DROS*类型的实例化指针，
    //planner_是boost::shared_ptr<nav_core::BaseGlobalPlanner>类型
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //初始化全局规划器,实例化全局规划器类型，进行规划器初始化
    try {
      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    //设置局部路径规划器,方法类似于全局路径规划
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //这个时候，tc_就成了局部规划器的实例对象。
    try {
      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    // 开始更新costmap：
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //创建一个服务端，进行全局规划
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    //开始清除地图服务：
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    //如果不小心关闭了costmap， 则停用，正常情况下只有动态调参时可能关闭代价地图
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //加载指定的恢复器，加载不出来则使用默认的，这里包括了找不到路自转360
    if(!loadRecoveryBehaviors(private_nh)){
      loadDefaultRecoveryBehaviors();
    }

    //导航过程基本结束，把状态初始化
    state_ = PLANNING;

    //we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;

    //开启move_base动作器
    as_->start();
    //启动动态参数服务器
    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }
  //回调函数MoveBase::reconfigureCB（），配置了各种参数
  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level)
  {
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    //一旦被调用，我们要确保有原始配置
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      config = default_config_;
      //如果有人在参数服务器上设置默认值，要防止循环
      config.restore_defaults = false;
    }

    if(planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if(controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    max_planning_retries_ = config.max_planning_retries;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;
    if(config.base_global_planner != last_config_.base_global_planner) {
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      //创建全局规划
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try {
        planner_ = bgp_loader_.createInstance(config.base_global_planner);

        // 等待当前规划结束
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

        // 在新规划开始前clear旧的
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

        lock.unlock();
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
        planner_ = old_planner;
        config.base_global_planner = last_config_.base_global_planner;
      }
    }

    if(config.base_local_planner != last_config_.base_local_planner){
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      //创建局部规划
      try {
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        // 清理旧的
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
        tc_ = old_planner;
        config.base_local_planner = last_config_.base_local_planner;
      }
    }

    make_plan_clear_costmap_ = config.make_plan_clear_costmap;
    make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

    last_config_ = config;
  }
  //其中，我们来看MoveBase::goalCB（）函数，传入的是goal，主要作用是为rviz等提供调用借口，
  //将geometry_msgs::PoseStamped形式的goal转换成move_base_msgs::MoveBaseActionGoal，
  //再发布到对应类型的goal话题中
  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y)
  {
    geometry_msgs::PoseStamped global_pose;

    //clear the planner's costmap
    getRobotPose(global_pose, planner_costmap_ros_);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.pose.position.x;
    double y = global_pose.pose.position.y;
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    //clear the controller's costmap
    getRobotPose(global_pose, controller_costmap_ros_);

    clear_poly.clear();
    x = global_pose.pose.position.x;
    y = global_pose.pose.position.y;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }
  //其中，调用了MoveBase::clearCostmapsService（）函数，提供清除一次costmap的功能
  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    //clear the costmaps
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    //值得注意的是，其中有一个函数resetLayers()，调用的是costmap包，该函数的功能是重置地图，内部包括重置总地图、重置地图各层：
    controller_costmap_ros_->resetLayers();

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }

  //其中，MoveBase::planService（）函数写了全局规划的策略，以多少距离向外搜索路径
  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
    //move base 处于激活的状态
    if(as_->isActive()){
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //确保代价地图已经被加载
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }
    //获取起始点，如果没有起始点，那就获取当前的全局位置为起始点
    geometry_msgs::PoseStamped start;
    //如果起始点为空，设置global_pose为起始点
    if(req.start.header.frame_id.empty())
    {
        geometry_msgs::PoseStamped global_pose;
        if(!getRobotPose(global_pose, planner_costmap_ros_)){
          ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
          return false;
        }
        start = global_pose;
    }
    else
    {
        start = req.start;
    }
    //清图
    if (make_plan_clear_costmap_) {
      //update the copy of the costmap the planner uses
      clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
    }
    //制定规划策略：
    std::vector<geometry_msgs::PoseStamped> global_plan;
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty())
    {
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance", 
          req.goal.pose.position.x, req.goal.pose.position.y);

      //寻找可行路径失败后，在规定的公差范围内向外寻找可行的路径
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;//以3倍分辨率的增量向外寻找
      if(req.tolerance > 0.0 && req.tolerance < search_increment)
        search_increment = req.tolerance;//从客户端获取步长值，如果小于3倍分辨率则重新赋值
      for(float max_offset = search_increment; max_offset <= req.tolerance &&
          !found_legal; max_offset += search_increment)
      {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal;
            y_offset += search_increment)
        {
          for(float x_offset = 0; x_offset <= max_offset &&
              !found_legal; x_offset += search_increment)
          {

            //不在本位置的外侧layer查找，太近的不找
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //从两个方向x、y查找精确的goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0)
            {
              //第一次遍历如果偏移量过小,则去除这个点或者上一点
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0)
              {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan))
                {
                  if(!global_plan.empty())
                  {
                    if (make_plan_add_unreachable_goal_)
                    {
                      //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                      //(the reachable goal should have been added by the global planner)
                      global_plan.push_back(req.goal);
                    }

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else
                {
                  ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //然后把规划后的global_plan附给resp，并且传出去：
    resp.plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }

  MoveBase::~MoveBase(){
    recovery_behaviors_.clear();

    delete dsrv_;

    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    tc_.reset();
  }

  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
  {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //规划初始化
    plan.clear();

    //激活句柄时调用
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //得到机器人起始点
    geometry_msgs::PoseStamped global_pose;
    if(!getRobotPose(global_pose, planner_costmap_ros_)) {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    const geometry_msgs::PoseStamped& start = global_pose;

    //如果规划失败或者返回一个长度为0的规划，则规划失败
    //Movebase使用的全局规划器默认为NavFn，默认使用Dijkstra算法，在地图上的起始点和目标点间规划出一条最优路径，
    //供局部规划器具体导航使用。NavFn的源码中实际上是有A*规划算法的函数的，但关于为什么不在NavFn中使用A*，
    //ROS Wiki上对提问者的回答是，早期NavFn包中的A*有bug，没有处理，后来发布了global_planner，
    //修改好了A*的部分。global_planner封装性更强，它和NavFn都是用于全局路径规划的包。
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
  }

  void MoveBase::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //1、首先检查四元数是否元素完整,元素包括nans or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //2、检查四元数是否趋近于0
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }
    //3、对四元数规范化，转化为vector
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg)
  {
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    geometry_msgs::PoseStamped goal_pose, global_pose;
    goal_pose = goal_pose_msg;

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.header.stamp = ros::Time();

    try{
      tf_.transform(goal_pose_msg, global_pose, global_frame);
    }
    catch(tf2::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    return global_pose;
  }

  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }
  //planner线程的入口。这个函数需要等待actionlib服务器的cbMoveBase::executeCb来唤醒启动。
  //主要作用是调用全局路径规划获取路径，同时保证规划的周期性以及规划超时清除goal
  void MoveBase::planThread()
  {
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    //创建递归锁
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    while(n.ok())
    {
      //判断是否阻塞线程
      while(wait_for_wake || !runPlanner_)
      {
        //如果不应该启动规划线程，则将规划线程阻塞
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        //当 std::condition_variable 对象的某个 wait 函数被调用的时候，
        //它使用 std::unique_lock(通过 std::mutex) 来锁住当前线程。
        //当前线程会一直被阻塞，直到另外一个线程在相同的 std::condition_variable 对象上调用了 notification 函数来唤醒当前线程。
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      ros::Time start_time = ros::Time::now();

      //将要进行规划，复制全局目标点，解锁
      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

      //获取规划的全局路径
      //这里的makePlan作用是获取机器人的位姿作为起点，然后调用全局规划器的makePlan返回规划路径，存储在planner_plan_
      planner_plan_->clear();
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);
      //如果规划出路径则更新相应路径，,将其赋值给latest_plan_，并将state_转换为CONTROLLING状态
      if(gotPlan)
      {
        ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;
        //如果能够获得全局规划结果，对规划相关变量进行赋值
        lock.lock();
        planner_plan_     = latest_plan_;
        latest_plan_      = temp_plan;//将最新的全局路径放到latest_plan_中，其在MoveBase::executeCycle中被传递到controller_plan_中，利用锁来进行同步
        last_valid_plan_  = ros::Time::now();
        planning_retries_ = 0;
        new_global_plan_  = true;

        ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

        //规划全局线路后，将状态置为CONTROLLING
        if(runPlanner_)
          state_ = CONTROLLING;
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      //5. 达到一定条件后停止路径规划,进入清障模式
      //如果没有规划出路径，并且处于PLANNING状态，则判断是否超过最大规划周期或者规划次数
      //如果是则进入自转模式，否则应该会等待MoveBase::executeCycle的唤醒再次规划
      else if(state_==PLANNING)
      {
        //仅在MoveBase::executeCb及其调用的MoveBase::executeCycle或者重置状态时会被设置为PLANNING
        //一般是刚获得新目标，或者得到路径但计算不出下一步控制时重新进行路径规划
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        //检查是否规划超时，或是超过最大的规划次数
        //出现如下条件则停止规划, 但是最大规划次数是负数，就忽略
        lock.lock();
        planning_retries_++;
        if(runPlanner_ &&
           (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
          //we'll move into our obstacle clearing mode
          state_ = CLEARING;
          runPlanner_ = false;  // proper solution for issue #523
          publishZeroVelocity();
          recovery_trigger_ = PLANNING_R;
        }

        lock.unlock();
      }

      //take the mutex for the next iteration
      lock.lock();

      //6.设置睡眠模式
      //如果还没到规划周期则定时器睡眠，在定时器中断中通过planner_cond_唤醒，这里规划周期为0
      if(planner_frequency_ > 0)
      {
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0)){
          wait_for_wake = true;
          //planner_cond_专门用于开启路径规划的线程，在其他地方也经常遇到；这一步中，实现了从latest_plan_
          //到planner_plan_ 的跨越；MoveBase::wakePlanner（）函数中用planner_cond_开启了路径规划的线程
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }
  //每次有goal发布的时候，都会去调用executeCb,executeCb会去调用executeCycle
  //进行local plan,发布cmd_vel的topic，根据小车处于的状态，进行相应的实现(会进行plan,control,clear obstacle)
  //此外全局规划的线程也在这里进行唤醒
  //第一次接收到goal时会进入该函数，但如果没有完成任务，尚未退出时，再有接收到goal并不会再新建线程进入一次（应该也可以这样操作，这里并没有这样选择），
  //而是通过抢断信号的形式通知该函数，所以在处理goal的时候需要经常查看isPreemptRequested函数的返回，看是否有抢占。
  //该函数流程是
  // 第一次进入后接收goal，判断有效性等，然后开启规划线程得到路径。
  //然后在while(n.ok())循环中调用executeCycle(goal, global_plan);来控制机器人进行相应跟随。
  //期间会不断检测是否有新的goal抢占，或者坐标系变换等，如果有则在while循环中重复步骤1的初始化再跟随
  //如果有被空抢占（如cancel等）则清除退出
  //如果跟随完成则退出
  //会进行控制周期约束。
  void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    //1.如果目标非法，则直接返回：其中，isQuaternionValid(const geometry_msgs::Quaternion& q)函数，
    //主要用于检查四元数的合法性
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation))
    {
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }
    //将目标的坐标系统一转换为全局坐标系：
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);
  
    publishZeroVelocity();
    //设置目标点并唤醒路径规划线程：
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();//唤醒正在wait的线程
    lock.unlock();
    //然后发布goal，设置控制频率
    current_goal_pub_.publish(goal);

    ros::Rate r(controller_frequency_);
    //当收到目标点时，检查地图是否已经关闭，如果关闭，则需要进行启动
    if(shutdown_costmaps_)
    {
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    //重置时间标志位
    last_valid_control_     = ros::Time::now();
    last_valid_plan_        = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_       = 0;
    //开启循环，循环判断是否有新的goal抢占（重要！！！）
    ros::NodeHandle n;
    while(n.ok())
    {
      //修改循环频率，主要是为动态调参使用
      if(c_freq_change_)
      {
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        r = ros::Rate(controller_frequency_);
        c_freq_change_ = false;
      }
      //如果获得一个抢占式目标
      //isPreemptRequested()允许轮询实现查询抢占请求
      //是否有抢占请求，根据参考1第8点的说法，SimpleActionServer的政策是，新的goal都会抢占旧的goal，这里应该只是为了清除新goal的一些状态。
      if(as_->isPreemptRequested())
      {
        //action的抢断函数,isNewGoalAvailable()判断新目标是否可用
        if(as_->isNewGoalAvailable())
        {
          //如果有新的目标，会接受的，但不会关闭其他进程
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();
          //如果目标点存在无效值,则直接返回
          if(!isQuaternionValid(new_goal.target_pose.pose.orientation))
          {
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }
          //将目标转换为全局坐标系
          goal = goalToGlobalFrame(new_goal.target_pose);

          //重置规划状态，为下一次的执行循环准备，设置状态为PLANNING
          recovery_index_ = 0;
          state_ = PLANNING;

          //确认新的目标点有效时，设置目标点并唤醒路径规划线程
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();//尝试把wait的线程唤醒，执行完这行，线程就被唤醒了
          lock.unlock();

          //把goal发布给可视化工具
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          //重置规划时间
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        else
        {
          //如果目标点不可用，重置状态,设置为抢占式任务
          resetState();
          //通知ActionServer已成功抢占
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          as_->setPreempted();

          //we'll actually return from execute after preempting
          return;
        }
      }
      //如果目标点的坐标系和全局地图的坐标系不相同
      if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID())
      {
        //转换目标点坐标系
        goal = goalToGlobalFrame(goal);

        //we want to go back to the planning state for the next execution cycle
        recovery_index_ = 0;
        state_ = PLANNING;

        //设置目标点并唤醒路径规划线程
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //把goal发布给可视化工具
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);

        //重置规划器相关时间标志位
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }

      //播放rosbag时，若参数/use_sim_time 为true，则此时
      //ros::WallTime::now()为当前的真实时间，也就是墙上的挂钟时间，一直在走。
      //ros::Time::now()为rosbag当时的时间，是由bag中/clock获取的。是仿真时间。
      ros::WallTime start = ros::WallTime::now();

      //到达目标点的真正工作，控制机器人进行跟随
      //其中，done的值即为MoveBase::executeCycle（）函数的返回值，这个值非常重要，
      //直接判断了是否到达目标点；MoveBase::executeCycle（）函数是控制机器人进行跟随的函数
      bool done = executeCycle(goal);

      //如果完成任务则返回
      if(done)
        return;

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());
      //执行休眠动作
      r.sleep();
      //确认控制周期与循环周期相同
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    //唤醒规划线程，以便它可以干净地退出
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //如果节点结束就终止并返回
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  }

  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }
  //两个参数分别是目标点位姿以及规划出的全局路径.通过这两个参数，实现以下功能：
  //利用局部路径规划器直接输出轮子速度，控制机器人按照路径走到目标点，成功返回真，否则返回假。在actionlib server的回调MoveBase::executeCb中被调用。
  //在MoveBase::executeCycle中，会分别对这三种状态做处理：
  //还在PLANNING中则唤醒规划线程让它干活
  //如果已经在CONTROLLING中，判断是否已经到达目的地，否则判断是否出现震动？否则调用局部路径规划，如果成功得到速度则直接发布到cmd_vel，失败则判断是否控制超时，不超时的话让全局再规划一个路径。
 // 如果出现了问题需要CLEARING（仅有全局规划失败、局部规划失败、长时间困在一片小区域三种原因），则每次尝试一种recovery方法，直到所有尝试完
  bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal)
  {
    //typedef unique_lock<recursive_mutex> scoped_lock;
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //定义速度消息类型
    geometry_msgs::Twist cmd_vel;

    //获取机器人当前位置
    geometry_msgs::PoseStamped global_pose;
    getRobotPose(global_pose, planner_costmap_ros_);
    const geometry_msgs::PoseStamped& current_position = global_pose;

    //push the feedback out
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);

    //check to see if we've moved far enough to reset our oscillation timeout
    //判断当前位置和是否振荡，其中distance函数返回的是两个位置的直线距离（欧氏距离），
    //recovery_trigger_是枚举RecoveryTrigger的对象
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_       = current_position;
      //如果上次的恢复是由振荡引起的，重置恢复指数
      if(recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }
    //地图数据超时，即观测传感器数据不够新，停止机器人，返回false
    if(!controller_costmap_ros_->isCurrent())
    {
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return false;
    }
    //如果获取新的全局路径,则将它传输给控制器，完成latest_plan_到controller_plan_的转换
    if(new_global_plan_)
    {
      //进入判断后，将标志位置为false
      new_global_plan_ = false;

      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

      //do a pointer swap under mutex
      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      controller_plan_ = latest_plan_;//将最新的全局规划路径传给controller_plan_
      latest_plan_ = temp_plan;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");
      //将全局路径传给局部路径规划器
      if(!tc_->setPlan(*controller_plan_))
      {
        //ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();

        //disable the planner thread
        lock.lock();
        runPlanner_ = false;
        lock.unlock();
        //设置动作中断,返回true
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;
      }

      //如果全局路径有效，则不需要recovery
      if(recovery_trigger_ == PLANNING_R)
        recovery_index_ = 0;
    }

    //5. move_base 状态机，处理导航的控制逻辑
    //一般默认状态或者接收到一个有效goal时是PLANNING，在规划出全局路径后state_会由PLANNING->CONTROLLING
    //如果规划失败则由PLANNING->CLEARING。
    switch(state_)
    {
      //机器人规划状态，尝试获取一条全局路径
      case PLANNING:
        {
          boost::recursive_mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();//还在PLANNING中则唤醒规划线程让它干活
        }
        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

      //机器人控制状态，尝试获取一个有效的速度命令
      case CONTROLLING:
        ROS_DEBUG_NAMED("move_base","In controlling state.");

        //如果到达目标点，重置状态，设置动作成功，返回true，其中isGoalReached()函数是TrajectoryPlannerROS的函数
        if(tc_->isGoalReached())
        {
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          //其中，resetState（）函数如下，配合上面的看，机器人到达目标点后把move_base状态设置为PLANNING
          resetState();

          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          return true;
        }

        //如果超过震荡时间，停止机器人，设置清障标志位
        if(oscillation_timeout_ > 0.0 &&
            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
        {
          publishZeroVelocity();
          state_ = CLEARING;
          recovery_trigger_ = OSCILLATION_R;
        }
        
        {
         boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
        //获取有效速度,如果获取成功，直接发布到cmd_vel
        if(tc_->computeVelocityCommands(cmd_vel))
        {
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          last_valid_control_ = ros::Time::now();
          //make sure that we send the velocity command to the base
          vel_pub_.publish(cmd_vel);
          if(recovery_trigger_ == CONTROLLING_R)
            recovery_index_ = 0;
        }
        else
        {
          ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
          ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

          //如果没有获取有效速度，则判断是否超过尝试时间，如果超时，则停止机器人，进入清障模式
          if(ros::Time::now() > attempt_end)
          {
            //we'll move into our obstacle clearing mode
            publishZeroVelocity();
            state_ = CLEARING;
            recovery_trigger_ = CONTROLLING_R;
          }
          else
          {
            //如果没有超时，则再全局规划一个新的路径
            last_valid_plan_ = ros::Time::now();
            planning_retries_ = 0;
            state_ = PLANNING;
            publishZeroVelocity();

            //如果没有获得局部规划，且没有超过允许的尝试时长，唤醒全局规划，重新规划全局路径
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();
          }
        }
        }

        break;

      //13. 规划或者控制失败，恢复或者进入到清障模式
      case CLEARING:
        ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
        //如果有可用恢复器，执行恢复动作，并设置状态为PLANNING
        if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size())
        {
          ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_, recovery_behaviors_.size());
          recovery_behaviors_[recovery_index_]->runBehavior();

          //we at least want to give the robot some time to stop oscillating
          //after executing the behavior
          last_oscillation_reset_ = ros::Time::now();

          //清障成功，对相关变量进行赋值
          ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
          last_valid_plan_ = ros::Time::now();
          planning_retries_ = 0;
          state_ = PLANNING;

          //update the index of the next recovery behavior that we'll try
          recovery_index_++;
        }
        //如果没有可用恢复器，结束动作，返回true：
        else
        {
          ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");

          if(recovery_trigger_ == CONTROLLING_R)
          {
            ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == PLANNING_R)
          {
            ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == OSCILLATION_R)
          {
            ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
          }
          resetState();
          return true;
        }
        break;
      default:
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        //disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
        return true;
    }

    //we aren't done yet
    return false;
  }
   //recovery是指恢复的规划器，其跟全局规划器和局部规划器是同一个等级的。
  //不同的是，它是在机器人在局部代价地图或者全局代价地图中找不到路时才会被调用，
  //比如rotate_recovery让机器人原地360°旋转，clear_costmap_recovery将代价地图恢复到静态地图的样子。
  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node)
  {
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.", 
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.", 
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  //we'll load our default recovery behaviors here
  void MoveBase::loadDefaultRecoveryBehaviors()
  {
    recovery_behaviors_.clear();
    try{
      //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      //1、加载recovery behavior清理costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(cons_clear);

      //2、加载recovery behavior 原地旋转
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_){
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behaviors_.push_back(rotate);
      }

      //3、加载 recovery behavior 重置 costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(ags_clear);

      //再旋转一次
      if(clearing_rotation_allowed_)
        recovery_behaviors_.push_back(rotate);
    }
    catch(pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

  void MoveBase::resetState()
  {
    // Disable the planner thread
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_)
    {
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }

  bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time(); // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    //转换到统一的全局坐标
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    //全局坐标时间戳是否在costmap要求下
    if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    {
      ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                        current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }
}
