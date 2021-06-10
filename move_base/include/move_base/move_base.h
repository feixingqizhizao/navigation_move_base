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
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"

namespace move_base {
  //typedefs to help us out with the action server so that we don't hace to type so much
  //声明server端，消息类型是move_base_msgs::MoveBaseAction
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;
  //枚举movebase状态表示
  //一般默认状态或者接收到一个有效goal时是PLANNING，在规划出全局路径后state_会由PLANNING变为CONTROLLING，
  //如果规划失败则由PLANNING变为CLEARING
  enum MoveBaseState {
    PLANNING,
    CONTROLLING,
    CLEARING
  };
  //枚举，触发恢复模式,比如从PLANNING 状态 进入 Recovery 则为 PLANNING_R
  enum RecoveryTrigger
  {
    PLANNING_R,//全局规划失败
    CONTROLLING_R,//局部规划失败
    OSCILLATION_R//长时间困在一片小区域
  };

  /**
   * @class MoveBase
   * @brief MoveBase类，使用actionlib：：ActionServer接口，该接口将robot移动到目标位置
   */
  class MoveBase {
    public:
      /**
       * @brief  构造函数，传入的参数是tf
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      MoveBase(tf2_ros::Buffer& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBase();

      /**
       * @brief  控制闭环、全局规划、 到达目标返回true，没有到达返回false
       * @param goal A reference to the goal to pursue
       * @return True if processing of the goal is done, false otherwise
       */
      bool executeCycle(geometry_msgs::PoseStamped& goal);

    private:
      /**
       * @brief 清除costmap的server端
       * @param req 表示server的request
       * @param resp 表示server的response
       * @return 如果server端被成功调用则为True，否则false
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  当action不活跃时，调用此函数，返回plan
       * @param  req 表示goal的request
       * @param  resp 表示plan的request
       * @return 规划成功返回reue，否则返回false
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

      /**
       * @brief  新的全局规划
       * @param  goal 规划的目标点
       * @param  plan 规划
       * @return  规划成功则返回True 否则false
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  从参数服务器加载导航参数Load the recovery behaviors
       * @param node 表示 ros::NodeHandle 加载的参数
       * @return 加载成功返回True 否则 false
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  加载默认导航恢复行为
       */
      void loadDefaultRecoveryBehaviors();


      /**
       * @brief  清楚机器人局部规划框的障碍物
       * @param size_x 局部规划框的长x
       * @param size_y 局部规划框的宽y
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  发布速度为0的指令
       */
      void publishZeroVelocity();

      /**
       * @brief  重置move_base action的状态，设置速度为0
       */
      void resetState();

      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      void planThread();

      /**
       * @brief  movebase这个actionlib服务的回调函数,收到goal，进行
       * @param size_x 局部规划框的长x
       * @param size_y 局部规划框的宽y
       */
      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      /**
       * @brief 周期性地唤醒规划器
       */
      void wakePlanner(const ros::TimerEvent& event);

      tf2_ros::Buffer& tf_;

      MoveBaseActionServer* as_;//就是actionlib的server端

      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;//局部规划器，加载并创建实例后的指针
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;//costmap的实例化指针

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;//全局规划器，加载并创建实例后的指针
      std::string robot_base_frame_, global_frame_;

      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;//可能是出错后的恢复
      unsigned int recovery_index_;

      geometry_msgs::PoseStamped global_pose_;
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_patience_, controller_patience_;
      int32_t max_planning_retries_;
      uint32_t planning_retries_;
      double conservative_reset_dist_, clearing_radius_;
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_;
      ros::Subscriber goal_sub_;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      bool make_plan_clear_costmap_, make_plan_add_unreachable_goal_;
      double oscillation_timeout_, oscillation_distance_;

      MoveBaseState state_;
      RecoveryTrigger recovery_trigger_;

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      geometry_msgs::PoseStamped oscillation_pose_;
      //用于在ROS包里面动态的加载或卸载plugin
      //想象一个场景, 假设现在有一个ROS包polygon_interface_package, 里面包含一个polygon基类,
      //同时, 另外还有两个不同的polygon, 一个是rectangle, 存在于rectangle_plugin包,
      //另外一个是triangle, 存在于triangle_plugin包. 我们希望rectangle和triangle都被系统支持.

      //ClassLoader(std::string package, std::string base_class,
      //std::string attrib_name = std::string("plugin"),
      //std::vector<std::string> plugin_xml_paths = std::vector<std::string>());
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      //触发哪种规划器
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;//保存最新规划的路径，传给latest_plan_
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;//在executeCycle中传给controller_plan_
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;//控制中用到的路径

       //规划器线程
      bool runPlanner_;//用来启动和关闭规划线程的标志位
      boost::recursive_mutex planner_mutex_;//用来为定义线程锁提供输入变量
      //包括了wait，notify_one 等函数，用于启动和终止线程
      //boost的一种结合了互斥锁的用法，可以使一个线程进入睡眠状态，然后在另一个线程触发唤醒。
      boost::condition_variable_any planner_cond_;
      //规划的目标点,作为一个桥梁，在MoveBase::executeCycle中传递给controller_plan_
      geometry_msgs::PoseStamped planner_goal_;
      boost::thread* planner_thread_;


      boost::recursive_mutex configuration_mutex_;
      dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;
      
      void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

      move_base::MoveBaseConfig last_config_;
      move_base::MoveBaseConfig default_config_;
      bool setup_, p_freq_change_, c_freq_change_;
      bool new_global_plan_;
  };
};
#endif

