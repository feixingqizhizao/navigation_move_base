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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
#ifndef NAV_CORE_BASE_LOCAL_PLANNER_H
#define NAV_CORE_BASE_LOCAL_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>

namespace nav_core {
  //局部规划接口类
  class BaseLocalPlanner{
    public:
    /**
     * @brief  被传入机器人当前位置、方向和速度，计算相对于base坐标系的速度
     * @param cmd_vel 机器人的速度
     * @return 有速度指令，则为true
     */
      virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) = 0;

    /**
     * @brief  检查局部规划器的目标位姿是否到达
     * @return 如果到达了返回true
     */
      virtual bool isGoalReached() = 0;

    /**
     * @brief  设置局部规划器正在执行的路径
     * @param plan 即将被传递给局部规划器的路径
     * @return 路径被成功标记则返回true
     */
      virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;

    /**
     * @brief  构建局部规划器
     * @param name 局部规划器的名称
     * @param tf tf的指针
     * @param costmap_ros 局部规划器的costmap
     */
      virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~BaseLocalPlanner(){}

    protected:
      BaseLocalPlanner(){}
  };
}  // namespace nav_core

#endif  // NAV_CORE_BASE_LOCAL_PLANNER_H
