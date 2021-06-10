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
#ifndef TRAJECTORY_ROLLOUT_COSTMAP_MODEL_
#define TRAJECTORY_ROLLOUT_COSTMAP_MODEL_

#include <base_local_planner/world_model.h>
// For obstacle data access
#include <costmap_2d/costmap_2d.h>

namespace base_local_planner {
  /**
   * @class CostmapModel
   * @brief A class that implements the WorldModel interface to provide grid
   * based collision checks for the trajectory controller using the costmap.
   */
  //这个类帮助局部规划器在Costmap上进行计算，
  //footprintCost、lineCost、pointCost三个类方法分别能通过Costmap计算出机器人足迹范围、两个cell连线、单个cell的代价，并将值返回给局部规划器。
  //若预设的足迹点数<3，考虑足迹的形状没有意义，这时只计算机器人位置点在costmap上的代价；
  //若预设的足迹点数≥3，把足迹视为多边形，循环调用lineCost计算多边形各边的cell，注意首尾闭合，最后返回代价。
  //返回值:-1.0表示覆盖至少一个障碍cell；-2.0表示覆盖至少一个未知cell；-3.0表示不在地图上；其他为正cost
  //
  //为什么这里只要多边形边缘不经过障碍就认为足迹不经过障碍呢？
  //因为这个函数是在轨迹点生成过程中，对新生成的轨迹点调用的，而既然能新生成轨迹点，说明该轨迹点必不是障碍物，
  //当多边形边缘不经过障碍、中心也不是障碍的时候，如果机器人比较小，可以认为足迹不经过障碍。
  class CostmapModel : public WorldModel {
    public:
      /**
       * @brief  Constructor for the CostmapModel
       * @param costmap The costmap that should be used
       * @return
       */
      CostmapModel(const costmap_2d::Costmap2D& costmap);

      /**
       * @brief  Destructor for the world model
       */
      virtual ~CostmapModel(){}
      using WorldModel::footprintCost;

      /**
       * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
       * @param  position The position of the robot in world coordinates
       * @param  footprint The specification of the footprint of the robot in world coordinates
       * @param  inscribed_radius The radius of the inscribed circle of the robot
       * @param  circumscribed_radius The radius of the circumscribed circle of the robot
       * @return Positive if all the points lie outside the footprint, negative otherwise:
       *            -1 if footprint covers at least a lethal obstacle cell, or
       *            -2 if footprint covers at least a no-information cell, or
       *            -3 if footprint is [partially] outside of the map
       */
      virtual double footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
          double inscribed_radius, double circumscribed_radius);

      /**
       * @brief  Rasterizes a line in the costmap grid and checks for collisions
       * @param x0 The x position of the first cell in grid coordinates
       * @param y0 The y position of the first cell in grid coordinates
       * @param x1 The x position of the second cell in grid coordinates
       * @param y1 The y position of the second cell in grid coordinates
       * @return A positive cost for a legal line... negative otherwise
       */
      double lineCost(int x0, int x1, int y0, int y1) const;

      /**
       * @brief  Checks the cost of a point in the costmap
       * @param x The x position of the point in cell coordinates
       * @param y The y position of the point in cell coordinates
       * @return A positive cost for a legal point... negative otherwise
       */
      double pointCost(int x, int y) const;

    private:
      const costmap_2d::Costmap2D& costmap_; ///< @brief Allows access of costmap obstacle information

  };
};
#endif
