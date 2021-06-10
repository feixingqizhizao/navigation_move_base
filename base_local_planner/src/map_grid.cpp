/*********************************************************************
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
 *********************************************************************/
#include <base_local_planner/map_grid.h>
#include <costmap_2d/cost_values.h>
using namespace std;

namespace base_local_planner{

  MapGrid::MapGrid()
    : size_x_(0), size_y_(0)
  {
  }

  MapGrid::MapGrid(unsigned int size_x, unsigned int size_y) 
    : size_x_(size_x), size_y_(size_y)
  {
    commonInit();
  }

  MapGrid::MapGrid(const MapGrid& mg){
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
  }

  void MapGrid::commonInit(){
    //don't allow construction of zero size grid
    ROS_ASSERT(size_y_ != 0 && size_x_ != 0);

    map_.resize(size_y_ * size_x_);

    //make each cell aware of its location in the grid
    for(unsigned int i = 0; i < size_y_; ++i){
      for(unsigned int j = 0; j < size_x_; ++j){
        unsigned int id = size_x_ * i + j;
        map_[id].cx = j;
        map_[id].cy = i;
      }
    }
  }

  size_t MapGrid::getIndex(int x, int y){
    return size_x_ * y + x;
  }

  MapGrid& MapGrid::operator= (const MapGrid& mg){
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
    return *this;
  }
  //检查路径地图尺寸是否和costmap相同，若不同，则按照costmap尺寸对其重新设置，并且检查MapCell在MapGrid中的index是否和它本身的坐标索引一致。
  void MapGrid::sizeCheck(unsigned int size_x, unsigned int size_y){
    if(map_.size() != size_x * size_y)
      map_.resize(size_x * size_y);

    if(size_x_ != size_x || size_y_ != size_y){
      size_x_ = size_x;
      size_y_ = size_y;

      for(unsigned int i = 0; i < size_y_; ++i){
        for(unsigned int j = 0; j < size_x_; ++j){
          int index = size_x_ * i + j;
          map_[index].cx = j;
          map_[index].cy = i;
        }
      }
    }
  }

  //关于updatePathCell函数，先获取该cell在costmap上的值，
  //若发现它是障碍物或未知cell或它在机器人足迹内，直接返回false，该cell将不会进入循环队列，也就是不会由它继续向下传播；
  //若非以上情况，让它的值=邻点值+1，若<原值，用它更新。
  inline bool MapGrid::updatePathCell(MapCell* current_cell, MapCell* check_cell,
      const costmap_2d::Costmap2D& costmap){

    //if the cell is an obstacle set the max path distance
    unsigned char cost = costmap.getCost(check_cell->cx, check_cell->cy);
    if(! getCell(check_cell->cx, check_cell->cy).within_robot &&
        (cost == costmap_2d::LETHAL_OBSTACLE ||
         cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
         cost == costmap_2d::NO_INFORMATION)){
      check_cell->target_dist = obstacleCosts();
      return false;
    }
    // 不是障碍物的话，比较它的target_dist（本来通过resetPathDist被设置为unreachableCellCosts）与>current_cell->target_dist + 1
    double new_target_dist = current_cell->target_dist + 1;
    if (new_target_dist < check_cell->target_dist) {
      check_cell->target_dist = new_target_dist;
    }
    return true;
  }


  //reset the path_dist and goal_dist fields for all cells
  void MapGrid::resetPathDist(){
    for(unsigned int i = 0; i < map_.size(); ++i) {
      map_[i].target_dist = unreachableCellCosts();
      map_[i].target_mark = false;
      map_[i].within_robot = false;
    }
  }
  //传入的全局规划是global系下的，调用adjustPlanResolution函数对其分辨率进行一个简单的调整，
  //使其达到costmap的分辨率，方法也很简单，当相邻点之间距离>分辨率，在其间插入点，以达到分辨率要求。
  void MapGrid::adjustPlanResolution(const std::vector<geometry_msgs::PoseStamped>& global_plan_in,
      std::vector<geometry_msgs::PoseStamped>& global_plan_out, double resolution) {
    if (global_plan_in.size() == 0) {
      return;
    }
    double last_x = global_plan_in[0].pose.position.x;
    double last_y = global_plan_in[0].pose.position.y;
    global_plan_out.push_back(global_plan_in[0]);

    double min_sq_resolution = resolution * resolution;
    for (unsigned int i = 1; i < global_plan_in.size(); ++i)
    {
      double loop_x = global_plan_in[i].pose.position.x;
      double loop_y = global_plan_in[i].pose.position.y;
      double sqdist = (loop_x - last_x) * (loop_x - last_x) + (loop_y - last_y) * (loop_y - last_y);
      if (sqdist > min_sq_resolution) {
        int steps = ceil((sqrt(sqdist)) / resolution);
        // add a points in-between
        double deltax = (loop_x - last_x) / steps;
        double deltay = (loop_y - last_y) / steps;
        // TODO: Interpolate orientation
        for (int j = 1; j < steps; ++j) {
          geometry_msgs::PoseStamped pose;
          pose.pose.position.x = last_x + j * deltax;
          pose.pose.position.y = last_y + j * deltay;
          pose.pose.position.z = global_plan_in[i].pose.position.z;
          pose.pose.orientation = global_plan_in[i].pose.orientation;
          pose.header = global_plan_in[i].header;
          global_plan_out.push_back(pose);
        }
      }
      global_plan_out.push_back(global_plan_in[i]);
      last_x = loop_x;
      last_y = loop_y;
    }
  }
  //接下来将全局路径的点转换到“路径地图”上，并将对应cell的target_dist设置为0，标记已计算过。
  //由于分辨率已经过调整，故不会出现路径不连续的情况。在“路径地图”上得到一条在地图范围内、且不经过costmap上未知cell的全局路径，每个点的target_dist都为0。
  void MapGrid::setTargetCells(const costmap_2d::Costmap2D& costmap,
      const std::vector<geometry_msgs::PoseStamped>& global_plan) {
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    bool started_path = false;

    queue<MapCell*> path_dist_queue;

    std::vector<geometry_msgs::PoseStamped> adjusted_global_plan;
    adjustPlanResolution(global_plan, adjusted_global_plan, costmap.getResolution());
    if (adjusted_global_plan.size() != global_plan.size()) {
      ROS_DEBUG("Adjusted global plan resolution, added %zu points", adjusted_global_plan.size() - global_plan.size());
    }
    unsigned int i;
    // put global path points into local map until we reach the border of the local map
    for (i = 0; i < adjusted_global_plan.size(); ++i) {
      double g_x = adjusted_global_plan[i].pose.position.x;
      double g_y = adjusted_global_plan[i].pose.position.y;
      unsigned int map_x, map_y;
      // 如果成功把一个全局规划上的点的坐标转换到地图坐标（map_x, map_y）上，且在代价地图上这一点不是未知的
      if (costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x, map_y) != costmap_2d::NO_INFORMATION) {
        // 用current来引用这个cell
        MapCell& current = getCell(map_x, map_y);
        current.target_dist = 0.0;// 将这个点的target_dist设置为0，即在全局路径上
        current.target_mark = true;// 标记已经计算了距离
        path_dist_queue.push(&current);// 把该点放进path_dist_queue队列中
        started_path = true;// 标记已经开始把点转换到地图坐标
      }
      // 当代价地图上这一点的代价不存在了（规划路径已经到达了代价地图的边界）
      // 并且标记了已经开始转换，退出循环
      else if (started_path) {
          break;
      }
    }
    // 如果循环结束后并没有确定任何在地图坐标上的点
    if (!started_path) {
      ROS_ERROR("None of the %d first of %zu (%zu) points of the global plan were in the local costmap and free",
          i, adjusted_global_plan.size(), global_plan.size());
      return;
    }
    //接下来调用类内computeTargetDistance函数，开始“填充”，最终得到一张完整的路径地图。
    computeTargetDistance(path_dist_queue, costmap);
  }

  //更新到目标点的距离地图
  void MapGrid::setLocalGoal(const costmap_2d::Costmap2D& costmap,
      const std::vector<geometry_msgs::PoseStamped>& global_plan) {
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    int local_goal_x = -1;
    int local_goal_y = -1;
    bool started_path = false;

    std::vector<geometry_msgs::PoseStamped> adjusted_global_plan;
    adjustPlanResolution(global_plan, adjusted_global_plan, costmap.getResolution());

    //然后，通过迭代找到全局路径的终点，即目标点，但如果迭代过程当中到达了局部规划costmap的边际或经过障碍物，立即退出迭代，将上一个有效点作为终点。
    for (unsigned int i = 0; i < adjusted_global_plan.size(); ++i) {
      double g_x = adjusted_global_plan[i].pose.position.x;
      double g_y = adjusted_global_plan[i].pose.position.y;
      unsigned int map_x, map_y;
      if (costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x, map_y) != costmap_2d::NO_INFORMATION) {
        // 不断迭代local_goal_x、local_goal_y，尝试找到终点
        local_goal_x = map_x;
        local_goal_y = map_y;
        started_path = true;
      } else {
        if (started_path) {
          break;
        }// else we might have a non pruned path, so we just continue
      }
    }
    if (!started_path) {
      ROS_ERROR("None of the points of the global plan were in the local costmap, global plan points too far from robot");
      return;
    }
    //将迭代得到的目标点对应在“目标地图”上的cell的target_dist标记为0。setTargetCells和setLocalGoal这两个函数，
    //前者在“路径地图”上将全局路径点target_dist标记为0，后者在“目标地图”上将目标点target_dist标记为0。最后同样调用computeTargetDistance函数。
    queue<MapCell*> path_dist_queue;
    if (local_goal_x >= 0 && local_goal_y >= 0) {
      MapCell& current = getCell(local_goal_x, local_goal_y);
      costmap.mapToWorld(local_goal_x, local_goal_y, goal_x_, goal_y_);
      current.target_dist = 0.0;
      current.target_mark = true;
      path_dist_queue.push(&current);
    }

    computeTargetDistance(path_dist_queue, costmap);
  }


  //这个函数传入target_dist=0.0的cell队列，然后从它们开始，向四周开始“传播”。
  void MapGrid::computeTargetDistance(queue<MapCell*>& dist_queue, const costmap_2d::Costmap2D& costmap){
    MapCell* current_cell;
    MapCell* check_cell;
    unsigned int last_col = size_x_ - 1;
    unsigned int last_row = size_y_ - 1;
    // 当队列非空，循环，更新每个网格的四邻域，直到队列为空
    while(!dist_queue.empty()){
      current_cell = dist_queue.front();
      dist_queue.pop();
    //依次检查“父cell”四周的cell并标记它已计算，对它调用updatePathCell函数，得到该cell的值。
    //若有效，加入循环队列里，弹出“父cell”，四周的cell成为新的“父cell”， 循环，直到所有cell更新完毕。
      //如果该点的横坐标大于0
      if(current_cell->cx > 0){
        // 检查它左侧相邻的点check_cell
        check_cell = current_cell - 1;
        // 如果check_cell未被标记,对该点进行标记，表示已经被访问;如果已经被访问，则不需要重新赋值
        if(!check_cell->target_mark){
          check_cell->target_mark = true;
          // updatePathCell功能：如果check_cell不是障碍物，将target_dist设置为current_cell+1；否则，设置为无穷
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);// check_cell入队
          }
        }
      }
      // 如果该点的横坐标小于图像宽度-1
      if(current_cell->cx < last_col){
        // 检查它右侧相邻的点check_cell
        check_cell = current_cell + 1;
        if(!check_cell->target_mark){
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }
      // 如果该点的纵坐标>0
      if(current_cell->cy > 0){
        // 检查它上面的点
        check_cell = current_cell - size_x_;
        if(!check_cell->target_mark){
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }
      // 如果该点的纵坐标<图像长度-1
      if(current_cell->cy < last_row){
        // 检查它下面的点
        check_cell = current_cell + size_x_;
        if(!check_cell->target_mark){
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }
    }
    // 所以已经计算过的点不断被加入，然后利用它找到周围点后，弹出它，直到地图上所有点的距离都被计算出来
  }

};
