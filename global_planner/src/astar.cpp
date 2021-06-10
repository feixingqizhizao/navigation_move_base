/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *         David V. Lu!!
 *********************************************************************/
#include<global_planner/astar.h>
#include<costmap_2d/cost_values.h>

namespace global_planner {

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
}
/*	function：计算规划代价的函数
  potential: 代价数组，
  costs： 地图指针，
  cycles：循环次数；
*/
bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    //queue_为启发式搜索到的向量队列：<i , cost>
    queue_.clear();
    //起点对应的序号
    int start_i = toIndex(start_x, start_y);
    //1 将起点放入队列
    queue_.push_back(Index(start_i, 0));
    //2 将所有点的potential都设为一个极大值1e10
    //potential就是估计值g, f=g+h
    //std::fill函数的作用是：将一个区间的元素都赋予指定的值，即在[first, last)范围内填充指定值
    std::fill(potential, potential + ns_, POT_HIGH);
    //3 起点的potential设为0；
    potential[start_i] = 0;
    //终点对应的序号
    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;

    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];
        //将首元素放到最后，其他元素按照Cost值从小到大排列
        //pop_heap() 是将堆顶元素与最后一个元素交换位置，之后用pop_back将最后一个元素删除
        //greater1（）是按小顶堆
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        //删除最后一个元素，即删除最小代价的点
        queue_.pop_back();

        int i = top.i;
        //如果到了目标点，就结束了
        if (i == goal_i)
            return true;
        //6. 对前后左右四个点执行add函数，将代价最小点i周围点加入搜索队里并更新代价值
        add(costs, potential, potential[i], i + 1, end_x, end_y);
        add(costs, potential, potential[i], i - 1, end_x, end_y);
        add(costs, potential, potential[i], i + nx_, end_x, end_y);
        add(costs, potential, potential[i], i - nx_, end_x, end_y);

        cycle++;
    }

    return false;
}
/*add函数：添加点并更新代价函数；
如果是已经添加的的点则忽略，根据costmap的值如果是障碍物的点也忽略。
potential[next_i]是起点到当前点的cost即g(n),
distance * neutral_cost_是当前点到目的点的cost即h(n)。
f(n) = g(n) + h(n)
*/
void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y) {
    //next_i 点不在网格内 忽略
    if (next_i < 0 || next_i >= ns_)
        return;
    //未搜索的点cost为POT_HIGH，如小于该值，则为已搜索点,跳过；
    if (potential[next_i] < POT_HIGH)
        return;
    //障碍物点 忽略
    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
        return;
    /*
      potential 存储所有点的g(n),即从初始点到节点n的实际代价
      costs[next_i] + neutral_cost_
      potential[next_i]   是起点到当前点的cost即g(n)
      neutral_cost_ 设定的一个默认值，为50
      prev_potentia   当前点的f

      calculatePotential()计算根据use_quadratic的值有下面两个选择
            若为TRUE， 则使用二次曲线计算
        若为False， 则采用简单方法计算， return prev_potential + cost
              即：costs[next_i] + neutral_cost_+ prev_potential
             地图代价+单格距离代价(初始化为50)+之前路径代价 为G
    */
    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    int x = next_i % nx_, y = next_i / nx_;
    //启发式函数：即h(n) 从节点n到目标点最佳路径的估计代价，这里选用了曼哈顿距离
    float distance = abs(end_x - x) + abs(end_y - y);

    //两个cost后，加起来即为f(n),代价 f = g +h;，将其存入队列中
    // f = potential[next_i] + distance * neutral_cost_ 为总
    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
    //对加入的再进行堆排序， 把最小代价点放到front队头queue_[0]
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

} //end namespace global_planner
