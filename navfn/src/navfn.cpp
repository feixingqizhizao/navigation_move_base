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
//
// Navigation function computation
// Uses Dijkstra's method
// Modified for Euclidean-distance computation
// 
// Path calculation uses no interpolation when pot field is at max in
//   nearby cells
//
// Path calc has sanity check that it succeeded
//


#include <navfn/navfn.h>
#include <ros/console.h>

namespace navfn {

  //
  // function to perform nav fn calculation
  // keeps track of internal buffers, will be more efficient
  //   if the size of the environment does not change
  //

  int
    create_nav_plan_astar(COSTTYPE *costmap, int nx, int ny,
        int* goal, int* start,
        float *plan, int nplan)
    {
      static NavFn *nav = NULL;

      if (nav == NULL)
        nav = new NavFn(nx,ny);

      if (nav->nx != nx || nav->ny != ny) // check for compatibility with previous call
      {
        delete nav;
        nav = new NavFn(nx,ny);      
      }

      nav->setGoal(goal);
      nav->setStart(start);

      nav->costarr = costmap;
      nav->setupNavFn(true);

      // calculate the nav fn and path
      nav->priInc = 2*COST_NEUTRAL;
      nav->propNavFnAstar(std::max(nx*ny/20,nx+ny));

      // path
      int len = nav->calcPath(nplan);

      if (len > 0)			// found plan
        ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
      else
        ROS_DEBUG("[NavFn] No path found\n");

      if (len > 0)
      {
        for (int i=0; i<len; i++)
        {
          plan[i*2] = nav->pathx[i];
          plan[i*2+1] = nav->pathy[i];
        }
      }

      return len;
    }




  //
  // create nav fn buffers 
  //

  NavFn::NavFn(int xs, int ys)
  {  
    // create cell arrays
    costarr = NULL;
    potarr = NULL;
    pending = NULL;
    gradx = grady = NULL;
    setNavArr(xs,ys);

    // priority buffers
    pb1 = new int[PRIORITYBUFSIZE];
    pb2 = new int[PRIORITYBUFSIZE];
    pb3 = new int[PRIORITYBUFSIZE];

    // for Dijkstra (breadth-first), set to COST_NEUTRAL
    // for A* (best-first), set to COST_NEUTRAL
    priInc = 2*COST_NEUTRAL;	

    // goal and start
    goal[0] = goal[1] = 0;
    start[0] = start[1] = 0;

    // display function
    displayFn = NULL;
    displayInt = 0;

    // path buffers
    npathbuf = npath = 0;
    pathx = pathy = NULL;
    pathStep = 0.5;
  }


  NavFn::~NavFn()
  {
    if(costarr)
      delete[] costarr;
    if(potarr)
      delete[] potarr;
    if(pending)
      delete[] pending;
    if(gradx)
      delete[] gradx;
    if(grady)
      delete[] grady;
    if(pathx)
      delete[] pathx;
    if(pathy)
      delete[] pathy;
    if(pb1)
      delete[] pb1;
    if(pb2)
      delete[] pb2;
    if(pb3)
      delete[] pb3;
  }


  //
  // set goal, start positions for the nav fn
  //

  void
    NavFn::setGoal(int *g)
    {
      goal[0] = g[0];
      goal[1] = g[1];
      ROS_DEBUG("[NavFn] Setting goal to %d,%d\n", goal[0], goal[1]);
    }

  void
    NavFn::setStart(int *g)
    {
      start[0] = g[0];
      start[1] = g[1];
      ROS_DEBUG("[NavFn] Setting start to %d,%d\n", start[0], start[1]);
    }

  //
  // Set/Reset map size
  //

  void
    NavFn::setNavArr(int xs, int ys)
    {
      ROS_DEBUG("[NavFn] Array is %d x %d\n", xs, ys);

      nx = xs;
      ny = ys;
      ns = nx*ny;

      if(costarr)
        delete[] costarr;
      if(potarr)
        delete[] potarr;
      if(pending)
        delete[] pending;

      if(gradx)
        delete[] gradx;
      if(grady)
        delete[] grady;

      costarr = new COSTTYPE[ns]; // cost array, 2d config space
      memset(costarr, 0, ns*sizeof(COSTTYPE));
      potarr = new float[ns];	// navigation potential array
      pending = new bool[ns];
      memset(pending, 0, ns*sizeof(bool));
      gradx = new float[ns];
      grady = new float[ns];
    }
  //用指针指向数组costarr，COSTTYPE宏定义为unsigned char，即ROS中使用的地图Costmap2D中用于储存地图数据的成员类型。
  void
    NavFn::setCostmap(const COSTTYPE *cmap, bool isROS, bool allow_unknown)
    {
      //全局规划用到的地图costarr
      COSTTYPE *cm = costarr;
      //接下来在地图的长宽范围内进行迭代：
      //① 若当前cell在costmap上的值 < COST_OBS_ROS(253)，即非致命障碍物（障碍物附近），重新将其赋值为COST_NEUTRAL(50)+当前cell在costmap上的值×比例0.8，最高253；
      //② 若当前cell在costmap上的值 == COST_UNKNOWN_ROS(255)，即未知区域，赋值为253；
      //③ 若当前cell在costmap上的值 == COST_OBS(254)，即致命障碍物（障碍物本身），值仍为254。
      if (isROS)			// ROS-type cost array
      {
        for (int i=0; i<ny; i++)
        {
          //k值记录二重迭代的次数
          int k=i*nx;
          //cmap指向costmap元素，cm指向costarr
          for (int j=0; j<nx; j++, k++, cmap++, cm++)
          {
            //默认最小权重值为COST_NEUTRAL＝50，最大权重值为COST_OBS＝254
            //注：最小权重值即行走单个free(无障碍物影响)栅格所付出的权重代价
            //最大权重值即行走单个障碍物栅格所付出的权重代价
            *cm = COST_OBS;
            int v = *cmap;
            //若当前cell的代价小于障碍物类型(253)，实际上253是膨胀型障碍
            if (v < COST_OBS_ROS)
            {
              //重新将其赋值为50+cost地图上的障碍物值×比例0.8
              v = COST_NEUTRAL+COST_FACTOR*v;
              //若值>=COST_OBS(254，致命层障碍)，统一设置为253，确保不要超出范围
              if (v >= COST_OBS)
                v = COST_OBS-1;
              *cm = v;
            }
            //若当前cell的值为COST_UNKNOWN_ROS(255)，未知区域
            else if(v == COST_UNKNOWN_ROS && allow_unknown)
            {
              v = COST_OBS-1;
              *cm = v;
            }
          }
        }
      }
      //当地图类型是其他类型（如PGM），也执行同样的“翻译”工作，设置costarr数组。
      else
      {
        for (int i=0; i<ny; i++)
        {
          int k=i*nx;
          for (int j=0; j<nx; j++, k++, cmap++, cm++)
          {
            *cm = COST_OBS;
            //避免处理边界cell
            if (i<7 || i > ny-8 || j<7 || j > nx-8)
              continue;	// don't do borders
            int v = *cmap;
            if (v < COST_OBS_ROS)
            {
              v = COST_NEUTRAL+COST_FACTOR*v;
              if (v >= COST_OBS)
                v = COST_OBS-1;
              *cm = v;
            }
            else if(v == COST_UNKNOWN_ROS)
            {
              v = COST_OBS-1;
              *cm = v;
            }
          }
        }

      }
    }
  //这个函数内完成了整个路径计算的流程，顺序调用了几个子部分的函数。
  //重新设置势场矩阵potarr的值、设置costarr的边际值、设置目标在costarr中的值为0，对四周cell进行处理，记录costarr中障碍物cell数
  bool
    NavFn::calcNavFnDijkstra(bool atStart)
    {
      //该函数对“翻译”生成的costarr数组进行了边际设置等处理，并初始化了potarr数组和梯度数组gradx、grady。
      setupNavFn(true);

      // 这个函数以目标点（Potential值已初始化为0）为起点，向整张地图的cell传播，填充potarr数组，直到找到起始点为止，
      //potarr数组的数据能够反映“走了多远”和“附近的障碍情况”，为最后的路径计算提供了依据。
      propNavFnDijkstra(std::max(nx*ny/20,nx+ny),atStart);

      //该函数负责在potarr数组的基础上选取一些cell点来生成最终的全局规划路径，从起点开始沿着最优行走代价值梯度下降的方向寻找到目标点的最优轨迹。
      int len = calcPath(nx*ny/2);

      if (len > 0)			// found plan
      {
        ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
        return true;
      }
      else
      {
        ROS_DEBUG("[NavFn] No path found\n");
        return false;
      }

    }


  //
  // calculate navigation function, given a costmap, goal, and start
  //

  bool
    NavFn::calcNavFnAstar()
    {
      setupNavFn(true);

      // calculate the nav fn and path
      propNavFnAstar(std::max(nx*ny/20,nx+ny));

      // path
      int len = calcPath(nx*4);

      if (len > 0)			// found plan
      {
        ROS_DEBUG("[NavFn] Path found, %d steps\n", len);
        return true;
      }
      else
      {
        ROS_DEBUG("[NavFn] No path found\n");
        return false;
      }
    }


  //
  // returning values
  //

  float *NavFn::getPathX() { return pathx; }
  float *NavFn::getPathY() { return pathy; }
  int    NavFn::getPathLen() { return npath; }

  // inserting onto the priority blocks
#define push_cur(n)  { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && curPe<PRIORITYBUFSIZE) \
  { curP[curPe++]=n; pending[n]=true; }}
#define push_next(n) { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && nextPe<PRIORITYBUFSIZE) \
  { nextP[nextPe++]=n; pending[n]=true; }}
#define push_over(n) { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && overPe<PRIORITYBUFSIZE) \
  { overP[overPe++]=n; pending[n]=true; }}


  // 该函数对“翻译”生成的costarr数组进行了边际设置等处理，并初始化了potarr数组和梯度数组gradx、grady。
  void
    NavFn::setupNavFn(bool keepit)
    {
      // 下面先循环初始化potarr矩阵元素全部为最大值POT_HIGH，并初始化梯度表初始值全部为0.0。
      for (int i=0; i<ns; i++)
      {
        potarr[i] = POT_HIGH;
        if (!keepit) costarr[i] = COST_NEUTRAL;
        gradx[i] = grady[i] = 0.0;
      }

      // 接下来设置costarr的四条边的cell的值为COST_OBS(致命层254)，封闭地图四周，以防产生边界以外的轨迹。
      COSTTYPE *pc;
      pc = costarr;
      //costarr第一行全部设置为COST_OBS(致命层254)
      for (int i=0; i<nx; i++)
        *pc++ = COST_OBS;
      pc = costarr + (ny-1)*nx;
      //costarr最后一行全部设置为COST_OBS(致命层254)
      for (int i=0; i<nx; i++)
        *pc++ = COST_OBS;
      pc = costarr;
      //costarr第一列全部设置为COST_OBS(致命层254)
      for (int i=0; i<ny; i++, pc+=nx)
        *pc = COST_OBS;
      pc = costarr + nx - 1;
      //costarr最后一列全部设置为COST_OBS(致命层254)
      for (int i=0; i<ny; i++, pc+=nx)
        *pc = COST_OBS;

      //初始化一些用于迭代更新potarr的数据，并初始化pending数组为全0，设置所有的cell状态都为非等待状态。
      curT = COST_OBS;//当前传播阈值，254
      curP = pb1; //当前用于传播的cell索引数组
      curPe = 0;//当前用于传播的cell的数量
      nextP = pb2;//用于下个传播过程的cell索引数组
      nextPe = 0;//用于下个传播过程的cell的数量
      overP = pb3;//传播界限外的cell索引数组
      overPe = 0;//传播界限外的cell的数量
      memset(pending, 0, ns*sizeof(bool));

      //接下来设置目标goal在potarr中的值为0，并把它四周非障碍物的cell加入curP数组（当前传播cell）中，为下一步的Potential值在整张地图上的传播做准备。
      //k为目标cell的索引
      int k = goal[0] + goal[1]*nx;
      //设置costarr的索引k（目标）的元素值为0，并对它四周的cell在pending[]中进行标记“等待状态”，并把索引存放入curP数组
      initCost(k,0);

      //更新nobs，记录costarr中的致命障碍物cell的数量
      pc = costarr;
      int ntot = 0;
      for (int i=0; i<ns; i++, pc++)
      {
        if (*pc >= COST_OBS)
          ntot++;			// number of cells that are obstacles
      }
      nobs = ntot;
    }


  // initialize a goal-type cost for starting propagation

  void
    NavFn::initCost(int k, float v)
    {
      potarr[k] = v;
      push_cur(k+1);
      push_cur(k-1);
      push_cur(k-nx);
      push_cur(k+nx);
    }


  // 
  // Critical function: calculate updated potential value of a cell,
  //   given its neighbors' values
  // Planar-wave update calculation from two lowest neighbors in a 4-grid
  // Quadratic approximation to the interpolated value 
  // No checking of bounds here, this function should be fast
  //

#define INVSQRT2 0.707106781
  //updateCell用于更新单个cell的Potential值，先获取当前cell四周邻点的potarr值，并取最小的值存入ta。
  inline void
    NavFn::updateCell(int n)
    {
      // 获取四邻域的值
      float u,d,l,r;
      l = potarr[n-1];
      r = potarr[n+1];		
      u = potarr[n-nx];
      d = potarr[n+nx];

      // 找到其中最小的邻居，ta为y方向，tc为x方向
      float ta, tc;
      if (l<r) tc=l; else tc=r;
      if (u<d) ta=u; else ta=d;

      // 下面执行一个判断，只有当当前cell不是致命障碍物时，才由它向四周传播，否则到它后停止，不传播。
      if (costarr[n] < COST_OBS)	// don't propagate into obstacles
      {
        float hf = (float)costarr[n]; // traversability factor
        float dc = tc-ta;		// relative cost between ta,tc
        if (dc < 0) 		// ta 更新为四邻域中最小值
        {
          dc = -dc;
          ta = tc;
        }

        // calculate new potential
        //在计算当前点Potential值时，有两种情况，即需要对“左右邻点最小pot值与上下邻点最小pot值之差的绝对值”和“当前cell的costarr值”比较，
        //有pot = ta+hf和另一个更复杂的公式，这两个公式的功能相同，但效果有区别，区别也是前面提到过的“圆形传播”和“菱形传播”，后者能够产生效果更好的菱形传播。

        //只有当前cell的Potential计算值<原本的Potential值，才更新，
        //这意味着从目标点开始，它的Potential值被初始化为0，不会被更新，接下来传播到它的四个邻点，才会开始更新他们的Potential值。

        //为便于理解，这里分析第一个公式，当前点Potential值=四周最小的Potential值+当前点cost值。
        //这说明，从目标点（Potential=0）向外传播时，它四周的可行cell的Potential值会变成0+cost，
        //可以假设他们的cost都是50，那么它们的Potential值都被更新为50（因为初始值无限大，故会被迭代）。
        //第二轮次的传播中，假设邻点的邻点cost也为50，那么它们的Potential值将被更新为100。
        //这种传播过程中cost的累加造成Potential值的上升能够反映离目标点的远近。

        //并且，cost大小与cell离障碍物的远近对应，更大的cost对应更大的Potential，并且障碍物点不更新Potential，使得其值停留在无限大，
        //故Potential值的大小也能反映点与障碍物的接近程度。
        float pot;
        if (dc >= hf)		// if too large, use ta-only update
          pot = ta+hf;
        else			// two-neighbor interpolation update
        {
          // use quadratic approximation
          // might speed this up through table lookup, but still have to 
          //   do the divide
          float d = dc/hf;
          float v = -0.2301*d*d + 0.5307*d + 0.7040;
          pot = ta + hf*v;
        }

        // 最后，将临近cell放入nextP或overP，供下次迭代使用。
        if (pot < potarr[n])
        {
          float le = INVSQRT2*(float)costarr[n-1];
          float re = INVSQRT2*(float)costarr[n+1];
          float ue = INVSQRT2*(float)costarr[n-nx];
          float de = INVSQRT2*(float)costarr[n+nx];
          potarr[n] = pot;
          //nextP和overP都来自从目标点开始传播的四周的cell，区别在于它们的“父cell”的pot值是否达到阈值curT，没达到则放入nextP，达到则放入overP。
          // 如果上下左右四个节点的权重值加上当前节点的potential值小于这四个节点的potential值时，才加入nexP数组进
          //行这四个节点potential值的更新，否则没有必要更新
          if (pot < curT)	// low-cost buffer block 
          {
            if (l > pot+le) push_next(n-1);
            if (r > pot+re) push_next(n+1);
            if (u > pot+ue) push_next(n-nx);
            if (d > pot+de) push_next(n+nx);
          }
          else			// 如果当前点Potential为
          {
            if (l > pot+le) push_over(n-1);
            if (r > pot+re) push_over(n+1);
            if (u > pot+ue) push_over(n-nx);
            if (d > pot+de) push_over(n+nx);
          }
        }

      }

    }


  //
  // Use A* method for setting priorities
  // Critical function: calculate updated potential value of a cell,
  //   given its neighbors' values
  // Planar-wave update calculation from two lowest neighbors in a 4-grid
  // Quadratic approximation to the interpolated value 
  // No checking of bounds here, this function should be fast
  //

#define INVSQRT2 0.707106781

  inline void
    NavFn::updateCellAstar(int n)
    {
      // get neighbors
      float u,d,l,r;
      l = potarr[n-1];
      r = potarr[n+1];		
      u = potarr[n-nx];
      d = potarr[n+nx];
      //ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n", 
      //	 potarr[n], l, r, u, d);
      // ROS_INFO("[Update] cost of %d: %d\n", n, costarr[n]);

      // find lowest, and its lowest neighbor
      float ta, tc;
      if (l<r) tc=l; else tc=r;
      if (u<d) ta=u; else ta=d;

      // do planar wave update
      if (costarr[n] < COST_OBS)	// don't propagate into obstacles
      {
        float hf = (float)costarr[n]; // traversability factor
        float dc = tc-ta;		// relative cost between ta,tc
        if (dc < 0) 		// ta is lowest
        {
          dc = -dc;
          ta = tc;
        }

        // calculate new potential
        float pot;
        if (dc >= hf)		// if too large, use ta-only update
          pot = ta+hf;
        else			// two-neighbor interpolation update
        {
          // 此处为计算当前节点potential值的过程，我试着猜测 float v = -0.2301*d*d + 0.5307*d + 0.7040 这个公式的含义，但
          //是都没成功，后来寻求到了作者的解释：https://github.com/ros-planning/navigation/issues/890
          float d = dc/hf;
          float v = -0.2301*d*d + 0.5307*d + 0.7040;
          pot = ta + hf*v;
        }

        //ROS_INFO("[Update] new pot: %d\n", costarr[n]);

        //判断当前节点计算的potential值是否小于本身的potential值，只有当小于时，才更新当前栅格节点的potential值
        if (pot < potarr[n])
        {
          // 计算当前节点上下左右四个节点的权重值
          float le = INVSQRT2*(float)costarr[n-1];
          float re = INVSQRT2*(float)costarr[n+1];
          float ue = INVSQRT2*(float)costarr[n-nx];
          float de = INVSQRT2*(float)costarr[n+nx];

          // calculate distance
          int x = n%nx;
          int y = n/nx;
          float dist = hypot(x-start[0], y-start[1])*(float)COST_NEUTRAL;

          potarr[n] = pot;
          pot += dist;
          // 当当前节点的potential值小于传播界限时，将上下左右四个节点传递给nextP
          if (pot < curT)	// low-cost buffer block 
          {
            if (l > pot+le) push_next(n-1);
            if (r > pot+re) push_next(n+1);
            if (u > pot+ue) push_next(n-nx);
            if (d > pot+de) push_next(n+nx);
          }
          else
          {
            if (l > pot+le) push_over(n-1);
            if (r > pot+re) push_over(n+1);
            if (u > pot+ue) push_over(n-nx);
            if (d > pot+de) push_over(n+nx);
          }
        }

      }

    }



  //
  // main propagation function
  // Dijkstra method, breadth-first
  // runs for a specified number of cycles,
  //   or until it runs out of cells to update,
  //   or until the Start cell is found (atStart = true)
  //
  //dijkstra算法广度优先传播更新potential数组，即可获得起点到传播过程中任意点的最优行走代价值．
  //此处cycles大小不用太纠结，设置足够大就行，能保证传播到目标点，到了目标点之后，会自动break掉
  bool
    NavFn::propNavFnDijkstra(int cycles, bool atStart)	
    {
      int nwv = 0;			//priority block的最大数量
      int nc = 0;			//priority blocks中的cell数
      int cycle = 0;		//当前迭代次数

      //记录起始位置的索引
      int startCell = start[1]*nx + start[0];
      //循环迭代更新potarr，判断条件：如果当前正在传播和下一步传播的集都为空，那么说明已经无法继续传播，可能有无法越过的障碍或其他情况，退出。
      for (; cycle < cycles; cycle++) // //循环迭代，次数为cycles
      {
        // 如果当前用于传播的节点的数量和接下来用于传播的节点的数量都为０，则直接退出传播过程，表示传播不下去了
        if (curPe == 0 && nextPe == 0) // priority blocks empty
          break;

        // stats
        nc += curPe;
        if (curPe > nwv)
          nwv = curPe;

        // 传播当前节点，并将当前节点上下左右四个节点保存到nextP数组当中，当当前节点传播完之后，将nextP数组中的
        // 节点数组传递给当前节点数组curP，又继续开始传播当前节点，循环往复．
        int *pb = curP;
        int i = curPe;			
        while (i-- > 0)		
          pending[*(pb++)] = false;//对pending数组进行设置


        //接下来传播curP，即当前cell，调用的函数为updateCell，它能更新当前cell在potarr数组中的值，并将其四周符合特定条件的点放入nextP或overP，用于下一步的传播。
        //调用完成后，将nextP数组中的cell传递给curP，继续上述传播，若nextP没有cell可以用来传播，则引入overP中的cell。

        //nextP和overP都来自从目标点开始传播的四周的cell，区别在于它们的“父cell”的pot值是否达到阈值curT，没达到则放入nextP，达到则放入overP。

        //在阅读其他博主的博客学习这部分时发现，设置一个阈值来区分nextP和overP的传播先后顺序，结果是以目标点为圆心向外圆形传播，
        //而不设阈值区分，则是以目标点为中心向外呈菱形传播，显然前者更合理。
        pb = curP; 
        i = curPe;
        while (i-- > 0)		
          updateCell(*pb++);

        if (displayInt > 0 &&  (cycle % displayInt) == 0)
          displayFn(this);

        // swap priority blocks curP <=> nextP
        curPe = nextPe;
        nextPe = 0;
        pb = curP;		// swap buffers
        curP = nextP;
        nextP = pb;

        // 当当前节点数组中节点个数为0时，增加传播界限阈值，并将overP传递给curP，然后再重复上面的过程
        if (curPe == 0)
        {
          curT += priInc;	// increment priority threshold
          curPe = overPe;	// set current to overflow block
          overPe = 0;
          pb = curP;		// swap buffers
          curP = overP;
          overP = pb;
        }

        // 在从目标点向全地图传播的过程中检查，当起点的Potential值不再是被初始化的无穷大，而是有一个实际的值时，说明到达了起点，传播停止。
        if (atStart)
          if (potarr[startCell] < POT_HIGH)
            break;
      }

      ROS_DEBUG("[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n", 
          cycle,nc,(int)((nc*100.0)/(ns-nobs)),nwv);

      if (cycle < cycles) return true; // finished up here
      else return false;
    }


  //
  // main propagation function
  // A* method, best-first
  // uses Euclidean distance heuristic
  // runs for a specified number of cycles,
  //   or until it runs out of cells to update,
  //   or until the Start cell is found (atStart = true)
  //

  bool
    NavFn::propNavFnAstar(int cycles)	
    {
      int nwv = 0;			// max priority block size
      int nc = 0;			// number of cells put into priority blocks
      int cycle = 0;		// which cycle we're on

      // set initial threshold, based on distance
      float dist = hypot(goal[0]-start[0], goal[1]-start[1])*(float)COST_NEUTRAL;
      curT = dist + curT;

      // set up start cell
      int startCell = start[1]*nx + start[0];

      // do main cycle
      for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
      {
        // 
        if (curPe == 0 && nextPe == 0) // priority blocks empty
          break;

        // stats
        nc += curPe;
        if (curPe > nwv)
          nwv = curPe;

        // reset pending flags on current priority buffer
        int *pb = curP;
        int i = curPe;			
        while (i-- > 0)		
          pending[*(pb++)] = false;

        // process current priority buffer
        pb = curP; 
        i = curPe;
        while (i-- > 0)		
          updateCellAstar(*pb++);

        if (displayInt > 0 &&  (cycle % displayInt) == 0)
          displayFn(this);

        // swap priority blocks curP <=> nextP
        curPe = nextPe;
        nextPe = 0;
        pb = curP;		// swap buffers
        curP = nextP;
        nextP = pb;

        // see if we're done with this priority level
        if (curPe == 0)
        {
          curT += priInc;	// increment priority threshold
          curPe = overPe;	// set current to overflow block
          overPe = 0;
          pb = curP;		// swap buffers
          curP = overP;
          overP = pb;
        }

        // check if we've hit the Start cell
        if (potarr[startCell] < POT_HIGH)
          break;

      }

      last_path_cost_ = potarr[startCell];

      ROS_DEBUG("[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n", 
          cycle,nc,(int)((nc*100.0)/(ns-nobs)),nwv);


      if (potarr[startCell] < POT_HIGH) return true; // finished up here
      else return false;
    }


  float NavFn::getLastPathCost()
  {
    return last_path_cost_;
  }


  //
  // Path construction
  // Find gradient at array points, interpolate path
  // Use step size of pathStep, usually 0.5 pixel
  //
  // Some sanity checks:
  //  1. Stuck at same index position
  //  2. Doesn't get near goal
  //  3. Surrounded by high potentials
  //
  //该函数负责在potarr数组的基础上选取一些cell点来生成最终的全局规划路径，从起点开始沿着最优行走代价值梯度下降的方向寻找到目标点的最优轨迹。
  int NavFn::calcPath(int n, int *st)
  {
    // 检查path数组，初始化
    if (npathbuf < n)
    {
      if (pathx) delete [] pathx;
      if (pathy) delete [] pathy;
      pathx = new float[n];
      pathy = new float[n];
      npathbuf = n;
    }
    // set up start position at cell
    // st is always upper left corner for 4-point bilinear interpolation
    if (st == NULL) st = start;//st指向起点
    int stc = st[1]*nx + st[0];//stc记录起点索引

    // set up offset
    float dx=0;
    float dy=0;
    npath = 0;//路径点索引

    // 最多计算n个循环
    for (int i=0; i<n; i++)
    {
      // 检查是否接近目标点
      int nearest_point=std::max(0,std::min(nx*ny-1,stc+(int)round(dx)+(int)(nx*round(dy))));
      //已经很靠近目标点，将目标点作为最后一个点存入路径
      if (potarr[nearest_point] < COST_NEUTRAL)
      {
        pathx[npath] = (float)goal[0];
        pathy[npath] = (float)goal[1];
        return ++npath;	// done!
      }
      // 接近第一层和最后一层网格，即认为超出边界
      if (stc < nx || stc > ns-nx)
      {
        ROS_DEBUG("[PathCalc] Out of bounds");
        return 0;
      }

      //添加至路径点，存入的是x,y方向的索引
      pathx[npath] = stc%nx + dx;
      pathy[npath] = stc/nx + dy;
      npath++;
      //震荡检测，某一步和上上一步的位置是否一样
      bool oscillation_detected = false;
      if( npath > 2 &&
          pathx[npath-1] == pathx[npath-3] &&
          pathy[npath-1] == pathy[npath-3] )
      {
        ROS_DEBUG("[PathCalc] oscillation detected, attempting fix.");
        oscillation_detected = true;
      }

      int stcnx = stc+nx;//当前点下方的点的索引
      int stcpx = stc-nx;//当前点上方的点的索引

      //检查当前到达节点的周边的8个节点是否有障碍物代价值，如果有的话，则直接将stc指向这8个节点中potential值最低的节点
      if (potarr[stc] >= POT_HIGH ||
          potarr[stc+1] >= POT_HIGH ||
          potarr[stc-1] >= POT_HIGH ||
          potarr[stcnx] >= POT_HIGH ||
          potarr[stcnx+1] >= POT_HIGH ||
          potarr[stcnx-1] >= POT_HIGH ||
          potarr[stcpx] >= POT_HIGH ||
          potarr[stcpx+1] >= POT_HIGH ||
          potarr[stcpx-1] >= POT_HIGH ||
          oscillation_detected)
      {
        ROS_DEBUG("[Path] Pot fn boundary, following grid (%0.1f/%d)", potarr[stc], npath);
        //检查八个邻点中的最小值，首先初始化两个变量，分别作为索引和对应的potential值
        int minc = stc;
        int minp = potarr[stc];
        //从左下角的邻域开始搜索，该点的编号最小，不断更新最小的索引和对应的potential值
        int st = stcpx - 1;
        if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
        st++;
        if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
        st++;
        if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
        st = stc-1;
        if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
        st = stc+1;
        if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
        st = stcnx-1;
        if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
        st++;
        if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
        st++;
        if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
        stc = minc;
        dx = 0;
        dy = 0;

        ROS_DEBUG("[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
                  potarr[stc], pathx[npath-1], pathy[npath-1]);

        if (potarr[stc] >= POT_HIGH)
        {
          ROS_DEBUG("[PathCalc] No path found, high potential");
          //savemap("navfn_highpot");
          return 0;
        }
      }

      //当周围八个邻点没有障碍物，则直接计算梯度，并沿着梯度方向查找下一个节点
      else
      {
        // 获得四个位置的梯度,这里为何选择这四个网格，猜测可能是因为无所谓那个方向，只要能算出梯度，进行插值获得X，y即可
        gradCell(stc);//该点
        gradCell(stc+1);//该点右侧点
        gradCell(stcnx);//该点下方点
        gradCell(stcnx+1);//该点右下方点

        // 接下来就是梯度插补值的计算
        float x1 = (1.0-dx)*gradx[stc] + dx*gradx[stc+1];
        float x2 = (1.0-dx)*gradx[stcnx] + dx*gradx[stcnx+1];
        float x = (1.0-dy)*x1 + dy*x2; // interpolated x
        float y1 = (1.0-dx)*grady[stc] + dx*grady[stc+1];
        float y2 = (1.0-dx)*grady[stcnx] + dx*grady[stcnx+1];
        float y = (1.0-dy)*y1 + dy*y2; // interpolated y

        // show gradients
        ROS_DEBUG("[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n",
                  gradx[stc], grady[stc], gradx[stc+1], grady[stc+1],
            gradx[stcnx], grady[stcnx], gradx[stcnx+1], grady[stcnx+1],
            x, y);

        // check for zero gradient, failed
        if (x == 0.0 && y == 0.0)
        {
          ROS_DEBUG("[PathCalc] Zero gradient");
          return 0;
        }

        //向梯度方向移动
        float ss = pathStep/hypot(x, y);
        dx += x*ss;
        dy += y*ss;

        // 最后检测dx和dy是否出界，如果大于1，就相当于是在下一个格子，这样还是会不光滑，跟grid一样。因此不能让其大于1
        if (dx > 1.0) { stc++; dx -= 1.0; }
        if (dx < -1.0) { stc--; dx += 1.0; }
        if (dy > 1.0) { stc+=nx; dy -= 1.0; }
        if (dy < -1.0) { stc-=nx; dy += 1.0; }

      }

      //      ROS_INFO("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
      //	     potarr[stc], x, y, pathx[npath-1], pathy[npath-1]);
    }

    //  return npath;			// out of cycles, return failure
    ROS_DEBUG("[PathCalc] No path found, path too long");
    //savemap("navfn_pathlong");
    return 0;			// out of cycles, return failure
  }


  //
  // gradient calculations
  //

  // 计算cell的梯度
  float				
    NavFn::gradCell(int n)
    {
      //如果这个梯度是增加的，就直接返回。 实际上在getPtah函数调用这个函数的时候
      //并没有调用返回值，而是对gradx_和grady_进行使用。增加的梯度相当于向这个方向
      //移动会使得potential增加。但是我们应该是让它减小。
      if (gradx[n]+grady[n] > 0.0)	// check this cell
        return 1.0;			
      //判断当前的n是不是越界，之前是否已经检测过了或者是超出了
      if (n < nx || n > ns-nx)
        return 0.0;

      float cv = potarr[n];
      float dx = 0.0;
      float dy = 0.0;

      //检测障碍物，如果当前的potential大于阈值，就判定有障碍，向小于阈值的方向移动。
      //因此dx和dy就给定在没有障碍方向的一个固定值 lethal_cost
      if (cv >= POT_HIGH) 
      {
        if (potarr[n-1] < POT_HIGH)
          dx = -COST_OBS;
        else if (potarr[n+1] < POT_HIGH)
          dx = COST_OBS;

        if (potarr[n-nx] < POT_HIGH)
          dy = -COST_OBS;
        else if (potarr[n+nx] < POT_HIGH)
          dy = COST_OBS;
      }
      //如果没有遇到障碍，就计算dx和dy。
      else				// not in an obstacle
      {
        // dx calc, average to sides
        if (potarr[n-1] < POT_HIGH)
          dx += potarr[n-1]- cv;	
        if (potarr[n+1] < POT_HIGH)
          dx += cv - potarr[n+1]; 

        // dy calc, average to sides
        if (potarr[n-nx] < POT_HIGH)
          dy += potarr[n-nx]- cv;	
        if (potarr[n+nx] < POT_HIGH)
          dy += cv - potarr[n+nx]; 
      }

      // normalize  对dx和dy进行归一，并且赋值给gradx_和grady_
      float norm = hypot(dx, dy);
      if (norm > 0)
      {
        norm = 1.0/norm;
        gradx[n] = norm*dx;
        grady[n] = norm*dy;
      }
      return norm;
    }


  //
  // display function setup
  // <n> is the number of cycles to wait before displaying,
  //     use 0 to turn it off

  void
    NavFn::display(void fn(NavFn *nav), int n)
    {
      displayFn = fn;
      displayInt = n;
    }


  //
  // debug writes
  // saves costmap and start/goal
  //

  void 
    NavFn::savemap(const char *fname)
    {
      char fn[4096];

      ROS_DEBUG("[NavFn] Saving costmap and start/goal points");
      // write start and goal points
      sprintf(fn,"%s.txt",fname);
      FILE *fp = fopen(fn,"w");
      if (!fp)
      {
        ROS_WARN("Can't open file %s", fn);
        return;
      }
      fprintf(fp,"Goal: %d %d\nStart: %d %d\n",goal[0],goal[1],start[0],start[1]);
      fclose(fp);

      // write cost array
      if (!costarr) return;
      sprintf(fn,"%s.pgm",fname);
      fp = fopen(fn,"wb");
      if (!fp)
      {
        ROS_WARN("Can't open file %s", fn);
        return;
      }
      fprintf(fp,"P5\n%d\n%d\n%d\n", nx, ny, 0xff);
      fwrite(costarr,1,nx*ny,fp);
      fclose(fp);
    }
};
