/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
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
#include <costmap_2d/static_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::StaticLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

StaticLayer::StaticLayer() : dsrv_(NULL) {}

StaticLayer::~StaticLayer()
{
  if (dsrv_)
    delete dsrv_;
}
//Costmap2DROS的构造函数会调用各Layer中的initialize函数，而initialize函数会调用onInitialize函数，真正的初始化工作在这里完成。
void StaticLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();
  //从参数服务器加载订阅静态地图的话题名称；默认静态地图只接收一次，不进行更新；以及默认追踪未知区域（主地图默认关闭）
  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));
  nh.param("first_map_only", first_map_only_, false);
  nh.param("subscribe_to_updates", subscribe_to_updates_, false);

  nh.param("track_unknown_space", track_unknown_space_, true);
  nh.param("use_maximum", use_maximum_, false);

  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  nh.param("trinary_costmap", trinary_costmap_, true);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;

  //订阅map_topic，回调函数：incomingMap，map_received_和has_updated_data_标志位设置为false（一旦收到地图，进入回调函数，它们会被置为真），没有接收到地图时，阻塞。
  //只有当话题改变后才重新订阅
  if (map_sub_.getTopic() != ros::names::resolve(map_topic))
  {
    // we'll subscribe to the latched topic that the map server uses
    ROS_INFO("Requesting the map...");
    map_sub_ = g_nh.subscribe(map_topic, 1, &StaticLayer::incomingMap, this);
    map_received_ = false;
    has_updated_data_ = false;
    //如果map_received_一直是false，则一直阻塞在这里，只有在接收到后进入回调函数更新为true后才会继续进行
    ros::Rate r(10);
    while (!map_received_ && g_nh.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());
    //判断是否接收static map的更新（默认否），若接收，则开启topic对map_topic + "_updates"的更新，最后开启参数动态配置服务。
    if (subscribe_to_updates_)
    {
      ROS_INFO("Subscribing to updates");
      map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &StaticLayer::incomingUpdate, this);

    }
  }
  else
  {
    has_updated_data_ = true;
  }

  if (dsrv_)
  {
    delete dsrv_;
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &StaticLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void StaticLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
  {
    enabled_ = config.enabled;
    has_updated_data_ = true;
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
  }
}

void StaticLayer::matchSize()
{
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
  if (!layered_costmap_->isRolling())
  {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }
}
//接收到的地图上：unknown_cost_value（默认为-1）为未知区域，lethal_cost_threshold（默认100）以上为致命障碍物
//当接收到的地图上为-1时，若追踪未知区域，则本层地图上赋值NO_INFORMATION（255）；否则，在本层地图上赋值FREE_SPACE（0）；
//当接收到的地图上>=100时，在本层地图上赋值LETHAL_OBSTACLE（254）；
//若以上都不是，则按比例返回代价值。
unsigned char StaticLayer::interpretValue(unsigned char value)
{
  //如果追踪未知区域，且传入的参数代表未知，设置为NO_INFORMATION
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;
  //如果不追踪未知区域，且传入的参数代表未知，设置为FREE_SPACE
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  //如果都不是，设置FREE_SPACE
  else if (trinary_costmap_)
    return FREE_SPACE;
  //如果以上都不是，返回对应比例的障碍值
  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}
//获取接收到的静态地图的尺寸，当地图不随机器人移动时，若接收到的静态地图和主地图的尺寸/分辨率/起点不同，以接收到的地图为准，调整主地图的参数。
void StaticLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

   //如果master map的尺寸、分辨率或原点与获取到的地图不匹配，重新设置master map
  Costmap2D* master = layered_costmap_->getCostmap();
  if (!layered_costmap_->isRolling() &&
      (master->getSizeInCellsX() != size_x ||
       master->getSizeInCellsY() != size_y ||
       master->getResolution() != new_map->info.resolution ||
       master->getOriginX() != new_map->info.origin.position.x ||
       master->getOriginY() != new_map->info.origin.position.y))
  {
    // Update the size of the layered costmap (and all layers, including this one)
    ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y,
                                true /* set size_locked to true, prevents reconfigureCb from overriding map size*/);
  }
  //若本层的数据和接收到的静态地图的参数不同时，继续以接收到的地图为准，调整本层参数。
  else if (size_x_ != size_x || size_y_ != size_y ||
           resolution_ != new_map->info.resolution ||
           origin_x_ != new_map->info.origin.position.x ||
           origin_y_ != new_map->info.origin.position.y)
  {
    // only update the size of the costmap stored locally in this layer
    ROS_INFO("Resizing static layer to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    resizeMap(size_x, size_y, new_map->info.resolution,
              new_map->info.origin.position.x, new_map->info.origin.position.y);
  }
  //将接收到的静态地图数据复制到本层地图，复制过程中调用interpretValue函数，进行“翻译”，内容后述。并设置好地图坐标系中的原点x_、y_，以及地图尺寸和标志位。
  //若first_map_only_开启，则在第一次接收到地图后，话题将不再接收消息。
  unsigned int index = 0;

  //用订阅到的map数据来初始化本层static map的数据成员costmap
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned char value = new_map->data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }
  map_frame_ = new_map->header.frame_id;

  // we have a new map, update full size of map
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;

  // shutdown the map subscrber if firt_map_only_ flag is on
  if (first_map_only_)
  {
    ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
    map_sub_.shutdown();
  }
}

void StaticLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)
{
  unsigned int di = 0;
  //将接收到的静态地图数据复制到本层地图，复制过程中调用interpretValue函数，进行“翻译”，内容后述。并设置好地图坐标系中的原点x_、y_，以及地图尺寸和标志位。
  for (unsigned int y = 0; y < update->height ; y++)
  {
    unsigned int index_base = (update->y + y) * size_x_;
    for (unsigned int x = 0; x < update->width ; x++)
    {
      unsigned int index = index_base + x + update->x;
      costmap_[index] = interpretValue(update->data[di++]);
    }
  }
  x_ = update->x;
  y_ = update->y;
  width_ = update->width;
  height_ = update->height;
  has_updated_data_ = true;
}

void StaticLayer::activate()
{
  onInitialize();
}

void StaticLayer::deactivate()
{
  map_sub_.shutdown();
  if (subscribe_to_updates_)
    map_update_sub_.shutdown();
}

void StaticLayer::reset()
{
  if (first_map_only_)
  {
    has_updated_data_ = true;
  }
  else
  {
    onInitialize();
  }
}
//若非rolling地图，在有地图数据更新时更新边界，否则，根据静态层更新的区域的边界更新传入的边界。
//由于默认不更新静态地图层，该层的bound将只更新一次。
void StaticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{
  //若地图不是窗口滚动的
  if( !layered_costmap_->isRolling() ){
    //has_extra_bounds_初始值为false，而在第一次结束前，has_updated_data_被设置为false
    //且订阅话题在第一次接收到消息后关闭，故对于static layer，updateBounds只进行一次
    //Bounds的范围是整张地图的大小，在updateBounds过程中没有对静态地图层做任何的更新
    if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
      return;
  }
  //若有extrabounds，使用之
  useExtraBounds(min_x, min_y, max_x, max_y);
  //将map系中的起点（x_， y_）与终点（x_ + width_,， y_ + height_）转换到世界系，确保传入的bound能包含整个map在世界系中的范围。
  double wx, wy;

  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  has_updated_data_ = false;
}
//这里将更新后的bound传入。
//若不是rolling地图，那么直接将静态地图层bound范围内的内容合并到主地图，因为二者的尺寸也一样，
//updateWithTrueOverwrite和updateWithMax采用不同的合并策略，在上一篇有记录。
void StaticLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!map_received_)
    return;

  if (!enabled_)
    return;
  //这里如果不是rolling选项，则直接将本层数据赋值到master map，因为他们尺寸也一样
  if (!layered_costmap_->isRolling())
  {
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
    if (!use_maximum_)
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    else
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
  //若为rolling地图，则找到静态地图和global系之间的坐标转换，通过主地图→global→静态地图的转换过程，找到主地图的cell在静态地图上对应的cost，赋值给主地图。
  else
  {
    //如果是rolling选项，master map就和该层数据坐标不一样
    unsigned int mx, my;
    double wx, wy;
    //首先获得map坐标系相对于global坐标系的位置，这个时候map坐标系随着机器人的运动而运动
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }

    tf2::Transform tf2_transform;
    tf2::convert(transform.transform, tf2_transform);
    for (unsigned int i = min_i; i < max_i; ++i)
    {
      for (unsigned int j = min_j; j < max_j; ++j)
      {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from global_frame_ to map_frame_
        tf2::Vector3 p(wx, wy, 0);
        p = tf2_transform*p;
        //global坐标p→静态地图坐标(mx, my)
        if (worldToMap(p.x(), p.y(), mx, my))
        {
          if (!use_maximum_)
            master_grid.setCost(i, j, getCost(mx, my));
          else
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
        }
      }
    }
  }
}

}  // namespace costmap_2d
