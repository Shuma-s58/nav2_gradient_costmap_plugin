#ifndef GRADIENT_LAYER_HPP_
#define GRADIENT_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace nav2_gradient_costmap_plugin
{

class GradientLayer : public nav2_costmap_2d::Layer
{
public:
  GradientLayer();

  virtual void onInitialize();
  virtual void updateBounds(
      double robot_x,
      double robot_y,
      double robot_yaw,
      double * min_x,
      double * min_y,
      double * max_x,
      double * max_y);

  virtual void updateCosts(
      nav2_costmap_2d::Costmap2D & master_grid,
      int min_i,
      int min_j,
      int max_i,
      int max_j);

  virtual void reset();
  virtual bool isClearable();

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  bool need_recalculation_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::TransformStamped tf;
  sensor_msgs::msg::PointCloud2 cloud_odom;
};

}

#endif
