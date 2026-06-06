#include "nav2_gradient_costmap_plugin/gradient_layer.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace nav2_gradient_costmap_plugin
{

GradientLayer::GradientLayer()
{
}

void GradientLayer::onInitialize()
{
  auto node = node_.lock();

  points_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/surestar_points", rclcpp::SensorDataQoS(), std::bind(&GradientLayer::pointCloudCallback, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  need_recalculation_ = true;
}

void GradientLayer::updateBounds(
  double,
  double,
  double,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  *min_x = -100;
  *min_y = -100;
  *max_x = 100;
  *max_y = 100;
}


// circle wave //
void GradientLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{

  if (!latest_cloud_) {
    return;
  }

  unsigned char * master_array =
    master_grid.getCharMap();

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {

      int index = master_grid.getIndex(i, j);

      master_array[index] = 0;
    }
  }

  sensor_msgs::msg::PointCloud2 cloud_odom;

  try {

    auto tf =
      tf_buffer_->lookupTransform(
        "odom",
        latest_cloud_->header.frame_id,
        tf2::TimePointZero);

    tf2::doTransform(
      *latest_cloud_,
      cloud_odom,
      tf);

  } catch (tf2::TransformException & ex) {

    RCLCPP_WARN(
      rclcpp::get_logger("GradientLayer"),
      "%s",
      ex.what());

    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float>
    iter_x(cloud_odom, "x");
  sensor_msgs::PointCloud2ConstIterator<float>
    iter_y(cloud_odom, "y");
  sensor_msgs::PointCloud2ConstIterator<float>
    iter_z(cloud_odom, "z");

  for (;
       iter_x != iter_x.end();
       ++iter_x, ++iter_y, ++iter_z)
  {
    float x = *iter_x;
    float y = *iter_y;
    float z = *iter_z;

    if (z < 0.1) {
      continue;
    }

    unsigned int mx;
    unsigned int my;

    if (master_grid.worldToMap(
          x,
          y,
          mx,
          my))
    {
      int index =
        master_grid.getIndex(mx, my);

      master_array[index] = 254;
    }
  }
}

// reset //
void GradientLayer::reset()
{
}

bool GradientLayer::isClearable()
{
  return false;
}

void GradientLayer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  latest_cloud_ = msg;
}

PLUGINLIB_EXPORT_CLASS(
  nav2_gradient_costmap_plugin::GradientLayer,
  nav2_costmap_2d::Layer)

}

