#include "nav2_gradient_costmap_plugin/gradient_layer.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <cmath>

namespace nav2_gradient_costmap_plugin
{

GradientLayer::GradientLayer()
{
}

void GradientLayer::onInitialize()
{
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
void GradientLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  unsigned char * master_array =
    master_grid.getCharMap();

  // Costmap中央を波の中心にする
  int cx = master_grid.getSizeInCellsX() / 2;
  int cy = master_grid.getSizeInCellsY() / 2;

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {

      int dx = i - cx;
      int dy = j - cy;

      // 中心からの距離
      double r = std::sqrt(
        static_cast<double>(dx * dx + dy * dy));

      // 正弦波生成
      double wave =
        127.0 + 127.0 * std::sin(r * 0.2);

      int index = master_grid.getIndex(i, j);

      master_array[index] =
        static_cast<unsigned char>(wave);
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

PLUGINLIB_EXPORT_CLASS(
  nav2_gradient_costmap_plugin::GradientLayer,
  nav2_costmap_2d::Layer)

}

