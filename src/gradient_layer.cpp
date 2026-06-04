#include "nav2_gradient_costmap_plugin/gradient_layer.hpp"

#include "pluginlib/class_list_macros.hpp"

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


// main update //
void GradientLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  unsigned char * master_array =
    master_grid.getCharMap();

  int gradient_index;

  for (int j = min_j; j < max_j; j++) {

    gradient_index = 0;

    for (int i = min_i; i < max_i; i++) {

      int index = master_grid.getIndex(i, j);

      unsigned char cost =
        (254 - gradient_index * 10) % 255;

      gradient_index++;

      master_array[index] = cost;
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



