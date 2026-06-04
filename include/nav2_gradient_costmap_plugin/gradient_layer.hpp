#ifndef GRADIENT_LAYER_HPP_
#define GRADIENT_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

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

private:
  bool need_recalculation_;
};

}

#endif
