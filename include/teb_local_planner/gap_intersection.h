#ifndef GAP_INTERSECTION_H_
#define GAP_INTERSECTION_H_

#include <ros/ros.h>
#include <math.h>
#include <algorithm>
#include <functional>
#include <vector>
#include <iterator>

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/egocircle_interface.h>


namespace teb_local_planner
{
  bool intersects(const GlobalGap& gap, const VertexPose* pose1, const VertexPose* pose2, Eigen::Vector2d* intersection=nullptr);
  bool intersects(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3, const Eigen::Vector2d& p4, Eigen::Vector2d* intersection=nullptr);
}


#endif //GAP_INTERSECTION_H_
