/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 *   * Neither the name of the institute nor the names of its
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
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann
 *********************************************************************/
#ifndef EDGE_GAP_H_
#define EDGE_GAP_H_

#include <teb_local_planner/egocircle_interface.h>
#include <Eigen/Geometry>
#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>



namespace teb_local_planner
{

  //using Gap=egocircle_utils::gap_finding::Gap;
  
//   struct GapBoundary
//   {
//     GapBoundary(Eigen::Vector2d pos, bool is_left):
//       pos(pos),
//       is_left(is_left)
//     {}
//       
//     Eigen::Vector2d pos; //boundary_normal_vector
//     //Eigen::ParametrizedLine<double, 2> boundary_line;
//     bool is_left;
//   };
  
  //using GapBoundary = Eigen::Vector2d;
  
/**
 * @class EdgeInflatedObstacle
 * @brief Edge defining the cost function for keeping a minimum distance from inflated obstacles.
 * 
 * The edge depends on a single vertex \f$ \mathbf{s}_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyBelow}( dist2point, min_obstacle_dist ) \cdot weight_inflation \f$. \n
 * Additional, a second penalty is provided with \n
 * \f$ \min \textrm{penaltyBelow}( dist2point, inflation_dist ) \cdot weight_inflation \f$.
 * It is assumed that inflation_dist > min_obstacle_dist and weight_inflation << weight_inflation.
 * \e dist2point denotes the minimum distance to the point obstacle. \n
 * \e penaltyBelow denotes the penalty function, see penaltyBoundFromBelow() \n
 * @see TebOptimalPlanner::AddEdgesObstacles, TebOptimalPlanner::EdgeObstacle
 * @remarks Do not forget to call setTebConfig() and setObstacle()
 */     
class EdgeGap : public BaseTebUnaryEdge<1, Eigen::Vector2d, VertexPose>
{
  
public:
    
  /**
   * @brief Construct edge.
   */    
  EdgeGap() 
  {
    //_measurement = NULL;
  }
 
 // cost function implementation for testing costs of other points
 double calculateError(Eigen::Vector2d pos);
 
 
  /**
   * @brief Actual cost function
   */    
  void computeError();
//   {
//     ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGap() on EdgeGap()");
//     const VertexPose* pose = static_cast<const VertexPose*>(_vertices[0]);
//         
//     Eigen::Vector2d bl = pose.pose().position() - initial_pos_;
//     
//     Eigen::Vector2d boundary_normal_vector(-bl.y,bl.x);
//     
//     double dot_prod = pose_v.dot(_measurement);
//     
//     double delta = cfg_.gaps.gap_boundary_ratio;
//     
// 
//     //double x = dot_prod/delta + 1 + cfg_.gaps.gap_boundary_tolerance;
//     double bounded = penaltyBoundFromBelowl(-dot_prod, 0, cfg_->optim.penalty_epsilon)
//     
//     double exp = math.exp(bounded,2*cfg_.gaps.gap_boundary_exponent);    
//     
//     _error[0] = exp;
// 
// 
//     ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeObstacle::computeError() _error[0]=%f\n",_error[0]);
//   }

  /**
   * @brief Set pointer to associated obstacle for the underlying cost function 
   * @param obstacle 2D position vector containing the position of the obstacle
   */ 
//   void setGap(const GapBoundary gap)
//   {
//     _measurement = gap;
//   }
//   
//   void setInitialPos(const Eigen::Vector2d pos)
//   {
//     initial_pos_ = pos
//   }
//       
  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle 2D position vector containing the position of the obstacle
   */ 
  void setParameters(const TebConfig& cfg, const Eigen::Vector2d& gap_start, const Eigen::Vector2d& gap_end);
//   {
//     cfg_ = &cfg;
//     
//     _measurement = getBoundaryNormalVector(gap, initial_pos);
//     initial_pos_ = initial_pos;
//   }
  
//protected:
public:
  Eigen::Vector2d initial_pos_, left_bnv_, right_bnv_, gap_start_, gap_end_, gap_center_;
  double gap_radius_;
  
public:         
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
    

} // end namespace

#endif
