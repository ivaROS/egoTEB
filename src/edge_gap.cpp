#include <teb_local_planner/g2o_types/edge_gap.h>
#include <cmath> 

namespace teb_local_planner
{

  double computeBoundaryError(Eigen::Vector2d pnt, Eigen::Vector2d bnv, double boundary_ratio, double boundary_exponent, double boundary_threshold)
  {
    double dot_prod = pnt.dot(bnv);
    
    //double x = dot_prod/delta + 1 + cfg_.gaps.gap_boundary_tolerance;
    double bounded = penaltyBoundFromBelow(-dot_prod/boundary_ratio, 0, 1+boundary_threshold/boundary_ratio);
    
    double exp = pow(bounded,2*boundary_exponent);
    return exp;
  }
  
  double EdgeGap::calculateError(Eigen::Vector2d pos)
  {
    Eigen::Vector2d pnt = pos- initial_pos_;
    
    double costl = computeBoundaryError(pnt, left_bnv_, cfg_->gaps.gap_boundary_ratio, cfg_->gaps.gap_boundary_exponent, cfg_->gaps.gap_boundary_threshold); 
    double costr = computeBoundaryError(pnt, right_bnv_, cfg_->gaps.gap_boundary_ratio, cfg_->gaps.gap_boundary_exponent, cfg_->gaps.gap_boundary_threshold); 
    ROS_DEBUG_STREAM_NAMED("gap_cost","Gap: [" << _measurement[0].x << "," << _measurement[0].y <<"] - [" << _measurement[1].x << "," << _measurement[1].y << "]. Pose: (" << pos.x() << "," << pos.y() << ") Costl: " << costl << ", Costr: " << costr);
    
    return std::max(costl, costr);
  }
  
  void EdgeGap::computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGap() on EdgeGap()");
    const VertexPose* pose = static_cast<const VertexPose*>(_vertices[0]);
    
    _error[0] = calculateError(pose->pose().position());

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeObstacle::computeError() _error[0]=%f\n",_error[0]);
  }
  
  
  Eigen::Vector2d getBoundaryNormalVector(const Eigen::Vector2d& pos, bool is_left, const Eigen::Vector2d initial_pos)
  {
    Eigen::Vector2d bl = pos - initial_pos;
    
    Eigen::Vector2d bnv; //boundary_normal_vector
    
    if(is_left)
    {
      bnv.x() = -bl.y();
      bnv.y() = bl.x();
    }
    else
    {
      bnv.x() = bl.y();
      bnv.y() = -bl.x();
    }
    
    return bnv;
  }
  
  void EdgeGap::setParameters(const TebConfig& cfg, const GlobalGap gap, const Eigen::Vector2d initial_pos)
  {
    cfg_ = &cfg;
    Eigen::Vector2d gapr(gap[0].x, gap[0].y);
    Eigen::Vector2d gapl(gap[1].x, gap[1].y);
    
    left_bnv_ = getBoundaryNormalVector(gapl, true, initial_pos);
    right_bnv_ = getBoundaryNormalVector(gapr, false, initial_pos);
    
    _measurement = gap;
    initial_pos_ = initial_pos;
  }


} // end namespace
