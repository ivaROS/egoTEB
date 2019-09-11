#include <teb_local_planner/g2o_types/edge_gap.h>
#include <cmath> 

namespace teb_local_planner
{

  double computeBoundaryError(Eigen::Vector2d gap_center, Eigen::Vector2d pnt, double gap_radius, double boundary_ratio, double boundary_exponent, double boundary_threshold)
  {
    //double dot_prod = pnt.dot(bnv);
    double dist = (gap_center-pnt).norm();
    double rem_dist = dist/gap_radius;
    
    //double x = dot_prod/delta + 1 + cfg_.gaps.gap_boundary_tolerance;
    double bounded = penaltyBoundFromBelow(-rem_dist/(boundary_ratio-boundary_threshold), 0, -boundary_threshold/(boundary_ratio-boundary_threshold));
    
    double exp = pow(bounded,2*boundary_exponent);
    
    ROS_DEBUG_STREAM("[EdgeGap] Gap center: [" << gap_center.x() << "," << gap_center.y() << "], pnt: [" << pnt.x() << "," << pnt.y() << "], radius: " << gap_radius << ", dist: " << dist << ", rem_dist: " << rem_dist << ", bounded: " << bounded);
    return exp;
  }
  
  double EdgeGap::calculateError(Eigen::Vector2d pos)
  {
    //Eigen::Vector2d pnt = pos- initial_pos_;
    
    double cost = computeBoundaryError(gap_center_, pos, gap_radius_, cfg_->gaps.gap_boundary_ratio, cfg_->gaps.gap_boundary_exponent, cfg_->gaps.gap_boundary_threshold); 
    //double costr = computeBoundaryError(pnt, right_bnv_, cfg_->gaps.gap_boundary_ratio, cfg_->gaps.gap_boundary_exponent, cfg_->gaps.gap_boundary_threshold); 
    ROS_DEBUG_STREAM_NAMED("gap_cost","Gap: [" << gap_start_.x() << "," << gap_start_.y() <<"] - [" << gap_end_.x() << "," << gap_end_.y() << "]. Pose: (" << pos.x() << "," << pos.y() << ") Cost: " << cost);
    

    return cost;  
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
  
  void EdgeGap::setParameters(const TebConfig& cfg, const Eigen::Vector2d& gap_start, const Eigen::Vector2d& gap_end)
  {
    cfg_ = &cfg;
    //Eigen::Vector2d gapr(gap.front().x, gap.front().y);
    //Eigen::Vector2d gapl(gap.back().x, gap.back().y);
    
    //left_bnv_ = getBoundaryNormalVector(gapl, true, initial_pos);
    //right_bnv_ = getBoundaryNormalVector(gapr, false, initial_pos);
    
    //_measurement = gap;
    //initial_pos_ = initial_pos;
    gap_center_ = (gap_start+gap_end)/2;
    //_measurement.push_back(gap_start);
    //_measurement.push_back(gap_end);
    gap_start_ = gap_start;
    gap_end_ = gap_end;
    gap_radius_ = (gap_start-gap_end).norm()/2;
  }


} // end namespace
