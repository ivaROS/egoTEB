#include <teb_local_planner/teb_validity_checker.h>

namespace teb_local_planner
{

bool checkTebValidity(const TimedElasticBand& teb, const EgoCircleInterface* egocircle)
{
  enum class CollisionState {Collision,Safe,Unknown};
  
  
  const egocircle_utils::Inflator* inflator = egocircle->getInflator();
  
  float inflation_radius = inflator->getInflationRadius();
  float egocircle_radius = inflator->getContainer().egocircle_radius;

  CollisionState prev_state = CollisionState::Safe;

  const PoseSequence& poses = teb.poses();
  unsigned int num_poses = poses.size();

  for(unsigned int i = 0; i < num_poses; ++i)
  {
    const VertexPose* vp = poses[i];
    const Eigen::Vector2d pose = vp->pose().position();
    
    CollisionState cur_state;
    
    {
      ego_circle::EgoCircularPoint ego_pose(pose.x(), pose.y());
      egocircle->toLocal(ego_pose); //transform pose from global to local
      ego_circle::PolarPoint polar_ego_pose(ego_pose);
      
      float freespace_range = egocircle->getInflatedEgoCircleRange(polar_ego_pose);
      
      if(freespace_range >= polar_ego_pose.r)  //definitely safe
      {
        cur_state = CollisionState::Safe;
      }
      else if(freespace_range <= polar_ego_pose.r-2*inflation_radius)  //possibly occluded
      {
        cur_state = CollisionState::Unknown;
      }
      else  //definite collision
      {
        cur_state = CollisionState::Collision;
      }
    
      if(polar_ego_pose.r < egocircle_radius && prev_state==CollisionState::Safe && cur_state==CollisionState::Collision)
      {
        //The current global plan will definitely result in collision, so should replan
        ROS_WARN_STREAM("Went from safe to collision!");
        return false;
      }
    }
    
    prev_state = cur_state;
    
  }


}

}
