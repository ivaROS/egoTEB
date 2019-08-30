#include <teb_local_planner/homotopy_class_planner


bool checkTebValidity(TimedElasticBand& teb, egocircle)
{


        bool good_to_go = false;
        target_obstructed_=true;
        float inflation_radius = ego_impl_->getInflationRadius();
        float egocircle_radius = ego_impl_->getEgoCircleRadius();

        CollisionState prev_state = CollisionState::Safe;
        
        unsigned int num_poses = teb.size();
        unsigned int target_ind = 0;
        
        for(unsigned int i = 0; i < num_poses; ++i)
        {
          VertexPose* vp = teb.PoseVertex(i);
          Eigen::Vector2d pose = vp->pose().position();
          
          CollisionState cur_state;
          
          {
            ego_circle::EgoCircularPoint ego_pose(pose.x, pose.y);
            egocircle.toLocal(ego_pose); //transform pose from global to local
            ego_circle::PolarPoint polar_ego_pose(ego_pose);
            
            float obstacle_range = //getInflatedEgoCircleRange(polar_ego_pose);
            ego_circle::PolarPoint polar_target = target_point;
            
            if(obstacle_range >= polar_ego_pose.r)  //definitely safe
            {
              cur_state = CollisionState::Safe;
              polar_target_point_ = polar_target;
              curr_best_pose = pose;
              target_obstructed_ = false;
              target_ind = i;
            }
            else if(target_clearance <= polar_target.r-2*inflation_radius)  //possibly occluded
            {
              cur_state = CollisionState::Unknown;
            }
            else  //definite collision
            {
              cur_state = CollisionState::Collision;
            }
          
            if(polar_target.r < egocircle_radius-0.6 && prev_state==CollisionState::Safe && cur_state==CollisionState::Collision)
            {
              //The current global plan will definitely result in collision, so should replan
              ROS_WARN_STREAM("Went from safe to collision!");
              return false;
            }
          }
          
          prev_state = cur_state;
          
        }


  
}
