#include <teb_local_planner/teb_local_planner_ros.h>


namespace teb_local_planner
{

void TebLocalPlannerROS::updateObstacleContainerWithEgocircle(const ros::Time stamp) //(const tf::Stamped<tf::Pose>& global_pose)
{
  if (cfg_.obstacles.include_egocircle_obstacles)
  //if(true)
  {  
    const auto& egocircle = *egocircle_wrapper_->getImpl();
    Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();
    egocircle_wrapper_->update();
    std_msgs::Header target_header;
    target_header.stamp = stamp;
    target_header.frame_id = global_frame_; //global_pose.frame_id_;
    ROS_DEBUG_STREAM("Target Header: " << target_header.frame_id << ", " << target_header.stamp);
    
    if(egocircle_wrapper_->isReady(target_header))
    {
      egocircle_wrapper_->setInflationRadius(robot_inscribed_radius_);
      std::vector<ego_circle::EgoCircularPoint> points = egocircle.getDecimatedEgoCircularPoints();//getLocalEgoCircularPoints();
      egocircle.transformToGlobal(points);
            
      
//       Eigen::Affine3d obstacle_to_map_eig;
//       try 
//       {
//         std_msgs::Header source_header = egocircle_wrapper_->getCurrentHeader();
//         
//         tf::StampedTransform obstacle_to_map;
//         tf_->waitForTransform(target_header.frame_id, target_header.stamp,
//                               source_header.frame_id, source_header.stamp,
//                               target_header.frame_id, ros::Duration(0.5));
//         tf_->lookupTransform(target_header.frame_id, target_header.stamp,
//                              source_header.frame_id, source_header.stamp,
//                              target_header.frame_id, obstacle_to_map);
//         tf::transformTFToEigen(obstacle_to_map, obstacle_to_map_eig);
//       }
//       catch (tf::TransformException ex)
//       {
//         ROS_ERROR("%s",ex.what());
//         obstacle_to_map_eig.setIdentity();
//       }
      
      for(auto point : points)
      {
            Eigen::Vector2d obs;
            obs.coeffRef(0) = point.x;
            obs.coeffRef(1) = point.y;
  //           costmap_->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));
  //           
  //           // check if obstacle is interesting (e.g. not far behind the robot)
  //           Eigen::Vector2d obs_dir = obs-robot_pose_.position();
  //           if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist  )
  //             continue;
            
            obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
            
      }
    }
  }
}

}
