
#include <teb_local_planner/optimal_planner.h>
#include <angles/angles.h>

namespace teb_local_planner
{
  
  

void TebOptimalPlanner::AddEdgesKinematicsDiffDrive()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_forward_drive==0)
    return; // if weight equals zero skip adding edges!
  
  double final_factor;
  if(gap_pose_ind_ <0)
  {
    final_factor = 1;
  }
  else
  {
    auto start_pose = teb_.Pose(0);
    auto gap_pose = teb_.Pose(gap_pose_ind_);
    auto end_pose = teb_.BackPose();
    
    auto P1 = start_pose.position();
    auto P2 = gap_pose.position();
    auto P3 = end_pose.position();
    double gap_angle = std::atan2(P3.y() - P1.y(), P3.x() - P1.x()) - std::atan2(P2.y() - P1.y(), P2.x() - P1.x());
    double normalized_gap_angle = angles::normalize_angle(gap_angle);
    
    double abs_gap_angle = std::abs(normalized_gap_angle);
   
    double PI = std::acos(-1);
    double start_descending_angle = cfg_->optim.gap_theta_start;
    double min_weight_angle = cfg_->optim.gap_theta_end;
    
    if(abs_gap_angle < start_descending_angle)
      final_factor = 1;
    else if(abs_gap_angle > min_weight_angle)
      final_factor = 0;
    else
    {
      final_factor = (min_weight_angle - abs_gap_angle)/(min_weight_angle-start_descending_angle);
    }
    
    ROS_INFO_STREAM("Gap angle: " << normalized_gap_angle << ", final_factor: " << final_factor);
    
    
  }
  
  


  
  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  
  int num_constraints = teb_.sizePoses()-1;
  
  for (int i=0; i < num_constraints; i++) // ignore twiced start only
  {
    double factor = final_factor + (1-final_factor)*(num_constraints - i) / num_constraints;
    information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh * factor;
    information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive * factor;
    
    if(factor<1)
      ROS_INFO_STREAM("[EdgeKinematics] Pose (" << teb_.Pose(i).position().x() << "," << teb_.Pose(i).position().y() << ") factor=" << factor);
    
    EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }	 
}

}
