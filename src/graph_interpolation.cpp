#include <teb_local_planner/graph_search.h>
#include <teb_local_planner/homotopy_class_planner.h>
#include <tf/transform_datatypes.h>

namespace teb_local_planner
{
  
std::vector<geometry_msgs::PoseStamped> GraphSearchInterface::InterpolateGraph(const HcGraph& g, const std::vector<HcGraphVertexType>& visited, double start_orientation,
                                      double goal_orientation, double diststep)
{
  std::vector<geometry_msgs::PoseStamped> plan;
  
  auto it = visited.begin();
  auto end = visited.end();
  
  Eigen::Vector2d cur_pos, next_pos;
  cur_pos = getVector2dFromHcGraph(*it, g);
  
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x=cur_pos[0];
  pose.pose.position.y=cur_pos[1];
  
  double roll = 0;
  double pitch = 0;
  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, start_orientation);
  
  plan.push_back(pose);
  
  if(diststep <= 0)
  {
    diststep = std::numeric_limits<double>::max();
  }
  
  int idx = 1;
  
  std::advance(it,1);
  for (; it != end; ++it)
  {
    next_pos = getVector2dFromHcGraph(*it, g);
    
    Eigen::Vector2d diff_last = next_pos - cur_pos; 
    double diff_norm = diff_last.norm();
    
    Eigen::Vector2d unit_diff = diff_last/diff_norm;
    
    ROS_DEBUG_STREAM("Next: " << toString(next_pos) << "; curr: " << toString(cur_pos) << "; diff: " << toString(diff_last) << "; diff_norm: " << diff_norm << "; unit_diff: " << toString(unit_diff));
    
    double remaining_dist = diff_norm;
    
    while(remaining_dist >0)
    {
      double sub_diff_norm = std::min(remaining_dist, diststep);
      cur_pos += unit_diff * sub_diff_norm;
      ++idx;
      
      
      
      remaining_dist-= sub_diff_norm;
      pose.pose.position.x=cur_pos[0];
      pose.pose.position.y=cur_pos[1];
      plan.push_back(pose);
      if(idx ==200)
      {
        ROS_ERROR_STREAM("HELP!");
        //break;
      }
      else if(idx < 200)
      {
        ROS_DEBUG_STREAM("idx: " << idx << "; sub_diff_norm: " << sub_diff_norm << "; curr_point: " << toString(cur_pos) << "; remaining_dist: " << remaining_dist);
      }
    }
    
  }
  
  plan.back().pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, goal_orientation);
  
  return plan;
}


} // teb_local_planner
