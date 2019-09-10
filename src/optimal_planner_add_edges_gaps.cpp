#include <teb_local_planner/optimal_planner.h>
#include <visualization_msgs/MarkerArray.h>
#include <teb_local_planner/gap_intersection.h>

namespace teb_local_planner
{

geometry_msgs::Point toPoint(const Eigen::Vector2d& vec)
{
  geometry_msgs::Point point;
  point.x = vec.x();
  point.y = vec.y();
  return point;
}
  
void addMarker(visualization_msgs::MarkerArray& markers, const EdgeGap* edge, double offset = 0, Eigen::Vector2d* pose=NULL)
{
  
  visualization_msgs::Marker& pose_marker = markers.markers[0];
  visualization_msgs::Marker& gap_border_marker = markers.markers[1];
  visualization_msgs::Marker& gap_normal_marker = markers.markers[2];
  

  std_msgs::ColorRGBA gap_color;
  gap_color.a = .5;
  gap_color.g = 1;
  std_msgs::ColorRGBA pose_color;
  pose_color.a = .5;
  pose_color.r = .5;
  
  std_msgs::ColorRGBA border_color;
  border_color.a = .5;
  border_color.b = 1;
  border_color.g = 1;
  
  std_msgs::ColorRGBA normal_color_l;
  normal_color_l.a = .5;
  normal_color_l.r = 1;
  normal_color_l.g = .75;
  
  std_msgs::ColorRGBA normal_color_r;
  normal_color_r.a = .5;
  normal_color_r.r = .66;
  normal_color_r.g = .33;
  normal_color_r.b = 1;

  
  const Eigen::Vector2d& gap_center = edge->measurement();
  
  geometry_msgs::Point gap_l;
  gap_l.x = edge->gap_start_.x();
  gap_l.y = edge->gap_start_.y();
  gap_l.z = offset;
  
  geometry_msgs::Point gap_r;
  gap_r.x = edge->gap_end_.x();
  gap_r.y = edge->gap_end_.y();
  gap_r.z = offset;
  
  gap_border_marker.points.push_back(gap_l);
  gap_border_marker.points.push_back(gap_r);
  gap_border_marker.colors.push_back(border_color);
  gap_border_marker.colors.push_back(border_color);

//   // draw borders of gap
//   geometry_msgs::Point init_pos = toPoint(edge->initial_pos_);
//   init_pos.z = offset;
//   
//   gap_border_marker.points.push_back(init_pos);
//   gap_border_marker.points.push_back(gap_l);
//   gap_border_marker.points.push_back(init_pos);
//   gap_border_marker.points.push_back(gap_r);
//   
//   gap_border_marker.colors.push_back(border_color);
//   gap_border_marker.colors.push_back(border_color);
//   gap_border_marker.colors.push_back(border_color);
//   gap_border_marker.colors.push_back(border_color);
  
  
//   // draw normal vectors of gap
//   geometry_msgs::Point border_l_v = toPoint(edge->left_bnv_ + edge->initial_pos_);
//   border_l_v.z = offset;
//   gap_normal_marker.points.push_back(init_pos);
//   gap_normal_marker.points.push_back(border_l_v);
//   
//   geometry_msgs::Point border_r_v = toPoint(edge->right_bnv_ + edge->initial_pos_);
//   border_r_v.z = offset;
//   gap_normal_marker.points.push_back(init_pos);
//   gap_normal_marker.points.push_back(border_r_v);
//   
//   gap_normal_marker.colors.push_back(normal_color_l);
//   gap_normal_marker.colors.push_back(normal_color_l);
//   gap_normal_marker.colors.push_back(normal_color_r);
//   gap_normal_marker.colors.push_back(normal_color_r);
  
  
  // draw connections between poses and gap edges
  if(pose)
  {
    geometry_msgs::Point pose_msg = toPoint(*pose);
    pose_msg.z = offset;
    //pose_msg.x = pose->x();
    //pose_msg.y = pose->y();
  
    pose_marker.points.push_back(pose_msg);
    pose_marker.points.push_back(gap_l);
    pose_marker.points.push_back(pose_msg);
    pose_marker.points.push_back(gap_r);
    
    pose_marker.colors.push_back(pose_color);
    pose_marker.colors.push_back(pose_color);
    pose_marker.colors.push_back(pose_color);
    pose_marker.colors.push_back(pose_color);
  }
  
  
  //gap_marker.scale.x = .04;
  
  //markers.markers.push_back(gap_marker);
}

  
void TebOptimalPlanner::AddEdgesGaps()
{
  gap_pose_ind_ = -1;
  
  if(cfg_->optim.weight_gap <= 0)
    return;
  
  Eigen::Vector2d start_pos = teb_.PoseVertex(0)->pose().position();
  
  //if(gap_markers_.markers.size()==0)
  
  gap_markers_.markers.clear();
  
  std_msgs::Header header;
  header.stamp = ros::Time::now(); //egocircle_->getCurrentHeader().stamp;
  header.frame_id = cfg_->map_frame;
  
  visualization_msgs::Marker gap_marker;
  gap_marker.type = visualization_msgs::Marker::LINE_LIST;
  gap_marker.header = header;
  gap_marker.ns = "gap_poses";
  gap_marker.id = 0;
  gap_marker.action = visualization_msgs::Marker::ADD;

  gap_marker.scale.x = .04;
  
  gap_markers_.markers.push_back(gap_marker);
  
  gap_marker.ns = "gap_borders";
  gap_marker.scale.x = .02;
  gap_markers_.markers.push_back(gap_marker);
  
  gap_marker.ns = "gap_normals";
  gap_markers_.markers.push_back(gap_marker);
  
  
  // create edge for staying within gaps
  Eigen::Matrix<double,1,1> information_gap;
  information_gap.fill(cfg_->optim.weight_gap);
  
  double offset = 0;
  
  //const std::vector<egocircle_utils::gap_finding::Gap>& gaps = egocircle_->getDiscontinuityGaps();
  const std::vector<GlobalGap>& gaps = egocircle_->getGlobalGaps();
  
  //Note: should be able
  for(int j = 0; j < gaps.size(); ++j)
  {

    int closest_pose = -1;
    Eigen::Vector2d intersection_point, gap_start, gap_end;
    
    double closest_distance = 0;
    for (int i=0; i < teb_.sizePoses()-2; i++)
    {
      if(intersects(gaps[j], teb_.PoseVertex(i), teb_.PoseVertex(i+1), &intersection_point, &gap_start, &gap_end))
      {
        closest_pose = i;
        break;
      }
    }
    
    if(closest_pose >=0)
    //int i = teb_.sizePoses()/2;
    {
      EdgeGap* gap_edge = new EdgeGap;
      gap_edge->setVertex(0,teb_.PoseVertex(closest_pose));
      gap_edge->setInformation(information_gap);
      gap_edge->setParameters(*cfg_, gap_start, gap_end);
      optimizer_->addEdge(gap_edge);

      ROS_INFO_STREAM("Adding gap: " << gap_start.x() << "," << gap_start.y() << "] " << gap_end.x() << "," << gap_end.y() << "}");
      addMarker(gap_markers_, gap_edge, offset, &teb_.PoseVertex(closest_pose)->pose().position());
      //visualization_->addGapEdge(gap_edge, &teb_.PoseVertex(closest_pose)->pose().position());
      
      offset += .06;
      
      gap_pose_ind_ = closest_pose;
      break;  //prevent adding multiple gap constraints per trajectory
    }
  }
  
  //gap_pub_.publish(markers);
}

}
