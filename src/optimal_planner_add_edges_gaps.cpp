#include <teb_local_planner/optimal_planner.h>
#include <visualization_msgs/MarkerArray.h>

namespace teb_local_planner
{
  
void addMarker(visualization_msgs::MarkerArray& markers, std_msgs::Header header, GlobalGap gap, Eigen::Vector2d* pose=NULL)
{
//   Eigen::Vector2d gap_l(gap[0].x, gap[0].y);
//   Eigen::Vector2d gap_r(gap[1].x, gap[1].y);
  
  visualization_msgs::Marker gap_marker;
  gap_marker.type = visualization_msgs::Marker::LINE_LIST;
  gap_marker.header = header;
  gap_marker.ns = "gap_edges";
  gap_marker.id = markers.markers.size();
  gap_marker.action = visualization_msgs::Marker::ADD;
  
  //gap_marker.color.a=.5;
  //gap_marker.color.g = 1;
  
  std_msgs::ColorRGBA gap_color;
  gap_color.a = .5;
  gap_color.g = 1;
  std_msgs::ColorRGBA pose_color;
  pose_color.a = .5;
  pose_color.r = .5;
  
  geometry_msgs::Point gap_l;
  gap_l.x = gap[0].x;
  gap_l.y = gap[0].y;
  
  geometry_msgs::Point gap_r;
  gap_r.x = gap[1].x;
  gap_r.y = gap[1].y;
  
  gap_marker.points.push_back(gap_l);
  gap_marker.points.push_back(gap_r);
  
  gap_marker.colors.push_back(gap_color);
  gap_marker.colors.push_back(gap_color);
  
  
  if(pose)
  {
    geometry_msgs::Point pose_msg;
    pose_msg.x = pose->x();
    pose_msg.y = pose->y();
  
    gap_marker.points.push_back(pose_msg);
    gap_marker.points.push_back(gap_l);
    gap_marker.points.push_back(pose_msg);
    gap_marker.points.push_back(gap_r);
    
    gap_marker.colors.push_back(pose_color);
    gap_marker.colors.push_back(pose_color);
    gap_marker.colors.push_back(pose_color);
    gap_marker.colors.push_back(pose_color);
  }
  
  gap_marker.scale.x = .04;
  
  markers.markers.push_back(gap_marker);
}

double intersects(GlobalGap gap, const VertexPose* pose1, const VertexPose* pose2)
{
  Eigen::Vector2d p3(gap[0].x, gap[0].y);
  Eigen::Vector2d p4(gap[1].x, gap[1].y);
  
  Eigen::Vector2d p1 = pose1->pose().position();
  Eigen::Vector2d p2 = pose2->pose().position();
  
  Eigen::Matrix2d m;
  m.col(0) = p4-p3;
  m.col(1) = -(p2-p1);
  
  Eigen::Vector2d res = m.inverse() * (p1-p3);
  
  if(res.x()>0 && res.x() < 1 && res.y() >0 && res.y() < 1)
  {
    return 1;
  }
  return -1;
}
  
void TebOptimalPlanner::AddEdgesGaps()
{
  Eigen::Vector2d start_pos = teb_.PoseVertex(0)->pose().position();
  
  visualization_msgs::MarkerArray markers;
  std_msgs::Header header;
  header.stamp = ros::Time::now(); //egocircle_->getCurrentHeader().stamp;
  header.frame_id = cfg_->map_frame;
  
  // create edge for staying within gaps
  Eigen::Matrix<double,1,1> information_gap;
  information_gap.fill(cfg_->optim.weight_gap);
  
  //Update with line intersection logic such as from here: http://www.cs.swan.ac.uk/~cssimon/line_intersection.html
  //Point to line: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
  for(GlobalGap gap : egocircle_->getGlobalGaps())
  {
    Eigen::Vector2d gap_l(gap[0].x, gap[0].y);
    Eigen::Vector2d gap_r(gap[1].x, gap[1].y);
    
    //Eigen::Vector2d gap_vec((gap[1].x - gap[0].x), (gap[1].y - gap[0].y));
    Eigen::Vector2d gap_vec = gap_r - gap_l;
    double gap_norm = gap_vec.norm();
    Eigen::Vector2d normed_vec = gap_vec/gap_norm;
    
    int closest_pose = -1;
    double closest_distance = 0;
    for (int i=0; i < teb_.sizePoses()-2; i++)
    {
      double res = intersects(gap, teb_.PoseVertex(i), teb_.PoseVertex(i+1));
      if(res >=0)
      {
        closest_distance = res;
        closest_pose = i;
      }
      /*
      Eigen::Vector2d pos = teb_.PoseVertex(i)->pose().position();
      
      Eigen::Vector2d pos_v = pos - gap_l;
      
      double vdot = pos_v.dot(normed_vec);
      if(vdot > 0 && vdot < gap_norm)
      {
        double gap_dist = (gap_vec.y()*pos.x() - gap_vec.x()*pos.y()+ gap_r.x()*gap_l.y() - gap_r.y()*gap_l.x())/gap_norm;
        if(closest_pose < 0 || gap_dist < closest_distance)
        {
          closest_distance = gap_dist;
          closest_pose = i;
        }
      }
      */
    }
    
    if(closest_pose >=0)
    //int i = teb_.sizePoses()/2;
    {
      EdgeGap* gap_edge = new EdgeGap;
      gap_edge->setVertex(0,teb_.PoseVertex(closest_pose));
      gap_edge->setInformation(information_gap);
      gap_edge->setParameters(*cfg_, gap, start_pos);
      optimizer_->addEdge(gap_edge);
      
      addMarker(markers, header, gap, &teb_.PoseVertex(closest_pose)->pose().position());
    }
  }
  
  gap_pub_.publish(markers);
}

}
