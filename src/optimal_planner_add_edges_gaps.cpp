#include <teb_local_planner/optimal_planner.h>
#include <visualization_msgs/MarkerArray.h>

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

  
  const GlobalGap& gap = edge->measurement();
  
  geometry_msgs::Point gap_l;
  gap_l.x = gap[0].x;
  gap_l.y = gap[0].y;
  gap_l.z = offset;
  
  geometry_msgs::Point gap_r;
  gap_r.x = gap[1].x;
  gap_r.y = gap[1].y;
  gap_r.z = offset;
  

  // draw borders of gap
  geometry_msgs::Point init_pos = toPoint(edge->initial_pos_);
  init_pos.z = offset;
  
  gap_border_marker.points.push_back(init_pos);
  gap_border_marker.points.push_back(gap_l);
  gap_border_marker.points.push_back(init_pos);
  gap_border_marker.points.push_back(gap_r);
  
  gap_border_marker.colors.push_back(border_color);
  gap_border_marker.colors.push_back(border_color);
  gap_border_marker.colors.push_back(border_color);
  gap_border_marker.colors.push_back(border_color);
  
  
  // draw normal vectors of gap
  geometry_msgs::Point border_l_v = toPoint(edge->left_bnv_ + edge->initial_pos_);
  border_l_v.z = offset;
  gap_normal_marker.points.push_back(init_pos);
  gap_normal_marker.points.push_back(border_l_v);
  
  geometry_msgs::Point border_r_v = toPoint(edge->right_bnv_ + edge->initial_pos_);
  border_r_v.z = offset;
  gap_normal_marker.points.push_back(init_pos);
  gap_normal_marker.points.push_back(border_r_v);
  
  gap_normal_marker.colors.push_back(normal_color_l);
  gap_normal_marker.colors.push_back(normal_color_l);
  gap_normal_marker.colors.push_back(normal_color_r);
  gap_normal_marker.colors.push_back(normal_color_r);
  
  
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

double intersects2(egocircle_utils::gap_finding::Gap gap, const VertexPose* pose1, const VertexPose* pose2, const EgoCircleInterface* egocircle)
{
  using ego_circle::PolarPoint;
  using ego_circle::EgoCircularPoint;
  
  EgoCircularPoint p1(pose1->pose().position().x(), pose1->pose().position().y());
  egocircle->toLocal(p1);
  PolarPoint pp1(p1); 
  
  EgoCircularPoint p2(pose2->pose().position().x(), pose2->pose().position().y());
  egocircle->toLocal(p2);
  PolarPoint pp2(p2);
  
  double dist = -1;
  if(pp1.theta >= gap.start.theta && pp1.theta <= gap.end.theta && pp2.theta >= gap.start.theta && pp2.theta <= gap.end.theta)
  {
    double r1e = (pp1.theta - gap.start.theta)/(gap.end.theta - gap.start.theta)*(gap.end.r - gap.start.r)+gap.start.r;
    double r2e = (pp2.theta - gap.start.theta)/(gap.end.theta - gap.start.theta)*(gap.end.r - gap.start.r)+gap.start.r;
    
    double avgr = (r1e + r2e)/2;
    
    if(pp2.r >= avgr && pp1.r <= avgr)
    {
      dist = (pp2.r - avgr) + (avgr - pp1.r);
    }
      
  }
  
  //ROS_INFO_STREAM("P1 [" << p1.x << "," << p1.y << "], P2 [" << p2.x << "," << p2.y << "]: " << dist);
  
  
  return dist;
  
  /*
  PolarPoint pmid = gap.getMid();
  EgoCircularPoint mid(pmid);
  
  Eigen::Vector2d emid(mid.x, mid.y);

  
  Eigen::Vector2d p1 = pose1->pose().position();
  Eigen::Vector2d p2 = pose2->pose().position();
  
  (p1-emid).norm() + (p2-emid).norm();
  
  Eigen::Matrix2d m;
  m.col(0) = p4-p3;
  m.col(1) = -(p2-p1);
  
  Eigen::Vector2d res = m.inverse() * (p1-p3);
  
  if(res.x()>0 && res.x() < 1 && res.y() >0 && res.y() < 1)
  {
    return 1;
  }
  return -1;*/
}
  
void TebOptimalPlanner::AddEdgesGaps()
{
  Eigen::Vector2d start_pos = teb_.PoseVertex(0)->pose().position();
  
  //if(gap_markers_.markers.size()==0)
  
  visualization_msgs::MarkerArray markers;
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
  
  markers.markers.push_back(gap_marker);
  
  gap_marker.ns = "gap_borders";
  gap_marker.scale.x = .02;
  markers.markers.push_back(gap_marker);
  
  gap_marker.ns = "gap_normals";
  markers.markers.push_back(gap_marker);
  
  
  // create edge for staying within gaps
  Eigen::Matrix<double,1,1> information_gap;
  information_gap.fill(cfg_->optim.weight_gap);
  
  double offset = 0;
  
  const std::vector<egocircle_utils::gap_finding::Gap>& gaps = egocircle_->getDiscontinuityGaps();
  const std::vector<GlobalGap>& global_gaps = egocircle_->getGlobalGaps();
  
  //Update with line intersection logic such as from here: http://www.cs.swan.ac.uk/~cssimon/line_intersection.html
  //Point to line: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
  for(int j = 0; j < gaps.size(); ++j)
  {
//     Eigen::Vector2d gap_l(gap[0].x, gap[0].y);
//     Eigen::Vector2d gap_r(gap[1].x, gap[1].y);
    
    //Eigen::Vector2d gap_vec((gap[1].x - gap[0].x), (gap[1].y - gap[0].y));
//     Eigen::Vector2d gap_vec = gap_r - gap_l;
//     double gap_norm = gap_vec.norm();
//     Eigen::Vector2d normed_vec = gap_vec/gap_norm;
    
    int closest_pose = -1;
    double closest_distance = 0;
    for (int i=0; i < teb_.sizePoses()-2; i++)
    {
      double res = intersects2(gaps[j], teb_.PoseVertex(i), teb_.PoseVertex(i+1), egocircle_);
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
      gap_edge->setParameters(*cfg_, global_gaps[j], start_pos);
      optimizer_->addEdge(gap_edge);
      
      
      
      addMarker(markers, gap_edge, offset, &teb_.PoseVertex(closest_pose)->pose().position());
      
      offset += .06;
    }
  }
  
  gap_pub_.publish(markers);
}

}
