#include <teb_local_planner/gap_finder.h>

namespace teb_local_planner
{


void GapFinderGraph::createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity)
{
  if(!egocircle_)
  {
    ROS_ERROR("No egocircle impl in gap finder!");
  }
  
  // Clear existing graph and paths

  ros::WallTime starttime = ros::WallTime::now();
  clearGraph();

  ros::WallTime endtime = ros::WallTime::now();
  ROS_INFO_STREAM("creatGraph, gap, clearGraph, " <<  (endtime - starttime).toSec() * 1e3 << "ms, " << ros::Time::now());
  starttime = ros::WallTime::now();

  // Direction-vector between start and goal and normal-vector:
  Eigen::Vector2d diff = goal.position()-start.position();
  
  if (diff.norm()<cfg_->goal_tolerance.xy_goal_tolerance)
  {
    // ROS_DEBUG("HomotopyClassPlanner::createProbRoadmapGraph(): xy-goal-tolerance already reached.");
    if (hcp_->getTrajectoryContainer().empty())
    {
      // ROS_INFO("HomotopyClassPlanner::createProbRoadmapGraph(): Initializing a small straight line to just correct orientation errors.");
      hcp_->addAndInitNewTeb(start, goal, start_velocity);
    }
    return;
  }

  diff.normalize();
  
  // Get vertices of gaps
//   std::vector<egocircle_utils::gap_finding::Gap> gaps = egocircle_->getDiscontinuityGaps(); //getHierarchicalGapPoints(.5);
//   
//   std::vector<ego_circle::EgoCircularPoint> gap_points;
//   for(egocircle_utils::gap_finding::Gap gap : gaps)
//   {
//     gap_points.push_back(gap.getMid());
//   }
//   
//   egocircle_->toGlobal(gap_points);
  
  std::vector<int> utilized_gaps;
  for(auto eq : equivalency_classes_)
  {
     const GapHSignature* sig = dynamic_cast<GapHSignature*>(eq.first.get());
     if(sig)
     {
        int gap_num= sig->getGap();
        utilized_gaps.push_back(gap_num);
        // ROS_INFO_STREAM("Added Gap #" << gap_num << " to the 'ignore' list for graph creation");
     }
     else
     {
      //  ROS_WARN_STREAM("Signature type is not GapHSignature, cannot prune gaps in graph");
     }
  }
  
  double gap_point_buffer_dist = 0.5;
  
  std::vector<GlobalGap> gap_points = egocircle_->getGlobalGaps();
    
  // ROS_INFO_STREAM("Got " << gap_points.size() << " gaps.");
  
  // Start sampling
  for (int i=0; i < gap_points.size(); ++i)
  {
    auto result1 = std::find(std::begin(utilized_gaps), std::end(utilized_gaps), i);
    
    if(result1 != std::end(utilized_gaps))
    {
      continue;
    }
    
    GlobalGap gap = gap_points[i];
    
    double best_dot_p = -1;
    Eigen::Vector2d best_gap_point;
    
    int num_segments = gap.size() - 1;
    for(int segment=0; segment < num_segments; segment++)
    {
      auto gap_p = gap[segment];
      auto gap_p2 = gap[segment + 1];
      
      Eigen::Vector2d midpoint;
      midpoint(0)=(gap_p.x+gap_p2.x)/2;
      midpoint(1)=(gap_p.y+gap_p2.y)/2;
      
      Eigen::Vector2d distgap = midpoint-start.position();
      distgap.normalize();
      
      double dot_p = midpoint.dot(diff);
      // Check if the direction is backwards:
      if (dot_p > best_dot_p)
      {
        best_dot_p = dot_p;
        best_gap_point = midpoint + distgap*gap_point_buffer_dist;
      }
    }
 
    
    // ROS_INFO_STREAM("Gap #" << i << ": [" << best_gap_point(0) << "," << best_gap_point(1) << "]"); 
    
    // Add new vertex
    HcGraphVertexType v = boost::add_vertex(graph_);
    graph_[v].pos = best_gap_point;
  }

  HcGraphVertexType start_vtx = boost::add_vertex(graph_); // start vertex
  graph_[start_vtx].pos = start.position();
  
  // Now add goal vertex
  HcGraphVertexType goal_vtx = boost::add_vertex(graph_); // goal vertex
  graph_[goal_vtx].pos = goal.position();

  
  // ROS_INFO_STREAM("Added (" << num_vertices << ") vertices to graph.");

  endtime = ros::WallTime::now();
  auto num_vertices = boost::num_vertices(graph_);
  ROS_INFO_STREAM("creatGraph, gap, numVertex, " << num_vertices << ", " << ros::Time::now());

  ROS_INFO_STREAM("creatGraph, gap, addVertex, " <<  (endtime - starttime).toSec() * 1e3 << "ms, " << ros::Time::now());
  starttime = ros::WallTime::now();
  

//   // Insert Edges
//   HcGraphVertexIterator it_i, end_i, it_j, end_j;
//   for (boost::tie(it_i,end_i) = boost::vertices(graph_); it_i!=boost::prior(end_i); ++it_i) // ignore goal in this loop
//   {
//     for (boost::tie(it_j,end_j) = boost::vertices(graph_); it_j!=end_j; ++it_j) // check all forward connections
//     {
//       if (it_i==it_j) // same vertex found
//         continue;
// 
//       Eigen::Vector2d distij = graph_[*it_j].pos-graph_[*it_i].pos;
//       distij.normalize(); // normalize in place
// 
//       // Check if the direction is backwards:
//       if (distij.dot(diff)<=obstacle_heading_threshold)
//           continue; // diff is already normalized
// 
// 
//       // Collision Check
//       bool collision = false;
//       for (ObstContainer::const_iterator it_obst = hcp_->obstacles()->begin(); it_obst != hcp_->obstacles()->end(); ++it_obst)
//       {
//         if ( (*it_obst)->checkLineIntersection(graph_[*it_i].pos,graph_[*it_j].pos, dist_to_obst) )
//         {
//           collision = true;
//           break;
//         }
//       }
//       if (collision)
//         continue;
// 
//       // Create Edge
//       boost::add_edge(*it_i,*it_j,graph_);
//     }
//   }
  
  // Insert Edges
  HcGraphVertexIterator it_i, end_i, it_j, end_j;
  for (boost::tie(it_i,end_i) = boost::vertices(graph_); it_i!=boost::prior(end_i,2); ++it_i) // ignore start and goal in this loop
  {
    // A gap vertex must be reachable from the start, so only check if it is in the right general direction
    {
      Eigen::Vector2d distij = graph_[*it_j].pos-start.position();
      distij.normalize(); // normalize in place
      
      // Check if the direction is backwards:
      if (distij.dot(diff)<=obstacle_heading_threshold)
      {
        continue; // diff is already normalized
      }
        
      // Add edge between start and gap verticies
      boost::add_edge(start_vtx, *it_i, graph_);
    }
    
    // Collision Check between gap vertex and goal
    {
      bool collision = false;
      for (ObstContainer::const_iterator it_obst = hcp_->obstacles()->begin(); it_obst != hcp_->obstacles()->end(); ++it_obst)
      {
        if ( (*it_obst)->checkLineIntersection(graph_[*it_i].pos, goal.position(), dist_to_obst) )
        {
          collision = true;
          break;
        }
      }
      if (collision)
        continue;
      
      // Create Edge
      boost::add_edge(*it_i, goal_vtx, graph_);
    }
  }
  
  //Finally, Collision Check between start and goal
  {
    bool collision = false;
    for (ObstContainer::const_iterator it_obst = hcp_->obstacles()->begin(); it_obst != hcp_->obstacles()->end(); ++it_obst)
    {
      if ( (*it_obst)->checkLineIntersection(start.position(), goal.position(), dist_to_obst) )
      {
        collision = true;
        break;
      }
    }
    if (!collision)
    {
      boost::add_edge(start_vtx, goal_vtx, graph_);
    }
  }
  
  
  // ROS_INFO_STREAM("Added (" << num_edges << ") edges to graph.");

  endtime = ros::WallTime::now();
  auto num_edges = boost::num_edges(graph_);
  ROS_INFO_STREAM("creatGraph, gap, numEdge, " << num_edges << ", " << ros::Time::now());
  ROS_INFO_STREAM("creatGraph, gap, insertEdge, " <<  (endtime - starttime).toSec() * 1e3 << "ms, " << ros::Time::now());
  starttime = ros::WallTime::now();

  /// Find all paths between start and goal!
  std::vector<HcGraphVertexType> visited;
  visited.push_back(start_vtx);
  //DepthFirstNI(graph_,visited,goal_vtx, start.theta(), goal.theta(), start_velocity);
  DepthFirst(graph_,visited,goal_vtx, start.theta(), goal.theta(), start_velocity);

  endtime = ros::WallTime::now();
  ROS_INFO_STREAM("creatGraph, gap, graphSearch, " <<  (endtime - starttime).toSec() * 1e3 << "ms, " << ros::Time::now());

  // ROS_INFO_STREAM("Finished depth first search.");
  
}

} //end namespace teb_local_planner
