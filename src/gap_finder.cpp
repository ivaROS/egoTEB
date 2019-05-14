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
  clearGraph();

  // Direction-vector between start and goal and normal-vector:
  Eigen::Vector2d diff = goal.position()-start.position();
  
  if (diff.norm()<cfg_->goal_tolerance.xy_goal_tolerance)
  {
    ROS_DEBUG("HomotopyClassPlanner::createProbRoadmapGraph(): xy-goal-tolerance already reached.");
    if (hcp_->getTrajectoryContainer().empty())
    {
      ROS_INFO("HomotopyClassPlanner::createProbRoadmapGraph(): Initializing a small straight line to just correct orientation errors.");
      hcp_->addAndInitNewTeb(start, goal, start_velocity);
    }
    return;
  }

  diff.normalize();
  
  // Get vertices of gaps
  std::vector<ego_circle::EgoCircularPoint> gap_points = egocircle_->getDiscontinuityGapPoints(); //getHierarchicalGapPoints(.5);
  egocircle_->transformToGlobal(gap_points);
    
  ROS_INFO_STREAM("Got " << gap_points.size() << " gaps.");

  // Start sampling
  for (int i=0; i < gap_points.size(); ++i)
  {
    Eigen::Vector2d sample;
    
    sample(0)=gap_points[i].x;
    sample(1)=gap_points[i].y;
    
    ROS_INFO_STREAM("Gap #" << i << ": [" << sample(0) << "," << sample(1) << "]"); 
    
    // Add new vertex
    HcGraphVertexType v = boost::add_vertex(graph_);
    graph_[v].pos = sample;
  }

  HcGraphVertexType start_vtx = boost::add_vertex(graph_); // start vertex
  graph_[start_vtx].pos = start.position();
  
  // Now add goal vertex
  HcGraphVertexType goal_vtx = boost::add_vertex(graph_); // goal vertex
  graph_[goal_vtx].pos = goal.position();

  auto num_vertices = boost::num_vertices(graph_);
  
  ROS_INFO_STREAM("Added (" << num_vertices << ") vertices to graph.");
  

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
        //continue; // diff is already normalized
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
  
  auto num_edges = boost::num_edges(graph_);
  
  ROS_INFO_STREAM("Added (" << num_edges << ") edges to graph.");
  

  /// Find all paths between start and goal!
  std::vector<HcGraphVertexType> visited;
  visited.push_back(start_vtx);
  DepthFirst(graph_,visited,goal_vtx, start.theta(), goal.theta(), start_velocity);
  ROS_INFO_STREAM("Finished depth first search.");
  
}

}
