#ifndef GAP_FINDER_GRAPH_SEARCH_H
#define GAP_FINDER_GRAPH_SEARCH_H

#include <teb_local_planner/graph_search.h>
#include <teb_local_planner/homotopy_class_planner.h>

namespace teb_local_planner
{

class GapFinderGraph : public GraphSearchInterface
{
public:
  GapFinderGraph(const TebConfig& cfg, HomotopyClassPlanner* hcp, const EgoCircleInterface* egocircle, const HomotopyClassPlanner::EquivalenceClassContainer& equivalency_classes) : GraphSearchInterface(cfg, hcp), egocircle_(egocircle), equivalency_classes_(equivalency_classes){}

  virtual ~GapFinderGraph(){}

  /**
   * @brief Create a graph containing points in the global frame that can be used to explore new possible paths between start and goal.
   *
   * This version of the graph creation places a keypoint on the left and right side of each obstacle w.r.t to the goal heading. \n
   * All feasible paths between start and goal point are extracted using a Depth First Search afterwards. \n
   * This version works very well for small point obstacles. For more complex obstacles call the createProbRoadmapGraph()
   * method that samples keypoints in a predefined area and hopefully finds all relevant alternative paths.
   *
   * @see createProbRoadmapGraph
   * @param start Start pose from wich to start on (e.g. the current robot pose).
   * @param goal Goal pose to find paths to (e.g. the robot's goal).
   * @param dist_to_obst Allowed distance to obstacles: if not satisfying, the path will be rejected (note, this is not the distance used for optimization).
   * @param obstacle_heading_threshold Value of the normalized scalar product between obstacle heading and goal heading in order to take them (obstacles) into account [0,1]
   * @param start_velocity start velocity (optional)
   */
  virtual void createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity);
  
protected:
  const EgoCircleInterface* egocircle_;
  const HomotopyClassPlanner::EquivalenceClassContainer& equivalency_classes_;
};

} //end namespace

#endif // GAP_FINDER_GRAPH_SEARCH_H
