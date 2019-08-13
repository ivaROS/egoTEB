/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/timed_elastic_band.h>
#include <teb_local_planner/homotopy_class_planner.h>
#include <teb_local_planner/graph_search.h>
#include <turtlebot_trajectory_functions/path.h>

namespace teb_local_planner
{

  template<typename BidirIter, typename Fun>
  bool TimedElasticBand::initTrajectoryToGoalNI(BidirIter path_start, BidirIter path_end, Fun fun_position, double max_vel_x, double max_vel_theta,
                                              boost::optional<double> max_acc_x, boost::optional<double> max_acc_theta,
                                              boost::optional<double> start_orientation, boost::optional<double> goal_orientation, int min_samples, bool guess_backwards_motion, double diststep, std::shared_ptr<turtlebot_trajectory_testing::GenAndTest>& traj_tester) 
  {
    BidirIter goal_it = boost::prior(path_end);
    Eigen::Vector2d start_position = fun_position( *path_start );
    Eigen::Vector2d goal_position = fun_position( *goal_it );
    
    if(diststep <=0)
    {
      diststep = std::numeric_limits<double>::max();
    }
    
    bool backwards = false;
    
    double start_orient, goal_orient;
    if (start_orientation)
    {
      start_orient = *start_orientation;
      
      // check if the goal is behind the start pose (w.r.t. start orientation)
      if (guess_backwards_motion && (goal_position-start_position).dot(Eigen::Vector2d(std::cos(start_orient), std::sin(start_orient))) < 0) 
        backwards = true;
    }
    else
    {
      Eigen::Vector2d start2goal =  goal_position - start_position;
      start_orient = atan2(start2goal[1],start2goal[0]);
    }
    
    double timestep = 1; // TODO: time
    
    if (goal_orientation)
    {
      goal_orient = *goal_orientation;
    }
    else
    {
      goal_orient = start_orient;
    }
    
    
    
    
    if (!isInit())
    {
      std::vector<Eigen::Vector2d> poses;
      
      poses.push_back(start_position);
      
      //addPose(start_position, start_orient, true); // add starting point and mark it as fixed for optimization		
      
      // we insert middle points now (increase start by 1 and decrease goal by 1)
      //std::advance(path_start,1);
      //std::advance(path_end,-1);
      int idx=0;
      for (; path_start != path_end; ++path_start) // insert middle-points
      {
        //Eigen::Vector2d point_to_goal = path.back()-*it;
        //double dir_to_goal = atan2(point_to_goal[1],point_to_goal[0]); // direction to goal
        // Alternative: Direction from last path
        Eigen::Vector2d next_point = fun_position(*path_start);
        Eigen::Vector2d curr_point = Pose(idx).position();
        Eigen::Vector2d diff_last = next_point - curr_point; // we do not use boost::prior(*path_start) for those cases,
        // where fun_position() does not return a reference or is expensive.
        double diff_norm = diff_last.norm();
        
        Eigen::Vector2d unit_diff = diff_last/diff_norm;
        
        double yaw = atan2(diff_last[1],diff_last[0]);
        
//         if (backwards)
//           yaw = g2o::normalize_theta(yaw + M_PI);
        
        
//         ROS_INFO_STREAM("Next: " << toString(next_point) << "; curr: " << toString(curr_point) << "; diff: " << toString(diff_last) << "; diff_norm: " << diff_norm << "; unit_diff: " << toString(unit_diff) << "; yaw: " << yaw);
        
        double remaining_dist = diff_norm;
        
        // If this is the last pose, stop 1 pose short so we can properly add it
        if(path_start==goal_it)
        {
          remaining_dist-= diststep;
        }
        
        while(remaining_dist >0)
        {
          double sub_diff_norm = std::min(remaining_dist, diststep);
          curr_point += unit_diff * sub_diff_norm;
          
          poses.push_back(curr_point);
          
          remaining_dist-= sub_diff_norm;

          ++idx;
          //ROS_ASSERT_MSG(idx > 200, "Uh oh, idx got really big.  Value = %d", idx);
          if(idx>200)
          {
            ROS_ERROR("HELP!");
          }
          else
          {
            ROS_INFO_STREAM("idx: " << idx << "; sub_diff_norm: " << sub_diff_norm << "; curr_point: " << toString(curr_point) << "; remaining_dist: " << remaining_dist);
          }
        }
      }
      
      poses.push_back(goal_position);
      
//       PoseSE2 goal(goal_position, goal_orient);
    
      if(!traj_tester)
      {
        traj_tester = std::make_shared<turtlebot_trajectory_testing::GenAndTest>(ros::NodeHandle(), ros::NodeHandle("~"));
      }
      
      nav_msgs::Path path_t;
      for(auto pose : poses)
      {
        geometry_msgs::PoseStamped stamped;
        stamped.pose.position.x = pose.x;
        stamped.pose.position.y = pose.y;
        
        path_t.poses.push_back(stamped);
      }
      
      
      double v_des = .3;
      turtlebot_trajectory_functions::Path::Ptr pathtraj = std::make_shared<turtlebot_trajectory_functions::Path>(path_t, v_des);
      turtlebot_trajectory_generator::desired_traj_func::Ptr dtraj = pathtraj;
      double v_max=.5;
      double w_max=4;
      double a_max=.55;
      double w_dot_max=1.78;
      
      turtlebot_trajectory_generator::near_identity ni(1,5,1,.01,v_max,w_max,a_max,w_dot_max);    
      traj_func_type::Ptr nc=std::make_shared<traj_func_type>(ni);
      nc->setTrajFunc(dtraj);
      
      double tf = pathtraj->getTF();
      
      ROS_INFO_STREAM("Tf=" << tf);
      
      
      trajectory_generator::traj_params_ptr params = std::make_shared<trajectory_generator::traj_params>();
      params->tf=tf;
      
      auto res = traj_tester_->run(nc, odom, params);
      auto path_traj = res[0];
    
    
      
      addPose(start_position, start_orient, true); // add starting point and mark it as fixed for optimization		
      
    
    
    
      for(int i = 1; i < path_traj.num_states()-1;
      addPoseAndTimeDiff(curr_point, yaw ,timestep);
      
    
    
    
    
    
    /*
    
      addPose(start_position, start_orient, true); // add starting point and mark it as fixed for optimization		
      
      // we insert middle points now (increase start by 1 and decrease goal by 1)
      //std::advance(path_start,1);
      //std::advance(path_end,-1);
      int idx=0;
      for (; path_start != path_end; ++path_start) // insert middle-points
      {
        //Eigen::Vector2d point_to_goal = path.back()-*it;
        //double dir_to_goal = atan2(point_to_goal[1],point_to_goal[0]); // direction to goal
        // Alternative: Direction from last path
        Eigen::Vector2d next_point = fun_position(*path_start);
        Eigen::Vector2d curr_point = Pose(idx).position();
        Eigen::Vector2d diff_last = next_point - curr_point; // we do not use boost::prior(*path_start) for those cases,
        // where fun_position() does not return a reference or is expensive.
        double diff_norm = diff_last.norm();
        
        Eigen::Vector2d unit_diff = diff_last/diff_norm;
        
        double yaw = atan2(diff_last[1],diff_last[0]);
        
        if (backwards)
          yaw = g2o::normalize_theta(yaw + M_PI);
        
        
        ROS_INFO_STREAM("Next: " << toString(next_point) << "; curr: " << toString(curr_point) << "; diff: " << toString(diff_last) << "; diff_norm: " << diff_norm << "; unit_diff: " << toString(unit_diff) << "; yaw: " << yaw);
        
        double remaining_dist = diff_norm;
        
        // If this is the last pose, stop 1 pose short so we can properly add it
        if(path_start==goal_it)
        {
          remaining_dist-= diststep;
        }
        
        while(remaining_dist >0)
        {
          double sub_diff_norm = std::min(remaining_dist, diststep);
          curr_point += unit_diff * sub_diff_norm;
          
          remaining_dist-= sub_diff_norm;
          
          double timestep_vel = sub_diff_norm/max_vel_x; // constant velocity
          double timestep_acc;
        
          if(max_acc_x)
          {
            timestep_acc = sqrt(2*sub_diff_norm/(*max_acc_x)); // constant acceleration
            if (timestep_vel < timestep_acc && max_acc_x)
            {
              timestep = timestep_acc;
            }
            else
            {
              timestep = timestep_vel;
            }
          }
          else timestep = timestep_vel;
          
          if (timestep<0) timestep=0.2; // TODO: this is an assumption
          
          
          addPoseAndTimeDiff(curr_point, yaw ,timestep);
          //setPoseVertexFixed(sizePoses()-1, true);
          
        
          ++idx;
          //ROS_ASSERT_MSG(idx > 200, "Uh oh, idx got really big.  Value = %d", idx);
          if(idx>200)
          {
            ROS_ERROR("HELP!");
          }
          else
          {
            ROS_INFO_STREAM("idx: " << idx << "; sub_diff_norm: " << sub_diff_norm << "; curr_point: " << toString(curr_point) << "; remaining_dist: " << remaining_dist << "; timestep: " << timestep);
          }
        }
      }
      
      // Note: with interpolation, unlikely that will be less than min_samples, so could simplify things and remove this part
      Eigen::Vector2d diff = goal_position-Pose(idx).position();
      double diff_norm = diff.norm();
      double timestep_vel = diff_norm/max_vel_x; // constant velocity
      if (max_acc_x)
      {
        double timestep_acc = sqrt(2*diff_norm/(*max_acc_x)); // constant acceleration
        if (timestep_vel < timestep_acc) timestep = timestep_acc;
        else timestep = timestep_vel;
      }
      else timestep = timestep_vel;*/
      
      
      PoseSE2 goal(goal_position, goal_orient);
      
      // if number of samples is not larger than min_samples, insert manually
      if ( sizePoses() < min_samples-1 )
      {
        ROS_DEBUG("initTEBtoGoal(): number of generated samples is less than specified by min_samples. Forcing the insertion of more samples...");
        while (sizePoses() < min_samples-1) // subtract goal point that will be added later
        {
          // Each inserted point bisects the remaining distance. Thus the timestep is also bisected.
          timestep /= 2;
          // simple strategy: interpolate between the current pose and the goal
          addPoseAndTimeDiff( PoseSE2::average(BackPose(), goal), timestep ); // let the optimier correct the timestep (TODO: better initialization	
        }
      }
      
      // now add goal
      addPoseAndTimeDiff(goal, timestep); // add goal point
      setPoseVertexFixed(sizePoses()-1,true); // GoalConf is a fixed constraint during optimization
    }
    else // size!=0
    {
      ROS_WARN("Cannot init TEB between given configuration and goal, because TEB vectors are not empty or TEB is already initialized (call this function before adding states yourself)!");
      ROS_WARN("Number of TEB configurations: %d, Number of TEB timediffs: %d", sizePoses(), sizeTimeDiffs());
      return false;
    }
    return true;
  }  


  template<typename BidirIter, typename Fun>
  TebOptimalPlannerPtr HomotopyClassPlanner::addAndInitNewTebNI(BidirIter path_start, BidirIter path_end, Fun fun_position, double start_orientation, double goal_orientation, const geometry_msgs::Twist* start_velocity)
  {
    TebOptimalPlannerPtr candidate = TebOptimalPlannerPtr( new TebOptimalPlanner(*cfg_, obstacles_, robot_model_));

    candidate->teb().initTrajectoryToGoalNI(path_start, path_end, fun_position, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta,
                                  cfg_->robot.acc_lim_x, cfg_->robot.acc_lim_theta, start_orientation, goal_orientation, cfg_->trajectory.min_samples,
                                  cfg_->trajectory.allow_init_with_backwards_motion, cfg_->trajectory.dist_step, traj_tester_);

    if (start_velocity)
      candidate->setVelocityStart(*start_velocity);

    EquivalenceClassPtr H = calculateEquivalenceClass(candidate->teb().poses().begin(), candidate->teb().poses().end(), getCplxFromVertexPosePtr, obstacles_,
                                                      candidate->teb().timediffs().begin(), candidate->teb().timediffs().end());

    if(addEquivalenceClassIfNew(H))
    {
      ROS_WARN_STREAM("Added new TEB! [iterators]");
      tebs_.push_back(candidate);
      return tebs_.back();
    }
    ROS_INFO_STREAM("Did not add new TEB! [iterators]");

    // If the candidate constitutes no new equivalence class, return a null pointer
    return TebOptimalPlannerPtr();
  }
  

  void GraphSearchInterface::DepthFirstNI(HcGraph& g, std::vector<HcGraphVertexType>& visited, const HcGraphVertexType& goal, double start_orientation,
                                        double goal_orientation, const geometry_msgs::Twist* start_velocity)
  {
    // see http://www.technical-recipes.com/2011/a-recursive-algorithm-to-find-all-paths-between-two-given-nodes/ for details on finding all simple paths

    if ((int)hcp_->getTrajectoryContainer().size() >= cfg_->hcp.max_number_classes)
      return; // We do not need to search for further possible alternative homotopy classes.

    HcGraphVertexType back = visited.back();

    /// Examine adjacent nodes
    HcGraphAdjecencyIterator it, end;
    for ( boost::tie(it,end) = boost::adjacent_vertices(back,g); it!=end; ++it)
    {
      if ( std::find(visited.begin(), visited.end(), *it)!=visited.end() )
        continue; // already visited

      if ( *it == goal ) // goal reached
      {
        visited.push_back(*it);
        ROS_INFO_STREAM_NAMED("graph_search", "Found path to goal through graph!");
        // Add new TEB, if this path belongs to a new homotopy class
        //std::vector<geometry_msgs::PoseStamped> plan = InterpolateGraph(g, visited, start_orientation, goal_orientation, .1);
        //hcp_->addAndInitNewTeb(plan, start_velocity);
        
        
        hcp_->addAndInitNewTebNI(visited.begin(), visited.end(), boost::bind(getVector2dFromHcGraph, _1, boost::cref(graph_)),
                              start_orientation, goal_orientation, start_velocity);

        visited.pop_back();
        break;
      }
    }

    /// Recursion for all adjacent vertices
    for ( boost::tie(it,end) = boost::adjacent_vertices(back,g); it!=end; ++it)
    {
      if ( std::find(visited.begin(), visited.end(), *it)!=visited.end() || *it == goal)
        continue; // already visited || goal reached


      visited.push_back(*it);

      // recursion step
      DepthFirst(g, visited, goal, start_orientation, goal_orientation, start_velocity);

      visited.pop_back();
    }
  }
  

} // namespace teb_local_planner



