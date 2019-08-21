
#include <teb_local_planner/gap_h_signature.h>

namespace teb_local_planner
{
    // TODO: outer loop should iterate over trajectories (to reduce how many times points must be transformed), and inner loops should check each gap against each link
    double intersects(egocircle_utils::gap_finding::Gap gap, const VertexPose* pose1, const VertexPose* pose2, const EgoCircleInterface* egocircle)
    {
      using ego_circle::PolarPoint;
      using ego_circle::EgoCircularPoint;
      
      EgoCircularPoint p1(pose1->pose().position().x(), pose1->pose().position().y());
      egocircle->toLocal(p1);
      PolarPoint pp1(p1); 
      
      EgoCircularPoint p2(pose2->pose().position().x(), pose2->pose().position().y());
      egocircle->toLocal(p2);
      PolarPoint pp2(p2);
      
      ROS_DEBUG_STREAM_NAMED("gap_signature", "Gap: " << toString(gap) << "P1(" << p1.x << "," << p1.y << "): [" << pp1.r << "m @" << pp1.theta << "] P2(" << p2.x << "," << p2.y << "): [" << pp2.r << "m @" << pp2.theta << "]");
      
      double dist = -1;
      if(pp1.theta >= gap.start.theta && pp1.theta <= gap.end.theta && pp2.theta >= gap.start.theta && pp2.theta <= gap.end.theta)
      {
        double r1e = (pp1.theta - gap.start.theta)/(gap.end.theta - gap.start.theta)*(gap.end.r - gap.start.r)+gap.start.r;
        double r2e = (pp2.theta - gap.start.theta)/(gap.end.theta - gap.start.theta)*(gap.end.r - gap.start.r)+gap.start.r;
        
        double avgr = (r1e + r2e)/2;
        
        ROS_DEBUG_STREAM_NAMED("gap_signature", "r1e: " << r1e << ", r2e: " << r2e);// << " avgr: " << avgr);
        
        if(pp2.r >= r2e && pp1.r <= r1e)
        {
          ROS_DEBUG_STREAM_NAMED("gap_signature", "Gap crossing!");
          dist = (pp2.r - avgr) + (avgr - pp1.r);
        }
        
      }
      return dist;
      
    }
    
  
} // namespace teb_local_planner

