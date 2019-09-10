
#include <teb_local_planner/gap_intersection.h>

namespace teb_local_planner
{
  bool intersects(const GlobalGap& gap, const VertexPose* pose1, const VertexPose* pose2, Eigen::Vector2d* intersection, Eigen::Vector2d* gap_start, Eigen::Vector2d* gap_end)
    {
      Eigen::Vector2d p1 = pose1->pose().position();
      Eigen::Vector2d p2 = pose2->pose().position();
      ROS_DEBUG_STREAM_NAMED("[intersects]","Checking intersections of " << p1.x() << "," << p1.y() << ") : (" << p2.x() << "," << p2.y() << ")");
      
      
      int num_segments = gap.size() - 1;
      for(int segment=0; segment < num_segments; segment++)
      {
        auto gap_p = gap[segment];
        auto gap_p2 = gap[segment + 1];
        
        Eigen::Vector2d p3(gap_p.x, gap_p.y);
        Eigen::Vector2d p4(gap_p2.x, gap_p2.y);
        
        if(intersects(p1, p2, p3, p4, intersection))
        {
          if(gap_start)
            *gap_start=p3;
          if(gap_end)
            *gap_end=p4;
          return true;
        }
      }
      
      return false;
    }
    
    //based on http://www.cs.swan.ac.uk/~cssimon/line_intersection.html
    bool intersects(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3, const Eigen::Vector2d& p4, Eigen::Vector2d* intersection)
    {
      Eigen::Matrix2d m;
      m.col(0) = p4-p3;
      m.col(1) = -(p2-p1);
      
      double eps = .01;
      
      Eigen::Vector2d res = m.inverse() * (p1-p3);
      
      ROS_DEBUG_STREAM_NAMED("[intersects]","Intersection results: (" << p3.x() << "," << p3.y() << ") : (" << p4.x() << "," << p4.y() << ") = " << res.x() << ", " << res.y());
      if(res.x()>=-eps && res.x() < 1+eps && res.y() >-eps && res.y() < 1+eps)
      {
        if(intersection)
        {
          *intersection = p1 + res.y()*(p2-p1);
        }
        return true;
      }
      return false;
    }
  
} // namespace teb_local_planner

