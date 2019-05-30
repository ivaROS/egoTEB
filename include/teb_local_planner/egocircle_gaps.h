#ifndef EGOCIRCLE_GAPS_H
#define EGOCIRCLE_GAPS_H


#include <egocircle_utils/inflator.h>

namespace egocircle_utils
{
  namespace gap_finding
  {
    using ego_circle::EgoCircularPoint;
    using ego_circle::PolarPoint;
    
    struct Gap 
    {
      Gap(PolarPoint start, PolarPoint end):
        start(start),
        end(end)
      {}
      
      PolarPoint getMid()
      {
        return PolarPoint((start.r+end.r)/2, (start.theta+end.theta)/2);
      }
      
      PolarPoint start, end;
    };
    
    inline std::vector<Gap> getDiscontinuityGaps(const egocircle_utils::Inflator& inflator)
    {
      const Container& container = inflator.getContainer();

      const sensor_msgs::LaserScan& scan = *container.scan;
      float angle_increment = scan.angle_increment;
      float current_angle = scan.angle_min;

      int num_points = scan.ranges.size();
      
      float inscribed_radius=inflator.getInflationRadius();
      float r2 = inscribed_radius*inscribed_radius;
      
      const float max_r = container.egocircle_radius;
      const std::vector<float>& inflated_depths = inflator.getDepths();
      
      
      std::vector<PolarPoint> polar_points;
      for(int i = 0; i < num_points; ++i)
      {
        PolarPoint polar_point(inflated_depths[i], current_angle);
        if(polar_point.r<max_r)
        {
          polar_point.r+=inscribed_radius;
        }
        
        current_angle += angle_increment;
        polar_points.push_back(polar_point);
      }
      
      std::vector<Gap> gaps;
      for(int start_ind = 0; start_ind < num_points; start_ind++)
      {
        int prev_ind = (start_ind + num_points-1) % num_points;
        
        PolarPoint curr_p = polar_points[start_ind];
        PolarPoint prev_p = polar_points[prev_ind];
        
        float curr_r = curr_p.r;
        float prev_r = prev_p.r;
        
        bool add_gap = false;
        
        if(curr_r == max_r)
        {
          if(prev_r < max_r)
          {
            add_gap = true;
          }
        }
        else if(prev_r == max_r)
        {
          if(curr_r < max_r)
          {
            add_gap = true;
          }
        }
        else if (std::abs(prev_r - curr_r) >= 2* inscribed_radius)
        {
          add_gap = true;
        }
        
        if(add_gap)
        {
          Gap gap(curr_p,prev_p);
          gaps.push_back(gap);
          PolarPoint mid = gap.getMid();
          ROS_DEBUG_STREAM("Start: " << prev_p.r << "m, @ " << prev_p.theta << ", Stop: " << curr_p.r << "m, @ " << curr_p.theta << "Mid: " << mid.r << "m, @ " << mid.theta);
        }
      }
      
      return gaps;
    }

  } //end namespace gap_finding
} //end namespace egocircle_utils

#endif //EGOCIRCLE_GAPS_H
