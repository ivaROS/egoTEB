#ifndef EGOCIRCLE_GAPS_H
#define EGOCIRCLE_GAPS_H

#include <egocircle_utils/inflator.h>
#include <visualization_msgs/MarkerArray.h>

namespace teb_local_planner
{
  using GlobalGap = std::vector<ego_circle::EgoCircularPoint>;
}

namespace egocircle_utils
{
  namespace gap_finding
  {
    using ego_circle::EgoCircularPoint;
    using ego_circle::PolarPoint;
    using teb_local_planner::GlobalGap;
    
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
    
    std::string toString(const Gap& gap);
    
    std::vector<Gap> getDiscontinuityGaps(const egocircle_utils::Inflator& inflator);
    
    std::vector<Gap> getCollapsedGaps(const std::vector<Gap>& gaps, double max_r);
    
    visualization_msgs::MarkerArray getMarkers(const std::vector<Gap>& raw_gaps, const std::vector<Gap>& collapsed_gaps, std_msgs::Header header);
    
    void addGlobalGapsToMarker(visualization_msgs::MarkerArray& markers, const std::vector<GlobalGap>& gaps, std_msgs::Header header);
    
    
    /*
    static std::string toString(const Gap& gap)
    {
      std::stringstream ss;
      ss << "(" <<
      gap.start.r << "m @ " << gap.start.theta <<
      ") : (" << 
      gap.end.r << "m @ " << gap.end.theta <<
      ")";
      
      return ss.str();
    }
    
    
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
          ROS_INFO_STREAM("Adding gap: " << toString(gap));
        }
        
      }
      
      ROS_INFO_STREAM("Collapsing gaps:");
      std::vector<Gap> collapsed_gaps;
      if(gaps.size()>=2)
      {
        Gap last_gap = gaps[gaps.size()-1];
        if(last_gap.end.r == max_r) // if last gap ends with infinity, the first gap must have started where this one starts
        {
          gaps[0].start = last_gap.start;
          gaps.pop_back();
        }
        
        
        for(int start_ind = 0, num_gaps=gaps.size(); start_ind < num_gaps; start_ind++)
        {
          int prev_ind = (start_ind + num_points-1) % num_gaps;
          
          Gap cur_gap = gaps[start_ind];
          Gap prev_gap = gaps[prev_ind];
          
          if(prev_gap.end.r == max_r && cur_gap.start.r == max_r)
          {
            Gap gap(prev_gap.start, cur_gap.end);
            collapsed_gaps.push_back(gap);
            start_ind++;
            ROS_INFO_STREAM("Collapsed " << toString(prev_gap) << " and " << toString(cur_gap) << " into " << toString(gap));
          }
          else
          {
            collapsed_gaps.push_back(cur_gap);
            ROS_INFO_STREAM("Added " << toString(cur_gap) << " as is");
          }
        }
      }
      else
      {
        collapsed_gaps = gaps;
      }
      
      return collapsed_gaps;
    }
    */

  } //end namespace gap_finding
} //end namespace egocircle_utils

#endif //EGOCIRCLE_GAPS_H
