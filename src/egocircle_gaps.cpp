#include <teb_local_planner/egocircle_gaps.h>



namespace egocircle_utils
{
  namespace gap_finding
  {
    std::string toString(const Gap& gap)
    {
      std::stringstream ss;
      ss << "(" <<
      gap.start.r << "m @ " << gap.start.theta <<
      ") : (" << 
      gap.end.r << "m @ " << gap.end.theta <<
      ")";
      
      return ss.str();
    }
    
    
    void addGapsToMarker(visualization_msgs::Marker& marker, const std_msgs::ColorRGBA& color, const std::vector<Gap>& gaps)
    {
      for( Gap gap : gaps)
      {
        geometry_msgs::Point gap_r;
        gap_r.x = gap.start.r*std::cos(gap.start.theta);
        gap_r.y = gap.start.r*std::sin(gap.start.theta);
        
        geometry_msgs::Point gap_l;
        gap_l.x = gap.end.r*std::cos(gap.end.theta);
        gap_l.y = gap.end.r*std::sin(gap.end.theta);
        
        marker.points.push_back(gap_r);
        marker.points.push_back(gap_l);
        
        marker.colors.push_back(color);
        marker.colors.push_back(color);
      }
    }
    
    
    visualization_msgs::MarkerArray getMarkers(const std::vector<Gap>& raw_gaps, const std::vector<Gap>& collapsed_gaps, std_msgs::Header header)
    {
      visualization_msgs::Marker raw_gap_marker;
      raw_gap_marker.type = visualization_msgs::Marker::LINE_LIST;
      raw_gap_marker.header = header;
      raw_gap_marker.ns = "raw_gaps";
      raw_gap_marker.id = 0;
      raw_gap_marker.action = visualization_msgs::Marker::ADD;
      raw_gap_marker.scale.x = .03;
      
      visualization_msgs::Marker collapsed_gap_marker = raw_gap_marker;
      
      std_msgs::ColorRGBA raw_gap_color;
      raw_gap_color.a = .5;
      raw_gap_color.b = 1;
      
      addGapsToMarker(raw_gap_marker, raw_gap_color, raw_gaps);
      
      
      collapsed_gap_marker.ns = "collapsed_gaps";
      
      std_msgs::ColorRGBA collapsed_gap_color;
      collapsed_gap_color.a = .5;
      collapsed_gap_color.g = 1;
      
      addGapsToMarker(collapsed_gap_marker, collapsed_gap_color, collapsed_gaps);
      
      visualization_msgs::MarkerArray markers;
      markers.markers.push_back(raw_gap_marker);
      markers.markers.push_back(collapsed_gap_marker);
      
      return markers;
    }

    std::vector<Gap> getDiscontinuityGaps(const egocircle_utils::Inflator& inflator)
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
          Gap gap(prev_p, curr_p);
          gaps.push_back(gap);
          PolarPoint mid = gap.getMid();
          ROS_INFO_STREAM("Adding gap: " << toString(gap));
        }
       
      }
      
      return gaps;
    }
    
    std::vector<Gap> getCollapsedGaps(const std::vector<Gap>& gaps, double max_r)
    {
      
      ROS_INFO_STREAM("Collapsing gaps:");
      std::vector<Gap> collapsed_gaps;
      if(gaps.size()>=2)
      {
        
        for(int start_ind = 0, num_gaps=gaps.size(); start_ind < num_gaps; start_ind++)
        {
          int prev_ind = (start_ind + num_gaps-1) % num_gaps;
          
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
    
  }
  
}
