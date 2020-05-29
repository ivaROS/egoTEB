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
        double num_divs = std::ceil((gap.end.theta - gap.start.theta) / .1);
        
        ROS_DEBUG_STREAM("Gap: " << toString(gap) << " num_divs=" << num_divs);
        
        for(int i=0; i <= num_divs; i++)
        {
          double r= gap.start.r + (gap.end.r-gap.start.r)*i/num_divs;
          double theta = gap.start.theta + (gap.end.theta-gap.start.theta)*i/num_divs;
          
          geometry_msgs::Point p;
          p.x = r*std::cos(theta);
          p.y = r*std::sin(theta);
          p.z = .1;
          
          ROS_DEBUG_STREAM("i: " << i << ", r: " << r << ", theta: " << theta);
          
          if(i > 1)
          {
            marker.points.push_back(marker.points.back());
            marker.colors.push_back(color);
          }
          marker.points.push_back(p);
          marker.colors.push_back(color);
        }
      }
    }
    
    void addGlobalGapsToMarker(visualization_msgs::MarkerArray& markers, const std::vector<GlobalGap>& gaps, std_msgs::Header header)
    {
      visualization_msgs::Marker global_gap_marker;
      global_gap_marker.type = visualization_msgs::Marker::LINE_LIST;
      global_gap_marker.header = header;
      global_gap_marker.ns = "global_gaps";
      global_gap_marker.id = 0;
      global_gap_marker.action = visualization_msgs::Marker::ADD;
      global_gap_marker.scale.x = .03;
      
      std_msgs::ColorRGBA global_gap_color;
      global_gap_color.a = 1;
      global_gap_color.b = 1;
      
      global_gap_color.g = .5;
      
      
      for(const GlobalGap& gap : gaps)
      {        
        
        //ROS_DEBUG_STREAM("Gap: " << toString(gap) << " num_divs=" << num_divs);
        
        int num_divs = gap.size();
        for(int i=0; i < num_divs; i++)
        {
          const ego_circle::EgoCircularPoint& pt = gap[i];
          
          geometry_msgs::Point p;
          p.x = pt.x;
          p.y = pt.y;
          p.z = .1;
          
          //ROS_DEBUG_STREAM("i: " << i << ", r: " << r << ", theta: " << theta);
          
          if(i > 1)
          {
            global_gap_marker.points.push_back(global_gap_marker.points.back());
            global_gap_marker.colors.push_back(global_gap_color);
          }
          global_gap_marker.points.push_back(p);
          global_gap_marker.colors.push_back(global_gap_color);
        }
      }
      
      markers.markers.push_back(global_gap_marker);
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
      raw_gap_color.a = 1;
      raw_gap_color.b = 1;
      
      addGapsToMarker(raw_gap_marker, raw_gap_color, raw_gaps);
      
      
      collapsed_gap_marker.ns = "collapsed_gaps";
      
      std_msgs::ColorRGBA collapsed_gap_color;
      collapsed_gap_color.a = 1;
      collapsed_gap_color.r = 1;
      collapsed_gap_color.g = .7;
      
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
          ROS_DEBUG_STREAM("Adding gap: " << toString(gap));
        }
       
      }
      
      return gaps;
    }
    
    std::vector<Gap> getCollapsedGaps(const std::vector<Gap>& gaps, double max_r)
    {
      constexpr double PI = std::acos(-1);
      
      ROS_DEBUG_STREAM("Collapsing gaps:");
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
            if(gap.start.theta > gap.end.theta)
            {
              gap.end.theta += 2*PI;
            }
            
            collapsed_gaps.push_back(gap);
            ROS_DEBUG_STREAM("Collapsed " << toString(prev_gap) << " and " << toString(cur_gap) << " into " << toString(gap));
          }
          else if(cur_gap.end.r !=  max_r)  // Only add gap if it wouldn't be collapsed with the next
          {
            collapsed_gaps.push_back(cur_gap);
            ROS_DEBUG_STREAM("Added " << toString(cur_gap) << " as is");
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
