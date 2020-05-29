
#include <teb_local_planner/egocircle_interface.h>
#include <pips/collision_testing/transforming_collision_checker.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace teb_local_planner
{

    EgoCircleInterface::EgoCircleInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name):
      pips::collision_testing::TransformingCollisionChecker(nh,pnh,name)
    {
      init();
    }
    
    bool EgoCircleInterface::init()
    {
      inflated_egocircle_pub_ = pnh_.advertise<sensor_msgs::LaserScan>("inflated_egocircle", 5, true);
      decimated_egocircle_pub_ = pnh_.advertise<visualization_msgs::Marker>("decimated_egocircle", 5, true);
      gap_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("gaps", 5, true);
      
      return true;
    }
    
    void EgoCircleInterface::setSearchRadius(double radius)
    {
      search_radius_ = radius;
    }
    
    void EgoCircleInterface::setInflationRadius(double radius)
    {
      inflation_radius_ = radius;
    }
    
    void EgoCircleInterface::update(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
      ros::WallTime starttime = ros::WallTime::now();
      container_ = std::make_shared<egocircle_utils::Container>(scan_msg);
      inflator_ = std::make_shared<egocircle_utils::Inflator>(*container_, inflation_radius_);
      min_dist_ = std::make_shared<egocircle_utils::MinDistanceCalculator>(*container_, search_radius_);
      decimator_ = std::make_shared<egocircle_utils::Decimator>(*container_, inflation_radius_);
      
      auto raw_gaps = egocircle_utils::gap_finding::getDiscontinuityGaps(*inflator_);
      
      gaps_ = egocircle_utils::gap_finding::getCollapsedGaps(raw_gaps, container_->egocircle_radius);
      
      std_msgs::Header local_header=scan_msg->header;
      //local_header.frame_id = getLocalFrameId();
      //local_header.stamp = getStamp();
      visualization_msgs::MarkerArray markers = egocircle_utils::gap_finding::getMarkers(raw_gaps, gaps_, local_header);
      
      
      global_gaps_.clear();
      int num_gaps = gaps_.size();
      ROS_INFO_STREAM("[EgoCircleInterface::update] There are " << num_gaps << " gaps");
      
      for(int i = 0; i < num_gaps; ++i)
      {
        std::vector<ego_circle::EgoCircularPoint> gap_vec;
        
        egocircle_utils::gap_finding::Gap& gap = gaps_[i];
        
        double MAX_GAP_RAD = std::acos(-1)/2;
        
        int num_segments = std::ceil((gap.end.theta-gap.start.theta) / MAX_GAP_RAD);
        
        {
          int start_ind = container_->indexer.getIndex(gap.start);
          int actual_start_ind = inflator_->getInds()[start_ind];
          auto actual_start_point = container_->points[actual_start_ind];
          
          ego_circle::EgoCircularPoint global_point(actual_start_point);
          toGlobal(global_point);
          gap_vec.push_back(global_point);
        }
        
        for(int segment = 0; segment < num_segments + 1; segment++)
        {
          double interp_r = gap.start.r + (gap.end.r - gap.start.r)*segment/num_segments;
          double interp_th = gap.start.theta + (gap.end.theta - gap.start.theta)*segment/num_segments;
          
          ego_circle::PolarPoint p(interp_r, interp_th);
          ego_circle::EgoCircularPoint global_point(p);
          toGlobal(global_point);
          gap_vec.push_back(global_point);
        }
        
        {
          int end_ind = container_->indexer.getIndex(gap.end);
          int actual_end_ind = inflator_->getInds()[end_ind];
          auto actual_end_point = container_->points[actual_end_ind];
          
          ego_circle::EgoCircularPoint global_point(actual_end_point);
          toGlobal(global_point);
          gap_vec.push_back(global_point);
        }
        
        global_gaps_.push_back(gap_vec);
      }
      ros::WallTime endtime = ros::WallTime::now();
      ROS_INFO_STREAM("egocircle_interface, time, " << (endtime - starttime).toSec() * 1e3 << "ms");
      if(gap_pub_.getNumSubscribers()>0)
      {
        std_msgs::Header global_header;
        global_header.stamp = scan_msg->header.stamp; //getStamp();
        global_header.frame_id = getGlobalFrameId();
        egocircle_utils::gap_finding::addGlobalGapsToMarker(markers, global_gaps_, global_header);
        {
          visualization_msgs::Marker global_egocircle_marker;
          global_egocircle_marker.type = visualization_msgs::Marker::POINTS;
          global_egocircle_marker.header = global_header;
          global_egocircle_marker.header.stamp = ros::Time::now();
          global_egocircle_marker.ns = "transformed_points";
          global_egocircle_marker.id = 0;
          global_egocircle_marker.action = visualization_msgs::Marker::ADD;
          global_egocircle_marker.scale.x = .03;
          
          std_msgs::ColorRGBA global_egocircle_color;
          global_egocircle_color.a = .5;
          global_egocircle_color.b = 1;
          
          global_egocircle_color.r = .5;
          
          global_egocircle_marker.color = global_egocircle_color;
          
          //for(auto it : ego_circle::LaserScanWrapper(*scan_msg))
          for(auto d : container_->points)
          {
            //auto d = ego_circle::PolarPoint(it);
            ego_circle::EgoCircularPoint pt = d;
            toGlobal(pt);
      
            
            geometry_msgs::Point p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = .1;
            
            global_egocircle_marker.points.push_back(p);
          }
          
          markers.markers.push_back(global_egocircle_marker);
        }

        gap_pub_.publish(markers);
      }
      
      
      if(inflated_egocircle_pub_.getNumSubscribers()>0)
      {
        sensor_msgs::LaserScanPtr inflated_scan = inflator_->getMsg();
        inflated_egocircle_pub_.publish(inflated_scan);
      }
      
      if(decimated_egocircle_pub_.getNumSubscribers()>0)
      {
        visualization_msgs::Marker::Ptr decimated_msg = decimator_->getMsg();
        decimated_egocircle_pub_.publish(decimated_msg);
      }
    }
    
    
    CCResult EgoCircleInterface::testCollisionImpl(CollisionChecker::PoseType pose, CCOptions options)
    {
      return getMinDist(ego_circle::EgoCircularPoint(pose.position.x,pose.position.y)) <=0;
    }
    
    void EgoCircleInterface::setTransform(const geometry_msgs::TransformStamped& base_optical_transform)
    {
      egocircle_utils::Transformer::setTransform(base_optical_transform);
    }

    
    float EgoCircleInterface::getEgoCircleRange(ego_circle::EgoCircularPoint point) const
    {
      return container_->getRange(point);
    }

    float EgoCircleInterface::getEgoCircleRange(ego_circle::PolarPoint point) const
    {
      return container_->getRange(point);
    }
    
    float EgoCircleInterface::getInflatedEgoCircleRange(ego_circle::EgoCircularPoint point) const
    {
      return inflator_->getRange(point);
    }
    
    float EgoCircleInterface::getInflatedEgoCircleRange(ego_circle::PolarPoint point) const
    {
      return inflator_->getRange(point);
    }
    
    ego_circle::EgoCircularPoint EgoCircleInterface::getLocalEgoCircularPoint(geometry_msgs::Pose pose) const
    {
      return getLocalEgoCircularPoint(pose.position.x, pose.position.y);
    }
    
    ego_circle::EgoCircularPoint EgoCircleInterface::getLocalEgoCircularPoint(float x, float y) const
    {
      ego_circle::EgoCircularPoint point(x,y);
      toLocal(point);
      return point;
    }
    
    float EgoCircleInterface::getMinDist(ego_circle::EgoCircularPoint point) const
    {
      ego_circle::EgoCircularPoint transformed_point = point;
      toLocal(transformed_point);
      
      return min_dist_->getMinDist(transformed_point);
    }
    
    const std::vector<egocircle_utils::gap_finding::Gap>& EgoCircleInterface::getDiscontinuityGaps() const
    {
      return gaps_;
    }
    
    const std::vector<std::vector<ego_circle::EgoCircularPoint> >& EgoCircleInterface::getGlobalGaps() const
    {
      return global_gaps_;
    }
    
    const std::vector<ego_circle::EgoCircularPoint>& EgoCircleInterface::getDecimatedEgoCircularPoints() const
    {
      return decimator_->getPoints();
    }
    
    const egocircle_utils::Inflator* EgoCircleInterface::getInflator() const
    {
      return inflator_.get();
    }
    
    
    std_msgs::Header EgoCircleInterface::getCurrentHeader() const
    {
      if(container_)
      {
        return container_->scan->header;
      }
      else
      {
        std_msgs::Header empty;
        return empty;
      }
    }
    

}


