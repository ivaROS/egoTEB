
#include <teb_local_planner/egocircle_interface.h>
#include <pips/collision_testing/transforming_collision_checker.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace teb_local_planner
{

    EgoCircleInterface::EgoCircleInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name) :
      egocircle_utils::UpdateableInterface(nh,pnh,name),
      name_(name),
      nh_(nh, name_),
      pnh_(pnh, name_)
    {
      inflated_egocircle_pub_ = pnh_.advertise<sensor_msgs::LaserScan>("inflated_egocircle", 5, true);
      decimated_egocircle_pub_ = pnh_.advertise<visualization_msgs::Marker>("decimated_egocircle", 5, true);
      
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
      container_ = std::make_shared<egocircle_utils::Container>(scan_msg);
      inflator_ = std::make_shared<egocircle_utils::Inflator>(*container_, inflation_radius_);
      min_dist_ = std::make_shared<egocircle_utils::MinDistanceCalculator>(*container_, search_radius_);
      decimator_ = std::make_shared<egocircle_utils::Decimator>(*container_, inflation_radius_);
      gaps_ = egocircle_utils::gap_finding::getDiscontinuityGaps(*inflator_);
      
      global_gaps_.clear();
      int num_gaps = gaps_.size();
      
      for(int i = 0; i < num_gaps; ++i)
      {
        std::vector<ego_circle::EgoCircularPoint> gap_vec;
        
        egocircle_utils::gap_finding::Gap& gap = gaps_[i];
        ego_circle::EgoCircularPoint point = gap.start;
        toGlobal(point);
        gap_vec.push_back(point);
        
        point = gap.end;
        toGlobal(point);
        gap_vec.push_back(point);
        
        global_gaps_.push_back(gap_vec);
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

    //TODO: rewrite functions to accept either EgoCircularPoint or PolarPoint (templated) to reduce repeated calculations
    float EgoCircleInterface::getEgoCircleRange(ego_circle::EgoCircularPoint point) const
    {
      return container_->getRange(point);
    }
    
    float EgoCircleInterface::getInflatedEgoCircleRange(ego_circle::EgoCircularPoint point) const
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
    
//     std::vector<EgoCircularPoint> EgoCircleCostImpl::getLocalEgoCircularPoints() const
//     {
//       float angle_increment = scan_->angle_increment;
//       float boundary_radius = getEgoCircleRadius();
//       
//       int num_points = scan_->ranges.size();
//       std::vector<EgoCircularPoint> points;
//       
//       float current_angle = scan_->angle_min;
//       for(int i = 0; i < num_points; ++i)
//       {
//         PolarPoint polar_point(scan_->ranges[i],current_angle);
//         if(polar_point.r < boundary_radius)
//         {          
//           points.push_back(polar_point);
//         }
//         current_angle += angle_increment;
//       }
//       
//       return points;
//     }
    
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

}


