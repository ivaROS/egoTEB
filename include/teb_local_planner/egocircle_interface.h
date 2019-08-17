#ifndef TEB_LOCAL_PLANNER_EGOCIRCLE_INTERFACE_H
#define TEB_LOCAL_PLANNER_EGOCIRCLE_INTERFACE_H

#include <egocircle_utils/updateable_interface.h>
#include <egocircle/ego_circle.h>
#include <egocircle_utils/container.h>
#include <egocircle_utils/inflator.h>
#include <egocircle_utils/min_dist.h>
#include <egocircle_utils/transformer.h>
#include <egocircle_utils/decimator.h>

#include <teb_local_planner/egocircle_gaps.h> //TODO: Move this to egocircle_utils once it has stabilized
#include <std_msgs/Header.h>

namespace teb_local_planner
{
  using GlobalGap = std::vector<ego_circle::EgoCircularPoint>;
  class EgoCircleInterface: public egocircle_utils::UpdateableInterface
  {
  public:
    
    EgoCircleInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME);
    
    void setSearchRadius(double radius);
    
    void setInflationRadius(double radius);
    
    void update(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    
    CCResult testCollisionImpl(CollisionChecker::PoseType pose, CCOptions options);

    //TODO: rewrite functions to accept either EgoCircularPoint or PolarPoint (templated) to reduce repeated calculations
    float getEgoCircleRange(ego_circle::EgoCircularPoint point) const;
    
    float getInflatedEgoCircleRange(ego_circle::EgoCircularPoint point) const;
    
    //TODO: only use 'EgoCircularPoint' type for points in ego frame, use different type for points in global frame.
    ego_circle::EgoCircularPoint getLocalEgoCircularPoint(geometry_msgs::Pose pose) const;
    
    ego_circle::EgoCircularPoint getLocalEgoCircularPoint(float x, float y) const;
    
    float getMinDist(ego_circle::EgoCircularPoint point) const;
    
    const std::vector<egocircle_utils::gap_finding::Gap>& getDiscontinuityGaps() const;
    
    const std::vector<GlobalGap>& getGlobalGaps() const;
    
    const std::vector<ego_circle::EgoCircularPoint>& getDecimatedEgoCircularPoints() const;
    
    std_msgs::Header getCurrentHeader() const;
    
    static constexpr const char* DEFAULT_NAME="ego_circle_cost_impl";
    
  private:
    float inflation_radius_, search_radius_;
    
    std::shared_ptr<egocircle_utils::Container> container_;    
    std::shared_ptr<egocircle_utils::Inflator> inflator_;
    std::shared_ptr<egocircle_utils::MinDistanceCalculator> min_dist_;
    std::shared_ptr<egocircle_utils::Decimator> decimator_;
//     egocircle_utils::Transformer transformer_;
    std::vector<egocircle_utils::gap_finding::Gap> gaps_;
    std::vector<GlobalGap> global_gaps_;
    
    std::string name_;    
    
    ros::Publisher inflated_egocircle_pub_, decimated_egocircle_pub_, gap_pub_;
    
    ros::NodeHandle nh_, pnh_;	// For now, separate node handles for base and derived
    
  };
} //namespace


#endif //TEB_LOCAL_PLANNER_EGOCIRCLE_INTERFACE_H
