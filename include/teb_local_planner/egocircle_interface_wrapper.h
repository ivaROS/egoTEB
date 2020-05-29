#ifndef TEB_LOCAL_PLANNER_EGOCIRCLE_INTERFACE_WRAPPER_H
#define TEB_LOCAL_PLANNER_EGOCIRCLE_INTERFACE_WRAPPER_H


#include <teb_local_planner/egocircle_interface.h>
#include <egocircle_utils/interface_updater.h>
#include <pips_trajectory_testing/pips_cc_wrapper.h>

namespace teb_local_planner
{

  class EgoCircleInterfaceWrapper: public pips_trajectory_testing::PipsCCWrapper, private egocircle_utils::InterfaceUpdater
  {
  public:
    using pips_trajectory_testing::PipsCCWrapper::isReady;
    
    EgoCircleInterfaceWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME, const tf2_utils::TransformManager& tfm=tf2_utils::TransformManager(false));
    
    EgoCircleInterfaceWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name=DEFAULT_NAME);
    
    std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> getCC()
    {
      return cc_;
    }
    
    std::shared_ptr<EgoCircleInterface> getImpl()
    {
      return cc_;
    }
    
    bool isReadyImpl()
    {
      return InterfaceUpdater::isReady();
    }
    
    bool init();
    
    void update()
    {
      InterfaceUpdater::update();
    }
    
    std_msgs::Header getCurrentHeader()
    {
      return InterfaceUpdater::getCurrentHeader();
    }
    
  protected:
    virtual void scanCb(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    
  private:
    static constexpr const char* DEFAULT_NAME="egocircle_interface_wrapper";
    std::shared_ptr<EgoCircleInterface> cc_;
    
    
    using PipsCCWrapper=pips_trajectory_testing::PipsCCWrapper;
    using InterfaceUpdater=egocircle_utils::InterfaceUpdater;
  };

} //end namespace teb_local_planner

#endif //TEB_LOCAL_PLANNER_EGOCIRCLE_INTERFACE_WRAPPER_H
