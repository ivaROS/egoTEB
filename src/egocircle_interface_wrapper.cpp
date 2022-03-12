#include <teb_local_planner/egocircle_interface_wrapper.h>

namespace teb_local_planner
{

  // EgoCircleInterfaceWrapper::EgoCircleInterfaceWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name) :
  EgoCircleInterfaceWrapper::EgoCircleInterfaceWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const std::string& name) :
    PipsCCWrapper(nh,pnh,name,tf_buffer),
    InterfaceUpdater(nh, pnh, tf_buffer, name)
  {
    cc_ = std::make_shared<EgoCircleInterface>(nh, pnh);
    InterfaceUpdater::setInterface(cc_);
  }

  // EgoCircleInterfaceWrapper::EgoCircleInterfaceWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const tf2_utils::TransformManager& tfm) :
  EgoCircleInterfaceWrapper::EgoCircleInterfaceWrapper(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer) :
    PipsCCWrapper(nh,pnh,name,tf_buffer),
    InterfaceUpdater(nh, pnh, tf_buffer, name)
  {
    cc_ = std::make_shared<EgoCircleInterface>(nh, pnh);
    InterfaceUpdater::setInterface(cc_);
  }
  
  bool EgoCircleInterfaceWrapper::init()
  {
    InterfaceUpdater::init();
    PipsCCWrapper::init();
  }
  
  void EgoCircleInterfaceWrapper::scanCb(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
  {
    InterfaceUpdater::scanCb(scan_msg);
    doCallback();
  }

} //end namespace teb_local_planner
