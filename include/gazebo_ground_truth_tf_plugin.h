#ifndef __GAZEBO_GROUND_TRUTH_TF_PLUGIN_H__
#define __GAZEBO_GROUND_TRUTH_TF_PLUGIN_H__

// gazebo plugin includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// ros includes
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>

#include <boost/bind.hpp>

#include <string>
#include <vector>
#include <iostream>

namespace gazebo {
class GazeboGroundTruthTf : public ModelPlugin {
public:
  GazeboGroundTruthTf() : ModelPlugin(), nh_("~") {};
  virtual ~GazeboGroundTruthTf();

public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  // Called by the world update start event
  void OnUpdate(const common::UpdateInfo & /*_info*/);

private:
  // set up ros
  ros::NodeHandle nh_;
  ros::Publisher path_pub_;
  ros::Subscriber sub_reset_pos_;
  // save path
  nav_msgs::Path path_;
  // is path saved
  bool calc_path_;
  // model and link name
  std::string model_name_;
  std::string link_name_;
  // Gazebo plugin need variables
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  void resetCb(const std_msgs::Empty &msg);
};
}

#endif /* end of include guard: __GAZEBO_GROUND_TRUTH_TF_PLUGIN_H__ */
