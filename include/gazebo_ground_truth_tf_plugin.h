/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gazebo_ground_truth_tf_plugin.h
 * Simple gazebo plugin to publish the exact position of a link as a ros tf msg
 *
 * @author Simone Guscetti <simonegu@student.ethz.ch>
 */

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

protected:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
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
  // gazebo plugin variables
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  void resetCb(const std_msgs::Empty &msg);
};
}

#endif /* __GAZEBO_GROUND_TRUTH_TF_PLUGIN_H__ */
