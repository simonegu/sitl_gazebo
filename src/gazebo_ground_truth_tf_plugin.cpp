#include "gazebo_ground_truth_tf_plugin.h"

namespace gazebo {

GazeboGroundTruthTf::~GazeboGroundTruthTf() {
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
};

void GazeboGroundTruthTf::Load(physics::ModelPtr _parent,
                               sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _parent;
// get model name
#if GAZEBO_MAJOR_VERSION >= 7
  model_name_ = model_->Name();
#else
  model_name_ = model_->GetName();
#endif

  // get sdf params
  if (_sdf->HasElement("calc_path")) {
    calc_path_ = _sdf->GetElement("calc_path")->Get<bool>();
  } else {
    gzwarn << "[gazebo_ground_truth_tf_plugin] defaulting to calc_path = "
              "false.\n";
    calc_path_ = false;
  }

  if (_sdf->HasElement("link_name")) {
    link_name_ = _sdf->GetElement("link_name")->GetValue()->GetAsString();
    link_ = model_->GetLink(link_name_);
  } else {
    link_ = model_->GetLink();
    link_name_ = link_->GetName();
  }

  path_pub_ = nh_.advertise<nav_msgs::Path>(model_name_ + "/path", 1);
  // clear all poses in path
  path_.poses.clear();
  sub_reset_pos_ = nh_.subscribe(model_name_ + "/reset", 1,
                                 &GazeboGroundTruthTf::resetCb, this);
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboGroundTruthTf::OnUpdate, this, _1));

  std::cout << "[gazebo_ground_truth_tf_plugin] loaded for link " << link_name_
            << std::endl;
}

// Called by the world update start event
void GazeboGroundTruthTf::OnUpdate(const common::UpdateInfo & /*_info*/) {
  // Initialize tf broadcaster
  static tf::TransformBroadcaster br_;
  tf::Transform transform_;
  double x, y, z, qw, qx, qy, qz;
  gazebo::math::Pose pose;
  pose = this->link_->GetWorldPose();
  x = pose.pos.x; // x coordinate
  y = pose.pos.y; // y coordinate
  z = pose.pos.z; // z coordinate
  qw = pose.rot.w;
  qx = pose.rot.x;
  qy = pose.rot.y;
  qz = pose.rot.z;
  // create ros::tf Quaternion
  // x and y are inverted because gazebo reference frame is rotated by 90 deg
  tf::Quaternion q1(-qy, qx, qz, qw);
  // factor needed to transform it to ROS ENU
  tf::Quaternion factor;
  factor.setRPY(0, 0, M_PI / 2);
  // rotate it to 90 deg in the z axis
  tf::Quaternion q = q1 * factor;
  // need to do like this because gazebo reference frame is 90 deg rotated
  transform_.setOrigin(tf::Vector3(-y, x, z));
  transform_.setRotation(q);
  // broadcast transformation
  br_.sendTransform(
      tf::StampedTransform(transform_, ros::Time::now(), "world", link_name_));

  // path
  if (calc_path_) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = -y;
    pose_msg.pose.position.y = x;
    pose_msg.pose.position.z = z;
    pose_msg.pose.orientation.x = -qy;
    pose_msg.pose.orientation.y = qx;
    pose_msg.pose.orientation.z = qz;
    pose_msg.pose.orientation.w = qw;
    path_.header = pose_msg.header;
    path_.poses.push_back(pose_msg);

    path_pub_.publish(path_);

    if (path_.poses.size() > 100000) {
      path_.poses.clear();
    }
  }
}

void GazeboGroundTruthTf::resetCb(const std_msgs::Empty &msg) {
  // clear path
  path_.poses.clear();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboGroundTruthTf)
}
