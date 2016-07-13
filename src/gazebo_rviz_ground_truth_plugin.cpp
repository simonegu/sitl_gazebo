#include <string>
#include <vector>
#include <iostream>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>

#include <boost/bind.hpp>

namespace gazebo
{
  class RvizGroundTruth : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      # if GAZEBO_MAJOR_VERSION >= 7
          model_name = this->model->Name();
      # else
          model_name = this->model->GetName();
      # endif

      //get sdf params
      if (_sdf->HasElement("calc_path")) {
        calc_path = _sdf->GetElement("calc_path")->Get<bool>();
      } else {
        gzwarn << "[gazebo_rviz_ground_truth_plugin] defaulting to calc_path = false.\n";
        calc_path = false;
      }

      std::string link_name_;
      if (_sdf->HasElement("link_name"))
      {
        link_name_ = _sdf->GetElement("link_name")->GetValue()->GetAsString();
        link = this->model->GetLink(link_name_);
      }
      else
      {
        link = this->model->GetLink();
        link_name_ = link->GetName();
      }

      //set up ros
      ros::NodeHandle nh("~");
      marker_pub = nh.advertise<visualization_msgs::Marker>(model_name + "/pose", 1);
      path_pub = nh.advertise<nav_msgs::Path>(model_name + "/path", 1);
      //clear all poses in path
      path.poses.clear();
      sub_reset_pos = nh.subscribe("/omni_vio_vis/reset", 1, &RvizGroundTruth::resetCb, this);

      //init params
      x_offset = 0.0;
      y_offset = 0.0;
      z_offset = 0.0;
      q_offset = tf::Quaternion(0, 0, 0, 1);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&RvizGroundTruth::OnUpdate, this, _1));

      std::cout << "[gazebo_rviz_ground_truth_plugin] loaded for link " << link_name_ << std::endl;
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      double x,y,z,qw,qx,qy,qz;
      gazebo::math::Pose pose;
      // pose = this->model->GetWorldPose();
      pose = this->link->GetWorldPose();
      x = pose.pos.x - x_offset; // x coordinate
      y = pose.pos.y - y_offset; // y coordinate
      z = pose.pos.z - z_offset; // z coordinate
      qw = pose.rot.w;
      qx = pose.rot.x;
      qy = pose.rot.y;
      qz = pose.rot.z;

      double scale = 0.5;
      double shaft_diameter = 0.05;
      double head_diameter = 0.1;
      double head_length = 0.2;
      double color_alpha = 0.7;
      double color_intensity = 1.0;

      tf::Quaternion q(qx, qy, qz, -qw);
      tf::Matrix3x3 m(q);
      tf::Matrix3x3 m_offset(q_offset);
      m *= m_offset.transpose();
      tf::Vector3 m_off_row0 = m_offset.getRow(0);
      tf::Vector3 m_off_row1 = m_offset.getRow(1);
      tf::Vector3 m_off_row2 = m_offset.getRow(2);

      double x_new = x*m_off_row0[0] + y*m_off_row0[1] + z*m_off_row0[2];
      double y_new = x*m_off_row1[0] + y*m_off_row1[1] + z*m_off_row1[2];
      double z_new = x*m_off_row2[0] + y*m_off_row2[1] + z*m_off_row2[2];
      tf::Vector3 x_axis = m.getRow(1); //orientation of omni cam
      tf::Vector3 y_axis = -m.getRow(2);
      tf::Vector3 z_axis = -m.getRow(0);

      visualization_msgs::Marker x_marker;
      visualization_msgs::Marker y_marker;
      visualization_msgs::Marker z_marker;

      // X direction arrow
      x_marker.header.frame_id = "world";
      x_marker.header.stamp = ros::Time::now();
      x_marker.type = visualization_msgs::Marker::ARROW;
      x_marker.action = visualization_msgs::Marker::ADD;
      x_marker.ns = "x_axis";
      x_marker.scale.x = shaft_diameter * scale;
      x_marker.scale.y = head_diameter * scale;
      x_marker.scale.z = head_length * scale;
      x_marker.color.r = color_intensity;
      x_marker.color.a = color_alpha;
      x_marker.points.resize(2);
      x_marker.points[0].x = x_new; //start
      x_marker.points[0].y = y_new;
      x_marker.points[0].z = z_new;
      x_marker.points[1].x = x_new + x_axis[0] * scale; //end
      x_marker.points[1].y = y_new + x_axis[1] * scale;
      x_marker.points[1].z = z_new + x_axis[2] * scale;

      // Y direction arrow
      y_marker.header.frame_id = "world";
      y_marker.header.stamp = ros::Time::now();
      y_marker.type = visualization_msgs::Marker::ARROW;
      y_marker.action = visualization_msgs::Marker::ADD;
      y_marker.ns = "y_axis";
      y_marker.scale.x = shaft_diameter * scale;
      y_marker.scale.y = head_diameter * scale;
      y_marker.scale.z = head_length * scale;
      y_marker.color.g = color_intensity;
      y_marker.color.a = color_alpha;
      y_marker.points.resize(2);
      y_marker.points[0].x = x_new; //start
      y_marker.points[0].y = y_new;
      y_marker.points[0].z = z_new;
      y_marker.points[1].x = x_new + y_axis[0] * scale; //end
      y_marker.points[1].y = y_new + y_axis[1] * scale;
      y_marker.points[1].z = z_new + y_axis[2] * scale;

      // Z direction arrow
      z_marker.header.frame_id = "world";
      z_marker.header.stamp = ros::Time::now();
      z_marker.type = visualization_msgs::Marker::ARROW;
      z_marker.action = visualization_msgs::Marker::ADD;
      z_marker.ns = "z_axis";
      z_marker.scale.x = shaft_diameter * scale;
      z_marker.scale.y = head_diameter * scale;
      z_marker.scale.z = head_length * scale;
      z_marker.color.b = color_intensity;
      z_marker.color.a = color_alpha;
      z_marker.points.resize(2);
      z_marker.points[0].x = x_new; //start
      z_marker.points[0].y = y_new;
      z_marker.points[0].z = z_new;
      z_marker.points[1].x = x_new + z_axis[0] * scale; //end
      z_marker.points[1].y = y_new + z_axis[1] * scale;
      z_marker.points[1].z = z_new + z_axis[2] * scale;

      //path
      if (calc_path) {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = "world";
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.position.z = z;
        pose_msg.pose.orientation.x = qx;
        pose_msg.pose.orientation.y = qy;
        pose_msg.pose.orientation.z = qz;
        pose_msg.pose.orientation.w = qw;
        path.header = pose_msg.header;
        path.poses.push_back(pose_msg);

        path_pub.publish(path);
        int size = path.poses.size();

        if (path.poses.size() > 100000) {
          path.poses.clear();
        }
      }

      // publish the markers
      marker_pub.publish(x_marker);
      marker_pub.publish(y_marker);
      marker_pub.publish(z_marker);
    }

    private: void resetCb(const std_msgs::Empty &msg)
    {
      gazebo::math::Pose pose_offset;
      pose_offset = this->link->GetWorldPose();
      x_offset = pose_offset.pos.x; // x coordinate
      y_offset = pose_offset.pos.y; // y coordinate
      z_offset = pose_offset.pos.z; // z coordinate
      q_offset = tf::Quaternion(pose_offset.rot.x, pose_offset.rot.y,
                              pose_offset.rot.z, -pose_offset.rot.w);
      tf::Quaternion q_vio_offset(0, 0, 1, 0); //180 degrees about z-axis
      q_offset *= q_vio_offset;
      path.poses.clear();
    }

    private:
      physics::ModelPtr model;
      physics::LinkPtr link;
      event::ConnectionPtr updateConnection;
      std::string model_name;
      nav_msgs::Path path;
      ros::Publisher marker_pub;
      ros::Publisher path_pub;
      ros::Subscriber sub_reset_pos;
      bool calc_path;
      double x_offset, y_offset, z_offset;
      tf::Quaternion q_offset;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RvizGroundTruth)
}
