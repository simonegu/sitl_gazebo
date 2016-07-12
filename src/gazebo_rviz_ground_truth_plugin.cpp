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
#include <nav_msgs/Path.h>

#include <boost/bind.hpp>

namespace gazebo
{
  class RvizGroundTruth : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      # if GAZEBO_MAJOR_VERSION >= 7
          model_name = this->model->Name();
      # else
          model_name = this->model->GetName();
      # endif

      //set up ros
      ros::NodeHandle nh("~");
      marker_pub = nh.advertise<visualization_msgs::Marker>(model_name + "/pose", 1);
      path_pub = nh.advertise<nav_msgs::Path>(model_name + "/path", 1);
      //clear all poses in path
      path.poses.clear();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&RvizGroundTruth::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      double x,y,z,qw,qx,qy,qz;
      gazebo::math::Pose pose;
      pose = this->model->GetWorldPose();
      x = pose.pos.x; // x coordinate
      y = pose.pos.y; // y coordinate
      z = pose.pos.z; // z coordinate
      qw = pose.rot.w;
      qx = pose.rot.x;
      qy = pose.rot.y;
      qz = pose.rot.z;

      double scale = 1.0;
      double shaft_diameter = 0.05;
      double head_diameter = 0.1;
      double head_length = 0.2;

      tf::Quaternion q(qx, qy, qz, qw);
      tf::Matrix3x3 m(q);
      tf::Vector3 x_axis = m.getRow(0);
      tf::Vector3 y_axis = m.getRow(1);
      tf::Vector3 z_axis = m.getRow(2);

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
      x_marker.color.r = 1.0;
      x_marker.color.a = 1.0;
      x_marker.points.resize(2);
      x_marker.points[0].x = x; //start
      x_marker.points[0].y = y;
      x_marker.points[0].z = z;
      x_marker.points[1].x = x + x_axis[0]; //end
      x_marker.points[1].y = y + x_axis[1];
      x_marker.points[1].z = z + x_axis[2];

      // Y direction arrow
      y_marker.header.frame_id = "world";
      y_marker.header.stamp = ros::Time::now();
      y_marker.type = visualization_msgs::Marker::ARROW;
      y_marker.action = visualization_msgs::Marker::ADD;
      y_marker.ns = "y_axis";
      y_marker.scale.x = shaft_diameter * scale;
      y_marker.scale.y = head_diameter * scale;
      y_marker.scale.z = head_length * scale;
      y_marker.color.g = 1.0;
      y_marker.color.a = 1.0;
      y_marker.points.resize(2);
      y_marker.points[0].x = x; //start
      y_marker.points[0].y = y;
      y_marker.points[0].z = z;
      y_marker.points[1].x = x + y_axis[0]; //end
      y_marker.points[1].y = y + y_axis[1];
      y_marker.points[1].z = z + y_axis[2];

      // Z direction arrow
      z_marker.header.frame_id = "world";
      z_marker.header.stamp = ros::Time::now();
      z_marker.type = visualization_msgs::Marker::ARROW;
      z_marker.action = visualization_msgs::Marker::ADD;
      z_marker.ns = "z_axis";
      z_marker.scale.x = shaft_diameter * scale;
      z_marker.scale.y = head_diameter * scale;
      z_marker.scale.z = head_length * scale;
      z_marker.color.b = 1.0;
      z_marker.color.a = 1.0;
      z_marker.points.resize(2);
      z_marker.points[0].x = x; //start
      z_marker.points[0].y = y;
      z_marker.points[0].z = z;
      z_marker.points[1].x = x + z_axis[0]; //end
      z_marker.points[1].y = y + z_axis[1];
      z_marker.points[1].z = z + z_axis[2];

      //path
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

      // publish the markers
      marker_pub.publish(x_marker);
      marker_pub.publish(y_marker);
      marker_pub.publish(z_marker);
    }

    private:
      physics::ModelPtr model;
      event::ConnectionPtr updateConnection;
      std::string model_name;
      nav_msgs::Path path;
      ros::Publisher marker_pub;
      ros::Publisher path_pub;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RvizGroundTruth)
}
