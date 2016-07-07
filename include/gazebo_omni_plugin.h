/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _GAZEBO_OMNI_CAMERA_PLUGIN_HH_
#define _GAZEBO_OMNI_CAMERA_PLUGIN_HH_

#include <string>
#include <vector>
#include <iostream>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/MultiCameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/gazebo.hh>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>

namespace gazebo
{
  class OmniCameraPlugin : public SensorPlugin
  {
    public:
      OmniCameraPlugin();
      // \brief Destructor
      virtual ~OmniCameraPlugin();
      virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
      virtual void OnNewFrameCam0(const unsigned char *_image,
                                unsigned int _width, unsigned int _height,
                                unsigned int _depth, const std::string &_format);
      virtual void OnNewFrameCam1(const unsigned char *_image,
                                unsigned int _width, unsigned int _height,
                                unsigned int _depth, const std::string &_format);
      virtual void OnNewFrameCam2(const unsigned char *_image,
                                unsigned int _width, unsigned int _height,
                                unsigned int _depth, const std::string &_format);
      virtual void OnNewFrameCam3(const unsigned char *_image,
                                unsigned int _width, unsigned int _height,
                                unsigned int _depth, const std::string &_format);
      virtual void OnNewFrameCam4(const unsigned char *_image,
                                unsigned int _width, unsigned int _height,
                                unsigned int _depth, const std::string &_format);
      virtual void OnNewFrameCam5(const unsigned char *_image,
                                unsigned int _width, unsigned int _height,
                                unsigned int _depth, const std::string &_format);
      virtual void OnNewFrameCam6(const unsigned char *_image,
                                unsigned int _width, unsigned int _height,
                                unsigned int _depth, const std::string &_format);
      virtual void OnNewFrameCam7(const unsigned char *_image,
                                unsigned int _width, unsigned int _height,
                                unsigned int _depth, const std::string &_format);

    protected:
      sensors::MultiCameraSensorPtr parentSensor;
      std::vector<unsigned int> width, height, depth;
      std::vector<std::string> format;
      std::vector<rendering::CameraPtr> camera;

    private:
      std::vector<event::ConnectionPtr> newFrameConnection;
      ros::Publisher image_publisher_0;
      ros::Publisher image_publisher_1;
      ros::Publisher image_publisher_2;
      ros::Publisher image_publisher_3;
      ros::Publisher image_publisher_4;
      ros::Publisher image_publisher_5;
      ros::Publisher image_publisher_6;
      ros::Publisher image_publisher_7;

      sensor_msgs::Image image_0;
      sensor_msgs::Image image_1;
      sensor_msgs::Image image_2;
      sensor_msgs::Image image_3;
      sensor_msgs::Image image_4;
      sensor_msgs::Image image_5;
      sensor_msgs::Image image_6;
      sensor_msgs::Image image_7;

      common::Time frame_time;
  };
}
#endif
