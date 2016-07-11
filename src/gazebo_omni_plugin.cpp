
/*
 * Copyright 2013 Open Source Robotics Foundation
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
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
// #include <gazebo_plugins/MultiCameraPlugin.h>
#include <gazebo_omni_plugin.h>
#include <gazebo_plugins/gazebo_ros_utils.h>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(OmniCameraPlugin)

/////////////////////////////////////////////////
OmniCameraPlugin::OmniCameraPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
OmniCameraPlugin::~OmniCameraPlugin()
{
  this->parentSensor.reset();
  this->camera.clear();
}

/////////////////////////////////////////////////
void OmniCameraPlugin::Load(sensors::SensorPtr _sensor,
  sdf::ElementPtr _sdf)
{
  if (!_sensor)
    gzerr << "Invalid sensor pointer.\n";

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ros::NodeHandle nh("~");

  this->parentSensor =
  #if GAZEBO_MAJOR_VERSION >= 7
    std::dynamic_pointer_cast<sensors::MultiCameraSensor>(_sensor);
  #else
    boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(_sensor);
  #endif

  if (!this->parentSensor)
  {
    gzerr << "CameraPlugin requires a CameraSensor.\n";
    #if GAZEBO_MAJOR_VERSION >= 7
      if (std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
    #else
      if (boost::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
    #endif
        gzmsg << "It is a depth camera sensor\n";
  }

  if (!this->parentSensor)
  {
    gzerr << "OmniCameraPlugin not attached to a camera sensor\n";
    return;
  }

  // this->world_ =
  // #if GAZEBO_MAJOR_VERSION >= 7
  //   this->parentSensor->World();
  // #else
  //   this->parentSensor->GetWorld();
  // #endif

# if GAZEBO_MAJOR_VERSION >= 7
  for (unsigned int i = 0; i < this->parentSensor->CameraCount(); ++i)
# else
  for (unsigned int i = 0; i < this->parentSensor->GetCameraCount(); ++i)
# endif
  {
# if GAZEBO_MAJOR_VERSION >= 7
    this->camera.push_back(this->parentSensor->Camera(i));
# else
    this->camera.push_back(this->parentSensor->GetCamera(i));
# endif

    // save camera attributes
# if GAZEBO_MAJOR_VERSION >= 7
    this->width.push_back(this->camera[i]->ImageWidth());
    this->height.push_back(this->camera[i]->ImageHeight());
    this->depth.push_back(this->camera[i]->ImageDepth());
    this->format.push_back(this->camera[i]->ImageFormat());
# else
    this->width.push_back(this->camera[i]->GetImageWidth());
    this->height.push_back(this->camera[i]->GetImageHeight());
    this->depth.push_back(this->camera[i]->GetImageDepth());
    this->format.push_back(this->camera[i]->GetImageFormat());
# endif

# if GAZEBO_MAJOR_VERSION >= 7
    std::string cameraName = this->parentSensor->Camera(i)->Name();
# else
    std::string cameraName = this->parentSensor->GetCamera(i)->GetName();
# endif

    if (cameraName.find("cam0") != std::string::npos)
    {
      std::cout << "found camera 0" << std::endl;
      image_publisher_0 = nh.advertise<sensor_msgs::Image>("omni_rig/cam0_image", 100);

      this->newFrameConnection.push_back(this->camera[i]->ConnectNewImageFrame(
        boost::bind(&OmniCameraPlugin::OnNewFrameCam0,
        this, _1, _2, _3, _4, _5)));
    }
    else if (cameraName.find("cam1") != std::string::npos)
    {
      std::cout << "found camera 1" << std::endl;
      image_publisher_1 = nh.advertise<sensor_msgs::Image>("omni_rig/cam1_image", 100);

      this->newFrameConnection.push_back(this->camera[i]->ConnectNewImageFrame(
        boost::bind(&OmniCameraPlugin::OnNewFrameCam1,
        this, _1, _2, _3, _4, _5)));
    }
    else if (cameraName.find("cam2") != std::string::npos)
    {
      std::cout << "found camera 2" << std::endl;
      image_publisher_2 = nh.advertise<sensor_msgs::Image>("omni_rig/cam2_image", 100);

      this->newFrameConnection.push_back(this->camera[i]->ConnectNewImageFrame(
        boost::bind(&OmniCameraPlugin::OnNewFrameCam2,
        this, _1, _2, _3, _4, _5)));
    }
    else if (cameraName.find("cam3") != std::string::npos)
    {
      std::cout << "found camera 3" << std::endl;
      image_publisher_3 = nh.advertise<sensor_msgs::Image>("omni_rig/cam3_image", 100);

      this->newFrameConnection.push_back(this->camera[i]->ConnectNewImageFrame(
        boost::bind(&OmniCameraPlugin::OnNewFrameCam3,
        this, _1, _2, _3, _4, _5)));
    }
    else if (cameraName.find("cam4") != std::string::npos)
    {
      std::cout << "found camera 4" << std::endl;
      image_publisher_4 = nh.advertise<sensor_msgs::Image>("omni_rig/cam4_image", 100);

      this->newFrameConnection.push_back(this->camera[i]->ConnectNewImageFrame(
        boost::bind(&OmniCameraPlugin::OnNewFrameCam4,
        this, _1, _2, _3, _4, _5)));
    }
    else if (cameraName.find("cam5") != std::string::npos)
    {
      std::cout << "found camera 5" << std::endl;
      image_publisher_5 = nh.advertise<sensor_msgs::Image>("omni_rig/cam5_image", 100);

      this->newFrameConnection.push_back(this->camera[i]->ConnectNewImageFrame(
        boost::bind(&OmniCameraPlugin::OnNewFrameCam5,
        this, _1, _2, _3, _4, _5)));
    }
    else if (cameraName.find("cam6") != std::string::npos)
    {
      std::cout << "found camera 6" << std::endl;
      image_publisher_6 = nh.advertise<sensor_msgs::Image>("omni_rig/cam6_image", 100);

      this->newFrameConnection.push_back(this->camera[i]->ConnectNewImageFrame(
        boost::bind(&OmniCameraPlugin::OnNewFrameCam6,
        this, _1, _2, _3, _4, _5)));
    }
    else if (cameraName.find("cam7") != std::string::npos)
    {
      std::cout << "found camera 7" << std::endl;
      image_publisher_7 = nh.advertise<sensor_msgs::Image>("omni_rig/cam7_image", 100);

      this->newFrameConnection.push_back(this->camera[i]->ConnectNewImageFrame(
        boost::bind(&OmniCameraPlugin::OnNewFrameCam7,
        this, _1, _2, _3, _4, _5)));
    }
  }

  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void OmniCameraPlugin::OnNewFrameCam0(const unsigned char * _image,
                              unsigned int _width,
                              unsigned int _height,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{

  // std::cout << "callback camera 0" << std::endl;
  frame_time = //this->world_->GetSimTime();
  #if GAZEBO_MAJOR_VERSION >= 7
    this->parentSensor->LastMeasurementTime();
  #else
    this->parentSensor->GetLastMeasurementTime();
  #endif

  image_0.header.frame_id = "cam0";
  image_0.header.stamp.sec = frame_time.sec;
  image_0.header.stamp.nsec = frame_time.nsec;
  image_0.width = _width;
  image_0.height = _height;
  image_0.encoding = "mono8";
  sensor_msgs::fillImage( image_0,
                          sensor_msgs::image_encodings::MONO8,
                          _height, // height
                          _width, // width
                          _width, // stepSize
                          _image);
  image_publisher_0.publish(image_0);
}

/////////////////////////////////////////////////
void OmniCameraPlugin::OnNewFrameCam1(const unsigned char * _image,
                              unsigned int _width,
                              unsigned int _height,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{

  // std::cout << "callback camera 1" << std::endl;
  image_1.header.frame_id = "cam1";
  image_1.header.stamp.sec = frame_time.sec;
  image_1.header.stamp.nsec = frame_time.nsec;
  image_1.width = _width;
  image_1.height = _height;
  image_1.encoding = "mono8";
  sensor_msgs::fillImage( image_1,
                          sensor_msgs::image_encodings::MONO8,
                          _height, // height
                          _width, // width
                          _width, // stepSize
                          _image);
  image_publisher_1.publish(image_1);
}

/////////////////////////////////////////////////
void OmniCameraPlugin::OnNewFrameCam2(const unsigned char * _image,
                              unsigned int _width,
                              unsigned int _height,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{

  // std::cout << "callback camera 2" << std::endl;
  image_2.header.frame_id = "cam2";
  image_2.header.stamp.sec = frame_time.sec;
  image_2.header.stamp.nsec = frame_time.nsec;
  image_2.width = _width;
  image_2.height = _height;
  image_2.encoding = "mono8";
  sensor_msgs::fillImage( image_2,
                          sensor_msgs::image_encodings::MONO8,
                          _height, // height
                          _width, // width
                          _width, // stepSize
                          _image);
  image_publisher_2.publish(image_2);
}

/////////////////////////////////////////////////
void OmniCameraPlugin::OnNewFrameCam3(const unsigned char * _image,
                              unsigned int _width,
                              unsigned int _height,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{

  // std::cout << "callback camera 3" << std::endl;
  image_3.header.frame_id = "cam3";
  image_3.header.stamp.sec = frame_time.sec;
  image_3.header.stamp.nsec = frame_time.nsec;
  image_3.width = _width;
  image_3.height = _height;
  image_3.encoding = "mono8";
  sensor_msgs::fillImage( image_3,
                          sensor_msgs::image_encodings::MONO8,
                          _height, // height
                          _width, // width
                          _width, // stepSize
                          _image);
  image_publisher_3.publish(image_3);
}

/////////////////////////////////////////////////
void OmniCameraPlugin::OnNewFrameCam4(const unsigned char * _image,
                              unsigned int _width,
                              unsigned int _height,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{

  // std::cout << "callback camera 4" << std::endl;
  image_4.header.frame_id = "cam4";
  image_4.header.stamp.sec = frame_time.sec;
  image_4.header.stamp.nsec = frame_time.nsec;
  image_4.width = _width;
  image_4.height = _height;
  image_4.encoding = "mono8";
  sensor_msgs::fillImage( image_4,
                          sensor_msgs::image_encodings::MONO8,
                          _height, // height
                          _width, // width
                          _width, // stepSize
                          _image);
  image_publisher_4.publish(image_4);
}

/////////////////////////////////////////////////
void OmniCameraPlugin::OnNewFrameCam5(const unsigned char * _image,
                              unsigned int _width,
                              unsigned int _height,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{

  // std::cout << "callback camera 5" << std::endl;
  image_5.header.frame_id = "cam5";
  image_5.header.stamp.sec = frame_time.sec;
  image_5.header.stamp.nsec = frame_time.nsec;
  image_5.width = _width;
  image_5.height = _height;
  image_5.encoding = "mono8";
  sensor_msgs::fillImage( image_5,
                          sensor_msgs::image_encodings::MONO8,
                          _height, // height
                          _width, // width
                          _width, // stepSize
                          _image);
  image_publisher_5.publish(image_5);
}

/////////////////////////////////////////////////
void OmniCameraPlugin::OnNewFrameCam6(const unsigned char * _image,
                              unsigned int _width,
                              unsigned int _height,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{

  // std::cout << "callback camera 6" << std::endl;
  image_6.header.frame_id = "cam6";
  image_6.header.stamp.sec = frame_time.sec;
  image_6.header.stamp.nsec = frame_time.nsec;
  image_6.width = _width;
  image_6.height = _height;
  image_6.encoding = "mono8";
  sensor_msgs::fillImage( image_6,
                          sensor_msgs::image_encodings::MONO8,
                          _height, // height
                          _width, // width
                          _width, // stepSize
                          _image);
  image_publisher_6.publish(image_6);
}

/////////////////////////////////////////////////
void OmniCameraPlugin::OnNewFrameCam7(const unsigned char * _image,
                              unsigned int _width,
                              unsigned int _height,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{

  // std::cout << "callback camera 7" << std::endl;
  image_7.header.frame_id = "cam7";
  image_7.header.stamp.sec = frame_time.sec;
  image_7.header.stamp.nsec = frame_time.nsec;
  image_7.width = _width;
  image_7.height = _height;
  image_7.encoding = "mono8";
  sensor_msgs::fillImage( image_7,
                          sensor_msgs::image_encodings::MONO8,
                          _height, // height
                          _width, // width
                          _width, // stepSize
                          _image);
  image_publisher_7.publish(image_7);
}
