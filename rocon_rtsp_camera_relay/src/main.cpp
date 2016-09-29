/*
 License: BSD
   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
*/

#include <ros/ros.h>
#include <rocon_rtsp_camera_relay/rocon_rtsp_camera_relay.hpp>

int main (int argc, char** argv) 
{
  ros::init(argc, argv, "rtsp_camera_relay");
  ros::NodeHandle pnh("~");
  std::string video_stream_url, user, password, camera_name, camera_config_url;

  pnh.getParam("video_stream_url", video_stream_url);

  //For camera configuration so that this camera can be calibrated
  pnh.getParam("camera_name", camera_name);
  pnh.getParam("camera_config_url", camera_config_url);
  ros::NodeHandle camera_nh(camera_name);

  rocon::RoconRtspCameraRelay rtsp(pnh, camera_nh);
  ROS_INFO("Rtsp Camera : Initialising..");
  if(!rtsp.init(video_stream_url, camera_config_url))
  {
    ROS_ERROR("Rtsp Camera : Failed to initialise stream");
    return -1;
  }

  ROS_INFO("Rtsp Camera : Initialised");
  rtsp.spin();
  ROS_INFO("Rtsp Camera : Bye Bye");

  return 0;
}
