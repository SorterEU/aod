#include "ros/ros.h"
#include "polled_camera/GetPolledImage.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "polled_camera_sync");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<polled_camera::GetPolledImage>("/rod/camera/rgb/request_image");
  polled_camera::GetPolledImage srv;
  srv.request.response_namespace = "/rod/camera/rgb";

  ros::Rate loop_rate(8);

  while (ros::ok()) {

    if (client.call(srv)) {
//    ROS_INFO("SYNC");
    } else {
      ROS_WARN("SYNC failes");
    }

    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}
