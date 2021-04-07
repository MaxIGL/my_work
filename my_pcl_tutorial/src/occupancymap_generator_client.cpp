#include "ros/ros.h"
#include "my_pcl_tutorial/occupancymap_generator.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "occupancymap_generator");
  if (argc != 6)
  {
    ROS_INFO("usage: occupancymap_generator file_in file_out frame_id resolution resolution_discretized");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<my_pcl_tutorial::occupancymap_generator>("occupancymap_generator");
  my_pcl_tutorial::occupancymap_generator srv;
  srv.request.file_in = argv[1];
  srv.request.file_out = argv[2];
  srv.request.frame_id=argv[3];
  srv.request.resolution = atof(argv[4]);
  srv.request.resolution_discretized = atof(argv[5]);
  if (client.call(srv))
  {
    ROS_INFO("The call of the service occupancy map generator was a success");
  }
  else
  {
    ROS_ERROR("Failed to call service occupancy map generator");
    return 1;
  }

  return 0;
}

