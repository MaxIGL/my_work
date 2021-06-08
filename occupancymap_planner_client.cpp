#include "ros/ros.h"
#include "my_pcl_tutorial/occupancymap_planner.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "occupancymap_planner");
  if (argc != 4)
  {
    ROS_INFO("usage: occupancymap_planner file_in file_out resolution_discretized");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<my_pcl_tutorial::occupancymap_planner>("occupancymap_planner");
  my_pcl_tutorial::occupancymap_planner srv;
  srv.request.file_in = argv[1];
  srv.request.file_out = argv[2];
  srv.request.resolution_discretized = atof(argv[3]);
  if (client.call(srv))
  {
    ROS_INFO("The call of the service occupancy map planner was a success");
  }
  else
  {
    ROS_ERROR("Failed to call service occupancy map planner");
    return 1;
  }

  return 0;
}
