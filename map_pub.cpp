#include <stdio.h>
#include <stdlib.h>

#include <nav_msgs/OccupancyGrid.h>
#include <fstream>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip> //used for formatting outputs
#include <sstream>
#include <vector>



int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh("~");
  std::string map_file;
 nh.getParam("occupancy_map_filename",map_file);
std::string topic_out;
nh.getParam("topic_out",topic_out);
ROS_INFO("Occupancy map publisher started");
ros::Rate rate(0.2);


while(nh.ok()){

//Databases initialization
        std::string wid,hei,frame_id,res; //variables from file are here
	std::string cell;

        std::vector<int>ID;
        int width;
	int height;
	double resolution;
	std::vector<int8_t> occupancygridlist;
	double origine_x,origine_y;

//Reading Database 1
        std::ifstream database(data); //opening the file.
        if (database.is_open()) //if the file is open
        {
                ROS_INFO("Reading the map");
                getline(database, res, '\n');
		resolution=stof(res);
		getline(database, wid, '\n');
		width=stoi(wid);
		getline(database,hei,'\n');
		height=stoi(hei);
		getline(database,frame_id,'\n');
		std::string line;
		getline(database,line,'\n');
		origine_x=stof(line);
		getline(database,line,'\n');
		origine_y=stof(line);

                while (true)
                {
                        getline(database, cell, ' ');
                        if (database.eof()) break;
                        occupancygridlist.push_back(stoi(cell));
                }
                ROS_INFO("Reading finished, closing the map file");
                database.close(); //closing the file
        }
        else ROS_INFO("Unable to open map file"); //if the file is not open output

nav_msgs::OccupancyGrid occupancygrid;
occupancygrid.header.frame_id=req.frame_id;
occupancygrid.info.resolution=resolution;
occupancygrid.info.width=width;
occupancygrid.info.height=height;
occupancygrid.info.origin.position.x=origin_x;
occupancygrid.info.origin.position.y=origin_y;
occupancygrid.info.origin.orientation.x = 0.0;
occupancygrid.info.origin.orientation.y = 0.0;
occupancygrid.info.origin.orientation.z = 0.0;
occupancygrid.info.origin.orientation.w = 1.0;
occupancygrid.data=occupancygridlist;

ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>(topic_out, 1);

  while (ros::ok())
  {
    
pub.publish(occupancygrid);
    loop_rate.sleep();
  }

return 0;
}