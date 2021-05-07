
#include <ros/ros.h>
#include <iostream>
#include <fstream>



ros::Publisher pub;

int existe(ar_track_alvar_msgs/AlvarMarkerConstPtr& ar, std::vector<int> liste)
{
int test=0;
for (unsigned i=0;i!=liste.lenght;i++){
if (liste[i]==ar.id) test=1;}
return test;
}

void cloud_cb(const ar_track_alvar_msgs/AlvarMarkersConstPtr& ar_tracker)
{

//Initialization
std::vector<int> ar_ids;
std::vector<int> ar_pose_confidences;
std::vector<std::vector<float>> ar_positions;
std::vector<std::vector<float>> ar_orientations;
int update=0;


//Reading Data
int existe=0;
for (unsigned i=0;i!=ar_tracker.markers.lenght){
if(existe(ar_tracker.markers[i],ar_ids)==0){
update=1;
//Add of the new detected marker to database
ar_ids.push_back(ar_tracker.markers[i].id);
ar_pose_confidence.push_back(ar_tracker.markers[i].confidence);

const geometry_msgs/Point ar_position_point=ar_tracker.markers[i].pose.pose.position;
std::vector<std::vector<float>> ar_position;
ar_position[0]=ar_position_point.x;
ar_position[1]=ar_position_point.y;
ar_position[2]=ar_position_point.z;
ar_positions.push_back(ar_position);

const geometry_msgs/Quaternion ar_orientation_quat=ar_tracker.markers[i].pose.pose.orientation;
std::vector<std::vector<float>> ar_orientation;
ar_orientation[0]=ar_orientation_quat.x;
ar_orientation[1]=ar_orientation_quat.y;
ar_orientation[2]=ar_orientation_quat.z;
ar_orientation[3]=ar_orientation_quat.w;
ar_orientations.push_back(ar_orientation);
}}


//Saving Data
if (update==1){

}

}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Spin
  ros::spin();
}
