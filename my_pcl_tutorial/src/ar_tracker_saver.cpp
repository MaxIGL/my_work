
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip> //used for formatting outputs
#include <sstream>
#include <vector>
#include <fstream>

#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/MultiMarkerBundle.h"
#include "ar_track_alvar/MultiMarkerInitializer.h"
#include "ar_track_alvar/Shared.h"
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

bool find_id(std::vector<int> list, int id){
for(unsigned int i=0;i!=list.size();i++){
if(id==list[i]){
return true;}
}
return false;
}

void ar_saver_cb(const ar_track_alvar_msgs::AlvarMarkersConstPtr& ar_tracker)
{

//Reading data from database
	std::string ID, POSITION_X, POSITION_Y, POSITION_Z, QUATERNION_X, QUATERNION_Y, QUATERNION_Z, QUATERNION_W, POSITION_CONFIDENCE; //variables from file are here
	std::vector<int>ID_v;
	std::vector<float>POSITION_X_v;
	std::vector<float>POSITION_Y_v;
	std::vector<float>POSITION_Z_v;
	std::vector<float>QUATERNION_X_v;
	std::vector<float>QUATERNION_Y_v;
	std::vector<float>QUATERNION_Z_v;
	std::vector<float>QUATERNION_W_v;
	std::vector<int>POSITION_CONFIDENCE_v;

	//input filename
	std::string filename="database.txt";

	//number of lines
	int number_detected=0;
	std::ifstream database(filename); //opening the file.
	if (database.is_open()) //if the file is open
	{
		//ignore first line
		std::string line;
		getline(database, line);

		while (true)
		{
			getline(database, ID, ',');
			if (database.eof()) break;
			ID_v.push_back(stoi(ID));
			getline(database, POSITION_CONFIDENCE, ',');
			POSITION_CONFIDENCE_v.push_back(stoi(POSITION_CONFIDENCE));
			getline(database, POSITION_X, '\n');
			/*POSITION_X_v.push_back(stof(POSITION_X));
			getline(database, POSITION_Y, ',');
			POSITION_Y_v.push_back(stof(POSITION_Y));
			getline(database, POSITION_Z, ',');
			POSITION_Z_v.push_back(stof(POSITION_Z));
			getline(database, QUATERNION_X, ',');
			QUATERNION_X_v.push_back(stof(QUATERNION_X));
			getline(database, QUATERNION_Y, ',');
			QUATERNION_Y_v.push_back(stof(QUATERNION_Y));
			getline(database, QUATERNION_Z, ',');
			QUATERNION_Z_v.push_back(stof(QUATERNION_Z));
			getline(database, QUATERNION_W, '\n');
			QUATERNION_W_v.push_back(stof(QUATERNION_W));*/
			number_detected+=1;
			
		}
		database.close(); //closing the file
	}
	else std::cout << "Unable to open file"; //if the file is not open output



//Reading Data from current message
int existe=0;
for (unsigned i=0;i!=ar_tracker->markers.size();i++){
if(!find_id(ID_v,ar_tracker->markers[i].id)){
//Add the new detected marker to the database
	std::ofstream writer(filename,std::ios::app);
	if(!writer){
	ROS_INFO("Error Opening file : ", filename,'\n');}
	writer <<ar_tracker->markers[i].id <<','<<' '<< ar_tracker->markers[i].confidence << ','<<' ' << ar_tracker->markers[i].pose.pose.position.x <<',' <<' '<< ar_tracker->markers[i].pose.pose.position.y << ','<<' '<<ar_tracker->markers[i].pose.pose.position.z << ','<<' '<< ar_tracker->markers[i].pose.pose.orientation.x <<','<<' '<<ar_tracker->markers[i].pose.pose.orientation.y<<','<<' '<<ar_tracker->markers[i].pose.pose.orientation.z<<','<<' '<<ar_tracker->markers[i].pose.pose.orientation.w << '\n';
	writer.close();}

}}

/*ar_ids.push_back(ar_tracker.markers[i].id);
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

*/


int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/ar_pose_marker", 1, ar_saver_cb);


  // Spin
  ros::spin();
}
