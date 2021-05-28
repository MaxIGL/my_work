 #include <ros/console.h>
#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip> //used for formatting outputs
#include <sstream>
#include <vector>
#include <fstream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>



std::vector<std::vector<int>> common(std::vector<int> liste1, std::vector<int> liste2)
{
//This function search for common elements in liste1 and liste2 along with their positions in the respective lists
std::vector<std::vector<int>> output;
int compteur=0;
for (unsigned int i=0; i!=liste1.size();i++){
for (unsigned int j=0;j!=liste2.size();j++){
if (liste1[i]==liste2[j]){
ROS_INFO("A common ar_tag has been found ! The Ar_tag number %d has been detected in both databases.",liste1[i]); 
std::vector<int> common_el;
common_el.push_back(liste1[i]);
common_el.push_back(i);
common_el.push_back(j);
output.push_back(common_el);
}
}}
return output;
}



int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh("~");
  std::string data_1,data_2;
 nh.getParam("database1",data_1);
nh.getParam("database2",data_2);
std::string parent_frame;
nh.getParam("parent_frame",parent_frame);
std::string child_frame;
nh.getParam("child_frame",child_frame);
ROS_INFO("Started Linking TF Node");
ros::Rate rate(0.2);


while(nh.ok()){

//Databases initialization
        std::string ID, POSITION_X, POSITION_Y, POSITION_Z, QUATERNION_X, QUATERNION_Y, QUATERNION_Z, QUATERNION_W, POSITION_CONFIDENCE; //variables from file are here

        std::vector<int>ID_1;
        std::vector<float>POSITION_X_1;
        std::vector<float>POSITION_Y_1;
        std::vector<float>POSITION_Z_1;
        std::vector<float>QUATERNION_X_1;
        std::vector<float>QUATERNION_Y_1;
        std::vector<float>QUATERNION_Z_1;
        std::vector<float>QUATERNION_W_1;
        std::vector<int>POSITION_CONFIDENCE_1;

        std::vector<int>ID_2;
        std::vector<float>POSITION_X_2;
        std::vector<float>POSITION_Y_2;
        std::vector<float>POSITION_Z_2;
        std::vector<float>QUATERNION_X_2;
        std::vector<float>QUATERNION_Y_2;
        std::vector<float>QUATERNION_Z_2;
        std::vector<float>QUATERNION_W_2;
        std::vector<int>POSITION_CONFIDENCE_2;

//Reading Database 1
        std::ifstream database1(data_1); //opening the file.
        if (database1.is_open()) //if the file is open
        {
                ROS_INFO("Reading the database 1");
                //ignore first line
                std::string line;
                getline(database1, line);

                while (true)
                {
                        getline(database1, ID, ',');
                        if (database1.eof()) break;
                        ID_1.push_back(stoi(ID));
                        getline(database1, POSITION_CONFIDENCE, ',');
                        POSITION_CONFIDENCE_1.push_back(stoi(POSITION_CONFIDENCE));
                        getline(database1, POSITION_X, ',');
                        POSITION_X_1.push_back(stof(POSITION_X));
                        getline(database1, POSITION_Y, ',');
                        POSITION_Y_1.push_back(stof(POSITION_Y));
                        getline(database1, POSITION_Z, ',');
                        POSITION_Z_1.push_back(stof(POSITION_Z));
                        getline(database1, QUATERNION_X, ',');
                        QUATERNION_X_1.push_back(stof(QUATERNION_X));
                        getline(database1, QUATERNION_Y, ',');
                        QUATERNION_Y_1.push_back(stof(QUATERNION_Y));
                        getline(database1, QUATERNION_Z, ',');
                        QUATERNION_Z_1.push_back(stof(QUATERNION_Z));
                        getline(database1, QUATERNION_W, '\n');
                        QUATERNION_W_1.push_back(stof(QUATERNION_W));

                }
                ROS_INFO("Reading finished, closing the database 1");
                database1.close(); //closing the file
        }
        else ROS_INFO("Unable to open database 1"); //if the file is not open output

//Reading Database 2
        std::ifstream database2(data_2); //opening the file.
        if (database2.is_open()) //if the file is open
        {
                ROS_INFO("Reading the database 2");
                //ignore first line
                std::string line;
                getline(database2, line);

                while (true)
                {
                        getline(database2, ID, ',');
                        if (database2.eof()) break;
                        ID_2.push_back(stoi(ID));
                        getline(database2, POSITION_CONFIDENCE, ',');
                        POSITION_CONFIDENCE_2.push_back(stoi(POSITION_CONFIDENCE));
                        getline(database2, POSITION_X, ',');
                        POSITION_X_2.push_back(stof(POSITION_X));
                        getline(database2, POSITION_Y, ',');
                        POSITION_Y_2.push_back(stof(POSITION_Y));
                        getline(database2, POSITION_Z, ',');
                        POSITION_Z_2.push_back(stof(POSITION_Z));
                        getline(database2, QUATERNION_X, ',');
                        QUATERNION_X_2.push_back(stof(QUATERNION_X));
                        getline(database2, QUATERNION_Y, ',');
                        QUATERNION_Y_2.push_back(stof(QUATERNION_Y));
                        getline(database2, QUATERNION_Z, ',');
                        QUATERNION_Z_2.push_back(stof(QUATERNION_Z));
                        getline(database2, QUATERNION_W, '\n');
                        QUATERNION_W_2.push_back(stof(QUATERNION_W));

                }
                ROS_INFO("Reading finished, closing the database 2");
                database2.close(); //closing the file
        }
        else ROS_INFO("Unable to open database 2"); //if the file is not open output


//Look at common ar_tags between databases
std::vector<std::vector<int>> common_elements=common(ID_1,ID_2);
//Compute the translation and rotation between the first common found element
//Need to be improve, maybe method Least Square ?

//Translation
std::vector<float> T_12;
T_12.push_back(POSITION_X_2[common_elements[0][2]]-POSITION_X_1[common_elements[0][1]]);
T_12.push_back(POSITION_Y_2[common_elements[0][2]]-POSITION_Y_1[common_elements[0][1]]);
T_12.push_back(POSITION_Z_2[common_elements[0][2]]-POSITION_Z_1[common_elements[0][1]]);

//Rotation
tf2::Quaternion q1(QUATERNION_X_1[common_elements[0][1]],QUATERNION_Y_1[common_elements[0][1]],QUATERNION_Z_1[common_elements[0][1]],-QUATERNION_W_1[common_elements[0][1]]);
tf2::Quaternion q2(QUATERNION_X_2[common_elements[0][2]],QUATERNION_Y_2[common_elements[0][2]],QUATERNION_Z_2[common_elements[0][2]],QUATERNION_W_2[common_elements[0][2]]);
tf2::Quaternion Q_12=q2*q1;

//Final TF from 1 to 2
tf2_ros::StaticTransformBroadcaster broadcaster;
geometry_msgs::TransformStamped TF_12;

TF_12.header.stamp = ros::Time::now();
TF_12.header.frame_id = parent_frame;
TF_12.child_frame_id = child_frame;

TF_12.transform.translation.x = T_12[0];
TF_12.transform.translation.y = T_12[1];
TF_12.transform.translation.z = T_12[2];
TF_12.transform.rotation.x = Q_12.x();
TF_12.transform.rotation.y = Q_12.y();
TF_12.transform.rotation.z = Q_12.z();
TF_12.transform.rotation.w = Q_12.w();

broadcaster.sendTransform(TF_12);
ROS_INFO("TF published");
rate.sleep();
}
return 0;
}



