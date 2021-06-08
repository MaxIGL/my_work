#include "ros/ros.h"
#include "my_pcl_tutorial/occupancymap_planner.h"

#include <cmath>
#include <vector>

#include <stdio.h>
#include <stdlib.h>

#include <fstream>

std::vector<std::vector<int8_t>> map_matrix(std::vector<int8_t> list, int width, int height){
std::vector<std::vector<int8_t>> matrix(height,std::vector<int8_t>(width));
for (unsigned int j=0; j!=width;j++){
for (unsigned int i=0;i!=height;i++){
matrix[i][j]=list[i*height+j];}}
return matrix;
}

int check_neighbors(i1,j1,i2,j2,matrix){
int path=0;
if (matrix[i1][j1]!=0 or matrix[i2][j2]!=0){
path=100;}
return path;}

std::vector<int>(2) xytoindex(x,y,origin_x,origin_y,resolution){
int i,j;
//Colonne
float cell_x= (x-origin_x)/resolution -1/2; //normalement compris entre j-1 et j
if (cell_x<=0) j=0;
else j= int(cell_x)+1;
//Ligne
float cell_y= (y-origin_y)/resolution -1/2; //normalement compris entre i-1 et i
if (cell_y<=0) i=0;
else i= int(cell_y)+1;
std::vector<int>(2) index;
index[0]=i;
index[1]=j;
return index;
}


bool map_planner(my_pcl_tutorial::occupancymap_planner::Request  &req,
               my_pcl_tutorial::occupancymap_planner::Response &res)
{

// Initialization

ROS_INFO("request: file_in_map=%s, file_in_artag=%s, file_out=%s, resolution_discretized=%lf, req.file_in_map.c_str(), req.file_in_artag.c_str(), req.file_out.c_str(), req.resolution_discretized);


//Databases initialization
        std::string wid,hei,frame_id,res; //variables from file are here
	std::string cell;
	std::string data=req.file_in_map;
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



//Code for Jasmine map, Discretization of initial grid map
double res_discret=req.resolution_discretized;
int cotes= res_discret/resolution+1;
int rectangle_large=width/cotes;
int restant_large=width-rectangle_large*cotes;
int rectangle_haut=height/cotes;
int restant_haut=height-rectangle_haut*cotes;
if(restant_haut!=0){
rectangle_haut+=1;}
if (restant_large!=0){
rectangle_large+=1;}
std::vector<std::vector<int>> mapdiscret(rectangle_large*rectangle_haut, std::vector<int>(3));
std::cout<< "\n Information about discretized grid map: \n";
std::cout<< "resolution=";
std::cout<< cotes*resolution;
std::cout<< " width=";
std::cout<< rectangle_large;
std::cout<< " height=";
std::cout<< rectangle_haut;

for (unsigned i=0;i!=height;i++){
for(unsigned j=0;j!=width;j++){
int x=i/cotes;
int y=j/cotes;
mapdiscret[y+x*rectangle_large][0]+=1;
if (occupancygridlist[j+width*i]==100) mapdiscret[y+x*rectangle_large][1]+=1;
if(occupancygridlist[j+width*i]==-1) mapdiscret[y+x*rectangle_large][2]+=1;
}}

std::vector<int8_t> griddiscret(rectangle_large*rectangle_haut);
for (unsigned i=0; i!=rectangle_large*rectangle_haut;i++){
double percentage_occ;
double percentage_unknown;
if (mapdiscret[i][2]!=mapdiscret[i][0]){
percentage_occ=mapdiscret[i][1]/double(mapdiscret[i][0]-mapdiscret[i][2]);
percentage_unknown=mapdiscret[i][2]/double(mapdiscret[i][0]);}
else {
percentage_occ=0;
percentage_unknown=1;}
if (percentage_occ>=0.05) griddiscret[i]=100;
else if(percentage_unknown>=0.4) griddiscret[i]=-1;}

//In a txt file 
//Save Waypoints 
std::string waypoints;
for(unsigned int i=0; i!=griddiscret.size();i++){
      waypoints.append("waypoint%d - waypoint\n", i);
      }
waypoints.append("\n");

//Create a matrix map out of the discretized map
std::vector<std::vector<int8_t>> map= map_matrix(griddiscret, rectangle_large,rectangle_haut);


std::string can_traverse;
std::string can_traverse_drone;
//Can traverse east-west ?
for(unsigned i=0;i!=rectangle_haut;i++){
for (unsigned j=0;j!=rectangle_large-1;j++){
can_traverse_drone.append("(can_traverse drone1 waypoint%d waypoint%d)\n", i*rectangle_large+j, i*rectangle_large+j+1);
can_traverse_drone.append("(can_traverse drone1 waypoint%d waypoint%d)\n", i*rectangle_large+j+1, i*rectangle_large+j);
if(matrix[i][j]==0){
if(matrix[i][j+1]==0]{
can_traverse.append("(can_traverse rover0 waypoint%d waypoint%d)\n", i*rectangle_large+j, i*rectangle_large+j+1);
can_traverse.append("(can_traverse rover0 waypoint%d waypoint%d)\n", i*rectangle_large+j+1, i*rectangle_large+j);
}}}}


//Can traverse north-south ?
for(unsigned i=0;i!=rectangle_haut-1;i++){
for (unsigned j=0;j!=rectangle_large;j++){
can_traverse_drone.append("(can_traverse drone1 waypoint%d waypoint%d)\n", i*rectangle_large+j, (i+1)*rectangle_large+j);
can_traverse_drone.append("(can_traverse drone1 waypoint%d waypoint%d)\n", (i+1)*rectangle_large+j, i*rectangle_large+j);
if(matrix[i][j]==0){
if(matrix[i+1][j]==0]{
can_traverse.append("(can_traverse rover0 waypoint%d waypoint%d)\n", i*rectangle_large+j, (i+1)*rectangle_large+j);
can_traverse.append("(can_traverse rover0 waypoint%d waypoint%d)\n", (i+1)*rectangle_large+j, i*rectangle_large+j);
}}}}

//Can traverse diagonal (northwest/southeast) ?
for (unsigned int i=0;i!=rectangle_haut-1;i++){
for (unsigned int j=1;j!=rectangle_large;j++){
can_traverse_drone.append("(can_traverse drone1 waypoint%d waypoint%d)\n", i*rectangle_large+j, (i+1)*rectangle_large+j-1);
can_traverse_drone.append("(can_traverse drone1 waypoint%d waypoint%d)\n", (i+1)*rectangle_large+j-1, i*rectangle_large+j);
if(matrix[i][j]==0){
if(check_neighbors(i,j-1,i+1,j,map)==0){
if(matrix[i+1][j-1]==0){
fprintf(txt, "(can_traverse rover0 waypoint%d waypoint%d)\n", i*rectangle_large+j, (i+1)*rectangle_large+j-1);
fprintf(txt, "(can_traverse rover0 waypoint%d waypoint%d)\n", (i+1)*rectangle_large+j-1, i*rectangle_large+j);
}}}}}

//Can traverse anti-diagonal (northeast/southwest) ?
for (unsigned int i=0;i!=rectangle_haut-1;i++){
for (unsigned int j=0;j!=rectangle_large-1;j++){
can_traverse_drone.append("(can_traverse drone1 waypoint%d waypoint%d)\n", i*rectangle_large+j, (i+1)*rectangle_large+j+1);
can_traverse_drone.append("(can_traverse drone1 waypoint%d waypoint%d)\n", (i+1)*rectangle_large+j+1, i*rectangle_large+j);
if(matrix[i][j]==0){
if(check_neighbors(i+1,j,i,j+1,map)==0){
if(matrix[i+1][j+1]==0){
can_traverse.append("(can_traverse rover0 waypoint%d waypoint%d)\n", i*rectangle_large+j, (i+1)*rectangle_large+j+1);
can_traverse.append("(can_traverse rover0 waypoint%d waypoint%d)\n", (i+1)*rectangle_large+j+1, i*rectangle_large+j);
}}}}}

can_traverse_drone.append("\n");

//Objectives as AR tag
//Reading the AR tag database
	std::string ID, POSITION_X,POSITION_Y,POSITION_Z,POSITION_CONFIDENCE; //variables from file are here
	std::vector<int>ID_v;
	std::vector<int>POSITION_CONFIDENCE_v;
	std::vector<float>POSITION_X_v,POSITION_Y_v;

	//input filename
	std::string filenametag=req.file_in_artag;

	//number of lines
	std::ifstream datatag(filenametag); //opening the file.
	if (datatag.is_open()) //if the file is open
	{
		//ignore first line
		std::string line;
		getline(datatag, line);

		while (true)
		{
			getline(datatag, ID, ',');
			if (datatag.eof()) break;
			ID_v.push_back(stoi(ID));
			getline(datatag, POSITION_CONFIDENCE, ',');
			getline(datatag, POSITION_X, ',');
			POSITION_X_v.push_back(stof(POSITION_X));
			getline(datatag, POSITION_Y, ',');
			POSITION_Y_v.push_back(stof(POSITION_Y));
			getline(datatag, POSITION_Z, '\n');
			/*POSITION_Z_v.push_back(stof(POSITION_Z));
			getline(datatag, QUATERNION_X, ',');
			QUATERNION_X_v.push_back(stof(QUATERNION_X));
			getline(datatag, QUATERNION_Y, ',');
			QUATERNION_Y_v.push_back(stof(QUATERNION_Y));
			getline(datatag, QUATERNION_Z, ',');
			QUATERNION_Z_v.push_back(stof(QUATERNION_Z));
			getline(datatag, QUATERNION_W, '\n');
			QUATERNION_W_v.push_back(stof(QUATERNION_W));*/
			
		}
		datatag.close(); //closing the file
	}
	else std::cout << "Unable to open AR tag database file"; //if the file is not open output


std::string objectives;
std::string visible_objectives_drone;
std::string visible_objectives_rover;
std::vector<int>(2) index(x,y,origin_x,origin_y,resolution){
for (unsigned int i=0;i!=ID_v.size();i++){
if(ID_v[i]<=23 and ID_v[i]>=0){
objectives.append("objective%d - objective\n",ID_v[i]);
index=xytoindex(POSITION_X_v[i],POSITION_Y_v[i],origin_x,origin_y,resolution_discretized);
visible_objectives_drone.append("(visible_from objective%d waypoint%d drone1)\n",ID_v[i],index[0]*rectangle_large+index[1]);
//Check if on border south
if(index[0]!=0){
if(matrix[index[0]-1][index[1]]==0){
visible_objectives_rover.append("(visible_from objective%d waypoint%d rover0)\n",ID_v[i],(index[0]-1)*rectangle_large+index[1]);
}}
//Check if on border north
if(index[0]!=rectangle_haut-1){
if(matrix[index[0]+1][index[1]]==0){
visible_objectives_rover.append("(visible_from objective%d waypoint%d rover0)\n",ID_v[i],(index[0]+1)*rectangle_large+index[1]);
}}
//Check if on border west
if(index[1]!=0){
if(matrix[index[0]][index[1]-1]==0){
visible_objectives_rover.append("(visible_from objective%d waypoint%d rover0)\n",ID_v[i],(index[0])*rectangle_large+index[1]-1);
}}
//Check if on border east
if(index[1]!=rectangle_large-1){
if(matrix[index[0]][index[1]+1]==0){
visible_objectives_rover.append("(visible_from objective%d waypoint%d rover0)\n",ID_v[i],(index[0])*rectangle_large+index[1]+1);
}}

}}
obectives.append("\n");





std::string mapplannertxtdatafile = mapplannername + ".txt";
 ROS_INFO("Writing waypoints data to %s", mapplannertxtdatafile.c_str());
      FILE* txt = fopen(mapplannertxtdatafile.c_str(), "w");
      if (!txt)
      {
        std::cout<< "Couldn't write on %s", maptxtdatafile.c_str();
        return false;
      }
 fprintf(txt,waypoints);
 fprintf(txt,objectives);
 fprintf(txt,can_traverse);
 fprintf(txt,can_traverse_drone);
 fprintf(txt,visible_objectives_rover);
 fprintf(txt,visible_objectives_drone);

 fclose(txt);

   ROS_INFO("Sending back response: The grid map plan was generated") ;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "occupancymap_planner_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("occupancymap_planner", map_planner);
  ROS_INFO("Ready to generate the grid map plan");
  ros::spin();

  return 0;
}
