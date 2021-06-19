#include "ros/ros.h"
#include "my_pcl_tutorial/occupancymap_planner.h"

#include <cmath>
#include <vector>

#include <stdio.h>
#include <stdlib.h>

#include <iomanip> //used for formatting outputs
#include <iostream>
#include <fstream>

//Chosen row/column convention to read the grid map
std::vector<std::vector<int8_t>> map_matrix(std::vector<int8_t> list, int width, int height){
std::vector<std::vector<int8_t>> matrix(height,std::vector<int8_t>(width));
if (list.size()!=width*height) ROS_INFO("problem");
for (unsigned int j=0; j!=width;j++){
for (unsigned int i=0;i!=height;i++){
matrix[i][j]=list[i*width+j];}}
return matrix;
}

int check_neighbors(int i1,int j1,int i2,int j2,std::vector<std::vector<int8_t>> matrix){
int path=0;
if (matrix[i1][j1]!=0 or matrix[i2][j2]!=0){
path=100;}
return path;}


//The chosen convention to index the grid map cells is the matrix "row/column" convention so 
//that means the axis y is read on the vertical axis which corresponds to the index i of rows of our matrix
//and the x is read on columns "j" indexes
//(0,0) is the origin minx, miny of the map.
std::vector<int> xytoindex(double x,double y,double origin_x,double origin_y,double resolution){
int i,j;
//Colonne
float cell_x= (x-origin_x)/resolution -1/2; //normalement compris entre j-1 et j
if (cell_x<=0) j=0;
else j= int(cell_x)+1;
//Ligne
float cell_y= (y-origin_y)/resolution -1/2; //normalement compris entre i-1 et i
if (cell_y<=0) i=0;
else i= int(cell_y)+1;
std::vector<int> index(2);
index[0]=i;
index[1]=j;
return index;
}

bool searchforindex(std::vector<int> index, std::vector<std::vector<int>> list){
bool controler=true;
for(unsigned int i=0;i!=list.size();i++){
if (list[i][0]==index[0] and list[i][1]==index[1]) controler=false;}
return controler;
}


bool map_planner(my_pcl_tutorial::occupancymap_planner::Request  &req,
               my_pcl_tutorial::occupancymap_planner::Response &res)
{

// Initialization

ROS_INFO("request: file_in_map=%s, file_in_artag=%s, file_out=%s, resolution_discretized=%lf, thr_occ=%lf, thr_unknown=%lf", req.file_in_map.c_str(), req.file_in_artag.c_str(), req.file_out.c_str(), req.resolution_discretized,req.thr_occ,req.thr_unknown);


//Databases initialization
        std::string wid,hei,frame_id,reso; //variables from file are here
        std::string cell;
        std::string data=req.file_in_map;
        int width;
        int height;
        double resolution;
        std::vector<int8_t> occupancygridlist;
        double origin_x,origin_y;
//Reading Database 1
        std::ifstream database(data); //opening the file.
        if (database.is_open()) //if the file is open
        {
                ROS_INFO("Reading the map");
                getline(database, reso, '\n');
                resolution=stof(reso);
                getline(database, wid, '\n');
                width=stoi(wid);
                getline(database,hei,'\n');
                height=stoi(hei);
                getline(database,frame_id,'\n');
                std::string line;
                getline(database,line,'\n');
                origin_x=stof(line);
                getline(database,line,'\n');
                origin_y=stof(line);

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

//Discretization of initial grid map
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
ROS_INFO( "%lf", cotes*resolution);
std::cout<< " width=";
ROS_INFO("%d", rectangle_large);
std::cout<< " height=";
ROS_INFO("%d", rectangle_haut);

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
if (percentage_occ>=req.thr_occ) griddiscret[i]=100;
else if(percentage_unknown>=req.thr_unknown) griddiscret[i]=-1;}

double threshold_free=25;
double threshold_occupied=65;

  //Remove Borders
//how many layers do you want to be removed 
int layers_removed=2;
int new_width=rectangle_large-2*layers_removed;
int new_height=rectangle_haut-2*layers_removed;
std::vector<int8_t> mapborders(new_width*new_height);
for (unsigned int i=layers_removed;i!=rectangle_haut-layers_removed;i++){
for(unsigned int  j=layers_removed;j!=rectangle_large-layers_removed;j++){
mapborders[(i-layers_removed)*new_width+(j-layers_removed)]=griddiscret[i*rectangle_large+j];
}}

rectangle_haut=new_height;
rectangle_large=new_width;
origin_x+=layers_removed*cotes*resolution;
origin_y+=layers_removed*cotes*resolution;

//As a pgm file 

std::string mapdatafile_discret = req.file_out + "_discretizedplan.pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile_discret.c_str());
      FILE* out_discret = fopen(mapdatafile_discret.c_str(), "w");
      if (!out_discret)
      {
        ROS_INFO( "Couldn't save map file to %s", mapdatafile_discret.c_str());
        return 0;
      }

      fprintf(out_discret, "P5\n# CREATOR: Grid_map_generator.cpp %.3f m/pix\n%d %d\n255\n",
              cotes*resolution, rectangle_large, rectangle_haut);
        for(unsigned int i=0;i!=rectangle_haut;i++){
        for(unsigned int j=0;j!=rectangle_large;j++){
        unsigned int x=j+rectangle_large*(rectangle_haut-1-i);
          if (mapborders[x] >= 0 && mapborders[x] <= threshold_free) { // Free is 254 : white
            fputc(254, out_discret);
          } else if (mapborders[x] >= threshold_occupied) { // Occupied is black : 000
            fputc(000, out_discret);
          } else { //unknown is 205 gray scale
            fputc(205, out_discret);
        }
          }
        }
      fclose(out_discret);


      std::string mapmetadatafile_discret = req.file_out + "_discretizedplan.yaml";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile_discret.c_str());
      FILE* yaml_discret = fopen(mapmetadatafile_discret.c_str(), "w");


      fprintf(yaml_discret, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile_discret.c_str(), cotes*resolution,origin_x ,origin_y , 0.);

      fclose(yaml_discret);


//In a txt file 


 //Create a matrix map out of the discretized map
std::vector<std::vector<int8_t>> matrix= map_matrix(mapborders, rectangle_large,rectangle_haut);
std::string can_traverse;
std::string can_traverse_drone;


//Can traverse for every cell is not considered anymore. We are gonna try to link the waypoints that are corresponding to an objective only.
/*
//Can traverse east-west ?
for(int i=0;i!=rectangle_haut;i++){
for (int j=0;j!=rectangle_large-1;j++){
can_traverse_drone.append("(can_traverse drone1 waypoint");
can_traverse_drone.append(std::to_string(i*rectangle_large+j));
can_traverse_drone.append(" waypoint");
can_traverse_drone.append(std::to_string(i*rectangle_large+j+1));
can_traverse_drone.append(")\n");

can_traverse_drone.append("(can_traverse drone1 waypoint");
can_traverse_drone.append(std::to_string(i*rectangle_large+j+1));
can_traverse_drone.append(" waypoint"); 
can_traverse_drone.append(std::to_string(i*rectangle_large+j));
can_traverse_drone.append(")\n");

if(matrix[i][j]==0){
if(matrix[i][j+1]==0){
can_traverse.append("(can_traverse rover0 waypoint");
can_traverse.append(std::to_string(i*rectangle_large+j));
can_traverse.append(" waypoint"); 
can_traverse.append(std::to_string(i*rectangle_large+j+1));
can_traverse.append(")\n");

can_traverse.append("(can_traverse rover0 waypoint");
can_traverse.append(std::to_string(i*rectangle_large+j+1));
can_traverse.append(" waypoint"); 
can_traverse.append(std::to_string(i*rectangle_large+j));
can_traverse.append(")\n");

}}}}


//Can traverse north-south ?
for(int i=0;i!=rectangle_haut-1;i++){
for (int j=0;j!=rectangle_large;j++){

can_traverse_drone.append("(can_traverse drone1 waypoint");
can_traverse_drone.append(std::to_string(i*rectangle_large+j));
can_traverse_drone.append(" waypoint"); 
can_traverse_drone.append(std::to_string((i+1)*rectangle_large+j));
can_traverse_drone.append(")\n");

can_traverse_drone.append("(can_traverse drone1 waypoint");
can_traverse_drone.append(std::to_string((i+1)*rectangle_large+j));
can_traverse_drone.append(" waypoint"); 
can_traverse_drone.append(std::to_string(i*rectangle_large+j));
can_traverse_drone.append(")\n");

if(matrix[i][j]==0){
if(matrix[i+1][j]==0){

can_traverse.append("(can_traverse rover0 waypoint");
can_traverse.append(std::to_string(i*rectangle_large+j));
can_traverse.append(" waypoint"); 
can_traverse.append(std::to_string((i+1)*rectangle_large+j));
can_traverse.append(")\n");

can_traverse.append("(can_traverse rover0 waypoint");
can_traverse.append(std::to_string((i+1)*rectangle_large+j));
can_traverse.append(" waypoint"); 
can_traverse.append(std::to_string(i*rectangle_large+j));
can_traverse.append(")\n");

}}}}

 //Can traverse diagonal (northwest/southeast) ?
for ( int i=0;i!=rectangle_haut-1;i++){
for ( int j=1;j!=rectangle_large;j++){

can_traverse_drone.append("(can_traverse drone1 waypoint");
can_traverse_drone.append(std::to_string(i*rectangle_large+j));
can_traverse_drone.append(" waypoint"); 
can_traverse_drone.append(std::to_string((i+1)*rectangle_large+j-1));
can_traverse_drone.append(")\n");

can_traverse_drone.append("(can_traverse drone1 waypoint");
can_traverse_drone.append(std::to_string((i+1)*rectangle_large+j-1));
can_traverse_drone.append(" waypoint"); 
can_traverse_drone.append(std::to_string(i*rectangle_large+j));
can_traverse_drone.append(")\n");


if(matrix[i][j]==0){
if(check_neighbors(i,j-1,i+1,j,matrix)==0){
if(matrix[i+1][j-1]==0){

can_traverse.append("(can_traverse rover0 waypoint");
can_traverse.append(std::to_string(i*rectangle_large+j));
can_traverse.append(" waypoint"); 
can_traverse.append(std::to_string((i+1)*rectangle_large+j-1));
can_traverse.append(")\n");

can_traverse.append("(can_traverse rover0 waypoint");
can_traverse.append(std::to_string((i+1)*rectangle_large+j-1));
can_traverse.append(" waypoint"); 
can_traverse.append(std::to_string(i*rectangle_large+j));
can_traverse.append(")\n");

}}}}}

//Can traverse anti-diagonal (northeast/southwest) ?
for (int i=0;i!=rectangle_haut-1;i++){
for (int j=0;j!=rectangle_large-1;j++){

can_traverse_drone.append("(can_traverse drone1 waypoint");
can_traverse_drone.append(std::to_string(i*rectangle_large+j));
can_traverse_drone.append(" waypoint"); 
can_traverse_drone.append(std::to_string((i+1)*rectangle_large+j+1));
can_traverse_drone.append(")\n");

can_traverse_drone.append("(can_traverse drone1 waypoint");
can_traverse_drone.append(std::to_string((i+1)*rectangle_large+j+1));
can_traverse_drone.append(" waypoint"); 
can_traverse_drone.append(std::to_string(i*rectangle_large+j));
can_traverse_drone.append(")\n");

if(matrix[i][j]==0){
if(check_neighbors(i+1,j,i,j+1,matrix)==0){
if(matrix[i+1][j+1]==0){
can_traverse.append("(can_traverse rover0 waypoint");
can_traverse.append(std::to_string(i*rectangle_large+j));
can_traverse.append(" waypoint"); 
can_traverse.append(std::to_string((i+1)*rectangle_large+j+1));
can_traverse.append(")\n");

can_traverse.append("(can_traverse rover0 waypoint");
can_traverse.append(std::to_string((i+1)*rectangle_large+j+1));
can_traverse.append(" waypoint"); 
can_traverse.append(std::to_string(i*rectangle_large+j));
can_traverse.append(")\n");

}}}}}
*/



  
//Objectives as AR tag
//Reading the AR tag database
        std::string ID, POSITION_X,POSITION_Y,POSITION_Z,POSITION_CONFIDENCE; //variables from file are here
        std::vector<int> ID_v;
        std::vector<int>POSITION_CONFIDENCE_v;
        std::vector<float>POSITION_X_v,POSITION_Y_v;

        //input filename
        std::string filenametag=req.file_in_artag;

        //number of lines
        std::ifstream datatag(filenametag); //opening the file.
        if (datatag.is_open()) //if the file is open
        {
                //ignore first line
                std::string lines;
                getline(datatag, lines);

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

ROS_INFO("OBJECTIVES");
std::string objectives;
std::string visible_objectives_drone;
std::string visible_objectives_rover;
std::vector<int> index;
//Take Waypoints that are only corresponding to a AR tag objective
std::string waypoints;
std::vector<std::vector<int>> indexsobjectives;
for  (int i=0;i!=ID_v.size();i++){
if(ID_v[i]<=60 and ID_v[i]>=0){


index=xytoindex(POSITION_X_v[i],POSITION_Y_v[i],origin_x,origin_y,resolution*cotes);
if(index[0]<=rectangle_haut-1 and index[1]<=rectangle_large-1){
//Check if this objective falls into a waypoint that already contains an objective.
if (searchforindex(index,indexsobjectives)==true){
indexsobjectives.push_back(index);
waypoints.append("waypoint");
waypoints.append(std::to_string(index[0]*rectangle_large+index[1]));
waypoints.append(" - waypoint\n");
visible_objectives_rover.append("(visible_from objective");
visible_objectives_rover.append(std::to_string(ID_v[i]));
visible_objectives_rover.append(" waypoint");
visible_objectives_rover.append(std::to_string(index[0]*rectangle_large+index[1]));
visible_objectives_rover.append(" rover0)\n");
visible_objectives_drone.append("(visible_from objective");
visible_objectives_drone.append(std::to_string(ID_v[i]));
visible_objectives_drone.append(" waypoint");
visible_objectives_drone.append(std::to_string(index[0]*rectangle_large+index[1]));
visible_objectives_drone.append(" drone1)\n");
objectives.append("objective");
objectives.append(std::to_string(ID_v[i]));
ROS_INFO("Waypoint number : %d",ID_v[i]);
objectives.append(" - objective\n");
}}}}

//Can traverse new version with objectives graph
std::vector<std::vector<int>> interm(indexsobjectives.size(),std::vector<int>(2));
std::vector<std::vector<std::vector<int>>> distancesobjectives(indexsobjectives.size(),interm);
//This is a table in which the cell (i,j) contain the information relative to the distance between objective i and objective j
//The cell [i][j] contains a pair (a,b) where a is the distance between i,j from which the distance b is traveled as a diagonal
int unit_distance=rectangle_large;
ROS_INFO("Size objective list %d",indexsobjectives.size());
for(unsigned int i=0;i!=indexsobjectives.size()-1;i++){
for(unsigned int j=i+1;j!=indexsobjectives.size();j++){
//Distance between objective i and objective j
int dist_x=indexsobjectives[i][1]-indexsobjectives[j][1];
if (dist_x<0) dist_x=-dist_x;
int dist_y=indexsobjectives[i][0]-indexsobjectives[j][0];
if (dist_y<0) dist_y=-dist_y;
if (dist_x<=dist_y) {
distancesobjectives[i][j][0]=dist_y;
distancesobjectives[i][j][1]=dist_x;}
if (dist_x>dist_y) {
distancesobjectives[i][j][0]=dist_x;
distancesobjectives[i][j][1]=dist_y;
}
if (distancesobjectives[i][j][0]<unit_distance) unit_distance=distancesobjectives[i][j][0];
}}
//Because every objectives are in a different cell, we ensure unit_distance to be different from 0 (the case where 2 objectives are in the same cell)

//A Database of primary numbers for ID generation
std::vector<int> primary_numbers {17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97,101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193, 197, 199, 211, 223, 227, 229, 233, 239, 241, 251, 257, 263, 269, 271,277, 281, 283, 293, 307, 311, 313, 317, 331, 337, 347, 349, 353, 359, 367, 373, 379, 383, 389, 397, 401, 409, 419, 421, 431, 433, 439, 443, 449, 457, 461, 463, 467, 479, 487, 491, 499, 503, 509,521, 523, 541, 547, 557, 563, 569, 571, 577, 587, 593, 599, 601, 607, 613, 617, 619, 631, 641, 643, 647, 653, 659, 661, 673, 677, 683, 691, 701, 709, 719, 727, 733, 739, 743, 751, 757, 761, 769, 773, 787, 797, 809, 811, 821, 823, 827, 829, 839, 853, 857, 859, 863, 877, 881, 883, 887, 907, 911, 919, 929, 937, 941, 947, 953, 967, 971, 977, 983, 991, 997, 1009, 1013, 1019, 1021, 1031, 1033, 1039, 1049, 1051, 1061, 1063, 1069, 1087, 1091, 1093, 1097, 1103, 1109, 1117, 1123, 1129, 1151, 1153, 1163, 1171, 1181, 1187, 1193, 1201, 1213, 1217, 1223, 1229, 1231, 1237, 1249, 1259, 1277, 1279, 1283, 1289, 1291, 1297, 1301, 1303, 1307, 1319, 1321, 1327, 1361, 1367, 1373, 1381, 1399, 1409, 1423, 1427, 1429, 1433, 1439, 1447, 1451, 1453, 1459, 1471, 1481, 1483, 1487, 1489, 1493, 1499, 1511, 1523, 1531, 1543, 1549, 1553, 1559, 1567, 1571, 1579, 1583, 1597, 1601, 1607, 1609, 1613, 1619, 1621, 1627, 1637, 1657, 1663, 1667, 1669, 1693, 1697, 1699, 1709, 1721, 1723, 1733, 1741, 1747, 1753, 1759, 1777, 1783, 1787, 1789, 1801, 1811, 1823, 1831, 1847, 1861, 1867, 1871, 1873, 1877, 1879, 1889, 1901, 1907, 1913, 1931, 1933, 1949, 1951, 1973, 1979, 1987, 1993, 1997, 1999, 2003, 2011, 2017, 2027, 2029, 2039, 2053, 2063, 2069, 2081, 2083, 2087, 2089, 2099, 2111, 2113, 2129, 2131, 2137, 2141, 2143, 2153, 2161, 2179, 2203, 2207, 2213, 2221, 2237, 2239, 2243, 2251, 2267, 2269, 2273, 2281, 2287, 2293, 2297, 2309, 2311, 2333, 2339, 2341, 2347, 2351, 2357, 2371, 2377, 2381, 2383, 2389, 2393, 2399, 2411, 2417, 2423, 2437, 2441, 2447, 2459, 2467, 2473, 2477, 2503, 2521, 2531, 2539, 2543, 2549, 2551, 2557, 2579, 2591, 2593, 2609, 2617, 2621, 2633, 2647, 2657, 2659, 2663, 2671, 2677, 2683, 2687, 2689, 2693, 2699, 2707, 2711, 2713, 2719, 2729, 2731, 2741, 2749, 2753, 2767, 2777, 2789, 2791, 2797, 2801, 2803, 2819, 2833, 2837, 2843, 2851, 2857, 2861, 2879, 2887, 2897, 2903, 2909, 2917, 2927, 2939, 2953, 2957, 2963, 2969, 2971, 2999, 3001, 3011, 3019, 3023, 3037, 3041, 3049, 3061, 3067, 3079, 3083, 3089, 3109, 3119, 3121, 3137, 3163, 3167, 3169, 3181, 3187, 3191, 3203, 3209, 3217, 3221, 3229, 3251, 3253, 3257, 3259, 3271, 3299, 3301, 3307, 3313, 3319, 3323, 3329, 3331, 3343, 3347, 3359, 3361, 3371, 3373, 3389, 3391, 3407, 3413, 3433, 3449, 3457, 3461, 3463, 3467, 3469, 3491, 3499, 3511, 3517, 3527, 3529, 3533, 3539, 3541, 3547, 3557, 3559, 3571, 3581, 3583, 3593, 3607, 3613, 3617, 3623, 3631, 3637, 3643, 3659, 3671, 3673, 3677, 3691, 3697, 3701, 3709, 3719, 3727, 3733, 3739, 3761, 3767, 3769, 3779, 3793, 3797, 3803, 3821, 3823, 3833, 3847, 3851, 3853, 3863, 3877, 3881, 3889, 3907, 3911, 3917, 3919, 3923, 3929, 3931, 3943, 3947, 3967, 3989, 4001, 4003, 4007, 4013, 4019, 4021, 4027, 4049, 4051, 4057, 4073, 4079, 4091, 4093, 4099, 4111, 4127, 4129, 4133, 4139, 4153, 4157, 4159, 4177, 4201, 4211, 4217, 4219, 4229, 4231, 4241, 4243, 4253, 4259, 4261, 4271, 4273, 4283, 4289, 4297, 4327, 4337, 4339, 4349, 4357, 4363, 4373, 4391, 4397, 4409, 4421, 4423, 4441, 4447, 4451, 4457, 4463, 4481, 4483, 4493, 4507, 4513, 4517, 4519, 4523, 4547, 4549, 4561, 4567, 4583, 4591, 4597, 4603, 4621, 4637, 4639, 4643, 4649, 4651, 4657, 4663, 4673, 4679, 4691, 4703, 4721, 4723, 4729, 4733, 4751, 4759, 4783, 4787, 4789, 4793, 4799, 4801, 4813, 4817, 4831, 4861, 4871, 4877, 4889, 4903, 4909, 4919, 4931, 4933, 4937, 4943, 4951, 4957, 4967, 4969, 4973, 4987, 4993, 4999, 5003, 5009, 5011, 5021, 5023, 5039, 5051, 5059, 5077, 5081, 5087, 5099, 5101, 5107, 5113, 5119, 5147, 5153, 5167, 5171, 5179, 5189, 5197, 5209, 5227, 5231, 5233, 5237, 5261, 5273, 5279, 5281, 5297, 5303, 5309, 5323, 5333, 5347, 5351, 5381, 5387, 5393, 5399, 5407, 5413, 5417, 5419, 5431, 5437, 5441, 5443, 5449, 5471, 5477, 5479, 5483, 5501, 5503, 5507, 5519, 5521, 5527, 5531, 5557, 5563, 5569, 5573, 5581, 5591, 5623, 5639, 5641, 5647, 5651, 5653, 5657, 5659, 5669, 5683, 5689, 5693, 5701, 5711, 5717, 5737, 5741, 5743, 5749, 5779, 5783, 5791, 5801, 5807, 5813, 5821, 5827, 5839, 5843, 5849, 5851, 5857, 5861, 5867, 5869, 5879, 5881, 5897, 5903, 5923, 5927, 5939, 5953, 5981, 5987, 6007, 6011, 6029, 6037, 6043, 6047, 6053, 6067, 6073, 6079, 6089, 6091, 6101, 6113, 6121, 6131, 6133, 6143, 6151, 6163, 6173, 6197, 6199, 6203, 6211, 6217, 6221, 6229, 6247, 6257, 6263, 6269, 6271, 6277, 6287, 6299, 6301, 6311, 6317, 6323, 6329, 6337, 6343, 6353, 6359, 6361, 6367, 6373, 6379, 6389, 6397, 6421, 6427, 6449, 6451, 6469, 6473, 6481, 6491, 6521, 6529, 6547, 6551, 6553, 6563, 6569, 6571, 6577, 6581, 6599, 6607, 6619, 6637, 6653, 6659, 6661, 6673, 6679, 6689, 6691, 6701, 6703, 6709, 6719, 6733, 6737, 6761, 6763, 6779, 6781, 6791, 6793, 6803, 6823, 6827, 6829, 6833, 6841, 6857, 6863, 6869, 6871, 6883, 6899, 6907, 6911, 6917, 6947, 6949, 6959, 6961, 6967, 6971, 6977, 6983, 6991,6997,7001, 7013, 7019, 7027, 7039, 7043, 7057};

for(unsigned int i=0;i!=indexsobjectives.size()-1;i++){
for(unsigned int j=i+1;j!=indexsobjectives.size();j++){
std::vector<int> indexi(2),indexj(2);
indexi[0]=indexsobjectives[i][0];
indexi[1]=indexsobjectives[i][1];
indexj[0]=indexsobjectives[j][0];
indexj[1]=indexsobjectives[j][1];
int distance_ij_rover=distancesobjectives[i][j][0];
double distance_ij_rover_normalized;
double distance_ij_drone_normalized;
//If the path is a pure diagonal, check if its free path for the rover
//If there is an  obstacle on the way then add +1 to the distance
if(distancesobjectives[i][j][1]==distancesobjectives[i][j][0]){
//Check the direction of the diagonal from objective i to objective j
int east=1;
int north=1;
if(indexi[0]>indexj[0]){
north=-1;}
if(indexi[1]>indexj[1]){
east=-1;}
int a=0;
bool obstacle=false;

while(a<=(distancesobjectives[i][j][0]-1) and (obstacle==false)){

//Check if there is an obstacle on the sub-diagonals that would block the diagonal path
if(matrix[indexi[0]+north*(a+1)][indexi[1]+east*a]!=0 or matrix[indexi[0]+a*north][indexi[1]+east*(a+1)]!=0)obstacle=true;
//Check if there is an obstacle on the diagonal itself

if(a!=0){
if(matrix[indexi[0]+north*a][indexi[1]+east*a]!=0) obstacle=true;}
//We suppose that the distance can be only be increased of 1 cell in case of an obstacle on the path
//because then it becomes a non pure diagonal path and we make a supposition on non pure diagonal path (see below)
a+=1;
}
if (obstacle==true) distance_ij_rover+=1;
}
//We suppose in the case of non pure diagonal path, that there are enough possibilities for the system to reach
//the other objective without encountering an obstacle


//Normalized distances with unit_distance
distance_ij_rover_normalized=distance_ij_rover/unit_distance;
distance_ij_drone_normalized=distancesobjectives[i][j][0]/unit_distance;
int drone_more=-1;
//Now that we have the normalized distances, check if we need to add intermediary waypoints for the drone
if(distance_ij_drone_normalized>2){
int how_many_more_drone=int((distance_ij_drone_normalized-0.05)/2);
drone_more=how_many_more_drone;
//Adding imaginary waypoints just to tell the task planner that there is some more distance to travel between objective i and j
unsigned long long int unique_waypoint_id=pow(primary_numbers[0],i+1)*pow(primary_numbers[1],j+1);

waypoints.append("waypoint");
waypoints.append(std::to_string(unique_waypoint_id));
waypoints.append(" - waypoint\n");

can_traverse_drone.append("(can_traverse drone1 waypoint");
can_traverse_drone.append(std::to_string(indexi[0]*rectangle_large+indexi[1]));
can_traverse_drone.append(" waypoint"); 
can_traverse_drone.append(std::to_string(unique_waypoint_id));
can_traverse_drone.append(")\n");

can_traverse_drone.append("(can_traverse drone1 waypoint");
can_traverse_drone.append(std::to_string(unique_waypoint_id));
can_traverse_drone.append(" waypoint"); 
can_traverse_drone.append(std::to_string(indexi[0]*rectangle_large+indexi[1]));
can_traverse_drone.append(")\n");
for (int adding=1;adding!=how_many_more_drone;adding++){
unsigned long long int unique_waypoint_new=pow(primary_numbers[adding],i+1)*pow(primary_numbers[adding+1],j+1);
waypoints.append("waypoint");
waypoints.append(std::to_string(unique_waypoint_new));
waypoints.append(" - waypoint\n");

can_traverse_drone.append("(can_traverse drone1 waypoint");
can_traverse_drone.append(std::to_string(unique_waypoint_id));
can_traverse_drone.append(" waypoint"); 
can_traverse_drone.append(std::to_string(unique_waypoint_new));
can_traverse_drone.append(")\n");
can_traverse_drone.append("(can_traverse drone1 waypoint");
can_traverse_drone.append(std::to_string(unique_waypoint_new));
can_traverse_drone.append(" waypoint"); 
can_traverse_drone.append(std::to_string(unique_waypoint_id));
can_traverse_drone.append(")\n");
unique_waypoint_id=unique_waypoint_new;
}

can_traverse_drone.append("(can_traverse drone1 waypoint");
can_traverse_drone.append(std::to_string(unique_waypoint_id));
can_traverse_drone.append(" waypoint"); 
can_traverse_drone.append(std::to_string(indexj[0]*rectangle_large+indexj[1]));
can_traverse_drone.append(")\n");

can_traverse_drone.append("(can_traverse drone1 waypoint");
can_traverse_drone.append(std::to_string(indexj[0]*rectangle_large+indexj[1]));
can_traverse_drone.append(" waypoint"); 
can_traverse_drone.append(std::to_string(unique_waypoint_id));
can_traverse_drone.append(")\n");

}
//or if the drone can immediatly traverse from i to j
if (distance_ij_drone_normalized<=2){
can_traverse_drone.append("(can_traverse drone1 waypoint");
can_traverse_drone.append(std::to_string(indexi[0]*rectangle_large+indexi[1]));
can_traverse_drone.append(" waypoint"); 
can_traverse_drone.append(std::to_string(indexj[0]*rectangle_large+indexj[1]));
can_traverse_drone.append(")\n");

can_traverse_drone.append("(can_traverse drone1 waypoint");
can_traverse_drone.append(std::to_string(indexj[0]*rectangle_large+indexj[1]));
can_traverse_drone.append(" waypoint"); 
can_traverse_drone.append(std::to_string(indexi[0]*rectangle_large+indexi[1]));
can_traverse_drone.append(")\n");
}

//Now that we have the normalized distances, check if we need to add intermediary waypoints for the rover
if(distance_ij_rover_normalized>2){
int how_many_more=int((distance_ij_rover_normalized-0.05)/2);
//Adding imaginary waypoints just to tell the task planner that there is some more distance to travel between objective i and j
unsigned long long int unique_waypoint_id=pow(primary_numbers[0],i+1)*pow(primary_numbers[1],j+1);

can_traverse.append("(can_traverse rover0 waypoint");
can_traverse.append(std::to_string(indexi[0]*rectangle_large+indexi[1]));
can_traverse.append(" waypoint"); 
can_traverse.append(std::to_string(unique_waypoint_id));
can_traverse.append(")\n");

can_traverse.append("(can_traverse rover0 waypoint");
can_traverse.append(std::to_string(unique_waypoint_id));
can_traverse.append(" waypoint"); 
can_traverse.append(std::to_string(indexi[0]*rectangle_large+indexi[1]));
can_traverse.append(")\n");


for (int adding=1;adding!=how_many_more;adding++){
unsigned long long int unique_waypoint_new=pow(primary_numbers[adding],i+1)*pow(primary_numbers[adding+1],j+1);

if(adding>=drone_more){
waypoints.append("waypoint");
waypoints.append(std::to_string(unique_waypoint_new));
waypoints.append(" - waypoint\n");}

can_traverse.append("(can_traverse rover0 waypoint");
can_traverse.append(std::to_string(unique_waypoint_id));
can_traverse.append(" waypoint"); 
can_traverse.append(std::to_string(unique_waypoint_new));
can_traverse.append(")\n");

can_traverse.append("(can_traverse rover0 waypoint");
can_traverse.append(std::to_string(unique_waypoint_new));
can_traverse.append(" waypoint"); 
can_traverse.append(std::to_string(unique_waypoint_id));
can_traverse.append(")\n");
unique_waypoint_id=unique_waypoint_new;
}

can_traverse.append("(can_traverse rover0 waypoint");
can_traverse.append(std::to_string(unique_waypoint_id));
can_traverse.append(" waypoint"); 
can_traverse.append(std::to_string(indexj[0]*rectangle_large+indexj[1]));
can_traverse.append(")\n");

can_traverse.append("(can_traverse rover0 waypoint");
can_traverse.append(std::to_string(indexj[0]*rectangle_large+indexj[1]));
can_traverse.append(" waypoint"); 
can_traverse.append(std::to_string(unique_waypoint_id));
can_traverse.append(")\n");

}
//or if the rover can immediatly traverse from i to j
if (distance_ij_rover_normalized<=2){
can_traverse.append("(can_traverse rover0 waypoint");
can_traverse.append(std::to_string(indexi[0]*rectangle_large+indexi[1]));
can_traverse.append(" waypoint"); 
can_traverse.append(std::to_string(indexj[0]*rectangle_large+indexj[1]));
can_traverse.append(")\n");

can_traverse.append("(can_traverse rover0 waypoint");
can_traverse.append(std::to_string(indexj[0]*rectangle_large+indexj[1]));
can_traverse.append(" waypoint"); 
can_traverse.append(std::to_string(indexi[0]*rectangle_large+indexi[1]));
can_traverse.append(")\n");
}

}}



can_traverse.append("\n");
can_traverse_drone.append("\n");

//Check if a waypoint is visible from an adjacent cell (for the rover, old consensus)
/*
//Check if on border south
if(index[0]!=0){
if(matrix[index[0]-1][index[1]]==0){
visible_objectives_rover.append("(visible_from objective");
visible_objectives_rover.append(std::to_string(ID_v[i]));
visible_objectives_rover.append(" waypoint");
visible_objectives_rover.append(std::to_string((index[0]-1)*rectangle_large+index[1]));
visible_objectives_rover.append(" rover0)\n");
}}
//Check if on border north
if(index[0]!=rectangle_haut-1){
if(matrix[index[0]+1][index[1]]==0){
visible_objectives_rover.append("(visible_from objective");
visible_objectives_rover.append(std::to_string(ID_v[i]));
visible_objectives_rover.append(" waypoint");
visible_objectives_rover.append(std::to_string((index[0]+1)*rectangle_large+index[1]));
visible_objectives_rover.append(" rover0)\n");

}}
//Check if on border west
if(index[1]!=0){
if(matrix[index[0]][index[1]-1]==0){
visible_objectives_rover.append("(visible_from objective");
visible_objectives_rover.append(std::to_string(ID_v[i]));
visible_objectives_rover.append(" waypoint");
visible_objectives_rover.append(std::to_string((index[0])*rectangle_large+index[1]-1));
visible_objectives_rover.append(" rover0)\n");

}}
//Check if on border east
if(index[1]!=rectangle_large-1){
if(matrix[index[0]][index[1]+1]==0){
visible_objectives_rover.append("(visible_from objective");
visible_objectives_rover.append(std::to_string(ID_v[i]));
visible_objectives_rover.append(" waypoint");
visible_objectives_rover.append(std::to_string((index[0])*rectangle_large+index[1]+1));
visible_objectives_rover.append(" rover0)\n");

}}
*/

objectives.append("\n");
waypoints.append("\n");

  
std::string mapplannertxtdatafile = req.file_out + ".txt";
 ROS_INFO("Writing waypoints data to %s", mapplannertxtdatafile.c_str());
      FILE* txt = fopen(mapplannertxtdatafile.c_str(), "w");
      if (!txt)
      {
        std::cout<< "Couldn't write on %s", mapplannertxtdatafile.c_str();
        return false;
      }
 fprintf(txt,"%s",waypoints.c_str());
 fprintf(txt,"%s",objectives.c_str());
 fprintf(txt,"%s",can_traverse.c_str());
 fprintf(txt,"%s",can_traverse_drone.c_str());
 fprintf(txt,"%s",visible_objectives_rover.c_str());
 fprintf(txt,"%s",visible_objectives_drone.c_str());

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
