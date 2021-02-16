#include <cmath>
#include <vector>




int main(int argc, char** argv)
{
//Initialization
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCDReader reader;
reader.read ("/home/dcas/m.dreier/Documents/PCD_FILES/Test2/D435/gridmappclDenoisedx1.pcd", *cloud);
int widht;
int height;
double origin_x;
double origin_y;
double resolution=0.03;

//Get Min and Max x and y and compute widht and height of the 2D map
float min_x = cloud->points[0].x, min_y = cloud->points[0].y, max_x = cloud->points[0].x, max_y = cloud->points[0].y,min_z=cloud->points[0].z,max_z=cloud->points[0].z;
for (size_t i = 1; i < cloud->points.size (); ++i){
       if(cloud->points[i].x <= min_x )
           min_x = cloud->points[i].x;
       else if(cloud->points[i].y <= min_y )
           min_y = cloud->points[i].y;
       else if(cloud->points[i].x >= max_x )
           max_x = cloud->points[i].x;
       else if(cloud->points[i].y >= max_y )
           max_y = cloud->points[i].y;
       else if(cloud->points[i].z <= min_z )
           min_z = cloud->points[i].z;
       else if(cloud->points[i].z >= max_z )
           max_z = cloud->points[i].z;
   }
origin_x=min_x;
origin_y=min_y;
//Dimensions in meter
float widht_float  = max_x - min_x;
float height_float = max_y - min_y;
//Number of cells necessary
float widht_scaled = widht_float  / resolution - 1/2;
float height_scaled= height_float / resolution - 1/2;
int widht  = int(widht_scaled)+2;
int heidht = int(height_scaled)+2; //Truncation to the lower int means we need to add 1 cell, and we want origin point to fall in the middle of the cell so we need to add 1 another cell. 


//Instantiate 2.5D map
std::vector<vector<float>> 25D_depth(widht*height);

//iteration on the point cloud to fill the 2D map

for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); it++) {
pcl::PointXYZ search_point;
search_point.x=it->x;
search_point.y=it->y;
search_point.z=it->z;
int i,j;
//Colonne
float cell_x= (search_point.x-origin_x)/resolution -1/2; //normalement compris entre k-1 et k
if (cell_x<=0) j=0;
else j= int(cell_x)+1;
//Ligne
float cell_y= (search_point.y-origin_y)/resolution -1/2; //normalement compris entre k-1 et k
if (cell_y<=0) i=0;
else i= int(cell_y)+1;

if(min_z <=0){
25D_depth[i*width+j].push_back(search_point.z-min_z);} //if some depths are negative, translate map to positive values
else 25D_depth[i*width+j].push_back(search_point.z);
}

std::vector<float> 2D_depth(widht*height);


//Assume that the 2.5D map is 1 layer thick. Further work later
//std::vector<std::string> trust(widht*height); //"A" for valid, "B" for invalid
//std::vector<float> 2D_depth(widht*height);
//Iteration sur la 2.5D map created 
//for (int num=0;num!=25D_depth.size();num++){
//if 25D_depth[num].size()==0 2D_depth[num]=-1; //The cell is unknown 
//float moyenne=0;

//std::sort (25D_depth[num].begin(),25D_depth[num].end());

//We compute the mean value of the column
//for (Iter=25D_depth[num].begin(); Iter!=25D_depth[num].end();Iter++){ 
//moyenne+=Iter;


//}
//moyenne=moyenne/25D_depth[num].size;

//We detect if there are some aberrant values in the column
//std::string trust_num="A";
//for (Iter=25D_depth[num].begin(); Iter!=25D_depth[num].end();Iter++){
//if (Iter<=moyenne){
//if(moyenne-Iter >= 0.02){
//trust_num="B";}}
//else if (Iter >=moyenne){
//if (Iter-moyenne >=0.02){
//trust_num="B";}}

//if (trust_num=="B"){

// , 
//2D_depth[num]=moyenne;
//}
//


//Convert 25D to 2D depth
for (int num=0;num!=25D_depth.size();num++){
std::sort (25D_depth[num].begin(),25D_depth[num].end());
if 25D_depth[num].size()==0 2D_depth[num]=-1; //The cell is unknown 
float moyenne=0;
float precedent=25D_depth[num].begin();
//We compute the mean value of the column
for (Iter=25D_depth[num].begin(); Iter!=25D_depth[num].end();Iter++){ 
moyenne+=Iter;
if Iter-precedent>=0.045 std::cout<< "Alerte au gogol !";
precedent=Iter;
}
2D_depth[num]=moyenne/25D_depth[num].size;
}

// compute the depth gradient and register obstacles into 2D list
std vector<int> occupancygridlist(widht*height);

//Iteration on horizontal neighbors
for (int j=0;j!=widht-1;j++){
for (int i=0;i!=height;i++){

//Check if the cell is unknown before computing depth gradient
if 2D_depth[j+widht*i]==-1 occupancygridlist[j+widht*i]=-1;
else if 2D_depth[j+widht*i+1]==-1 occupancygridlist[j+1+widht*i]=-1;
else if 2D_depth[j+widht*i]!=-1 and 2D_depth[j+1+widht*i]!=-1{
if(2D_depth[j+widht*i]-2D_depth[j+1+widht*i])>=0.07{ //Determine if the cell is the cell of an obstacle
occupancygridlist[j+widht*i]=100;
occupancygridlist[j+1+widht*i]=100; //Add obstacle and only obstacle on the grid
}}}}

//Iteration on vertical neighbors
for (int j=0;j!=widht;j++){
for (int i=0;i!=height-1;i++){
//Check if the cell is unknown before computing depth gradient
if 2D_depth[j+widht*i]==-1 occupancygridlist[j+widht*i]=-1;
else if 2D_depth[j+widht*(i+1)]==-1 occupancygridlist[j+widht*(i+1)]=-1;
else if 2D_depth[j+widht*i]!=-1 and 2D_depth[j+widht*(i+1)]!=-1{
if(2D_depth[j+widht*i]-2D_depth[j+widht*(i+1)])>=0.07{ //Determine if the cell is the cell of an obstacle
occupancygridlist[j+widht*i]=100;
occupancygridlist[j+widht*(i+1)]=100; //Add obstacle and only obstacle on the grid
}}}}

//Iteration on diagonials
for (int j=0;j!=widht-1;j++){
for (int i=1;i!=height;i++){
//Check if the cell is unknown before computing depth gradient
if 2D_depth[j+widht*i]==-1 occupancygridlist[j+widht*i]=-1;
else if 2D_depth[j+1+widht*(i-1)]==-1 occupancygridlist[j+1+widht*(i-1)]=-1;
else if 2D_depth[j+widht*i]!=-1 and 2D_depth[j+1+widht*(i-1)]!=-1{
if(2D_depth[j+widht*i]-2D_depth[j+1+widht*(i-1)])>=0.07{ //Determine if the cell is the cell of an obstacle
occupancygridlist[j+widht*i]=100;
occupancygridlist[j+1+widht*(i-1)]=100; //Add obstacle and only obstacle on the grid
}}}}

//Iteration on anti-diagonals
for (int j=0;j!=widht-1;j++){
for (int i=0;i!=height-1;i++){
//Check if the cell is unknown before computing depth gradient
if 2D_depth[j+widht*i]==-1 occupancygridlist[j+widht*i]=-1;
else if 2D_depth[j+1+widht*(i+1)]==-1 occupancygridlist[j+1+widht*(i+1)]=-1;
else if 2D_depth[j+widht*i]!=-1 and 2D_depth[j+1+widht*(i+1)]!=-1{
if(2D_depth[j+widht*i]-2D_depth[j+1+widht*(i+1)])>=0.07{ //Determine if the cell is the cell of an obstacle
occupancygridlist[j+widht*i]=100;
occupancygridlist[j+1+widht*(i+1)]=100; //Add obstacle and only obstacle on the grid
}}}}

//Create the final sensor::msg occupancygrid 2D occupancy map

nav_msgs::OccupancyGrid occupancygrid;
occupancygrid.header.frame_id="map";
occupancygrid.info.resolution=resolution;
occupancygrid.info.widht=widht;
occupancyGrid.info.height=height;
occupancyGrid.info.origin.position.x=origin_x;
occupancyGrid.info.origin.position.y=origin_y;
occupancyGrid.info.origin.orientation.x = 0.0;
occupancyGrid.info.origin.orientation.y = 0.0;
occupancyGrid.info.origin.orientation.z = 0.0;
occupancyGrid.info.origin.orientation.w = 1.0;
occupancyGrid.data=occupancygridlist;


}
//http://www.cplusplus.com/reference/algorithm/max_element/
