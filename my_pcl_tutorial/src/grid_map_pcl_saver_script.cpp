
//Load from bag
#include "grid_map_core/GridMap.hpp"
#include <grid_map_msgs/GridMap.h>

#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_ros/GridMapMsgHelpers.hpp"
#include <grid_map_cv/grid_map_cv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv)
{

//Load From Bag
const std::string pathToBag = "/home/dcas/m.dreier/Documents/PCD_FILES/Test2/Lidar/elevation_map.bag";
const std::string topic= "grid_map";
grid_map::GridMap gridMap;

grid_map::GridMapRosConverter::loadFromBag( pathToBag, topic, gridMap);
const std::string pointLayer ="elevation";
//Transform Grid to PC2
sensor_msgs::PointCloud2 cloud_grid_PC2;
grid_map::GridMapRosConverter::toPointCloud (gridMap, gridMap.getLayers(), pointLayer, cloud_grid_PC2);

//Partie Transform PC2 TO PCD
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_grid (new pcl::PointCloud<pcl::PointXYZ>);
pcl::fromROSMsg (cloud_grid_PC2,*cloud_grid);

// Save PCD to .PCD file
pcl::PCDWriter writer;
writer.write ("/home/dcas/m.dreier/Documents/PCD_FILES/Test2/Lidar/grid_map_lidar_in_script.pcd", *cloud_grid, false);

return(0);
}




