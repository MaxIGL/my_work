#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>


int
 main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

// Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read ("/home/dcas/m.dreier/Documents/PCD_FILES/Test3/Laser/Outside_raw_banc_laser.pcd", *cloud);

pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.06);
    outrem.setMinNeighborsInRadius (4);
    outrem.setKeepOrganized(false);
    // apply filter
    outrem.filter (*cloud_filtered);

pcl::PCDWriter writer;
  writer.write ("/home/dcas/m.dreier/Documents/PCD_FILES/Test3/Laser/Outside_banc_radiusx1.pcd", *cloud_filtered, false);

return (0);
}
