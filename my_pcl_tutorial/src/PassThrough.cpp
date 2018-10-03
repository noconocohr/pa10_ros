#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int
 main (int argc, char** argv)
{
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3 (new pcl::PointCloud<pcl::PointXYZ>);

  reader.read ("Cup_real_scale_z_inverted.pcd", *cloud);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.1, 0.1);
  pass.filter (*cloud_filtered2);
  pass.setInputCloud (cloud_filtered2);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.1, 0.1);
  pass.filter (*cloud_filtered3);
  pass.setInputCloud (cloud_filtered3);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.7, 0.1);
  pass.filter (*cloud_filtered);

  writer.write("Cup_real_scale_z_inverted.pcd", *cloud_filtered, false);

  return(0);
}
