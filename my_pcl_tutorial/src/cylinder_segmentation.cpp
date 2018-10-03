#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/mls.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZRGB PointT;

int
main (int argc, char** argv)
{
  // All the objects needed
  pcl::PCDReader reader;
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr kdtree (new pcl::search::KdTree<PointT> ());

  // Datasets
  double z1, z2, x1, x2, y1, y2;
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered3 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered_cylinder (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered_extract (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered_smoothed (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_cylinder (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  // Read in the cloud data
  reader.read ("Cup_11.pcd", *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  pcl::console::parse_argument (argc, argv, "--z1", z1);
  pcl::console::parse_argument (argc, argv, "--z2", z2);
  pcl::console::parse_argument (argc, argv, "--x1", x1);
  pcl::console::parse_argument (argc, argv, "--x2", x2);
  pcl::console::parse_argument (argc, argv, "--y1", y1);
  pcl::console::parse_argument (argc, argv, "--y2", y2);

  // Build a passthrough filter to remove spurious NaNs
  /*pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z1, z2);
  pass.filter (*cloud_filtered);
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(x1,x2);
  pass.filter (*cloud_filtered2);
  pass.setInputCloud (cloud_filtered2);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(y1,y2);
  pass.filter (*cloud_filtered3);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;
  writer.write("Cup_1_passthrough.pcd", *cloud_filtered, false);*/

  // Smoothing object (we choose what point types we want as output).
  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> filter;
  filter.setInputCloud(cloud_filtered3);
  // Use all neighbors in a radius of 3cm.
  filter.setSearchRadius(0.03);
  // If true, the surface and normal are approximated using a polynomial estimation
  // (if false, only a tangent one).
  filter.setPolynomialFit(true);
  // We can tell the algorithm to also compute smoothed normals (optional).
  filter.setComputeNormals(true);
  // kd-tree object for performing searches.
  filter.setSearchMethod(kdtree);

  filter.process(*smoothedCloud);
  writer.write("Cup_11Table_closer_smoothed.pcd", *smoothedCloud, false);

  reader.read ("Cup_11Table_closer_smoothed.pcd", *cloud_filtered_smoothed);

  // Estimate point normals
  ne.setSearchMethod (kdtree);
  ne.setInputCloud (cloud_filtered_smoothed);
  ne.setRadiusSearch(0.03);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered_smoothed);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered_smoothed);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered_extract);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);
  writer.write("Cup_Table_closer_extract.pcd", *cloud_filtered_extract, false);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.25);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);
  seg.setInputCloud (cloud_filtered_cylinder);
  seg.setInputNormals (cloud_normals_cylinder);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered_cylinder);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
	  writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
  }

  // visualize normals
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.5);
  viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud_filtered_cylinder, cloud_normals_cylinder, 5, 0.01, "normals");

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return (0);
}
