#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
 
int
main(int argc, char** argv)
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	// Object for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing both the points and the normals.
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals(new pcl::PointCloud<pcl::PointNormal>);
 
	// Read in the cloud data
	reader.read ("Cup_SDU.pcd", *cloud);
	std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;
 
	// Normal estimation.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
 
	// The triangulation object requires the points and normals to be stored in the same structure.
	pcl::concatenateFields(*cloud, *normals, *cloudNormals);
	// Tree object for searches in this new object.
	pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree2(new pcl::search::KdTree<pcl::PointNormal>);
	kdtree2->setInputCloud(cloudNormals);
 
	// Triangulation object.
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> triangulation;
	// Output object, containing the mesh.
	pcl::PolygonMesh triangles;
	// Maximum distance between connected points (maximum edge length).
	triangulation.setSearchRadius(5);
	// Maximum acceptable distance for a point to be considered,
	// relative to the distance of the nearest point.
	triangulation.setMu(10);
	// How many neighbors are searched for.
	triangulation.setMaximumNearestNeighbors(100);
	// Points will not be connected to the current point
	// if their normals deviate more than the specified angle.
	triangulation.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees.
	// If false, the direction of normals will not be taken into account
	// when computing the angle between them.
	triangulation.setNormalConsistency(false);
	// Minimum and maximum angle there can be in a triangle.
	// The first is not guaranteed, the second is.
	triangulation.setMinimumAngle(M_PI / 18); // 10 degrees.
	triangulation.setMaximumAngle(2 * M_PI / 3); // 120 degrees.
 
	// Triangulate the cloud.
	triangulation.setInputCloud(cloudNormals);
	triangulation.setSearchMethod(kdtree2);
	triangulation.reconstruct(triangles);
 
	// Save to disk.
	pcl::io::saveVTKFile("mesh.vtk", triangles);
 
	return 0;
}
