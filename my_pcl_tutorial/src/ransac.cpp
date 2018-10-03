#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>

using namespace std;

#define PI 3.14159265

struct points {
	float x;
	float y;
	float z;
	float rgb;
};

namespace pcl {
template<>
struct SIFTKeypointFieldSelector<PointXYZ> {
	inline float operator ()(const PointXYZ &p) const {
		return p.z;
	}
};
}

bool notCollinear(points A, points B, points C) {
	points AC = { (C.x - A.x), (C.y - A.y), (C.z - A.z) }; // Gives the vector AC
	points AB = { (B.x - A.x), (B.y - A.y), (B.z - A.z) }; // Gives the vector AB
	points CP = { ((AB.y * AC.z) - (AB.z * AC.y)), ((AB.z * AC.x)
			- (AB.x * AC.z)), ((AB.x * AC.y) - (AB.y * AC.x)) }; // Gives the crossproduct of vector AB and AC
	if (CP.x != 0 && CP.y != 0 && CP.z != 0) // Checks if they are on the same line or not
		return true; // Returns true if they are not on the same line
	else
		return false; // Returns false if they are on the same line
}

bool notCollinear2(vector<points> samples) {
	points AC = { (samples.at(2).x - samples.at(0).x), (samples.at(2).y
			- samples.at(0).y), (samples.at(2).z - samples.at(0).z) }; // Gives the vector AC
	points AB = { (samples.at(1).x - samples.at(0).x), (samples.at(1).y
			- samples.at(0).y), (samples.at(1).z - samples.at(0).z) }; // Gives the vector AB
	points CP = { ((AB.y * AC.z) - (AB.z * AC.y)), ((AB.z * AC.x)
			- (AB.x * AC.z)), ((AB.x * AC.y) - (AB.y * AC.x)) }; // Gives the crossproduct of vector AB and AC
	if (CP.x != 0 && CP.y != 0 && CP.z != 0) // Checks if they are on the same line or not
		return true; // Returns true if they are not on the same line
	else
		return false; // Returns false if they are on the same line
}

float angle(points pt1, points pt2, points refpoint) {
	float dx1 = pt1.x - refpoint.x;
	float dy1 = pt1.y - refpoint.y;
	float dz1 = pt1.z - refpoint.z;
	float dx2 = refpoint.x - pt2.x;
	float dy2 = refpoint.y - pt2.y;
	float dz2 = refpoint.z - pt2.z;
	float v1[] = { dx1, dy1, dz1 };
	float v2[] = { dx2, dy2, dz2 };
	float v1mag = sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]);
	float v2mag = sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2]);
	float dotp = (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]);
	float result = acos(dotp / (v1mag * v2mag));
	float angle = result;
	return angle;
}
pcl::PointCloud<pcl::PointWithScale> SIFT_comp(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
// Parameters for sift computation
	const float min_scale = 0.01f;
	const int n_octaves = 6;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.01f;

// Estimate the sift interest points using z values from xyz as the Intensity variants
	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr Kdtree(
			new pcl::search::KdTree<pcl::PointXYZRGB>());
	sift.setSearchMethod(Kdtree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud);
	sift.compute(result);

	return result;
}

void Spin_Image(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out){

}

void Ransac(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object) {

	int iterations = 150;
	int K = 10;
	double t = 0.0005;
	double d_frac = 0.25;
	float radius = 1.0;

	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud_scene);
	vector<int> pointIdxNKNSearch(K);
	vector<float> pointNKNSquaredDistance(K);
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;

	vector<pcl::PointXYZRGB> sps; // searchPoint samples
	vector<pcl::PointXYZRGB> spv; // searchPoint vector
	int pointcloud_size = cloud_scene->width * cloud_scene->height;
	cout << "Number of points in the cloud: " << pointcloud_size << endl;
	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud_scene->begin();
			it != cloud_scene->end(); it++) {

		pcl::PointXYZRGB sp;
		sp.x = it->x;
		sp.y = it->y;
		sp.z = it->z;
		sp.rgb = it->rgb;
		spv.push_back(sp);

	}
	for (int maxSample = 0; maxSample < 3; maxSample++) {
		int number = rand() % pointcloud_size;
		sps.push_back(spv.at(number));
		cout << "The number is: " << number << endl;
	}
	pcl::PointCloud<pcl::PointWithScale> features_scene = SIFT_comp(
			cloud_scene);
	pcl::PointCloud<pcl::PointWithScale> features_object = SIFT_comp(
			cloud_object);
	for (int i = 0; i < sps.size(); i++) {
		if (kdtree.nearestKSearch(sps.at(i), K, pointIdxNKNSearch,
				pointNKNSquaredDistance) > 0) {
			for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
				cout << " " << cloud_scene->points[pointIdxNKNSearch[i]].x
						<< " " << cloud_scene->points[pointIdxNKNSearch[i]].y
						<< " " << cloud_scene->points[pointIdxNKNSearch[i]].z
						<< " (squared distance: " << pointNKNSquaredDistance[i]
						<< ")" << endl;
		}
	}
}

int main(int argc, char *argv[]) {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("Scene_Cup_1.pcd", *cloud)
			== -1) {
		PCL_ERROR("Couldn't read file Scene_Cup_1.pcd \n");
		return (-1);
	}

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("test2.pcd", *cloud3) == -1) {
		PCL_ERROR("Couldn't read file test2.pcd \n");
		return (-1);
	}
	vector<int> indices;
	vector<int> indices2;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	pcl::removeNaNFromPointCloud(*cloud2, *cloud2, indices);
	pcl::VoxelGrid<pcl::PointXYZRGB> vox;
	vox.setInputCloud(cloud);
	vox.setLeafSize(0.05f, 0.05f, 0.05f);
	vox.filter(*cloud_filtered);
	pcl::VoxelGrid<pcl::PointXYZRGB> vox2;
	vox.setInputCloud(cloud2);
	vox.setLeafSize(0.05, 0.05f, 0.05f);
	vox.filter(*cloud_filtered2);
	vector<points> pclp;
	vector<points> samples;
	int pointcloud_size = cloud_filtered->width * cloud_filtered->height;
	cout << "Number of points in the cloud: " << pointcloud_size << endl;
	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it =
			cloud_filtered->begin(); it != cloud_filtered->end(); it++) {

		points poinT;
		poinT.x = it->x;
		poinT.y = it->y;
		poinT.z = it->z;
		poinT.rgb = it->rgb;
		pclp.push_back(poinT);

	}
	for (int maxSample = 0; maxSample < 20; maxSample++) {
		int number = rand() % pointcloud_size;
		samples.push_back(pclp.at(number));
		cout << "The number is: " << number << endl;
	}

	for (vector<points>::iterator it = samples.begin(); it != samples.end();
			it++) {
		cout << "The samples are: " << "X: " << it->x << ", " << " Y: " << it->y
				<< ", " << " Z: " << it->z << ", " << " RGB: " << it->rgb
				<< endl;
	}
	cout << "Size: " << samples.size() << endl;

	if (notCollinear(samples.at(0), samples.at(1), samples.at(2)))
		cout << "Not on the same line" << endl;
	else
		cout << "On the same line" << endl;
	// If the points are on the same line, we need to add something that will find another point
	// to use, and then repeat it, until they are not on the same line

	if (notCollinear2(samples))
		cout << "Not on the same line2" << endl;
	else
		cout << "On the same line2" << endl;

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne2;
	ne.setInputCloud(cloud_filtered);
	ne2.setInputCloud(cloud_filtered2);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZRGB>());
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2(
			new pcl::search::KdTree<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
			new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(
			new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.025);
	ne2.setRadiusSearch(0.025);
	ne.compute(*cloud_normals);
	ne2.compute(*cloud_normals2);

	// Parameters for sift computation
	const float min_scale = 0.01f;
	const int n_octaves = 6;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.01f;

	// Estimate the sift interest points using z values from xyz as the Intensity variants
	pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr Kdtree(
			new pcl::search::KdTree<pcl::PointXYZ>());
	sift.setSearchMethod(Kdtree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud3);
	sift.compute(result);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr test(
			new pcl::PointCloud<pcl::PointXYZRGB>);
	cout << "Amount of features: " << result.points.size() << endl;
	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift2;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr Kdtree2(
			new pcl::search::KdTree<pcl::PointXYZRGB>());
	sift2.setSearchMethod(Kdtree2);
	sift2.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift2.setMinimumContrast(min_contrast);
	sift2.setInputCloud(cloud);
	sift2.compute(result);
	cout << "Amount of features in full scene: " << result.points.size()
			<< endl;

	return 0;
}
