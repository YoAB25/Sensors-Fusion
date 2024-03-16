/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#ifndef BOOST_ALL_DYN_LINK
#   define BOOST_ALL_DYN_LINK
#endif 
#include <boost/filesystem.hpp>


#include <thread>
#include <eigen3/Eigen/Dense>
#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

// A useless helper function
// std::pair<int, int> generateRandomIndices(int cloudSize){
// 	int index1 = rand() % cloudSize;
// 	int index2 = rand() % cloudSize;
// 	while (index2 == index1) {
//         index2 = rand() % cloudSize; // Make sure index2 is different from index1
//     }
// 	return std::make_pair(index1, index2);
// }



std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// Start the timer : We'll used later to see how much time RANSAC Algo takes
	auto startTime = std::chrono::steady_clock::now();
	// Initialize the set to hold the inliers found : Inliers are points of interest within the plane/line
	std::unordered_set<int> inliersResult;
	// Seed the random number generator
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	while(maxIterations--){
		// Initialize a set to hold the inliers for the current model
		std::unordered_set<int> inliers;
		//std::pair<int, int> pointPairs = generateRandomIndices(cloud->size());
		while(inliers.size()<2)
			// Randomly select two points (indices) to form the initial model
			inliers.insert(rand()%(cloud->points.size()));

		float x1,y1,x2,y2;

		// Extract the coordinates of the two randomly selected points
		auto itr = inliers.begin(); 		// This a pointer where the list of inliers begins

		x1 = cloud->points[*itr].x;			// Extract the first point : Dereferencing the itr in order to get the indice
		y1 = cloud->points[*itr].y;
		itr++;								// Move to the next one
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		// Here are the coefficients of the line : A*X+B*Y+C = 0
		float A = (y1-y2);
		float B = (x2-x1);
		float C = x1*y2-y1*x2;
		// We iterate over all points in order to find inliers
		for (int index=0; index < cloud->points.size(); index++){
			if(inliers.count(index)>0)
				continue;
			// A regular point from the point cloud
			pcl::PointXYZ point = cloud->points[index];
			// Extract coordinates
			float X = point.x;
			float Y = point.y;
			// Calculate distance using a given formula
			float d = fabs(A*X+B*Y+C)/sqrt(A*A+B*B);
			// Compare the calculated distance to the tolerance one
			if(d <=distanceTol){
				inliers.insert(index);
			}

		}
		if(inliers.size()> inliersResult.size())
			inliersResult = inliers;
		
	}
	// Calculate the elapsed time : Algorithm efficiency
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransam took " << elapsedTime.count() << " ms" << std::endl;

	return inliersResult;

}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// Start the timer : We'll used later to see how much time RANSAC Algo takes
	auto startTime = std::chrono::steady_clock::now();
	// Initialize the set to hold the inliers found : Inliers are points of interest within the plane/line
	std::unordered_set<int> inliersResult;
	// Seed the random number generator
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	while(maxIterations--){
		// Initialize a set to hold the inliers for the current model
		std::unordered_set<int> inliers;
		//std::pair<int, int> pointPairs = generateRandomIndices(cloud->size());
		while(inliers.size()<3)
			// Randomly select two points (indices) to form the initial model
			inliers.insert(rand()%(cloud->points.size()));

		float x1,y1,z1,x2,y2,z2,x3,y3,z3;

		// Extract the coordinates of the two randomly selected points
		auto itr = inliers.begin(); 		// This a pointer where the list of inliers begins

		// Coordinates
		x1 = cloud->points[*itr].x;			// Extract the first point : Dereferencing the itr in order to get the indice
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;								// Move to the next one
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;								// Move to the next one
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		// Vectors
		Eigen::Vector3d V1(x2-x1,y2-y1,z2-z1);
		Eigen::Vector3d V2(x3-x1,y3-y1,z3-z1);
		// Product Cross
		Eigen::Vector3d V1V2 = V1.cross(V2);

		// Here are the coefficients of the line : A*X+B*Y+C*Z+D = 0
		float A = V1V2.x();
		float B = V1V2.y();
		float C = V1V2.z();
		float D = -(A*x1+B*y1+C*z1); 

		// We iterate over all points in order to find inliers
		for (int index=0; index < cloud->points.size(); index++){
			if(inliers.count(index)>0)
				continue;
			// A regular point from the point cloud
			pcl::PointXYZ point = cloud->points[index];
			// Extract coordinates
			float X = point.x;
			float Y = point.y;
			float Z = point.z;
			// Calculate distance using a given formula
			float d = fabs(A*X+B*Y+C*Z+D)/sqrt(A*A+B*B+C*C);
			// Compare the calculated distance to the tolerance one
			if(d <=distanceTol){
				inliers.insert(index);
			}

		}
		if(inliers.size()> inliersResult.size())
			inliersResult = inliers;
		
	}
	// Calculate the elapsed time : Algorithm efficiency
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransam took " << elapsedTime.count() << " ms" << std::endl;

	return inliersResult;

}












int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2D = CreateData();
	// Create 3D data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spin ();
  	}
  	
}



	// for(int iteration; iteration <maxIterations; iteration++){
	// 	std::pair<int, int> pointPairs = generateRandomIndices(cloud->size());
	// 	// Retreive the point indices
	// 	int indexPoint1 = pointPairs.first;
	// 	int indexPoint2 = pointPairs.second;
	// 	// Retreive the points coordinates
	// 	pcl::PointXYZ Point1 = cloud->points[indexPoint1];
	// 	pcl::PointXYZ Point2 = cloud->points[indexPoint2];
	// 	// Line equation
	// 	(Point1.y - Point2.y)*


	// }