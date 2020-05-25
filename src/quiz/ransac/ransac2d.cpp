/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

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



std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand((unsigned)time(NULL));
 	std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

		// Randomly sample two points 
		// Get the maximum number of possible indices.
	int max_inliers = 0;

	std::unordered_set<int> inliersIter;
	std::set<int> rand_set;
	// int arr[3];
	while (maxIterations > 0) {
		// cloud->points.size();
		// int firstInd = rand()%((cloud->points.size() - 1) + 1);
		// int secInd = rand()%((cloud->points.size() - 1) + 1);
		// int thirdInd = rand()%((cloud->points.size() - 1) + 1);
		// // cout << "first rand " << firstInd << " sec rand"  << secInd << "\n";
		// while (firstInd == secInd) {
		// 	secInd = rand()%((cloud->points.size() - 1) + 1);
		// }
		// int _i = 0;
			std::uniform_int_distribution<size_t> dis(0, cloud->points.size() - 1);

			while (rand_set.size() < 3) {
				// int rand = rand()%((cloud->points.size() - 1) + 1);
				rand_set.insert(dis(gen));
				// arr[_i] = 
			}
			// cout << rand_set.size() << "\n";
			std::set<int>::iterator it= rand_set.begin();
			
			// Get the points and get the coeffcients
			pcl::PointXYZ p1 = cloud->points[*it];
			it++;
			pcl::PointXYZ p2 = cloud->points[*it];
			it++;
			pcl::PointXYZ p3 = cloud->points[*it];

			// p1 is reference and v1 vector travels from p1 to p2.
			// vector p1 to p3

			// v1 x v2 = <i, j, k>
			// i = (y2 - y1)(z3 - z1) - (z2 - z1)(y3 - y1)
			// j = (z2 - z1)(x3 - x1) - (x2 - x1)(z3 - z1)
			// k = (x2 - x1)(y3 - y1) - (y2 - y1)(x3 - x1)
			// A = i, B = j, C = k, D = -(ix1 +j y1 + kz1), where A, B, C, D are coeffs
			// defining the plane Ax + By + Cz + D = 0.

			float A = (p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y);
			float B = (p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z);
			float C = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
			float D = -(A*(p1.x) + B*(p1.y) + C*(p1.z));
			float inliers = 0;
			for (int i = 0; i < cloud->points.size(); i++) {
				pcl::PointXYZ p = cloud->points[i];
				float d = std::abs(A*(p.x) + B*(p.y) + C*(p.z) + D)/(std::sqrt(A*A + B*B + C*C));
				if (d < distanceTol) {
					inliersIter.insert(i);
					inliers++;
				}
			}
			if (inliers > max_inliers) {
				max_inliers = inliers;
				// cout << max_inliers << "\n";
				inliersResult = inliersIter;
			} 
			inliersIter.clear();
			maxIterations--;
			// cout << maxIterations << "\n";
		}
	// }
	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10000, 0.5);

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
  	  viewer->spinOnce ();
  	}
  	
}
