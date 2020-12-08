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
	// Store the highest number of inliers in inliersResult
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	while (maxIterations--)
	{
		/*## Randomly sample subset and fit a plane ##*/

		//Randomly pick three points

		/* underorder_set: set means it won't contain the same index twice
		 * as elements have to be unique */

		
		std::unordered_set<int> inliers;

		// <3: to insert three points into inliers
		while (inliers.size() < 3)
			// use % to return values within cloud points range 
			inliers.insert(rand()%(cloud->points.size()));

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.begin();

		// dereference itr index triple to get the corres. points for plane
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;

		itr ++;

		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;	
		z2 = cloud->points[*itr].z;

		itr ++;

		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;	
		z3 = cloud->points[*itr].z;

		// Plane  coefficients 

		/* take (x1,y1,z1) as a reference to define two vectors v1, v2
		 * v1 = <x2-x1, y2-y1, z2-z1> , v2 = <x3-x1, y3-y1, z3-z1> */

		// cross product of v1xv2=<i,j,k>, to find normal vector to the plane

		// cross product coefficients
		float i,j,k;

		i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);

		//  plane coefficients
		float a, b, c, d;

		a = i;
		b = j;
		c = k;
		d = -(i*x1 + j*y1 + k*z1);


		/*## Measure distance between every point and fitted plane ##*/

		// Looping through cloud points 
		for(int index = 0; index < cloud->points.size(); index++){

			// if this point is one of the 3 points of (testing) plane, skip it
			if(inliers.count(index)>0)
				continue;


			pcl::PointXYZ point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;


			// calculate plane-point distance
			float d = fabs(a*x4 + b*y4 + c*z4 + d)/sqrt(a*a+b*b+c*c);

			/*## If distance is smaller than threshold count it as inlier ##*/
			// Add the inlier within distance tolerance
			if (d <= distanceTol)
				inliers.insert(index);
		}


		// keep line with largest inliers as the best solution
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// CreateData() >> changed to CreateData3D()
	// CreateData3D() simply loads clouds from simpleHighway.pcd
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);

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


	// Render 3D point cloud with inliers and outliers
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
