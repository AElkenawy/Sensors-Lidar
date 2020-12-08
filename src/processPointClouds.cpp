// PCL lib Functions for processing point clouds 

/* Templates allow to write the function once and use the template
   like an argument*/

/* PointT is a template parameter, which holds the place for different 
   types of point clouds*/

#include "processPointClouds.h"
#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


// FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
/* the word typename tells the compiler that 
   pcl::PointCloud<PointT>::Ptr is a type not a value */
// typename will be always used when extension of pointer ::Ptr

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}

//SeparateClouds function to separate obstacles from inliers
template<typename PointT>

std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
//std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  
	/* TODO: Create two new point clouds, one cloud with obstacles 
	 (non-plane points) and other with segmented plane (plane points)*/
	typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());


	/* ### PLANE CLOUD GENERATION ### */
	/* Inliers can be added to the plane by looping over inlier indices
	and pushing the corresponding inlier point into the plane cloud's 
	point vector */
	
	// Road points are inliers
	/* inliers represents all the indices of the points that
	   belongs to the plane, that plane (planeCloud) is the road*/

	for (int index : inliers->indices){
		// points: cloud member of plane cloud's point vector
		planeCloud->points.push_back(cloud->points[index]);
	}

	/* ### OBSTACLE CLOUD GENERATION ### */

	/* Extraction: Subtracting the plane cloud from input cloud
	   so all points aren't inliers are kept, other removed*/

	// Create the filtering object
	pcl::ExtractIndices<PointT> extract;

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);

    // obstCloud is a pointer, dereference it when passing to filter
    extract.filter (*obstCloud);

    /* ### RETURN BOTH GENERATED CLOUDS ### */

    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

/* SegmentPlane Function: fits a plane to points and uses distance
   tolerance to decide whic points belong to thaat plane*/
// SegmentPlane returns a std::pair, which holds point cloud pointer types
// pair object holds pair of PCs: obstacle PC and road PC.
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process

    /* Timer to check how long segmentation takes 
    (not useful if takes long) for a real self-driving car */
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
	/* TODO:: Fill in this function to segment cloud into two parts
	   drivable plane and obstacles.*/

	
	// /*## RANSAC USING PCL BUILT-IN FUNCTION ##*/

	// // Create the segmentation object (with PointT type to handle any)
	// pcl::SACSegmentation<PointT> seg;
	// // Create inliers (to breakup the point cloud into two pieces)
	// pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
	// /* Defining (SACMODEL_PLANE) plane/plane coefficients. could be used 
	//    to render this plane in the viewer*/
	// pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
	// // Optional
	// seg.setOptimizeCoefficients (true);
	// // Mandatory
	// seg.setModelType (pcl::SACMODEL_PLANE);
	// // Do RANSAC on plane model
	// seg.setMethodType (pcl::SAC_RANSAC);
	// // RANSAC hyperparameters
	// seg.setMaxIterations (maxIterations);
	// seg.setDistanceThreshold (distanceThreshold);


	// // Segment the largest planar component from the remaining cloud

	// // Give the input cloud (last updated .PCD file)
	// seg.setInputCloud (cloud);
	// // generating inliers (passing by reference/dereferencing)
	// seg.segment (*inliers, *coefficients);

	// if (inliers->indices.size () == 0)
	// {
	//   std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	// }

    // /*## RANSAC END ##*/


    // /*## RANSAC USING MANUAL Ransac FUNCTION (same as in ransac2d.cpp) ##*/

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
			if (d <= distanceThreshold)
				inliers.insert(index);
		}


		// keep line with largest inliers as the best solution
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	/* Instantiate PointIndices variable (inliers) to store  
	 * inliers to match SeparateClouds function input type*/
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

	for (int idx : inliersResult){
		inliers->indices.push_back(idx);
	}


 	/* SeparateClouds is used inside here (SegmentPlane) 
 	   after inliers is calculated as input with input cloud*/

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    
    /*## Euclidean Clustering USING PCL BUILT-IN FUNCTION ##*/

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

	/* Clustering hyperpatameters */

	/* Distance tolerance to group points together 
	 * (total object distance range) */
	ec.setClusterTolerance (clusterTolerance); // [m]

	// Min (avoid noise) & Max (separte large) points to form a cluster 
	ec.setMinClusterSize (minSize);
	ec.setMaxClusterSize (maxSize);

	//KD-Tree structure to speed up look up time from O(n) to O(log(n))
	ec.setSearchMethod (tree);

	// Input cloud
	ec.setInputCloud (cloud);

	// Output clusters
	ec.extract (clusterIndices);

	// Create a Point cloud for each cluster (from cluster indices)
	for(pcl::PointIndices getIndices:clusterIndices){
		typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
		
		for(int index:getIndices.indices)
			cloudCluster->points.push_back (cloud->points[index]);

		// Set width and height of the cloud
		cloudCluster->width = cloudCluster->points.size();
		cloudCluster->height = 1;
		cloudCluster->is_dense = true;

		// clusters vector of point cloud to be returned by function
		clusters.push_back(cloudCluster);
	}





    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
// typedef boost::shared_ptr<PointCloud<PointT>> pcl::PointCloud<PointT>::Ptr
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}