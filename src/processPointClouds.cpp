// PCL lib Functions for processing point clouds 

/* Templates allow to write the function once and use the template
   like an argument*/

/* PointT is a template parameter, which holds the place for different 
   types of point clouds*/

#include "processPointClouds.h"


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

	
	
	// Create the segmentation object (with PointT type to handle any)
	pcl::SACSegmentation<PointT> seg;
	// Create inliers (to breakup the point cloud into two pieces)
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
	/* Defining (SACMODEL_PLANE) plane/plane coefficients. could be used 
	   to render this plane in the viewer*/
	pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatorys
	seg.setModelType (pcl::SACMODEL_PLANE);
	// Do RANSAC on plane model
	seg.setMethodType (pcl::SAC_RANSAC);
	// RANSAC hyperparameters
	seg.setMaxIterations (maxIterations);
	seg.setDistanceThreshold (distanceThreshold);


	// Segment the largest planar component from the remaining cloud

	// Give the input cloud (last updated .PCD file)
	seg.setInputCloud (cloud);
	// generating inliers (passing by reference/dereferencing)
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
	  std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
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