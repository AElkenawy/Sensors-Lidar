// PCL lib Functions for processing point clouds 

/* Templates allow to write the function once and use the template
   like an argument
 * PointT is a template parameter, which holds the place for different 
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
   pcl::PointCloud<PointT> is a type, not a value (cloud) */
// typename will be always used with extension ::Ptr of Pointer 

//*################ VOXEL AND FILTERING FUNCTION (FilterCloud Method of ProcessPointClouds class) ################*//
// Downsample the input cloud and then removes the roof area
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

  // Voxel grid class template filter
  pcl::VoxelGrid<PointT> vg;
  typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>); //memory allocation of new variable cloudFiltered
  vg.setInputCloud (cloud); // Public Member Function inherited from pcl::PCLBase <PointT> 
  vg.setLeafSize (filterRes, filterRes, filterRes);
  
  // Pass pointer to apply changes to cloudFiltered
  vg.filter(*cloudFiltered); //Calls the filtering method and returns the filtered dataset in output?

  typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

  // CropBox is a filter that allows the user to filter all the data inside of a given box. 
  // allocation of cloudFilter region
  // pcl::CropBox< PointT>::CropBox  (bool extract_removed_indices = false)   
  // extract_removed_indices  Set to true if you want to be able to extract the indices of points being removed
  // true is the class constructor argument, used here to manipulate points inside the CropBox 
  pcl::CropBox<PointT> region(true); 

  // ROI points
  region.setMin(minPoint);
  region.setMax(maxPoint);
  
  region.setInputCloud (cloudFiltered);
  region.filter(*cloudRegion);

  // Removing car's roof points 

  // vector to store roof points indices
  std::vector <int> indices;

  pcl::CropBox<PointT> roof(true);

  // roof dimensions (determined using renderBox)
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));

  roof.setInputCloud (cloudRegion);

  // indices inside roof's box
  roof.filter(indices);

  //pcl::PointIndices::Ptr struct
  pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

  // Store roof indices inside inliers 
  for(int point : indices)
    inliers->indices.push_back(point);

  // pcl::ExtractIndices< PointT > Class Template
  // Similar to segmentation SeparateClouds function
  pcl::ExtractIndices<PointT> extract;

  // Inliers extract
  extract.setInputCloud (cloudRegion);
  extract.setIndices (inliers);
  
  // Set whether the regular conditions for points filtering should apply, or the inverted conditions. 
  // setNegative(true): inverted behavior of extraction to remove inliers
  extract.setNegative (true);
  extract.filter(*cloudRegion);


  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  return cloudRegion;
}

//*################ CLOUDS GENERATION/SEPARATION FUNCTION ################*//
/* Generate two point clouds using inliers 
 * One cloud of obstacles (non-plane points) and the other of 
   segmented plane (plane/inliers points)
 * Inliers are extracted using SegmentPlane function (RANSAC)
 * SeparateClouds is a part of SegmentPlane
 */

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());


  /* ### PLANE CLOUD GENERATION ### */
  /* Inliers can be added to the plane by looping over inlier indices
  and pushing the corresponding inlier point into the plane cloud's 
  point vector */

  // Road points are inliers
  /* inliers (output of SegmentPlane function represent all the indices of the points that
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
  
  // setNegative(true): inverted behavior of extraction to remove inliers/road plane points
  extract.setNegative (true);

  // obstCloud is a pointer, dereference it when passing to filter
  extract.filter (*obstCloud);

  /* ### RETURN BOTH GENERATED CLOUDS ### */

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
  return segResult;
}

//*################ RANSAC SEGMENTATION FUNCTION ################*//
/* SegmentPlane Function: fits a plane to (road) points and uses distance
   tolerance to decide which points (inliers) belong to that plane 
 * SegmentPlane returns a std::pair (stored in segResult variable) 
 * pair object holds pair of PCs: obstacle & road */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
  // Time segmentation process

  /* Timer to check how long segmentation takes 
  (not useful if takes long) for a real self-driving car */
  auto startTime = std::chrono::steady_clock::now();

  // /*## RANSAC USING PCL'S BUILT IN FUNCTION ##*/

  // // Create the segmentation object (with PointT type to handle any PC type)
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

  // /*## PCL'S BUILT IN RANSAC END ##*/


  /*## IMPLEMENTED RANSAC FUNCTION (quiz/ransac/ransac2d.cpp) ##*/

  // Store the highest number of inliers in inliersResult
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

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

    // Plane coefficients 

    /* take (x1,y1,z1) as a reference to define two vectors v1, v2
     * v1 = <x2-x1, y2-y1, z2-z1> , v2 = <x3-x1, y3-y1, z3-z1> */
    // cross product of v1xv2=<i,j,k>, to find normal vector to the plane
    // cross product coefficients
    float i,j,k;

    i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
    j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
    k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);

    // plane (RANSAC) coefficients
    float A, B, C, D;

    A = i;
    B = j;
    C = k;
    D = -(i*x1 + j*y1 + k*z1);


    /*## Measure distance between every point and fitted plane ##*/

    // Looping through cloud points 
    for(int index = 0; index < cloud->points.size(); index++){
      
      // if this point is one of the 3 points of (testing) plane, skip it
      if(inliers.count(index)>0)
        continue;

      PointT point = cloud->points[index];

      float x4 = point.x;
      float y4 = point.y;
      float z4 = point.z;

      // calculate plane-point distance
      float d = fabs(A*x4 + B*y4 + C*z4 + D)/sqrt(A*A+B*B+C*C);

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

  for (int idx : inliersResult)
    inliers->indices.push_back(idx);

  /* SeparateClouds is used inside here (SegmentPlane) 
     after inliers is calculated as input with input cloud*/

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

  return segResult;
}

/*## PCL'S BUILT IN EUCLIDEAN CLUSTERING FUNCTION ##*/
// perform Euclidean Clustering to group detected obstacles
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  /* Clustering hyperpatameters */

  // Distance tolerance to group points together (total object distance range)
  ec.setClusterTolerance (clusterTolerance); // [m]

  // Min (avoid noise) & Max (separate large) points to form a cluster 
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

/*## IMPLEMENTED EUCLIDEAN CLUSTERING FUNCTION ##*/
/* Consists of 3 parts:
 * 1) euclideanCluster: instantiates clusters, points queue and calls clusterHelper
   for not processed points
 * 2) clusterHelper: finds neasrest points to target point (tree->search) necessary 
   for clusters construction
 * 3) completeCluster: Inserts points to the tree (tree->insert), calls euclideanCluster and renders
   clusters in Bounding boxes
 * tree methods are included from kdtree.h
  */

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

  // Clusters Ids
  std::vector<std::vector<int>> clusters;
  
  // Processed points queue
  std::vector <bool> processed (points.size(), false);
  
  int j = 0;
  
  while (j < points.size()){
    
    // Skip already processed point
    if (processed[j]){
      j++; continue;
    }
    
    std::vector<int> cluster;
    clusterHelper (j, points, cluster, processed, tree, distanceTol);
    clusters.push_back(cluster);
    j++;
  }
  
  return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper (int index, const std::vector<std::vector<float>> points, std::vector<int> &cluster, std::vector <bool> &processed, KdTree* tree, float distanceTol){
  
  // Mark current point as processed
  processed[index] = true;
  
  // Instantiate the cluster with this point
  cluster.push_back(index);
  
  // list of nearby points using search method
  std::vector<int> nearest = tree->search (points[index], distanceTol);
  
  // Iterate through nearby points (recursively with clusterHelper)
  for (int id : nearest){
    if(!processed[id])
      clusterHelper(id, points, cluster, processed, tree, distanceTol);
  }
}

template<typename PointT>
void ProcessPointClouds<PointT>::completeCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<std::vector<int>> clusters;

    // Creating the KdTree object
    KdTree* tree = new KdTree;
    
    // Store points of Point Cloud in a vector of points/vectors
    std::vector<std::vector<float>> points;
      
    // constructing KD-tree
    for(int i = 0; i < cloud->points.size(); i++)
    {
      // Store PC current cloud point in PointT variable
      PointT current_point = cloud->points[i];
      
      points.push_back({current_point.x, current_point.y, current_point.z});
      tree->insert(points[i], i);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    
    // clusters are points (with specific indices) inside cloud
    clusters = euclideanCluster(points, tree, clusterTolerance);
    std::cout << "Number of found clusters: " << clusters.size() << std::endl;
  
  // Enabling bounding box
  int render_box = 1;
  
  // Render clusters
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,1), Color(0,0,1)};
  for(std::vector<int> cluster : clusters)
  {
    typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
    for(int indice : cluster)
      clusterCloud->points.push_back(cloud->points[indice]);
    
    renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
    
    if (render_box){
      // Creating Bounding box object (for clusters)
      Box box = BoundingBox(clusterCloud);
      
      // Rendering Bounding boxes to the viewer
      renderBox(viewer,box,clusterId);
    }    
    ++clusterId;
  }
  
  if(clusters.size()==0)
    renderPointCloud(viewer,cloud,"data");
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
      PCL_ERROR ("Couldn't read file \n");

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