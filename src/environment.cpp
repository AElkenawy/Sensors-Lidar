/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


// Viewer is used to handle all the graphics
// 3DViewer is passed as a reference again
std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

  // Creating different cars objects
  Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
  Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
  Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
  Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");


  // Create cars vector
  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if(renderScene)
  {
    // Render this highway to the viewer
    renderHighway(viewer);
    // Render this car to the viewer
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  
  // Instantiate point processor (Intensity) on the heap (for single frame testing)
  // ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  
  // Turned OFF to render Voxel filtered scene
  //renderPointCloud(viewer,inputCloud,"inputCloud");
  
  //*################ 1-VOXEL FILTERING ################*//
  // Downsampling number of points of cloud for efficient processing
  
  // FilterCloud(cloud, filterRes, minPoint, maxPoint)
  // minPoint, maxPoint: indicates ROI of scene
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.5f , Eigen::Vector4f (-8, -5, -3, 1), Eigen::Vector4f (23, 7, 0, 1));
  renderPointCloud(viewer, filterCloud, "filterCloud");
  
  //*################ 2-RANSAC SEGMENTATION ################*//
  /* renedering two segmented clouds (different colors), by calling segmentCloud 
    function (pointProcessor class) on input   */
    
  // use pointProcessor.SegmentPlane in case of point processor created on the stack 
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);

  // TURNED OFF segmentCloud to render Clustered clouds
  renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
  renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

  //*################ 3-EUCLIDEAN CLUSTERING ################*//
  // Clustering obstacle cloud into separate objects/obstacles
  
  pointProcessorI->completeCluster(segmentCloud.first, 0.67, viewer);   
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS

  // set renderScene to false, to only show the cloud (without cars/obstacles)
  bool renderScene = false;
  std::vector<Car> cars = initHighway(renderScene, viewer);

  // TODO:: Create lidar sensor 

  /*  
   * lidar >> Lidar pointer type (on the heap) with "new" keyword
   * Lidar(cars,0) >> cars: vector of cars from initHighway
     0 is ground plane slope */
  Lidar* lidar = new Lidar(cars, 0);


  // ### CREATE 'inputCloud' ###
  /* Ptr type indicates that object is a pointer (contains address
     of point cloud object) */

  /* <pcl::PointXYZ> is a template, type for the point cloud is pcl::PointXYZ
     other template options: <pcl::PointXYZI> or <pcl::PointXYZRGB>
   * template definition in processPointClouds.cpp */

  /* Create 'inputCloud' object */
  // As lidar is a pointer and created on the heap, -> used to call functions
  // we can give the cloud name of inputCloud or rays

  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();

  // Rendering lidar rays lines
  // Position is attribute in Lidar struct, where lidar is located
  //renderRays(viewer, lidar->position, inputCloud);

  // Name attribute used to differentiate between multi point clouds

  // TURNED OFF inputCloud to render segmented clouds
  //renderPointCloud(viewer, inputCloud, "inputCloud");

  // ### CREATE point processor ###
  
  // Instantiate point processor on the heap 
  //ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointCloud<pcl::PointXYZ>();

  // Instantiate point processor object on the stack
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  
  //*################ 2-RANSAC SEGMENTATION ################*// 
  /* segmentation by Calling SegmentPlane function on input and reneder the two segmented point 
     clouds in different colors */

  // point processor created on the heap then pointProcessor->SegmentPlane
  //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);

  // point processor created on the stack then pointProcessor.SegmentPlane
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);

  // TURNED OFF segmentCloud to render clustered clouds
  //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
  //renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

  //*################ 3-EUCLIDEAN CLUSTERING (PCL's built in function) ################*//

  /* Calling PCL built in clustering function of pointProcessor object
     on the obstacles/first cloud member "obstCloud" */
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

  // Enabling bounding box
  int render_box = 1;

  // rendering each cluster point cloud
  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters){
      std::cout << "cluster size ";
      pointProcessor.numPoints(cluster);

      /* renderPointCloud is expecting each cluster to have a unique identifier, so clusters are counted
       with clusterId and appended to obstCloud string (eg. obstCloud1) */
      renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);

      if (render_box){
      // Creating Bounding box object (for clusters)
      Box box = pointProcessor.BoundingBox(cluster);

      // Rendering Bounding boxes to the viewer
      renderBox(viewer,box,clusterId);
      }    
      ++clusterId; 
  }
}


// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
// Helps to set up different viewing angles in your window
// Viewer is passed as a reference (any changes done to it, will persist outside this function)
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

  //Changes done to the 3DViewer camera parameters
  viewer->setBackgroundColor (0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  // setAngle has 4 different options (camera position)
  switch(setAngle)
  {
    // XY is 45 degrees angle view
    case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
    case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
    case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;

    //FPS: First Person Sense, give sensation of being in driver seat
    case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if(setAngle!=FPS)
      viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
  std::cout << "starting enviroment" << std::endl;

  // 3DViewer (PCL Visualizer) is created and elements could be added to it
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  CameraAngle setAngle = XY;

  // Intensity point processor is created on the heap
  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  // Streaming .pcd files 
  std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();

  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  // Passing viewer to initCamera/simpleHighway/cityBlock functions (for single frame)
  initCamera(XY, viewer);
  //simpleHighway(viewer);
  //cityBlock(viewer, pointProcessorI, inputCloudI);
  //cityBlock(viewer);

  // Streaming frames
  while (!viewer->wasStopped ())
  {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process (using cityBlock pipeline)
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);

    streamIterator++;
    if(streamIterator == stream.end())
      streamIterator = stream.begin();

    viewer->spinOnce ();
  }
}