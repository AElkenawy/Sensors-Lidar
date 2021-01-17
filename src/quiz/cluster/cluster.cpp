/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, window.z_min, window.z_max, 1, 1, 1, "window");
  return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    for(int i = 0; i < points.size(); i++)
    {
      pcl::PointXYZ point;
      point.x = points[i][0];
      point.y = points[i][1];
      point.z = points[i][2];

      cloud->points.push_back(point);
    }
  
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;
}


void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

  if(node!=NULL)
  {
    Box upperWindow = window;
    Box lowerWindow = window;
    // split on x axis
    if(depth%3==0)
    {
      viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, window.z_min),pcl::PointXYZ(node->point[0], window.y_max, window.z_max),0,0,1,"line"+std::to_string(iteration));
      lowerWindow.x_max = node->point[0];
      upperWindow.x_min = node->point[0];
    }
    // split on y axis
    else if(depth%3==1)
    {
      viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], window.z_min),pcl::PointXYZ(window.x_max, node->point[1], window.z_max),1,0,0,"line"+std::to_string(iteration));
      lowerWindow.y_max = node->point[1];
      upperWindow.y_min = node->point[1];
    }
    // split on z axis
    else
    {
      viewer->addLine(pcl::PointXYZ(window.x_min, window.y_min, node->point[2]),pcl::PointXYZ(window.x_max, window.y_max, node->point[2]),0,1,0,"line"+std::to_string(iteration));
      lowerWindow.z_max = node->point[2];
      upperWindow.z_min = node->point[2];
    }    
    
    iteration++;

    render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
    render2DTree(node->right,viewer, upperWindow, iteration, depth+1);

  }
}


int main ()
{

  // Create viewer
  Box window;
  window.x_min = -10;
  window.x_max =  10;
  window.y_min = -10;
  window.y_max =  10;
  window.z_min = -10;
  window.z_max =  10;
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

  // Create data
  // Origin point (0,0) is in the middle 
  // std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
    std::vector<std::vector<float>> points = {{-6.2,7,3}, {-6.3,8.4,4}, {-5.2,7.1,3.5}, {-5.7,6.3,3}, {7.2,6.1,-2}, {8.0,5.3,-4}, {7.2,7.1,-3.5}, {0.2,-7.1,2}, {1.7,-6.9,3}, {-1.2,-7.2,4}, {2.2,-8.9,3.5}};

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

  KdTree* tree = new KdTree;
  
  for (int i=0; i<points.size(); i++) 
    tree->insert(points[i],i); 

  int it = 0;
  render2DTree(tree->root,viewer,window, it);

  std::cout << "Test Search" << std::endl;
  std::vector<int> nearby = tree->search({7.2,6.1,-2},4.0);
  //std::vector<int> nearby = tree->search({-0,-8.0,3},3.0);
  for(int index : nearby)
    std::cout << index << ",";
  std::cout << std::endl;

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 3.0);
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  // Render clusters
  int clusterId = 0;
std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  for(std::vector<int> cluster : clusters)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
    for(int indice: cluster)
      clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],0));
    renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
    ++clusterId;
  }
  if(clusters.size()==0)
    renderPointCloud(viewer,cloud,"data");

  while (!viewer->wasStopped ())
    viewer->spinOnce ();

}