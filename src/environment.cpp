/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors
#include <time.h>
#include <unordered_set>
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
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

  pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_cloud;
  filterd_cloud = pointProcessorI->FilterCloud(inputCloud, 0.3f , Eigen::Vector4f (-10.0f, -5.0f, -2.0f, 1), Eigen::Vector4f ( 30.0f, 8.0f, 1.0f, 1));

#if 0
  // Render the whole cloud
  renderPointCloud(viewer,filterd_cloud,"filterCloud");
#endif

    // Render the plane segments
    ProcessPointClouds<pcl::PointXYZI> cloud_processor;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_segs = 
        cloud_processor.CustomSegmentPlane(filterd_cloud, 25, 0.3);

    // render the plane
    renderPointCloud(viewer, cloud_segs.first, "Plane", Color(0, 1, 0));

#if 0
    // All obstacles before clustering
    renderPointCloud(viewer, cloud_segs.second, "Obstacles", Color(1, 0, 0));
#endif

    // Cluster the obstacles into different colors
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = cloud_processor.Clustering(cloud_segs.second, 0.5, 20, 300);

    // third phase cluster the segments into individual objects
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,1),Color(0,1,1),Color(1,1,0)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        cloud_processor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);
        Box box = cloud_processor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }

}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // simulation of Lidar Output
    Lidar * lidar = new Lidar(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();

    // first phase, render the lines of lidar
    //renderRays(viewer, lidar->position, cloud);

    // first phase, render lidar cloud
    //renderPointCloud(viewer, cloud, "Lidar cloud", Color(1, 1, 1));
    
    ProcessPointClouds<pcl::PointXYZ> cloud_processor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segs = 
        cloud_processor.CustomSegmentPlane(cloud, 100, 0.2);
    
    // second phase segment plane and obstacles and render them
#if 0
    renderPointCloud(viewer, cloud_segs.first, "Plane", Color(1, 0, 0));
    renderPointCloud(viewer, cloud_segs.second, "Obstacles", Color(0, 1, 0));
#endif

#if 1
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = cloud_processor.Clustering(cloud_segs.second, 1.0, 3, 30);

    // third phase cluster the segments into individual objects
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        cloud_processor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = cloud_processor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
#endif

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}