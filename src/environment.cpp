/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
 //   bool renderScene = true;
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    boost::shared_ptr<Lidar> iansLidar(new Lidar(cars, 0.0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = iansLidar->scan();

    boost::shared_ptr<ProcessPointClouds<pcl::PointXYZ>> pcProcessor(new ProcessPointClouds<pcl::PointXYZ>());
   
    // Find the ground plane in the point cloud
    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmented = pcProcessor->SegmentPlane(pointCloud, 100, 0.2f);

    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pcProcessor->Clustering(segmented.second, 1.0, 3, 100);

    int clusterId = 0;
    std::vector<Color> colours = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : clusters)
    {
        pcProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obj Cloud"+std::to_string(clusterId), colours[clusterId]);
        Box box = pcProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
//        BoxQ box = pcProcessor->BoundingBoxQ(cluster);
//        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

void CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> *pcProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display a City Block -----
    // ----------------------------------------------------
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pcProcessorI->FilterCloud(inputCloud, 0.2, {-15, -5.5, -2, 1.0}, {30, 5.5, 1, 1.0});
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmented = pcProcessorI->SegmentPlane(filteredCloud, 20, 0.17f);
    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pcProcessorI->Clustering(segmented.second, 0.45, 20, 1500);

    renderPointCloud(viewer, segmented.first, "Plane Cloud", Color(0,1,0));

    int clusterId = 0;
    std::vector<Color> colours = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters)
    {
        renderPointCloud(viewer, cluster, "cluster"+std::to_string(clusterId), colours[clusterId]);
        Box box = pcProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
//        BoxQ box = pcProcessorI->BoundingBoxQ(cluster);
//        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
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
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI> *pcProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pcProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    while (!viewer->wasStopped())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloud = pcProcessorI->loadPcd((*streamIterator).string());
        CityBlock(viewer, pcProcessorI, inputCloud);
        ++streamIterator;

        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }

    if (pcProcessorI != nullptr)
    {
        delete pcProcessorI;
        pcProcessorI = nullptr;
    }
}