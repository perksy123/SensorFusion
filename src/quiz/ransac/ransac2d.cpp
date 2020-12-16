/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

#include "Generator.h"

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

/*std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	std::cout << "Creating generator for cloud size " << cloud->size() << std::endl;
	Generator lineGenerator(cloud->size() - 1);

	// For max iterations 
	for (int iter = 0; iter < maxIterations; ++iter)
	{
		// Select start and end points
		CloudLine line;
		lineGenerator.CreateLine(line);

		line.SetLineCoefficients(cloud);

		std::unordered_set<int> iterationRes;
		int index = 0;
		for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); ++it)
		{
			if (!line.Contains(index))
			{
				pcl::PointXYZ &point = *it;

				if (line.IsInlier(point, distanceTol))
				{
					iterationRes.insert(index);
				} 
			}
			index++;
		}

		iterationRes.insert(line.GetStart());
		iterationRes.insert(line.GetEnd());

		if (inliersResult.size() < iterationRes.size())
		{
			inliersResult = iterationRes;
		}
	}


	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}*/

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	std::cout << "Creating plane generator for cloud size " << cloud->size() << std::endl;
	Generator planeGenerator(cloud->size() - 1);

	// For max iterations 
	for (int iter = 0; iter < maxIterations; ++iter)
	{
		// Select start and end points
		CloudPlane plane;
		planeGenerator.CreatePlane(plane);

		plane.SetPlaneCoefficients(cloud);

		std::unordered_set<int> iterationRes;
		int index = 0;
		for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); ++it)
		{
			if (!plane.Contains(index))
			{
				pcl::PointXYZ &point = *it;

				if (plane.IsInlier(point, distanceTol))
				{
					iterationRes.insert(index);
				} 
			}
			index++;
		}

		iterationRes.insert(plane.GetP1());
		iterationRes.insert(plane.GetP2());
		iterationRes.insert(plane.GetP3());

		if (inliersResult.size() < iterationRes.size())
		{
			inliersResult = iterationRes;
		}
	}


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
	std::unordered_set<int> inliers = Ransac(cloud, 20, 0.2);

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
