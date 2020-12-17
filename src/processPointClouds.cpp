// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "Generator.h"
#include "CloudPlane.h"
#include "kdtree.h"

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


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr inlierCloud(new pcl::PointCloud<PointT>(*cloud, inliers->indices));
    typename pcl::PointCloud<PointT>::Ptr outlierCloud(new pcl::PointCloud<PointT>());
    
    std::vector<bool> inlier(cloud->size(), false);
    for (std::vector<int>::iterator it = inliers->indices.begin(); it != inliers->indices.end(); ++it)
    {
        inlier[*it] = true;
    }

    int index = 0;
    for (typename std::vector<bool>::iterator inlierIt = inlier.begin(); inlierIt != inlier.end(); ++inlierIt)
    {
        if (*inlierIt == false)
        {
            outlierCloud->push_back(cloud->at(index));
        }
        index++;
    }

/*    typename pcl::PointCloud<PointT>::Ptr inlierCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr outlierCloud(new pcl::PointCloud<PointT>());

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    extract.setNegative(false);
    extract.filter(*inlierCloud);

    extract.setNegative(true);
    extract.filter(*outlierCloud);*/

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(inlierCloud, outlierCloud);
    return segResult;
}


/*template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
 
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setMaxIterations(maxIterations);
    seg.setInputCloud(cloud);

    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not find a plane in the point cloud" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
 
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    return segResult;
}*/

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	srand(time(NULL));
	
	// TODO: Fill in this function
	std::cout << "Creating plane generator for cloud size " << cloud->size() << std::endl;
	Generator<PointT> planeGenerator(cloud->size() - 1);

	// For max iterations 
	for (int iter = 0; iter < maxIterations; ++iter)
	{
		// Select start and end points
		CloudPlane<PointT> plane;
		planeGenerator.CreatePlane(plane);

		plane.SetPlaneCoefficients(cloud);

	    pcl::PointIndices::Ptr iterationRes(new pcl::PointIndices());
		int index = 0;
		for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); ++it)
		{
			if (!plane.Contains(index))
			{
				pcl::PointXYZ &point = *it;

				if (plane.IsInlier(point, distanceThreshold))
				{
                    iterationRes->indices.push_back(index);
				} 
			}
			index++;
		}

		iterationRes->indices.push_back(plane.GetP1());
		iterationRes->indices.push_back(plane.GetP2());
		iterationRes->indices.push_back(plane.GetP3());

		if (inliers->indices.size() < iterationRes->indices.size())
		{
			inliers->indices = iterationRes->indices;
		}
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(int point, std::vector<int> &cluster, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<bool> &processedPoints, KdTree<PointT> *tree, float distanceTol)
{
	if (processedPoints[point] == false)
	{
		processedPoints[point] = true;
		cluster.push_back(point);
		std::vector<int> nearbyPoints = tree->search(cloud->at(point), distanceTol);
		for(std::vector<int>::iterator it = nearbyPoints.begin(); it != nearbyPoints.end(); ++it)
		{
			Proximity(*it, cluster, cloud, processedPoints, tree, distanceTol);
		}
	}
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
/*    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for (std::vector<pcl::PointIndices>::iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());

        pcl::PointIndices clusterIndices = *it;
        for (std::vector<int>::iterator indicesIt = clusterIndices.indices.begin(); indicesIt != clusterIndices.indices.end(); ++indicesIt)
        {
            cluster->push_back(cloud->at(*indicesIt));
        }

        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;*/

	KdTree<PointT>* tree = new KdTree<PointT>;
  
    for (int i = 0; i < cloud->points.size(); i++) 
    	tree->insert(cloud->at(i), i); 

	// Point index based array of processed flags.
	std::vector<bool> processedPoints(cloud->points.size(), false);
	std::vector<std::vector<int>> clusterIds;

	for (int point = 0; point < cloud->points.size(); ++point)
	{
		if (processedPoints[point] == false)
		{
			std::vector<int> cluster;
			Proximity(point, cluster, cloud, processedPoints, tree, clusterTolerance);
			clusterIds.push_back(cluster);
		}
	}

    for (std::vector<std::vector<int>>::const_iterator clusterIt = clusterIds.begin(); clusterIt != clusterIds.end(); ++clusterIt)
    {
         const std::vector<int> &ids = *clusterIt;

        if (ids.size() >= minSize && ids.size() <= maxSize)
        {
            typename pcl::PointCloud<PointT>::Ptr clusterPts(new pcl::PointCloud<PointT>());

            for (std::vector<int>::const_iterator idsIt = ids.begin(); idsIt != ids.end(); ++idsIt)
            {
                clusterPts->points.push_back(cloud->points.at(*idsIt));
            }
            clusters.push_back(clusterPts);
        }
    }

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
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}