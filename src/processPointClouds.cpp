// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "Generator.h"
#include "CloudPlane.h"
#include "kdtree.h"
#include "pcl/common/pca.h"

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

    typename pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cropped(new pcl::PointCloud<PointT>());
 
    std::cout << "Filtering Input cloud size " << cloud->points.size() << std::endl;

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*filtered);

    std::cout << "Voxel Filtered cloud size " << filtered->points.size() << std::endl;

    pcl::CropBox<PointT> crop(true);
    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.setInputCloud(filtered);
    crop.filter(*cropped);

    std::cout << "Cropped cloud size " << cropped->points.size() << std::endl;

    std::vector<int> indices;
    pcl::CropBox<PointT> roofPts(true);
    roofPts.setMin(Eigen::Vector4f(-1.5, -1.7, -1.0, 1));
    roofPts.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roofPts.setInputCloud(cropped);
    roofPts.filter(indices);

    std::cout << "Roof filtering removed  " << indices.size() <<  " points" << std::endl;

    pcl::PointIndices::Ptr roofInliers (new pcl::PointIndices);
    for (std::vector<int>::const_iterator it = indices.begin(); it != indices.end(); ++it)
        roofInliers->indices.push_back(*it);

    pcl::ExtractIndices<PointT> ex;
    ex.setInputCloud(cropped);
    ex.setIndices(roofInliers);
    ex.setNegative(true);
    ex.filter(*cropped);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cropped;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
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

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(inlierCloud, outlierCloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();
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
		for (typename pcl::PointCloud<PointT>::iterator it = cloud->begin(); it != cloud->end(); ++it)
		{
			if (!plane.Contains(index))
			{
				PointT &point = *it;

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

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
 
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

    if (tree != nullptr)
    {
        delete tree;
        tree == nullptr;
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

// I can't make this work......
// I think its giving me rotations in all 3 dimensions. I don't have time to sort out how to just find the minimum
// bounded box rotated just about Z. Also the boxe axes don't align with the cluster points. Again, I don't know why.
template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    BoxQ box;

    typename pcl::PointCloud<PointT>::Ptr projectedPCACloud(new pcl::PointCloud<PointT>);
    pcl::PCA<PointT> pca;

    pca.setInputCloud(cluster);
    pca.project(*cluster, *projectedPCACloud); 
    Eigen::Matrix3f &cloudEigenVectors = pca.getEigenVectors();
 
    Eigen::Vector4f cloudCentroid;
    pcl::compute3DCentroid(*cluster, cloudCentroid);

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = cloudEigenVectors.transpose();
    projectionTransform.block<3,1>(0,3) = -1.0f * (projectionTransform.block<3,3>(0,0) * cloudCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

    PointT minPoint;
    PointT maxPoint;

    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    const Eigen::Quaternionf bboxQuaternion(cloudEigenVectors);
    box.bboxQuaternion = bboxQuaternion;
    box.bboxTransform = cloudEigenVectors * meanDiagonal + cloudCentroid.head<3>();
    box.cube_height = maxPoint.z - minPoint.z;
    box.cube_length = maxPoint.y - minPoint.y;
    box.cube_width = maxPoint.x - minPoint.x;

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

// Old code....

// PCD clustering
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

// PCD Segment plane
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

// PCD Separate clouds
/*    typename pcl::PointCloud<PointT>::Ptr inlierCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr outlierCloud(new pcl::PointCloud<PointT>());

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    extract.setNegative(false);
    extract.filter(*inlierCloud);

    extract.setNegative(true);
    extract.filter(*outlierCloud);*/
