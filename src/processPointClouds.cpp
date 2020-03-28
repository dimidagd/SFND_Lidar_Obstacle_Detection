// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{

	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	while (maxIterations--)
	{
		
	
	// Randomly sample subset and fit line
	std::unordered_set<int> inliers;

	while (inliers.size() < 3 )
		inliers.insert(rand()%(cloud->points.size()));
	
	auto itr = inliers.begin();

	float x1, y1, z1, x2, y2, z2, x3,y3,z3;
	x1 = cloud->points[*itr].x;
	y1 = cloud->points[*itr].y;
	z1 = cloud->points[*itr].z;
	itr++;
	x2 = cloud->points[*itr].x;
	y2 = cloud->points[*itr].y;
	z2 = cloud->points[*itr].z;
	itr++;
	x3 = cloud->points[*itr].x;
	y3 = cloud->points[*itr].y;
	z3 = cloud->points[*itr].z;

	
	float cross_i = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
	float cross_j = (z2 - z1)*(x3 - x1) - (x2 -x1)*(z3- z1);
	float cross_k = (x2 -x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);


	float A = cross_i;
	float B = cross_j;
	float C = cross_k;
	float D = -(cross_i*x1 + cross_j*y1 + cross_k*z1);

	for (int index = 0; index < cloud->points.size(); index++)
	{

		if (inliers.count(index)>0)
			continue;

		pcl::PointXYZ point = cloud->points[index];
		float x4 = point.x;
		float y4 = point.y;
		float z4 = point.z;

		
		float d = abs(A*x4 + B*y4 + C*z4 + D)/sqrt(A*A + B*B +C*C);// Measure distance between every point and fitted line

		if (d < distanceTol)// If distance is smaller than threshold count it as inlier
			inliers.insert(index);
	}

	
	
	

		if(inliers.size()>inliersResult.size())
		{
			//std::cout << "new found" << std::endl;
			inliersResult = inliers;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac duration " << elapsedTime.count() << " milliseconds"  << std::endl;
	return inliersResult;

}


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

   
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*cloudFiltered);


    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f {-1.5, -1.7, -1 ,1});
    roof.setMax(Eigen::Vector4f {2.6, 1.7, -.4 ,1});
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter(*cloudRegion);
    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for (int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    //Create filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter (*obstCloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices()};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients()};// Model parametsr
    // TODO:: Fill in this function to find inliers for the cloud.

    //Initialise segmentation object on the stack.
    pcl::SACSegmentation<PointT> seg;

    // Optionally set optimize coefficients
    seg.setOptimizeCoefficients(true);

    //Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations)    ;
    seg.setDistanceThreshold (distanceThreshold);


    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients );    
    if (inliers->indices.size()==0)
    {
        std::cout << "Could not estimate planar model for the given pointcloud." << std::endl;
        
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); //2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for(pcl::PointIndices getIndices : cluster_indices) // for loop on a vector
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster {new pcl::PointCloud<PointT>};

        for (int index : getIndices.indices)
            cloudCluster->points.push_back (cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size() ;
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

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
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlaneOwn(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers {new pcl::PointIndices()};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients()};// Model parametsr
    // TODO:: Fill in this function to find inliers for the cloud.

   
	int numIters = 100;
	float distThreshold = 0.1;
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, numIters, distThreshold);

	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    return segResult;
}
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ClusteringOwn(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{


    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    
    
    KdTree* tree = new KdTree;
    for (int i=0; i<cloud->points.size(); i++)
		tree->insert(cloud->points[i],i); 

    //std::vector<pcl::PointIndices> cluster_indices;

    std::vector<std::vector<int>> cluster_indices = euclideanCluster(cloud->points, tree, 0.5);

    for(std::vector<int> getIndices : cluster_indices) // for loop on a vector
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster {new pcl::PointCloud<PointT>};

        for (int index : getIndices)
            cloudCluster->points.push_back (cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size() ;
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;


    return clusters;
}


std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);

	int i =0;

	while (i < points.size())
	{
		if (processed[i])
		{
			i++;
			continue;
		}

		std::vector<int> cluster;
		clusterHelper(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;
	}
	

	return clusters;

}

void clusterHelper(int index, const std::vector<std::vector<float>> points, std::vector<int> &cluster, std::vector<bool> &processed, KdTree* tree, float distanceTol)
{
	processed[index] = true;
	cluster.push_back(index);
	std::vector<int> nearest = tree->search(points[index], distanceTol);

	for (int id : nearest)
	{
		if (!processed[id])
			clusterHelper(id, points, cluster, processed, tree, distanceTol);
	}

}