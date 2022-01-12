// PCL lib Functions for processing point clouds 
#include <unordered_set>
#include "processPointClouds.h"
#include "quiz/cluster/kdtree.h"
#include <vector>

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
    
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);

    // the voxel to down size the dense cloud into the leaf size (in meters, ie 0.01 = 1cm)
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*filtered_cloud);

    // Crop box is to remove all data that are away from the car, for example 30 meters aways obj
    // and further are ignored, we take a region

    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filtered_cloud);
    region.filter(*cloud_region);

    // remove the vertices that represent the roof we take a cropbox around the roof.
    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloud_region);
    roof.filter(indices);

    // the inliers are the points reflected by the roof
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (int point: indices)
        inliers->indices.push_back(point);

    // we set the indices of the roof as negatives and remove them from the cloud_region
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_region);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_region);

    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>), cloud_f (new pcl::PointCloud<PointT>);

 
    // create the indices of the points lies in a Plan
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

    // coeffients of the plan
    typename pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());

    // segmentaion object and parameters of the segmentation
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);


    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given data set.\n";
    }

    // extract the inliers if the size > 0
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_p);

    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);

    // update the cloud by removing the indices of the plane.
    //cloud_filtered.swap (cloud_f);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult; // = SeparateClouds(inliers,cloud);

    segResult.first = cloud_f;
    segResult.second = cloud_p;

    return segResult;
}

/*****************************************************************************
 * RANSAC Algorithm
 *****************************************************************************/

template<typename PointT>
class Segment
{
public:
	Segment(int size, typename pcl::PointCloud<PointT>::Ptr _cloud)
	{
		this->indices.clear();
		this->cloud = _cloud;

        // Pick k first random indices from the cloud
		while (this->indices.size() < size)
			this->indices.insert(rand() % this->cloud->points.size());
	}

	/* */
	std::unordered_set<int> get_inliers(float distanceTol)
	{
		for (int i = 0; i < this->cloud->points.size(); i++)
		{
			// skip if the point is already in
			if (this->indices.count(i) > 0)
				continue;

			// pick a point
			auto p = this->cloud->points[i];

			if (this->get_distance(p) <= distanceTol)
				this->indices.insert(i);
		}
		return this->indices;
	}
	
	virtual float get_distance(PointT p) = 0;

	~Segment() {}

protected:
	std::array<float, 4> m_coef;
	std::unordered_set<int> indices;
private:
	typename pcl::PointCloud<PointT>::Ptr cloud;
};

template<typename PointT>
class LineSeg : public Segment<PointT>
{
public:
	LineSeg(typename pcl::PointCloud<PointT>::Ptr cloud)
	:Segment<PointT>(2, cloud)
	{
		auto iter = this->indices.begin();

		auto p1 = cloud->points[*iter++];
		auto p2 = cloud->points[*iter];
		
        // calculate the coefficients of the line segment
		this->m_coef[0] = p1.y - p2.y; 
		this->m_coef[1] = p2.x - p1.x; 
		this->m_coef[2] = p1.x * p2.y - p2.x*p1.y;
	}

	float get_distance(PointT p)
	{
		return fabs(this->m_coef[0] * p.x + 
								this->m_coef[1] * p.y + 
								this->m_coef[2]) / 
								(sqrt(this->m_coef[0] * this->m_coef[0] + 
									  this->m_coef[1] * this->m_coef[1]));
	}

	~LineSeg()
	{
	}
};
template<typename PointT>
class PlaneSeg : public Segment<PointT>
{
public:
	PlaneSeg(typename pcl::PointCloud<PointT>::Ptr cloud)
	:Segment<PointT>(3, cloud)
	{
		auto iter = this->indices.begin();

		auto p1 = cloud->points[*iter++];
		auto p2 = cloud->points[*iter++];
		auto p3 = cloud->points[*iter];

        // calculate the coefficients of the Plane segment
		float i = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
		float j = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
		float k = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);

		this->m_coef[0] = i; 
		this->m_coef[1] = j; 
		this->m_coef[2] = k;
		this->m_coef[3] = -(i * p1.x + j * p1.y + k * p1.z);

	}

	float get_distance(PointT p)
	{
			return fabs(this->m_coef[0] * p.x + 
									this->m_coef[1] * p.y + 
									this->m_coef[2] * p.z + 
									this->m_coef[3]) / 
									(sqrt(this->m_coef[0] * this->m_coef[0] + 
										  this->m_coef[1] * this->m_coef[1] +
										  this->m_coef[2] * this->m_coef[2]));
	}
	
	~PlaneSeg()
	{
	}
};

template<typename PointT>
std::unordered_set<int> Ransac2D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	while(maxIterations--)
	{
		LineSeg<PointT> segment(cloud);

		std::unordered_set<int> inliers = segment.get_inliers(distanceTol);

		if (inliers.size() > inliersResult.size())
			inliersResult = inliers; 
	}

	return inliersResult;

}

template<typename PointT>
std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	while(maxIterations--)
	{
		PlaneSeg<PointT> segment(cloud);

		std::unordered_set<int> inliers = segment.get_inliers(distanceTol);

		if (inliers.size() > inliersResult.size())
			inliersResult = inliers; 
	}

	return inliersResult;

}

/*****************************************************************************
 *  End Custom Ransac
 *****************************************************************************/

/*****************************************************************************
 *  Custom Plane Segmentation
 *****************************************************************************/

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::CustomSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliers = Ransac3D<PointT>(cloud, maxIterations, distanceThreshold);

	typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
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

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);

    return segResult;
}


/*****************************************************************************
 *  END Custom Plane Segmentation
 *****************************************************************************/

/*****************************************************************************
 *  Custom Clustering
 *****************************************************************************/

std::vector<std::vector<int>> euclideanCluster(
                                const std::vector<std::vector<float>>& points,
                                custom_impl::KdTree* tree,
                                float distanceTol, int min_size, int max_size)
{
	std::vector<bool> visited(points.size(), false);
	std::vector<int> p_stack(points.size());

	std::vector<std::vector<int>> clusters_list;

	for (int curr_id = 0; curr_id < points.size(); curr_id++)
	{

		if (!visited[curr_id])
		{
			std::vector<int> cluster;

			// put the current indice in the cluster and find all the neighboors
			p_stack.push_back(curr_id);
			while (!p_stack.empty())
			{
				int p_id = p_stack.back();
				p_stack.pop_back();

				if (!visited[p_id])
				{
					visited[p_id] = true;
					cluster.push_back(p_id);
                    // if we reach the max, Drop the current stack and start a new cluster
                    if (cluster.size() == max_size)
                    {
                        p_stack.clear();
                        break;
                    }

		  		    std::vector<int> temp_cluster = tree->search(points[p_id], distanceTol);
					p_stack.insert(p_stack.end(), temp_cluster.begin(), temp_cluster.end());
				}

			}
			// add cluster to list of clusters_list
            if (cluster.size() >= min_size)
			    clusters_list.push_back(cluster);
		}
	}

	return clusters_list;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::CustomClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    custom_impl::KdTree* tree = new custom_impl::KdTree(3);

    std::vector<std::vector<float>> points;

    // convert the data cloud into float vector
    for (int i = 0; i < cloud->points.size(); i++)
    {
        std::vector<float> p_temp = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        points.push_back(p_temp);
	    tree->insert(points[i],i);
    }

    std::vector<std::vector<int>> cluster_indices = euclideanCluster(points, tree, clusterTolerance, minSize, maxSize);
    
    for (int i = 0; i < cluster_indices.size(); i++)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto& idx : cluster_indices[i])
        {
            cloud_cluster->push_back ((*cloud)[idx]);
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        clusters.push_back(cloud_cluster);
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

/*****************************************************************************
 *  END Custom Clustering
 *****************************************************************************/

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;

    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j = 0;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);

        for (const auto& idx : it->indices)
        {
            cloud_cluster->push_back ((*cloud)[idx]);
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        clusters.push_back(cloud_cluster);
        j++;
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