/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <time.h>

/**
 * @brief Pick randomly a segment (line or plane) from a data cloud
 * 	and return the data belonging to this segment
 * 
 */
class	Segment
{
public:
	Segment(int size, pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud)
	{
		this->indices.clear();
		this->cloud = _cloud;

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
	
	virtual float get_distance(pcl::PointXYZ p) = 0;

	~Segment() {}

protected:
	std::array<float, 4> m_coef;
	std::unordered_set<int> indices;
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

class LineSeg : public Segment
{
public:
	LineSeg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	:Segment(2, cloud)
	{
		auto iter = this->indices.begin();

		auto p1 = cloud->points[*iter++];
		auto p2 = cloud->points[*iter];
		
		this->m_coef[0] = p1.y - p2.y; 
		this->m_coef[1] = p2.x - p1.x; 
		this->m_coef[2] = p1.x * p2.y - p2.x*p1.y;
	}

	float get_distance(pcl::PointXYZ p)
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

class PlaneSeg : public Segment
{
public:
	PlaneSeg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	:Segment(3, cloud)
	{
		auto iter = this->indices.begin();

		auto p1 = cloud->points[*iter++];
		auto p2 = cloud->points[*iter++];
		auto p3 = cloud->points[*iter];

		float i = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
		float j = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
		float k = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);

		this->m_coef[0] = i; 
		this->m_coef[1] = j; 
		this->m_coef[2] = k;
		this->m_coef[3] = -(i * p1.x + j * p1.y + k * p1.z);

	}

	float get_distance(pcl::PointXYZ p)
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

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	while(maxIterations--)
	{
		LineSeg segment(cloud);

		std::unordered_set<int> inliers = segment.get_inliers(distanceTol);

		if (inliers.size() > inliersResult.size())
			inliersResult = inliers; 
	}

	return inliersResult;

}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	while(maxIterations--)
	{
		PlaneSeg segment(cloud);

		std::unordered_set<int> inliers = segment.get_inliers(distanceTol);

		if (inliers.size() > inliersResult.size())
			inliersResult = inliers; 
	}

	return inliersResult;

}
