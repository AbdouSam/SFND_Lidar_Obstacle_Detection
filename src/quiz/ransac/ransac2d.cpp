/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <time.h>
#include <limits>

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

class	Segment
{
public:
	Segment(int size, pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud)
	{
		this->cloud = _cloud;

		while (this->indices.size() < size)
			this->indices.insert(rand() % this->cloud->points.size());	
	}

	std::unordered_set<int> get_inliers(float distanceTol)
	{
		std::unordered_set<int> inliers;

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
		inliers = this->indices;
		this->indices.clear();

		return inliers;
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
		this->m_coef[3] = (i * p1.x + j * p1.y + k * p1.z);

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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	while(maxIterations--)
	{
		PlaneSeg segment(cloud);

		std::unordered_set<int> inliers = segment.get_inliers(distanceTol);

		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers; 
		}
		
	}
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 0.22);

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
