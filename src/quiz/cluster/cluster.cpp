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
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

  	viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 0, 0, 0, "window");
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
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

void render2DTree(custom_impl::Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%2==0)
		{
			viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),pcl::PointXYZ(node->point[0], window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		else
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),pcl::PointXYZ(window.x_max, node->point[1], 0),1,0,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

}

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
                    #if 0
                    if (cluster.size() == max_size)
                    {
                        p_stack.clear();
                        break;
                    }
                    #endif

		  		    std::vector<int> temp_cluster = tree->search(points[curr_id], distanceTol);
					// put all the points in the stack
					p_stack.insert(p_stack.end(), temp_cluster.begin(), temp_cluster.end());
				}

			}
			// add cluster to list of clusters_list
            std::cout << "Found cluster with size " << cluster.size() << std::endl;
            if (cluster.size() >= min_size)
			    clusters_list.push_back(cluster);
		}
	}

	return clusters_list;
}

int main ()
{

	// Create viewer
	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min =   0;
  	window.z_max =   0;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

	// Create data
	std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

	custom_impl::KdTree* tree = new custom_impl::KdTree(2);
  
	for (int i=0; i<points.size(); i++) 
		tree->insert(points[i],i); 

	int it = 0;
	render2DTree(tree->root,viewer,window, it);

	std::cout << "Test Search" << std::endl;
	std::vector<float> p_t = {-6, 7};

	std::vector<int> nearby = tree->search(p_t,3.0);
	for(int index : nearby)
		std::cout << index << ",";
	std::cout << std::endl;

	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now();
	//
	std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 3.0, 1, 10);
	//
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

	// Render clusters
	int clusterId = 0;

	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,1,1)};
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
	{
		viewer->spinOnce ();
	}
  	
}
