#ifndef RANSAC_H
#define RANSAC_H

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol);

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol);

#endif  /* RANSAC_H */