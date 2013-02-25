/*
 * pcl_filters_util.cpp
 *
 *  Created on: Feb 25, 2013
 *      Author: carlos
 */

#include "pcl_filters_utils.h"

void spatialFilter(const PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, double neighbor_max_proximity)
{
  long int number_of_points = cloud_in->points.size();
  printf("%d points in the currently filtered cloud\n", number_of_points);
  std::vector<bool> valid_indices(number_of_points, true);

  PointT searchPoint;
  // ... populate the cloud and the search point
  // create a kd-tree instance
  pcl::KdTreeFLANN<PointT> kdtree_naive;
  // assign a point cloud - this builds the tree
  kdtree_naive.setInputCloud (cloud_in);
  std::vector<int> pointIdxRadius;
  std::vector<float> pointsSquaredDistRadius;
  float radius = neighbor_max_proximity;
  // radius search

  for(int i = 0; i<number_of_points; i++)
  {
    if(valid_indices[i])
    {
      pointIdxRadius.clear();
      pointsSquaredDistRadius.clear();
      searchPoint = cloud_in->points[i];
      int count = kdtree_naive.radiusSearch (searchPoint, radius,
                                             pointIdxRadius, pointsSquaredDistRadius);

      for(int c = 1; c<pointIdxRadius.size(); c++)
      {
        valid_indices[pointIdxRadius[c]] = false;
      }

    }
  }

  int count_valid_indices = 0;
  for(int i = 0; i<number_of_points; i++)
  {
    if(valid_indices[i])
      count_valid_indices++;
  }

  printf("%d points in the cloud for meshing after filtered with knn radius search: %f\n", count_valid_indices, radius);

  // Form the new tree
  cloud_out->points.clear();
  cloud_out->points.resize(count_valid_indices);
  int pt = 0;
  for(int i = 0; i<number_of_points; i++)
  {
    if(valid_indices[i])
    {
      PointT valid_point = cloud_in->points[i];
      cloud_out->points[pt] = valid_point;
      pt++;
    }
  }

  printf("done!\n");

}



