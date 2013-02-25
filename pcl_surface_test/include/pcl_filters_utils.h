/*
 * pcl_filters_utils.h
 *
 *  Created on: Feb 25, 2013
 *      Author: carlos
 */

#ifndef PCL_FILTERS_UTILS_H_
#define PCL_FILTERS_UTILS_H_

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZRGBNormal PointNormalT;

void spatialFilter(const PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out, double neighbor_max_proximity);



#endif /* PCL_FILTERS_UTILS_H_ */
