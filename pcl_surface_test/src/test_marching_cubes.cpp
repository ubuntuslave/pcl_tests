/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: test_surface.cpp 6579 2012-07-27 18:57:32Z rusu $
 *
 */

//#include <gtest/gtest.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/common/common.h>

// Filters:
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>


using namespace pcl;
using namespace pcl::io;
using namespace std;


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBNormal PointNormalT;


PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal>);
search::KdTree<PointXYZ>::Ptr tree;
search::KdTree<PointNormal>::Ptr tree2;

// add by ktran to test update functions
PointCloud<PointXYZ>::Ptr cloud1 (new PointCloud<PointXYZ>);
PointCloud<PointNormal>::Ptr cloud_with_normals1 (new PointCloud<PointNormal>);
search::KdTree<PointXYZ>::Ptr tree3;
search::KdTree<PointNormal>::Ptr tree4;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
main (int argc, char** argv)
{
  if(argc != 6)
  {
    printf("ERROR: Usage is %s pcd_filename_prefix voxel_grid_filter_res neighbor_max_prox smoothing_res mesh_search_radius\n", argv[0]);
    return -1;
  }
  double vgf_res = atof(argv[2]);
  double neighbor_max_proximity = atof(argv[3]);
  double smoothing_res = atof(argv[4]);
  double mesh_search_radius = atof(argv[5]);

  bool with_smoothing = false;
  if(smoothing_res > 0.0)
    with_smoothing = true;

  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_for_mesh (new pcl::PointCloud<PointT>);
//  pcl::PointCloud<PointT>::Ptr cloud_filtered_twice(new pcl::PointCloud<PointT>);
  sensor_msgs::PointCloud2 cloud_blob;
//  pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);

  pcl::io::loadPCDFile (std::string(argv[1]) + ".pcd", cloud_blob);
  pcl::fromROSMsg (cloud_blob, *cloud);

  // Create the filtering object
  pcl::VoxelGrid<PointT> sor1;
  sor1.setInputCloud (cloud);
  sor1.setLeafSize (vgf_res, vgf_res, vgf_res);
  sor1.filter (*cloud_filtered);







  // --------------------------------------------------------------------------------------------
  // Create search tree
  tree.reset (new search::KdTree<PointXYZ> (false));
  tree->setInputCloud (cloud);

  // Normal estimation
  NormalEstimation<PointXYZ, Normal> n;
  PointCloud<Normal>::Ptr normals (new PointCloud<Normal> ());
  n.setInputCloud (cloud);
  //n.setIndices (indices[B);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);

  // Concatenate XYZ and normal information
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
      
  // Create search tree
  tree2.reset (new search::KdTree<PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Process for update cloud
  if(argc == 3){
    sensor_msgs::PointCloud2 cloud_blob1;
    loadPCDFile (argv[2], cloud_blob1);
    fromROSMsg (cloud_blob1, *cloud1);
        // Create search tree
    tree3.reset (new search::KdTree<PointXYZ> (false));
    tree3->setInputCloud (cloud1);

    // Normal estimation
    NormalEstimation<PointXYZ, Normal> n1;
    PointCloud<Normal>::Ptr normals1 (new PointCloud<Normal> ());
    n1.setInputCloud (cloud1);

    n1.setSearchMethod (tree3);
    n1.setKSearch (20);
    n1.compute (*normals1);

    // Concatenate XYZ and normal information
    pcl::concatenateFields (*cloud1, *normals1, *cloud_with_normals1);
    // Create search tree
    tree4.reset (new search::KdTree<PointNormal>);
    tree4->setInputCloud (cloud_with_normals1);
  }

  MarchingCubesHoppe<PointNormal> hoppe;
    hoppe.setIsoLevel (0);
    hoppe.setGridResolution (30, 30, 30);
    hoppe.setPercentageExtendGrid (0.3f);
    hoppe.setInputCloud (cloud_with_normals);
    PointCloud<PointNormal> points;
    std::vector<Vertices> vertices;
    hoppe.reconstruct (points, vertices);

    MarchingCubesRBF<PointNormal> rbf;
    rbf.setIsoLevel (0);
    rbf.setGridResolution (20, 20, 20);
    rbf.setPercentageExtendGrid (0.1f);
    rbf.setInputCloud (cloud_with_normals);
    rbf.setOffSurfaceDisplacement (0.02f);
    rbf.reconstruct (points, vertices);
  return 0;
}
/* ]--- */
