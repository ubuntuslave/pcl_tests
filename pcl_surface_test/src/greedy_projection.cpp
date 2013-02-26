#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>


//#ifdef __APPLE__
//#include <OpenGL/OpenGL.h>
//#include <GLUT/glut.h>
//#else
//#include <GL/glut.h>
//#endif
//
//#include <pthread.h> // POSIX threads library

#include "pcl_filters_utils.h"

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
  PointCloudT::Ptr cloud (new PointCloudT);
  PointCloudT::Ptr cloud_filtered (new PointCloudT);
  PointCloudT::Ptr cloud_for_mesh (new PointCloudT);
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

  /*
  // Displace cloud by a notch
  tf::Transform tf_for_cloud_filtering;
  tf_for_cloud_filtering.setIdentity();
  tf_for_cloud_filtering.setOrigin(btVector3(vgf_res/2.0, vgf_res/2.0, vgf_res/2.0));
  pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, ccny_rgbd::eigenFromTf(tf_for_cloud_filtering));

  // Filter again:
  pcl::VoxelGrid<PointT> sor2;
  sor2.setInputCloud (cloud_filtered);
  sor2.setLeafSize (vgf_res, vgf_res, vgf_res);
  sor2.filter (*cloud_filtered_twice);

  // Displace back the cloud
  tf_for_cloud_filtering.setOrigin(btVector3(-vgf_res/2.0, -vgf_res/2.0, -vgf_res/2.0));
  pcl::transformPointCloud(*cloud_filtered_twice, *cloud_filtered_twice, ccny_rgbd::eigenFromTf(tf_for_cloud_filtering));
  */

  spatialFilter(cloud_filtered, cloud_for_mesh, neighbor_max_proximity);

  if(with_smoothing)
  {
  // smooth using mls
    printf("Creating kd-tree for smoothing\n");
    pcl::search::KdTree<PointT>::Ptr mls_tree;
    mls_tree.reset(new pcl::search::KdTree<PointT>());

    printf("MLS...\n");
    pcl::MovingLeastSquares<PointT, PointT> mls;
    mls.setInputCloud (cloud_for_mesh);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (mls_tree);
    mls.setSearchRadius (smoothing_res);
    mls.process(*cloud_for_mesh);
    // the dense data should be available in cloud_for_mesh
    printf("done!\n");

  }


  pcl::PCDWriter writer;
  std::string downsampled_pcd_filename;
  if(with_smoothing)
    downsampled_pcd_filename = std::string(argv[1]) + "_downsampled_smoothed.pcd";
  else
    downsampled_pcd_filename = std::string(argv[1]) + "_downsampled.pcd";

  int result_pcd = writer.writeBinary<PointT>(downsampled_pcd_filename, *cloud_for_mesh);

  // Normal estimation*
  pcl::NormalEstimation<PointT, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud_for_mesh);
  n.setInputCloud (cloud_for_mesh);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  printf("Computing Normals...\n");
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures
  printf("done!\n");

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<PointNormalT>::Ptr cloud_with_normals (new pcl::PointCloud<PointNormalT>);
  pcl::concatenateFields (*cloud_for_mesh, *normals, *cloud_with_normals);
  // cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<PointNormalT>::Ptr tree2 (new pcl::search::KdTree<PointNormalT>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  printf("Greedy Triangulation\n");

  pcl::GreedyProjectionTriangulation<PointNormalT> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (mesh_search_radius);

  // Set typical values for the parameters
//  gp3.setMu (2.5);
  gp3.setMu (10);
  gp3.setMaximumNearestNeighbors (50);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  printf("Reconstructing mesh...\n");
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  std::string mesh_filename;
  if(with_smoothing)
    mesh_filename = std::string(argv[1]) + "_mesh_smoothed";
  else
    mesh_filename = std::string(argv[1]) + "_mesh";

  pcl::io::saveVTKFile (mesh_filename+".vtk", triangles);
  pcl::io::savePLYFile(mesh_filename+".ply", triangles);

  // Finish
  return (0);
}
