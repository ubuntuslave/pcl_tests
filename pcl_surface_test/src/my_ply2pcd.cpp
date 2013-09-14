#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include "pcl_filters_utils.h"

int
main (int argc, char** argv)
{
  if(argc > 3 || argc < 2)
  {
    printf("ERROR: Usage is %s filename.ply filename.pcd\n", argv[0]);
    return -1;
  }

  // Load input file into a PointCloud<T> with an appropriate type
  PointCloudT::Ptr cloud_output (new PointCloudT);

  sensor_msgs::PointCloud2 cloud_msg_from_ply;
//  pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);

  pcl::io::loadPLYFile(std::string(argv[1]), cloud_msg_from_ply);
  // Convert to Point cloud pointer type
  pcl::fromROSMsg (cloud_msg_from_ply, *cloud_output);

  pcl::PCDWriter writer;
  std::string output_pcd_filename;
  if(argc != 3)
    output_pcd_filename = std::string(argv[1]) + ".pcd";
  else
    output_pcd_filename = std::string(argv[2]);


  int result_pcd = writer.writeBinary<PointT>(output_pcd_filename, *cloud_output);

  // Finish
  return (0);
}
