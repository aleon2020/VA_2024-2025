#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main()
{
  // PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Read pcd file
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("../../PCL_data/test_pcd.pcd", *cloud) == -1) { //* load the file
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    return -1;
  }

  // Show data
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

  // Using auto
  std::cout << std::endl << "FOR AUTO:" << std::endl;
  for (const auto & point: *cloud) {
    std::cout << "    " << point.x
              << " " << point.y
              << " " << point.z << std::endl;
  }

  // Using iterator
  std::cout << std::endl << "FOR ITERATOR:" << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::const_iterator it;
  for (it = cloud->begin(); it != cloud->end(); ++it) {
    std::cout << "    " << it->x
              << " " << it->y
              << " " << it->z << std::endl;
  }
  return 0;
}
