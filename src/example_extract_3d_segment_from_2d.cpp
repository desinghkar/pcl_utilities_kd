#include "pcl_utilities_kd/rgbd_association.h"
#include <fstream>
using namespace std;

int main(int arc, char** arv)
{
  if(arc<2)
  {
    cout << "No point cloud is provided" << endl;
    return 1;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  //Read a pointcloud that is with respect to camera
  string pcd_name = arv[1];
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_name.c_str(), *cloud_in)==-1)
  {
    PCL_ERROR("Couldn't read the file ");
    return -1;
  }
  //Random pixel bounds
  vector<int> pix_bounds;
  pix_bounds.push_back(340);
  pix_bounds.push_back(94);
  pix_bounds.push_back(424);
  pix_bounds.push_back(221);
  //Use the filter_table object
  RGBD_Association as;
  as.setInputOrganizedCloud(cloud_in);
  as.setPixelBounds(pix_bounds);
  cloud_out = as.extract3DSegment();

  //save as a pcd
  pcl::io::savePCDFileASCII ("test_seg.pcd", *cloud_out);
  //pcl::io::savePCDFileASCII ("test_pcd_b.pcd", *cloud_b);
  return 0;
}
