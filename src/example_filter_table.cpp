#include "pcl_utilities_kd/filter_table.h"
#include <fstream>
using namespace std;
tf::Transform readTransformation(string file)
{
  ifstream file_ptr(file.c_str());
  tf::Vector3 v;
  file_ptr >> v[0];
  file_ptr >> v[1];
  file_ptr >> v[2];
  tf::Quaternion q;
  file_ptr >> q[0];
  file_ptr >> q[1];
  file_ptr >> q[2];
  file_ptr >> q[3];

  tf::Transform tf_p;
  tf_p.setOrigin(v);
  tf_p.setRotation(q);
  return tf_p;

}
int main(int arc, char** arv)
{
  if(arc<2)
  {
    cout << "No point cloud is provided" << endl;
    return 1;
  }
  if(arc<3)
  {
    cout << "Transformation file was not provided" << endl;
    return 1;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b (new pcl::PointCloud<pcl::PointXYZ>);
  //Read a pointcloud that is with respect to camera
  string pcd_name = arv[1];
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_name.c_str(), *cloud_in)==-1)
  {
    PCL_ERROR("Couldn't read the file ");
    return -1;
  }
  //Initialze a transformation
  string file_trans = arv[2];
  tf::Transform trans = readTransformation(file_trans);

  //Use the filter_table object
  Filter_Table ft;
  ft.setTableHeight(0.798);
  ft.setTransform(trans);
  ft.setInputCloudwrtCamera(cloud_in);
  ft.transformToBase();
//  cloud_b = ft.getCloudwrtBase();

  ft.filterTable(cloud_out);
  //save as a pcd
  pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud_out);
  //pcl::io::savePCDFileASCII ("test_pcd_b.pcd", *cloud_b);
  return 0;
}
