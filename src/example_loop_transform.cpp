#include "pcl_utilities_kd/filter_table.h"
#include <fstream>
#include <string>
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

void pcd2obj(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& outputFilename)
{
  const size_t size = cloud->points.size();
  std::ofstream os(outputFilename.c_str());

  for(unsigned int i=0 ; i<size ; i++)
  {
    // Remove nan
    if(!std::isnan(cloud->points[i].x))
    {
      os << "v ";
      os << cloud->points[i].x << " ";
      os << cloud->points[i].y << " ";
      os << cloud->points[i].z << "\n";
    }
  }

  os.close();
}
void writeTransformedCloud(string pcd_file, string file_trans, string output_file, string output_file_obj)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b (new pcl::PointCloud<pcl::PointXYZ>);
  //Read a pointcloud that is with respect to camera

  if(pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file.c_str(), *cloud_in)==-1)
  {
    PCL_ERROR("Couldn't read the file ");
    return;
  }
  //Initialze a transformation
  tf::Transform trans = readTransformation(file_trans);

  //Use the filter_table object
  Filter_Table ft;
  ft.setTableHeight(0.798);
  ft.setTransform(trans);
  ft.setInputCloudwrtCamera(cloud_in);
  ft.transformToBase();
  cloud_b = ft.getCloudwrtBase();

  //ft.filterTable(cloud_out);
  //save as a pcd
  //pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud_out);
  cout << "Writing " << output_file << endl;
  pcl::io::savePCDFileASCII (output_file.c_str(), *cloud_b);
  pcd2obj(cloud_b, output_file_obj);

}
int main(int arc, char** arv)
{
  //Text file with list of point clouds to be used
  //Path to the folder

  if(arc<2)
  {
    cout << "Text file with list of point clouds is not provided" << endl;
    return 1;
  }
  if(arc<3)
  {
    cout << "Path to the text file is not provided" << endl;
    return 1;
  }
  string pcd_list_name = arv[1];
  string path = arv[2];
  string file_path = path+'/'+pcd_list_name;
  ifstream file_id(file_path.c_str());
  
  while(1)
  {
    //Get the pcd name along with its file_path
    string pcd;
    file_id >> pcd;
    if(file_id.eof())
      break;
    string pcd_file = path+'/'+pcd;
    size_t lastindex = pcd_file.find_last_of(".");
    string trans = pcd_file.substr(0, lastindex);
    string trans_file =trans+"_cam_tf.txt";
    string output_file = trans+"_odom.pcd";
    string output_file_obj = trans+"_odom.obj";
    writeTransformedCloud(pcd_file, trans_file, output_file, output_file_obj);
    //cout << pcd_file << endl;
    //cout << trans_file << endl;
    
  }
  return 0;
}
