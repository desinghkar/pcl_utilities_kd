/*
 * By Karthik Desingh
 * Input:
 *      1) Table height 
 *      2) Transform of rosrun tf tf_echo /base_link /head_camera_rgb_optical_frame with x y z qx qy qz qw in text
 *      3) Point cloud with respect to the camera frame
 * Output:
 *      1) Table filtered point cloud with respect to the camera frame.
 */

#ifndef FILTER_TABLE
//CPP
#include <string.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/impl/transforms.hpp>

using namespace std;
using namespace pcl;

class Filter_Table
{
  //Input contains table_height, transform between the point clouds and point clouds
  private:
    double table_height;
    tf::Transform transform;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base;
    pcl::PointIndices::Ptr orig_indices;

  //
  public:
    Filter_Table();
    ~Filter_Table(){ };
    
    void setTableHeight(double);
    void setTransform(tf::Transform);
    void setInputCloudwrtCamera(pcl::PointCloud<pcl::PointXYZ>::Ptr);
    void transformToBase();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudwrtBase();
    void filterTable(pcl::PointCloud<pcl::PointXYZ>::Ptr);

};

#endif
