#include "pcl_utilities_kd/filter_table.h"

Filter_Table::Filter_Table()
{
  cloud_camera = boost::make_shared<pcl::PointCloud<PointXYZ> >();
  cloud_base = boost::make_shared<pcl::PointCloud<PointXYZ> >();

}
void Filter_Table::setTableHeight(double h)
{
  table_height = h;
}

void Filter_Table::setTransform(tf::Transform tf_camera_base)
{
  transform = tf_camera_base;
}

void Filter_Table::setInputCloudwrtCamera(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
  cloud_camera = cloud_in;
}

void Filter_Table::transformToBase()
{
  pcl_ros::transformPointCloud(*cloud_camera, *cloud_base, transform);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Filter_Table:: getCloudwrtBase()
{
  return cloud_base;
}
void Filter_Table::filterTable(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>);
  
  //Do the passthrough filtering
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_base);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (table_height, 3);
  pass.filter (*temp);

  //Transform back to the camera frame
  tf::Transform inverse_tf = transform.inverse();
  pcl_ros::transformPointCloud(*temp, *cloud_out, inverse_tf); 

}
