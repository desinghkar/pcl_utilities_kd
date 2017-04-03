#include "pcl_utilities_kd/rgbd_association.h"

RGBD_Association::RGBD_Association()
{
  cloud_in = boost::make_shared<pcl::PointCloud<PointXYZ> >();
}

void RGBD_Association::setInputOrganizedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  cloud_in = cloud;
}

pcl::PointXYZ RGBD_Association::get3DPointFrom2DPoint(int px, int py)
{
  pcl::PointXYZ p_3;
  p_3 = cloud_in->points[640*py+px]; //TODO make it better with pixel offset
  return p_3;
}

void RGBD_Association::setPixelBounds(vector<int> pxs)
{
  pixel_bounds.push_back(pxs[0]);
  pixel_bounds.push_back(pxs[1]);
  pixel_bounds.push_back(pxs[2]);
  pixel_bounds.push_back(pxs[3]);
}
pcl::PointCloud<pcl::PointXYZ>::Ptr RGBD_Association::extract3DSegment()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg (new pcl::PointCloud<pcl::PointXYZ>);
  for(int i=pixel_bounds[0]; i<=pixel_bounds[2]; i++)
    for(int j=pixel_bounds[1]; j<=pixel_bounds[3]; j++)
     cloud_seg->points.push_back(get3DPointFrom2DPoint(i, j));
  cloud_seg->height = 1;
  cloud_seg->width = cloud_seg->points.size();
  return cloud_seg;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RGBD_Association::get3DLimits(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr limits_3d (new pcl::PointCloud<pcl::PointXYZ>);
  double min_x=99, min_y=99, min_z=99;
  double max_x=-99, max_y=-99, max_z=-99;
  for(size_t i=0; i<cloud_seg->points.size(); i++)
  {
    pcl::PointXYZ p = cloud_seg->points[i];
   if(min_x>p.x) 
     min_x=p.x;
   if(min_y>p.y)
     min_y=p.y;
   if(min_z>p.z)
     min_z=p.z;
   
   if(max_x<p.x)
     max_x=p.x;
   if(max_y<p.y)
     max_y=p.y;
   if(max_z<p.z)
     max_z=p.z;
  }
  pcl::PointXYZ min_pt;
  pcl::PointXYZ max_pt;
  min_pt.x = min_x; min_pt.y = min_y; min_pt.z = min_z;
  max_pt.x = max_x; max_pt.y = max_y; max_pt.z = max_z;
  limits_3d->points.push_back(min_pt);
  limits_3d->points.push_back(max_pt);
  return limits_3d;
}

