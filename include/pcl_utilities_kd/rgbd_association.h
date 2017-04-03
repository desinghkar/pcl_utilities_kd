/*
 * By Karthik Desingh  
 * Input:
 *      1) Pixel bounds (start_i, start_j, end_i, end_j)
 *      2) Organized point cloud
 * Outputs:
 *      1) 3D point cloud in that bounds
 *      2) 3D bounds (left_front_xyz, bottom_back_xyz) (Optional)
 */
#ifndef RGBD_ASSOCIATION
//CPP
#include <string.h>
//PCL
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
using namespace std;
using namespace pcl;

class RGBD_Association
{
  //Input contains the pixel bouds as a vector of 4 values
  private:
    vector<int> pixel_bounds;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;

  public:
    RGBD_Association();
    ~RGBD_Association(){ };

    void setInputOrganizedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);
    void setPixelBounds(vector<int>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr extract3DSegment();
    pcl::PointXYZ get3DPointFrom2DPoint(int, int);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr get3DBounds();
    //To use outside for general pointclouds
    //Gives the 3Dbounds as left_top_front_xyz and right_bottom_back_xyz of the segment wrt provided frameofref
    pcl::PointCloud<pcl::PointXYZ>::Ptr get3DLimits(pcl::PointCloud<pcl::PointXYZ>::Ptr);

};

#endif

