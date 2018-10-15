#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/hitech/rs_ws/data/1539638968743881.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width  << " " << cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);  
  pcl::copyPointCloud (*cloud, *cropped_cloud);


  pcl::visualization::PCLVisualizer viewer("PCL Viewer"); 
  // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB>rgb(cropped_cloud);
  // viewer.addPointCloud<pcl::PointXYZRGB> (cropped_cloud, rgb, "sample cloud"); 
  // viewer.showCloud (cropped_cloud);
  // while (!viewer.wasStopped ())
  // {
      
  // }
  // return (0);
}