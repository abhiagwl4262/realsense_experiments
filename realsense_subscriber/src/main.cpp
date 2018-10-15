#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <boost/foreach.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/hitech/rs_ws/data/1539638968743881.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width  << " " << cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  
  sensor_msgs::Image image_;
  pcl::toROSMsg (*cloud, image_);

  cv::Mat image = cv::Mat( cloud->height, cloud->width, CV_8UC3);
  for( int y = 0; y < image.rows; y++ ) {
      for( int x = 0; x < image.cols; x++ ) {
          pcl::PointXYZRGB point = cloud->at( x, y );
          image.at<cv::Vec3b>( y, x )[0] = point.b;
          image.at<cv::Vec3b>( y, x )[1] = point.g;
          image.at<cv::Vec3b>( y, x )[2] = point.r;
      }
  }
  cv::Rect2d r = cv::selectROI(image);
  std::cout << "rect is " << r << std::endl;;
  float topleft_x     = r.x;
  float topleft_y     = r.y;
  float bottomright_x = r.x + r.width;
  float bottomright_y = r.y + r.height;

  for( int y = 0; y < cloud->width; y++ ) {
      for( int x = 0; x < cloud->height; x++ ) {

          

      }
  }


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);  
  pcl::copyPointCloud (*cloud, *cropped_cloud);
  cropped_cloud->points.resize (r.width * r.height);
  
  pcl::PointXYZRGB minPt, maxPt;
  pcl::getMinMax3D (*cropped_cloud, minPt, maxPt);
  std::cout << "Max x: " << maxPt.x << std::endl;
  std::cout << "Max y: " << maxPt.y << std::endl;
  std::cout << "Max z: " << maxPt.z << std::endl;
  std::cout << "Min x: " << minPt.x << std::endl;
  std::cout << "Min y: " << minPt.y << std::endl;
  std::cout << "Min z: " << minPt.z << std::endl;

  pcl::visualization::PCLVisualizer viewer("PCL Viewer"); 
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cropped_cloud);
  viewer.addPointCloud<pcl::PointXYZRGB> (cropped_cloud, rgb, "sample cloud");
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000)); 
  }
  return (0);
}