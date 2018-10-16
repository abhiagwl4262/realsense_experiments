#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <boost/foreach.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <cmath>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

std::vector<cv::Point> vec_point;

void mouseHandler(int event, int x, int y, int flags, void* param) 
{
    if(event == CV_EVENT_LBUTTONDOWN){
        cv::Point *p = (cv::Point*)param;
        p->x = x;
        p->y = y;
        std::cout << p->x  << " " << p->y<< std::endl;
        vec_point.push_back(*p);
    }
}


class CloudBoundaries {
  public:
    float minX;
    float maxX;
    float minY;
    float maxY;
    float minZ;
    float maxZ;    
};
 

void findBoundaries(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, CloudBoundaries& boundaries) 
{
    boundaries.minX = cloud->points[0].x;
    boundaries.maxX = cloud->points[0].x;
    boundaries.minY = cloud->points[0].y;
    boundaries.maxY = cloud->points[0].y;
    boundaries.minZ = cloud->points[0].z;
    boundaries.maxZ = cloud->points[0].z;

    for (auto it = cloud->points.begin(); it != cloud->points.end(); ++it) {
        if ((*it).x > boundaries.maxX) {
                boundaries.maxX = (*it).x;
        }
        if ((*it).x < boundaries.minX) {
                boundaries.minX = (*it).x;
        }
        if ((*it).y > boundaries.maxY) {
                boundaries.maxY = (*it).y;
        }
        if ((*it).y < boundaries.minY) {
                boundaries.minY = (*it).y;
        }
        if ((*it).z > boundaries.maxZ) {
                boundaries.maxZ = (*it).z;
        }
        if ((*it).z < boundaries.minZ) {
                boundaries.minZ = (*it).z;
        }
    }
}

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  // if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/hitech/rs_ws/data/novel/1539704918328530.pcd", *cloud) == -1) //* load the file  
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/hitech/rs_ws/data/syska_box/1539700159119905.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width  << " " << cloud->height
            << "data points from test_pcd.pcd with the following fields: "
            << std::endl;
  

  cv::Mat image = cv::Mat( cloud->height, cloud->width, CV_8UC3);
  for( int y = 0; y < image.rows; y++ ) {
      for( int x = 0; x < image.cols; x++ ) {
          pcl::PointXYZRGB point = cloud->at( x, y );
          image.at<cv::Vec3b>( y, x )[0] = point.b;
          image.at<cv::Vec3b>( y, x )[1] = point.g;
          image.at<cv::Vec3b>( y, x )[2] = point.r;
      }
  }

  std::cout << image.cols << " " << image.rows << "\n" ;
  cv::Point p;
  cv::imshow("image",image);
  
  cv::setMouseCallback("image",mouseHandler,&p);
  cv::waitKey(0);


  for  (int i =0; i<vec_point.size();i++)
  {
    cout << vec_point[i].x << " " << vec_point[i].y << std::endl;
    cout << cloud->at(vec_point[i].y,vec_point[i].x) << std::endl;
  }

  pcl::PointXYZRGB point1 = cloud->at(vec_point[0].y,vec_point[0].x);
  pcl::PointXYZRGB point2 = cloud->at(vec_point[1].y,vec_point[1].x);

  float dist = sqrt(pow((point1.x - point2.x), 2) + pow((point1.y - point2.y),2) + pow((point1.z - point2.z),2));
  std::cout << "distance is " << dist << std::endl;;
  // float eucl_dist = pcl::euclideanDistance(cloud->at(vec_point[0].y, vec_point[0].x), cloud->at(vec_point[1].y, vec_point[1].x));

  cv::Rect2d r = cv::selectROI(image);
  std::cout << "rect is " << r << std::endl;;
  int topleft_x     = r.x;
  int topleft_y     = r.y;
  int bottomright_x = r.x + r.width;
  int bottomright_y = r.y + r.height;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);  

   for( int y = 0; y < cloud->height; y++ ) {
      for( int x = 0; x < cloud->width; x++ ) {

        pcl::PointXYZRGB point = cloud->at(x,y);
      	if(x > topleft_x &&  x < bottomright_x && y > topleft_y && y < bottomright_y )
      	{
      		cropped_cloud->push_back(point);
      	}

      }
  }

  std::cout << "Extract the point cloud" << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>); 

  pcl::copyPointCloud (*cropped_cloud, *cropped_cloud_xyz);
  

  CloudBoundaries CloudBoundaries_obj; 
  findBoundaries(cropped_cloud, CloudBoundaries_obj);
  std::cout << "z min is " << CloudBoundaries_obj.minZ << std::endl;;
  std::cout << "z max is " << CloudBoundaries_obj.maxZ << std::endl;;

  // pcl::PointXYZ minPt, maxPt;
  // pcl::getMinMax3D (*cropped_cloud_xyz, minPt, maxPt);
  // std::cout << "Max x: " << maxPt.x << std::endl;
  // std::cout << "Max y: " << maxPt.y << std::endl;
  // std::cout << "Max z: " << maxPt.z << std::endl;
  // std::cout << "Min x: " << minPt.x << std::endl;
  // std::cout << "Min y: " << minPt.y << std::endl;
  // std::cout << "Min z: " << minPt.z << std::endl;


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
