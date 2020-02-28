#include <iostream>
#include <chrono>
#include <pcl_uca/pcl_uca.h>
#include <ros/service_server.h>
#include <PCL_Perception/set_filter_params.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// double lower_limit = 0.1;
// double upper_limit = 1.5;
// std::string filterFieldName = "z";

double lower_limit;
double upper_limit;
std::string filterFieldName;
bool negative;

bool set_filter_params(PCL_Perception::set_filter_params::Request& req,
                       PCL_Perception::set_filter_params::Response& res)
{
    lower_limit = req.lower_lim;
    upper_limit = req.upper_lim;
    filterFieldName = req.filterFieldName;
 
    return true;
}

int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "passThrough_filter");
  ros::NodeHandle nh;
//   pcl_uca pcl ("octomap_point_cloud_centers", nh );
  pcl_uca pcl ("velodyne_points", nh );
  ros::Publisher pub = nh.advertise<PointCloud>("/filtered_cloud", 10);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  
  
  ros::ServiceServer service = nh.advertiseService("set_filter_params", set_filter_params);
  
  if (!nh.hasParam("lower_limit") || !nh.hasParam("upper_limit") || !nh.hasParam("filterFieldName"))
  {
    std::cerr << "Filter parameters missing!" << std::endl;
  }
  else
  {
    nh.getParam("lower_limit", lower_limit);
    nh.getParam("upper_limit", upper_limit);
    nh.getParam("filterFieldName", filterFieldName);
    nh.getParam("filter_behavior", negative);
  
  ros::Rate loop_rate(10);
  
  

  while (ros::ok()) 
  {  
    cloud_filtered = pcl.passThroughFilter(pcl.inputCloud(), filterFieldName, lower_limit, upper_limit, negative);
    
    pub.publish(cloud_filtered);
    
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  }
}


