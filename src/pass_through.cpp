#include <ros/ros.h>
#include <iostream>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>

#include <boost/foreach.hpp>



typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_rcv (new pcl::PointCloud<pcl::PointXYZ>);

void callback(const PointCloud::ConstPtr& msg)
{
  // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
  
  cloud_rcv = msg;
     
}

int main(int argc, char** argv)
{     
  ros::init(argc, argv, "passThrough_filter");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/octomap_point_cloud_centers", 1, callback);
  
  ros::Publisher pub = nh.advertise<PointCloud>("/filtered_cloud", 10);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);   
  
  // tf::TransformListener listener;
  // tf::StampedTransform transform;

  
  ros::Rate loop_rate(10);
  
  while (ros::ok()) 
  {   
    // listener.lookupTransform("velodyne_calib", "wheel_1", ros::Time::now(), transform);   
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_rcv);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.1, 1.5);
    pass.filter(*cloud_filtered); 
    PointCloud msg_to_pub= *cloud_filtered;
    // msg_to_pub.header.frame_id = "pelvis";
    
    for (std::size_t i = 0; i < cloud_filtered->points.size (); ++i)
    {
        std::cout << "x: " << cloud_filtered->points[i].x
                  << "  y: " << cloud_filtered->points[i].y
                  << "  z: " << cloud_filtered->points[i].z << std::endl;   
    }      
    
    std::cout << " points #: " << cloud_filtered->points.size ()<< std::endl;
    
    pub.publish(msg_to_pub);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}


