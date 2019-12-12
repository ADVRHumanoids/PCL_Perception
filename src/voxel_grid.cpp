#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/foreach.hpp>

/* typedef pcl::PCLPointCloud2 PointCloud;

pcl::PCLPointCloud2::ConstPtr cloud_rcv (new pcl::PCLPointCloud2);

void callback(const PointCloud::ConstPtr& msg)
{
  // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  //BOOST_FOREACH (const pcl::PCLPointCloud2& pt, msg->data )
  // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
  
  cloud_rcv = msg;
     
}

int main(int argc, char** argv)
{     
  ros::init(argc, argv, "voxel_filter");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/octomap_point_cloud_centers", 1, callback);
  
  ros::Publisher pub = nh.advertise<PointCloud>("/VoxelGrid_filtered_cloud", 10);
  
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);   
  
  ros::Rate loop_rate(10);
  
  while (ros::ok()) 
  { 
    std::cerr << "PointCloud before filtering: " << cloud_rcv->width * cloud_rcv->height << std::endl;

    pcl::VoxelGrid<pcl::PCLPointCloud2> pass;
    pass.setInputCloud(cloud_rcv);
    pass.setLeafSize (0.3f, 0.3f, 0.3f);
    pass.filter(*cloud_filtered); 
    PointCloud msg_to_pub = *cloud_filtered;    
        
    pub.publish(msg_to_pub);
    
       
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << std::endl;
    
    ros::spinOnce();
    loop_rate.sleep();
  }
} */


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_rcv (new pcl::PointCloud<pcl::PointXYZ>);

void callback(const PointCloud::ConstPtr& msg)
{
  // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  //BOOST_FOREACH (const pcl::PCLPointCloud2& pt, msg->data )
  // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
  
  cloud_rcv = msg;
     
}

int main(int argc, char** argv)
{     
  ros::init(argc, argv, "voxel_filter");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/octomap_point_cloud_centers", 1, callback);
  
  ros::Publisher pub = nh.advertise<PointCloud>("/voxel_grid_filtered_cloud", 10);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);   
  
  ros::Rate loop_rate(10);
  
  while (ros::ok()) 
  { 
    std::cerr << "PointCloud before filtering: " << cloud_rcv->width * cloud_rcv->height << std::endl;

    pcl::VoxelGrid<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_rcv);
    pass.setLeafSize (0.3f, 0.3f, 0.3f);
    pass.filter(*cloud_filtered); 
    PointCloud msg_to_pub = *cloud_filtered;    
        
    pub.publish(msg_to_pub);
    
       
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << std::endl;
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}
