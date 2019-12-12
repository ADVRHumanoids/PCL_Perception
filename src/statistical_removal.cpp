#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>


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
  ros::init(argc, argv, "statistical_removal_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/velodyne_points", 1, callback);
  
  ros::Publisher pub = nh.advertise<PointCloud>("/outlier_removal", 10);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);   
  
  ros::Rate loop_rate(10);
  
  while (ros::ok()) 
  { 
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_rcv);
    pass.setMeanK (atof(argv[1]));
    pass.setStddevMulThresh (atof(argv[2]));
    pass.filter(*cloud_filtered); 
    PointCloud msg_to_pub = *cloud_filtered;    
        
    pub.publish(msg_to_pub);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}
