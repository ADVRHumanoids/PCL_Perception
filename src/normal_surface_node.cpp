#include<pcl_uca/pcl_uca.h>

#define RADIUS_NORMAL 0.3


int main (int argc, char** argv)
{
    ros::init(argc, argv, "normal_estimator");
    ros::NodeHandle nh;
    pcl_uca pcl_normal("filtered_cloud", nh);
    ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray> ("normals", 100);
    
    ros::Rate loop_rate (10);
    
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    visualization_msgs::MarkerArray ma;
    while (ros::ok())
    {             
        if (pcl_normal.isCallbackDone())
        {
           cloud_normals = pcl_normal.computeNormals(pcl_normal.inputCloud(), RADIUS_NORMAL);
           
           ma = pcl_normal.fromNormaltoMarkerArray(cloud_normals, pcl_normal.inputCloud(), "world", 0.1);
           
           pub.publish (ma);
           
        }
    
        ros::spinOnce ();
        loop_rate.sleep ();
    }
}