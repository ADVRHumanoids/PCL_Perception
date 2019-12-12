#ifndef _NORMAL_SURFACE_H_
#define _NORMAL_SURFACE_H_

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/PointIndices.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigen>


class pcl_uca
{
public:
    pcl_uca(const std::string& from_topic, ros::NodeHandle& nh);
    
    /**
     * return internal Point Cloud used forn normal computation
     */
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr getPointCloud()
    { 
        return _cloud_rcv;
    }
    
    /**
     * Computes normals from internal Point Cloud
     * @param: inputCloud -> Point Cloud Data in input
     *         radius_search -> area used for normal computation
     */
    pcl::PointCloud<pcl::Normal>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud,
                                                     double radius_search);
    
    /**
     * Advertise the computed normals as a MarkerArray
     * @param: cloud_normals -> computed normals
     *         inputCloud -> Point Cloud Data from which the normals have been computed
     *         frame_id -> base frame 
     */
    visualization_msgs::MarkerArray fromNormaltoMarkerArray(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                                                            pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud,
                                                            std::string frame_id, const double arrow_scale = 1.0);
    
    bool isCallbackDone();
    
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud();
    
    void publish (pcl::PointCloud<pcl::Normal>::Ptr msg_to_pub);
    
    void publish (pcl::PointCloud<pcl::PointXYZ>::Ptr msg_to_pub);
    
    /**
     * Compute the average point starting from a Point Cloud
     * @param: inputCloud -> Point Cloud Data in input
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr averagePoints (pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud);
    
     /**
     * Compute the average normal starting from a Point Cloud
     * @param: inputCloud -> Point Cloud Data Normal in input
     */
    pcl::PointCloud<pcl::Normal>::Ptr averageNormals (pcl::PointCloud<pcl::Normal>::Ptr inputCloudNormals);
    
    Eigen::Matrix4f fromNormalToHomogeneous(pcl::PointCloud<pcl::PointXYZ>::Ptr origin,
                                            pcl::PointCloud<pcl::Normal>::Ptr normal,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
    
    void broadcastTF(Eigen::Matrix4f inputHM,
                     std::string parent_frame,
                     std::string child_frame);
     
    
private:
    /**
     * Internal Point Cloud
     */
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr _cloud_rcv;
    
    ros::Publisher _pub;
    ros::Subscriber _sub; 
    
    ros::NodeHandle& _nh;
    
    bool _callback_done = false;
    
    void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
    
    Eigen::Matrix3f _rotationMatrix;
    
    tf::TransformBroadcaster _broadcaster;
    

};

#endif