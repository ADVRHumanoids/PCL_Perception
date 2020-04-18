#ifndef _NORMAL_SURFACE_H_
#define _NORMAL_SURFACE_H_

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
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
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>


class pcl_uca
{
public:
    pcl_uca(const std::string& from_topic, ros::NodeHandle& nh);
    
    /**
     * Pass Through FIlter along a specified direction with set boundaries
     * @param: inputCloud -> Point Cloud Data in input
     *         filterFieldName -> Axis along which the filtering is performed
     *         lower_lim -> lower boundary
     *         upperl_lim -> upper boundary
     */ 
    pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud,
                                                          const std::string filterFieldName,
                                                          const float lower_lim,
                                                          const float upper_lim,
                                                          const bool negative);
    
    /**
     * Computes normals from internal Point Cloud
     * @param: inputCloud -> Point Cloud Data in input
     *         radius_search -> area used for normal computation
     */
    pcl::PointCloud<pcl::Normal>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud,
                                                     double radius_search);
    
    /**
     * Perform a planar segmentation using the normals of each point inside the Point Cloud
     * @param: inputCloud -> Point Cloud Data in input
     *         coefficients -> Contains the coefficients of the extracted plane (ax + by + cz + d = 0)
     *         inliers -> Contains the indices of the inliers relative to the extracted plane
     */
    void planarSegmentationFromNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud,
                                       pcl::ModelCoefficients::Ptr coefficients,
                                       pcl::PointIndices::Ptr inliers,
                                       double normalDistanceWeight,
                                       int maxIterations,
                                       double distanceThreshold);
    
    /**
     * Advertise the computed normals as a MarkerArray
     * @param: cloud_normals -> computed normals
     *         inputCloud -> Point Cloud Data from which the normals have been computed
     *         frame_id -> base frame 
     */
    visualization_msgs::MarkerArray fromNormaltoMarkerArray(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                                                            pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud,
                                                            std::string frame_id, const double arrow_scale = 1.0);
    
    /**
     * Returns a boolean = 1 when the callback is done 
     */
    bool isCallbackDone();
    
    /**
     * Returns the input cloud loaded from from topic
     */
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud();
    
    
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
    
    /**
     * Returns the homogenous matrix that goes from an arbitrary parent frame to a frame with the z axis coincident with the computed normal
     * @param: origin -> Pivot point of the normal
     *         normal -> normal components 
     *         inputCloud -> the Point Cloud Data used to compute the averaged normal
     */
    Eigen::Affine3d fromNormalToHomogeneous(pcl::PointCloud<pcl::PointXYZ>::Ptr origin,
                                            pcl::PointCloud<pcl::Normal>::Ptr normal);
    /**
     * Broadcast the homogenous matrix 
     */
    void broadcastTF(Eigen::Affine3d inputHM,
                     std::string parent_frame,
                     std::string child_frame);
     
    
private:
    /**
     * Internal Point Cloud
     */
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr _cloud_rcv;
    
    ros::Subscriber _sub; 
    
    ros::NodeHandle& _nh;
    
    bool _callback_done = false;
    
    void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
    
    Eigen::Matrix3d _rotationMatrix;
    
    tf::TransformBroadcaster _broadcaster;
    

};

#endif