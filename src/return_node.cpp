#include <pcl_uca/pcl_uca.h>
#include <cartesian_interface/ros/RosClient.h>

int main ( int argc, char** argv ) {
    
    ros::init ( argc, argv, "cartesian_task" );
    
    XBot::Cartesian::RosClient ci_client;
    
    tf::TransformListener listener_wc;
    tf::StampedTransform transform_wc;

    try {
        listener_wc.waitForTransform ( "ci/world", "ci/car_frame", ros::Time(0), ros::Duration ( 3.0 ) );
        listener_wc.lookupTransform ( "ci/world", "ci/car_frame",ros::Time(0), transform_wc );
    } catch ( tf::TransformException ex ) {
        ROS_ERROR ( "%s",ex.what() );
        ros::Duration ( 1.0 ).sleep();
    }
    
    Eigen::Affine3d T_wc;
    tf::transformTFToEigen ( transform_wc, T_wc );
    
    Eigen::Matrix3d R;
    R << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    
    T_wc.translation().x() = 0;
    T_wc.translation().y() = 0;
    T_wc.translation().z() = T_wc.translation().z();
    T_wc.linear() = R;
    
    ci_client.setTargetPose("car_frame", T_wc, 5.0);
    ci_client.waitReachCompleted("car_frame");   
}