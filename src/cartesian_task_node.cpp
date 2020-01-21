#include <pcl_uca/pcl_uca.h>
#include <cartesian_interface/ros/RosImpl.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <PCL_Perception/SetReferenceFrame.h>

int main ( int argc, char** argv ) {
    
    ros::init ( argc, argv, "cartesian_task" );  
    ros::NodeHandle nh("~");
    bool start_broadcast = false;

    // Dataset
    XBot::Cartesian::RosImpl ci_client;
    Eigen::Affine3d T_ref; //FINAL REFERENCE SENT TO ROBOT
    
    // Create the reference car_frame   
    tf::TransformBroadcaster broadcaster;
    
    
    auto set_reference = [&broadcaster, &T_ref, &start_broadcast](PCL_Perception::SetReferenceFrame::Request& req, PCL_Perception::SetReferenceFrame::Response& res) -> bool
    {
        
        tf::TransformListener listener_wc, listener_wn;
        tf::StampedTransform transform_wc, transform_wn;
        Eigen::Affine3d T_wn;
        Eigen::Matrix3d R_wn;
        Eigen::Vector3d x_nf;

        Eigen::Array<double, 1, 1> value;

        // Get the transform world/normal and  world/car_frame
        ros::Time now = ros::Time(0);
        try {
            listener_wc.waitForTransform ( "ci/world_odom", "ci/car_frame", now, ros::Duration ( 3.0 ) );
            listener_wc.lookupTransform ( "ci/world_odom", "ci/car_frame", now, transform_wc );
            listener_wn.waitForTransform ( "ci/world_odom", "normal_frame", now, ros::Duration ( 3.0 ) );
            listener_wn.lookupTransform ( "ci/world_odom", "normal_frame", now, transform_wn );
        } catch ( tf::TransformException ex ) {
            ROS_ERROR ( "%s",ex.what() );
            ros::Duration ( 1.0 ).sleep();
        }
          
        // Create the affine matrix that will be sent to the robot. The robot will move towards 
        // the plane at a distance equal to the normal length, keeping the same initial z
        tf::transformTFToEigen ( transform_wn, T_wn );
        T_ref = T_wn;
        T_ref.translation().x() += req.pose_offset.position.x;
        T_ref.translation().y() += req.pose_offset.position.y;
        T_ref.translation().z() = transform_wc.getOrigin().getZ() + req.pose_offset.position.z;
        const Eigen::Quaternion<double> rotation(req.pose_offset.orientation.w, req.pose_offset.orientation.x, req.pose_offset.orientation.y, req.pose_offset.orientation.z);
        T_ref.rotate(rotation);
          
        start_broadcast = true;
        return true;
    };
    
    auto set_service = nh.advertiseService<PCL_Perception::SetReferenceFrame::Request, PCL_Perception::SetReferenceFrame::Response>("set_reference", set_reference);
    
    // Move the robot
    auto send_reference = [&T_ref, &ci_client](std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) -> bool
    {
        ci_client.setTargetPose("car_frame", T_ref, 10.0);
        ci_client.waitReachCompleted("car_frame");
        return true;
    };
    
    auto send_service = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("send_reference", send_reference);
    
    tf::Transform transform;
    ros::Rate rate(10);
    while(ros::ok())
    {
        if(start_broadcast)
        {
            tf::transformEigenToTF(T_ref, transform);
            broadcaster.sendTransform(tf::StampedTransform ( transform, ros::Time::now(), "ci/world_odom", "reference_frame" ) );
        }
        ros::spinOnce();
        rate.sleep();
    }
    

}





