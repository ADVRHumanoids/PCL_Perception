#include <iostream>
#include <pcl/filters/extract_indices.h>
#include <pcl_uca/pcl_uca.h>


int main ( int argc, char** argv ) {
    ros::init ( argc, argv, "planar_segmentation" );
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >( "plane_segmentation", 1 );
    ros::Publisher pub_avg = nh.advertise<visualization_msgs::MarkerArray> ( "normal_plane", 1 );
    ros::Publisher pub_seg = nh.advertise<visualization_msgs::MarkerArray> ( "normals_plane", 1);
    pcl_uca pcl_segmentation ( "filtered_cloud", nh );

    ros::Rate loop_rate ( 10 );

    // Dataset
    pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
    pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normal;

    while ( ros::ok() ) {
        if ( pcl_segmentation.isCallbackDone() ) {
            // Extract normals
            cloud_normals = pcl_segmentation.computeNormals ( pcl_segmentation.inputCloud(), 0.3 );

            // Create the segmentation object
            pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
            // Optional
            seg.setModelType ( pcl::SACMODEL_NORMAL_PLANE );
            seg.setNormalDistanceWeight ( 0.1 );
            seg.setMethodType ( pcl::SAC_RANSAC );
            seg.setMaxIterations ( 100 );
            seg.setDistanceThreshold ( 0.03 );
            seg.setInputCloud ( pcl_segmentation.inputCloud() );
            seg.setInputNormals ( cloud_normals );
            seg.segment ( *inliers, *coefficients );

            // Extract inliers
            extract.setInputCloud ( pcl_segmentation.inputCloud() );
            extract.setIndices ( inliers );
            extract.setNegative ( false );

            // Write the planar inliers
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane ( new pcl::PointCloud<pcl::PointXYZ> );
            extract.filter ( *cloud_plane );
            pub.publish ( cloud_plane );
            
            if ( inliers->indices.size () == 0 ) {
                PCL_ERROR ( "Could not estimate a planar model for the given dataset." );
                return ( -1 );
            }

            // Calculate normals of cloud_plane
            pcl::PointCloud<pcl::Normal>::Ptr cloud_plane_normals ( new pcl::PointCloud<pcl::Normal> );
            pcl::PointCloud<pcl::PointXYZ>::Ptr averagePoints ( new pcl::PointCloud<pcl::PointXYZ> );
            pcl::PointCloud<pcl::Normal>::Ptr averageNormals ( new pcl::PointCloud<pcl::Normal> );

            visualization_msgs::MarkerArray maAverage;
            visualization_msgs::MarkerArray ma_seg;

            cloud_plane_normals = pcl_segmentation.computeNormals ( cloud_plane, 0.06 );
            averagePoints = pcl_segmentation.averagePoints ( cloud_plane );
            averageNormals = pcl_segmentation.averageNormals ( cloud_plane_normals );
            
            // Transorm the computed normal in a MarkerArray
            maAverage = pcl_segmentation.fromNormaltoMarkerArray ( averageNormals, averagePoints, "world", 0.5 );
            ma_seg = pcl_segmentation.fromNormaltoMarkerArray ( cloud_plane_normals, cloud_plane, "world", 0.1 );
        
            // Publish it on rviz
            pub_avg.publish ( maAverage );
            pub_seg.publish ( ma_seg );

            // Create the homogenous matrix related to the computed normal
            Eigen::Matrix4f hM;
            hM = pcl_segmentation.fromNormalToHomogeneous(averagePoints, averageNormals, cloud_plane);
            
            // Brodcast the created frame
            pcl_segmentation.broadcastTF(hM, "world", "normal_frame");
            
        }


        ros::spinOnce ();
        loop_rate.sleep ();
    }

}
