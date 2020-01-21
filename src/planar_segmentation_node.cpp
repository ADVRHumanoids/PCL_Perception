#include <iostream>
#include <pcl/filters/extract_indices.h>
#include <pcl_uca/pcl_uca.h>
#include <ros/service_server.h>
#include <PCL_Perception/set_segmentation_params.h>
#include <chrono>

double normalDistanceWeight = 0.1;
int maxIterations = 100;
double distanceThreshold = 0.1;

bool set_segmentation_params(PCL_Perception::set_segmentation_params::Request& req,
                       PCL_Perception::set_segmentation_params::Response& res)
{
    normalDistanceWeight = req.normalDistanceWeight;
    maxIterations = req.maxIterations;
    distanceThreshold = req.distanceThreshold;
 
    return true;
}

int main ( int argc, char** argv ) {
    ros::init ( argc, argv, "planar_segmentation" );
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ( "plane_segmentation", 1 );
    ros::Publisher pub_avg = nh.advertise<visualization_msgs::MarkerArray> ( "normal_plane", 1 );
    ros::Publisher pub_seg = nh.advertise<visualization_msgs::MarkerArray> ( "normals_plane", 1 );
    ros::ServiceServer service = nh.advertiseService("set_segmentation_params", set_segmentation_params);
    pcl_uca pcl_segmentation ( "filtered_cloud", nh );

    ros::Rate loop_rate ( 10 );

    // Dataset
    pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
    pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normal;

    while ( ros::ok() ) {
        if ( pcl_segmentation.isCallbackDone() ) {
            
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

            pcl_segmentation.planarSegmentationFromNormals ( pcl_segmentation.inputCloud(), coefficients, inliers, normalDistanceWeight, maxIterations, distanceThreshold);
            
            extract.setInputCloud ( pcl_segmentation.inputCloud() );
            extract.setIndices ( inliers );
            extract.setNegative ( false );

            // Write the planar inliers
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane ( new pcl::PointCloud<pcl::PointXYZ> );
            extract.filter ( *cloud_plane );
            pub.publish ( cloud_plane );
            
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
          //  std::cout << "Planar segmentation took " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << "us.\n";

            if ( inliers->indices.size () == 0 ) {
                PCL_ERROR ( "Could not estimate a planar model for the given dataset." );
                return ( -1 );
            }

            // Calculate normals of cloud_plane
            start = std::chrono::steady_clock::now();
            
            pcl::PointCloud<pcl::Normal>::Ptr cloud_plane_normals ( new pcl::PointCloud<pcl::Normal> );
            pcl::PointCloud<pcl::PointXYZ>::Ptr averagePoints ( new pcl::PointCloud<pcl::PointXYZ> );
            pcl::PointCloud<pcl::Normal>::Ptr averageNormals ( new pcl::PointCloud<pcl::Normal> );
           
            visualization_msgs::MarkerArray maAverage;
            visualization_msgs::MarkerArray ma_seg;

            cloud_plane_normals = pcl_segmentation.computeNormals ( cloud_plane, 0.17 );
            averagePoints = pcl_segmentation.averagePoints ( cloud_plane );
            // averageNormals = pcl_segmentation.averageNormals ( cloud_plane_normals );
            averageNormals->width = 1;
            averageNormals->height = 1;
            averageNormals->points.resize ( averageNormals->width * averageNormals->height );
            averageNormals->points[0].normal_x = cloud_plane_normals->points[ ( cloud_plane_normals->width ) /2 + 10].normal_x;
            averageNormals->points[0].normal_y = cloud_plane_normals->points[ ( cloud_plane_normals->width ) /2 + 10].normal_y;
            averageNormals->points[0].normal_z = cloud_plane_normals->points[ ( cloud_plane_normals->width ) /2 + 10].normal_z;

            // Transorm the computed normal in a MarkerArray
            maAverage = pcl_segmentation.fromNormaltoMarkerArray ( averageNormals, averagePoints, "ci/world_odom", 0.5 );
            ma_seg = pcl_segmentation.fromNormaltoMarkerArray ( cloud_plane_normals, cloud_plane, "ci/world_odom", 0.1 );

            // Publish it on rviz
            pub_avg.publish ( maAverage );
            pub_seg.publish ( ma_seg );
            end = std::chrono::steady_clock::now();
           // std::cout << "Marker generation took " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << "us.\n";

            // Create the homogenous matrix related to the computed normal
            Eigen::Affine3d hM;
            start = std::chrono::steady_clock::now();
            
            hM = pcl_segmentation.fromNormalToHomogeneous ( averagePoints, averageNormals); 

            // Brodcast the created frame
            pcl_segmentation.broadcastTF ( hM, "ci/world_odom", "normal_frame" );
            
            end = std::chrono::steady_clock::now();
            //std::cout << "Frame generation took " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << "us.\n";

        }


        ros::spinOnce ();
        loop_rate.sleep ();
    }

}
