#include <pcl_uca/pcl_uca.h>
#include <cmath>
#include <cfloat>
#include <iostream>

void pcl_uca::callback ( const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg ) {
    _cloud_rcv = msg;
    _callback_done = true;
}

pcl_uca::pcl_uca ( const std::string& from_topic, ros::NodeHandle& nh ) :
    _cloud_rcv ( new pcl::PointCloud<pcl::PointXYZ> ),
    _nh ( nh ),
    _rotationMatrix() {

    _sub = _nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > ( from_topic,
            1,
            &pcl_uca::callback,
            this );
}

bool pcl_uca::isCallbackDone() {
    return _callback_done;
}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl_uca::inputCloud() {
    return _cloud_rcv;
}



pcl::PointCloud<pcl::Normal>::Ptr pcl_uca::computeNormals ( pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud,
        const double radius_search ) {
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals ( new pcl::PointCloud<pcl::Normal> );

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud ( inputCloud );

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given)
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree ( new pcl::search::KdTree<pcl::PointXYZ> );
    ne.setSearchMethod ( tree );

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch ( radius_search );

    // Compute normals
    ne.compute ( *cloud_normals );

    return cloud_normals;
}

visualization_msgs::MarkerArray pcl_uca::fromNormaltoMarkerArray ( pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud,
        std::string frame_id, const double arrow_scale ) {

    geometry_msgs::Point pnt_start, pnt_end;
    visualization_msgs::MarkerArray ma;


    ros::Time t = ros::Time::now();
    for ( int i = 0; i < inputCloud->width; i++ ) {

        // Assign values to Marker fields
        if ( !std::isnan ( cloud_normals->points[i].normal_x ) && !std::isnan ( cloud_normals->points[i].normal_y ) && !std::isnan ( cloud_normals->points[i].normal_z ) ) {
            visualization_msgs::Marker m;

            m.header.frame_id = frame_id;
            m.header.stamp = t;
            m.id = i;
            m.action = visualization_msgs::Marker::ADD;
            m.type = visualization_msgs::Marker::ARROW;
            pnt_start.x = inputCloud->points[i].x;
            pnt_start.y = inputCloud->points[i].y;
            pnt_start.z = inputCloud->points[i].z;
            m.points.push_back ( pnt_start );

            pnt_end.x = pnt_start.x + arrow_scale*cloud_normals->points[i].normal_x;
            pnt_end.y = pnt_start.y + arrow_scale*cloud_normals->points[i].normal_y;
            pnt_end.z = pnt_start.z + arrow_scale*cloud_normals->points[i].normal_z;
            m.points.push_back ( pnt_end );
            m.scale.x = 0.01;
            m.scale.y = 0.02;
            m.scale.z = 0.02;
            m.color.r = 255;
            m.color.g = 0;
            m.color.b = 0;
            m.color.a = 1;

            // Assign Marker to MarkerArray
            ma.markers.push_back ( m );
        }
    }
    return ma;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_uca::averagePoints ( pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud ) {
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;

    for ( int i = 0; i < inputCloud->width; i++ ) {
        sum_x += inputCloud->points[i].x;
        sum_y += inputCloud->points[i].y;
        sum_z += inputCloud->points[i].z;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr average ( new pcl::PointCloud<pcl::PointXYZ>); 

    average->width = 1;
    average->height = 1;
    average->points.resize (average->width * average->height);
    average->points[0].x = sum_x/inputCloud->width;
    average->points[0].y = sum_y/inputCloud->width;
    average->points[0].z = sum_z/inputCloud->width;

    return average;
}

pcl::PointCloud< pcl::Normal >::Ptr pcl_uca::averageNormals ( pcl::PointCloud< pcl::Normal >::Ptr inputCloudNormals ) {
    float sumNormal_x = 0;
    float sumNormal_y = 0;
    float sumNormal_z = 0;

    pcl::PointCloud<pcl::Normal>::Ptr average ( new pcl::PointCloud<pcl::Normal> );

    for ( int i = 0; i < inputCloudNormals->width; i++ ) {
        if (!std::isnan (inputCloudNormals->points[i].normal_x) ){
        sumNormal_x += inputCloudNormals->points[i].normal_x;}
        if (!std::isnan (inputCloudNormals->points[i].normal_y) ){
        sumNormal_y += inputCloudNormals->points[i].normal_y;}
        if (!std::isnan (inputCloudNormals->points[i].normal_z) ){
        sumNormal_z += inputCloudNormals->points[i].normal_z;}
    }

    average->width = 1;
    average->height = 1;
    average->points.resize (average->width * average->height);
    average->points[0].normal_x = sumNormal_x/inputCloudNormals->width;
    average->points[0].normal_y = sumNormal_y/inputCloudNormals->width;
    average->points[0].normal_z = sumNormal_z/inputCloudNormals->width;

    return average;
}

Eigen::Matrix4f pcl_uca::fromNormalToHomogeneous(pcl::PointCloud<pcl::PointXYZ>::Ptr origin,
                                                pcl::PointCloud<pcl::Normal>::Ptr normal,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud) {
    // Dataset
    Eigen::Vector3f x_child, y_child, z_child, pointChildFrame;
    Eigen::Vector3f x_parent (1, 0, 0), y_parent (0, 1, 0), z_parent (0, 0, 1);
    
    z_child << normal->points[0].normal_x,
               normal->points[0].normal_y,
               normal->points[0].normal_z;
    
    
    // Choose an arbitrary point inside the plane Point Cloud and create the x_child versor
    int ind = inputCloud->width / 2;
    pointChildFrame << inputCloud->points[ind].x - origin->points[0].x, 
                       inputCloud->points[ind].y - origin->points[0].y,
                       inputCloud->points[ind].z - origin->points[0].z;
    
    x_child << pointChildFrame / pointChildFrame.norm();
    
    // Create the y_child versor using the cross product between z_child and x_child
    y_child = z_child.cross(x_child);
    
    // Create the rotation matrix between parent and child frames
    _rotationMatrix << x_child.transpose() * x_parent, y_child.transpose() * x_parent, z_child.transpose() * x_parent,
                       x_child.transpose() * y_parent, y_child.transpose() * y_parent, z_child.transpose() * y_parent,
                       x_child.transpose() * z_parent, y_child.transpose() * z_parent, z_child.transpose() * z_parent;
    
    // Create the homogeneous matrix
    Eigen::Matrix4f hM;
    hM.block<3,3>(0,0) = _rotationMatrix;
    hM.block<1,4>(3,0) << 0, 0, 0, 1;
    hM.block<3,1>(0,3) << origin->points[0].x, origin->points[0].y, origin->points[0].z;
    
    return hM;
}    

void pcl_uca::broadcastTF (Eigen::Matrix4f inputHM,
                           std::string parent_frame,
                           std::string child_frame) {
    tf::Transform transform;
    tf::TransformBroadcaster broadcaster;
    
    const tfScalar x[3] = {inputHM(3,0), inputHM(3,1), inputHM(3,2)};
    const tfScalar rot[3][3] = {{inputHM(0,0), inputHM(0,1), inputHM(0,2)},
                                {inputHM(1,0), inputHM(1,1), inputHM(1,2)},
                                {inputHM(2,0), inputHM(2,1), inputHM(2,2)}};
    
    transform.setOrigin ( tf::Vector3(x[0], x[1], x[2]) );
    transform.setBasis( tf::Matrix3x3(rot[0][0], rot[0][1], rot[0][2],
                                      rot[1][0], rot[1][1], rot[1][2],
                                      rot[2][0], rot[2][1], rot[2][2]) ); 
    
    std::cout << inputHM.block<3,3>(0,0) << std::endl << inputHM.block<3,1>(0,2) << std::endl << std::endl;
    
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame) );
}


void pcl_uca::publish ( pcl::PointCloud<pcl::Normal>::Ptr msg_to_pub ) {
    _pub.publish ( msg_to_pub );
}

void pcl_uca::publish ( pcl::PointCloud<pcl::PointXYZ>::Ptr msg_to_pub ) {
    _pub.publish ( msg_to_pub );
}


