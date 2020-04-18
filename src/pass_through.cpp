#include <iostream>
#include <chrono>
#include <pcl_uca/pcl_uca.h>
#include <ros/service_server.h>
#include <PCL_Perception/set_filter_params.h>
#include <sstream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

double lower_limit;
double upper_limit;
std::string filterFieldName;
bool negative;

bool set_filter_params(PCL_Perception::set_filter_params::Request& req,
                       PCL_Perception::set_filter_params::Response& res)
{
    lower_limit = req.lower_lim;
    upper_limit = req.upper_lim;
    filterFieldName = req.filterFieldName;
    negative = req.negative;
 
    return true;
}

int main(int argc, char** argv)
{     
    ros::init(argc, argv, "passThrough_filter");
    ros::NodeHandle nh;
    
    std::cout << "passed :\n" << argv[1] << " " << argv[2] << " " << argv[3] << " " << argv[4] << std::endl;
    
    if (argc != 5)
        throw std::runtime_error("Error: pass exactly the 4 arguments 'lower_limit', 'upper_limit', 'filterFieldName', 'negative'");
    lower_limit = atof(argv[1]);
    upper_limit = atof(argv[2]);
    filterFieldName = argv[3];
    
    std::stringstream ss(argv[4]);
    ss >> std::boolalpha >> negative;
    
    std::cout << "corresponding values are:\n" << lower_limit << " " << upper_limit << " " << filterFieldName << " " << negative << std::endl;
    
    pcl_uca pcl ("cloud_in", nh );
    ros::Publisher pub = nh.advertise<PointCloud>("filtered_cloud", 10);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    
    
    ros::ServiceServer service = nh.advertiseService("set_filter_params", set_filter_params);
    
    ros::Rate loop_rate(10);
    
    

    while (ros::ok()) 
    {  
        cloud_filtered = pcl.passThroughFilter(pcl.inputCloud(), filterFieldName, lower_limit, upper_limit, negative);
        
        pub.publish(cloud_filtered);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}


