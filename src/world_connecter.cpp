#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "world_connecter");
    
    if(argc != 5)
    {
        std::cout << "Usage: " << argv[0] << " world_1 world_2 root_1 root_2" << std::endl;
        exit(1);
    }
    
    std::string world_1(argv[1]), world_2(argv[2]), root_1(argv[3]), root_2(argv[4]);
    
    tf::TransformListener listener;
    
    tf::TransformBroadcaster br;

    ros::Rate rate(10);
  
    
    while(ros::ok())
    {
        
        tf::StampedTransform w1_T_r1, w2_T_r2;
        
        try{
            listener.waitForTransform(world_1, root_1, ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform(world_1, root_1,  
                                     ros::Time(0), w1_T_r1);
            listener.waitForTransform(world_2, root_2, ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform(world_2, root_2,  
                                     ros::Time(0), w2_T_r2);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        
        w1_T_r1.setOrigin(w2_T_r2.getOrigin());
        
        br.sendTransform(tf::StampedTransform(w1_T_r1, ros::Time::now(), world_1, root_1));
        
        rate.sleep();
    }
    
    return true;
    
}