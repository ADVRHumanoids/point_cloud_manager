#include <ros/ros.h>

#include <iostream>
#include <point_cloud_manager/PointCloudManager.h>


int main(int argc, char** argv){

    ros::init(argc, argv, "cloud_manager");
    
    PointCloudManager pcm({"/zedx_left/zed_node/point_cloud/cloud_registered",
                           "/zedx_right/zed_node/point_cloud/cloud_registered"});

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        pcm.run();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
