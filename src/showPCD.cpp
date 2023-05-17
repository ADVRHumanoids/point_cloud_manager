#include <ros/ros.h>
#include <iostream>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "showPCD");
    ros::NodeHandle nh;

    ros::Publisher cloud_pub = nh.advertise<PointCloud>("point_cloud", 1);

    PointCloud::Ptr cloud (new PointCloud);

    std::string file_name = argv[1];

    ros::Rate loop_rate(5);

    while(ros::ok()){

        if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            return (-1);
        }

        cloud->header.frame_id = "map";
        pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
        cloud_pub.publish(cloud);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
