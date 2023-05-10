// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <point_cloud_manager/PointCloudManager.h>

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudT;

using namespace Eigen;

//Global Vars
PointCloudT::Ptr _cloud (new PointCloudT);
PointCloudT::Ptr _cloud2 (new PointCloudT);
std::vector<PointCloudT::Ptr> _cloud_vector;

bool _acquired = false;
//------

void acquire_cloud(const PointCloudT::ConstPtr& msg)
{
    if(msg->points.size() < 200)
        return ;

    *_cloud = *msg;
    _acquired = true;
}

void senseTF(tf::TransformListener * listener, tf::StampedTransform *tr, Affine3d *transf, std::string source, std::string target){
    try{
        listener->waitForTransform (target, source, ros::Time(0), ros::Duration (1.0));
        listener->lookupTransform(target, source, ros::Time(0) , (*tr));

        tf::transformTFToEigen((*tr), (*transf));
    } catch ( tf::TransformException ex ) {
        ROS_ERROR ( "%s",ex.what() );
        ros::Duration ( 1.0 ).sleep();
    }
}

visualization_msgs::Marker boxMarker(PointXYZ min_point_OBB, PointXYZ max_point_OBB, PointXYZ position_OBB, Eigen::Matrix3f rotational_matrix_OBB, int id, std::string link_name){
    visualization_msgs::Marker marker;

    Quaternionf quat (rotational_matrix_OBB);

    marker.header.stamp = ros::Time();
    marker.header.frame_id = link_name;
    marker.ns = "bounding_box";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = position_OBB.x;
    marker.pose.position.y = position_OBB.y;
    marker.pose.position.z = position_OBB.z;
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();

    marker.scale.x = max_point_OBB.x - min_point_OBB.x+0.001;
    marker.scale.y = max_point_OBB.y - min_point_OBB.y+0.001;
    marker.scale.z = max_point_OBB.z - min_point_OBB.z+0.001;
    marker.color.a = 0.7; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    ROS_INFO("Object at: %f, %f, %f -- Size: %f, %f, %f", marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, marker.scale.x, marker.scale.y, marker.scale.z);

    return marker;
}


int main(int argc, char** argv){

    ros::init(argc, argv, "cloud_manager");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<PointCloudT>("/filtered_cloud", 2);
    ros::Publisher pub_marker = nh.advertise<visualization_msgs::MarkerArray>("/bounding_markers", 2);
    ros::Subscriber subscriber = nh.subscribe("/velodyne_points", 1, acquire_cloud);

    PointCloudT::Ptr msg (new PointCloudT);

    PointCloudManager pcm;

    tf::TransformListener _listener;
    tf::StampedTransform _tr;
    Affine3d _transf;

    PointXYZ min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    visualization_msgs::MarkerArray marker_array;

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        if(_acquired){

            senseTF(&_listener, &_tr, &_transf, _cloud->header.frame_id, "pelvis");

            pcm.voxelDownsampling(_cloud, _cloud, 0.01, 0.01, 0.01);
            pcm.outlierRemoval(_cloud, _cloud, 50, 1.0);

            pcm.transformCloud(_cloud, _cloud, _transf);

            pcm.filterCloudAxis(_cloud, _cloud, -0.7, 2.0, "z", false);
            _cloud_vector = pcm.euclideanClustering(_cloud, 0.05, 30, 15000);

            *_cloud2 = *_cloud;

            msg->points.clear();

            for(int i = 0; i < _cloud_vector.size(); i++){

                pcm.horizontalPlaneSegmentation(_cloud_vector[i], _cloud);

                pcm.getConcaveHull(_cloud, _cloud, 2, 0.05);
                *msg = *msg + *_cloud;

                pcm.extractBoundingBox(_cloud, &min_point_OBB, &max_point_OBB, &position_OBB, &rotational_matrix_OBB);

                marker_array.markers.clear();
                marker_array.markers.push_back(boxMarker(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB, i+1, "pelvis"));

                pub_marker.publish(marker_array);
            }

            msg->header.frame_id = "pelvis";

            _acquired = false;

            pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
            pub.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
