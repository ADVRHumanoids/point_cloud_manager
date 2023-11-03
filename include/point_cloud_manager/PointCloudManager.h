#ifndef __CLOUD_MANAGER__
#define __CLOUD_MANAGER__

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/moment_of_inertia_estimation.h>


typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloud;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;

using namespace Eigen;

class PointCloudManager
{
    void point_cloud_callback(const PointCloudRGB::ConstPtr& msg, int i);

    ros::NodeHandle _nh;
    tf::TransformListener _listener;
    tf::StampedTransform _transform;

    Affine3d _camera_transf;
    
    PointCloudRGB::Ptr _merged_cloud;

    std::vector<ros::Subscriber> _cloud_subs;
    ros::Publisher _cloud_pub;
    std::vector<std::string> _input_topics;
    std::vector<PointCloudRGB::Ptr> _cloud_color_vect;
    std::vector<bool> _is_callback_done;

    pcl::search::KdTree<PointXYZ>::Ptr _tree;
    pcl::search::Search<PointRGB>::Ptr _tree_rgb;

    pcl::VoxelGrid<PointXYZ> _vox_grid;
    pcl::VoxelGrid<PointRGB> _vox_grid_color;
    pcl::PassThrough<PointXYZ> _pass_filter;
    pcl::PassThrough<PointRGB> _pass_filter_color;
    pcl::StatisticalOutlierRemoval<PointXYZ> _sor;

    std::vector<pcl::PointIndices> _cluster_indices;
    std::vector<PointCloud::Ptr> _cloud_vect;

    pcl::PointCloud <pcl::Normal>::Ptr _normals;
    pcl::NormalEstimation<PointXYZ, pcl::Normal> _normal_estimator;
    pcl::RegionGrowing<PointXYZ, pcl::Normal> _reg;
    pcl::RegionGrowingRGB<PointRGB> _color_reg;
    pcl::EuclideanClusterExtraction<PointXYZ> _ec;
    pcl::MomentOfInertiaEstimation <PointXYZ> _feature_extractor;
    pcl::ConvexHull<PointXYZ> _conv_hull;
    pcl::ConcaveHull<PointXYZ> _conc_hull;
    //pcl::ExtractIndices<pcl::PointXYZ> _extract;

    pcl::ModelCoefficients::Ptr _coefficients;
    pcl::SACSegmentation<PointXYZ> _sac_seg;
    pcl::PointIndices::Ptr _inliers;
    pcl::ProjectInliers<PointXYZ> _proj;

    // Functions -------------------------------------------------------------------------------
public:
    PointCloudManager(std::vector<std::string> input_topics);
    ~PointCloudManager();

    void resetParams();

    //Filtering
    void voxelDownsampling(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, double dim_x, double dim_y, double dim_z);
    void voxelDownsampling(PointCloudRGB::Ptr cloud_in, PointCloudRGB::Ptr cloud_out, double dim_x, double dim_y, double dim_z);
    void filterCloudXYZ(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
    void filterCloudAxis(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, double min, double max, std::string axis, bool negative = false);
    void filterCloudAxis(PointCloudRGB::Ptr cloud_in, PointCloudRGB::Ptr cloud_out, double min, double max, std::string axis, bool negative = false);
    void outlierRemoval(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, int mean_k = 50, double std_th = 1.0);

    //Transform
    void transformCloud(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, Affine3d& transf);
    void transformCloud(PointCloudRGB::Ptr cloud_in, PointCloudRGB::Ptr cloud_out, Affine3d& transf);

    //Segment
    void segmentRegionGrowing(PointCloud::Ptr cloud_in, PointCloudRGB::Ptr cloud_out, int min_points = 50, int max_points = 100000, double n_neigh = 0.10, double smootheness_th = 0.10, double curvature_th = 0.10);
    void segmentRegionGrowingRGB(PointCloudRGB::Ptr cloud_in, PointCloudRGB::Ptr cloud_out, int min_points = 50, int max_points = 100000, float dist_th = 0.05, float color_point_th = 0.2, float region_point_th = 0.2);
    void horizontalPlaneSegmentation(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out);
    void planeSegmentation(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out);
    std::vector<PointCloud::Ptr> euclideanClustering(PointCloud::Ptr cloud, double tolerance = 0.05, int min_size = 30, int max_size = 15000);

    //Boxes and Hulls
    void extractBoundingBox(PointCloud::Ptr cloud, pcl::PointXYZ *min_point_OBB, pcl::PointXYZ *max_point_OBB, pcl::PointXYZ *position_OBB, Eigen::Matrix3f *rotational_matrix_OBB);
    void getConvexHull(PointCloud::Ptr cloud, PointCloud::Ptr hull, int dimension = 3);
    void getConcaveHull(PointCloud::Ptr cloud, PointCloud::Ptr hull, int dimension = 3, double alpha = 0.1);

    void run();
};

#endif // __CLOUD_MANAGER__
