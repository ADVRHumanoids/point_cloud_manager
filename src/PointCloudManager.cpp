#include <point_cloud_manager/PointCloudManager.h>

PointCloudManager::PointCloudManager (std::vector<std::string> input_topics):
_input_topics(input_topics),
_nh("")
{
    resetParams();
    _cloud_subs.resize(_input_topics.size());
    _cloud_color_vect.resize(_input_topics.size());
    _is_callback_done.resize(_input_topics.size());    
    
    for (int i = 0; i < _input_topics.size(); i++)
    {   

        auto cb = [this, i](const PointCloudRGB::ConstPtr& msg)
        { 
            point_cloud_callback(msg, i);
        };
        _cloud_subs[i] = _nh.subscribe<PointCloudRGB>(_input_topics[i], 1, cb);

    }

    _cloud_pub = _nh.advertise<PointCloudRGB>("/merged_zed_cloud", 1000);

    _merged_cloud = boost::shared_ptr<PointCloudRGB>(new PointCloudRGB());
}

void PointCloudManager::point_cloud_callback(const PointCloudRGB::ConstPtr& msg, int i)
{
    if (!_cloud_color_vect[i])
    {
        _cloud_color_vect[i].reset(new PointCloudRGB);
    }

    *(_cloud_color_vect[i]) = *msg;
    _is_callback_done[i] = true;
}

void PointCloudManager::run() 
{
    // if all callback called
    if(std::find(_is_callback_done.begin(), _is_callback_done.end(), false) == _is_callback_done.end())
    {
        _merged_cloud->points.clear();
        _merged_cloud->header.frame_id = "zedx_left_left_camera_frame";
        for (auto& cloud : _cloud_color_vect)
        {
            //Reduce number of points
            voxelDownsampling(cloud, cloud, 0.05, 0.05, 0.05);

            if(cloud->header.frame_id.compare("zedx_left_left_camera_frame") != 0){
                //Transform cloud in common frame
                try{
                    _listener.lookupTransform("zedx_left_left_camera_frame", cloud->header.frame_id,
                                              ros::Time(0), _transform);

                   tf::transformTFToEigen(_transform, _camera_transf);
                }
                catch (tf::TransformException &ex) {
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }
                transformCloud(cloud, cloud, _camera_transf);
            }

            //Filter along Z axis to remove ceiling
            filterCloudAxis(cloud, cloud, -1.5, 2.0, "y"); //Left camera has Y pointing to the ground

            //Sum clouds
            *_merged_cloud += *cloud;
        }

        _cloud_pub.publish(_merged_cloud);


        for (int i = 0; i < _is_callback_done.size(); i++)
        {
            _is_callback_done[i] = false;
        }
    }
    
}

void PointCloudManager::resetParams(){
    _normals = boost::shared_ptr<pcl::PointCloud <pcl::Normal>>(new pcl::PointCloud <pcl::Normal>);

    _tree = boost::shared_ptr<pcl::search::KdTree<PointXYZ>>(new pcl::search::KdTree<PointXYZ>());
    _tree_rgb = boost::shared_ptr<pcl::search::KdTree<PointRGB>>(new pcl::search::KdTree<PointRGB>());

    _camera_transf = Affine3d::Identity();
}

void PointCloudManager::voxelDownsampling(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, double dim_x, double dim_y, double dim_z){

    //NOTE: Maybe check on N° points needed
    _vox_grid.setInputCloud (cloud_in);
    _vox_grid.setLeafSize (dim_x, dim_y, dim_z);
    _vox_grid.filter (*cloud_out);
}

void PointCloudManager::voxelDownsampling(PointCloudRGB::Ptr cloud_in, PointCloudRGB::Ptr cloud_out, double dim_x, double dim_y, double dim_z){

    //NOTE: Maybe check on N° points needed
    _vox_grid_color.setInputCloud (cloud_in);
    _vox_grid_color.setLeafSize (dim_x, dim_y, dim_z);
    _vox_grid_color.filter (*cloud_out);
}

void PointCloudManager::transformCloud(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, Affine3d& transf){
    pcl::transformPointCloud(*cloud_in, *cloud_out, transf);
}

void PointCloudManager::transformCloud(PointCloudRGB::Ptr cloud_in, PointCloudRGB::Ptr cloud_out, Affine3d& transf){
    pcl::transformPointCloud(*cloud_in, *cloud_out, transf);
}

void PointCloudManager::filterCloudXYZ(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, double x_min, double x_max, double y_min, double y_max, double z_min, double z_max){
    _pass_filter.setInputCloud (cloud_in);
    _pass_filter.setFilterFieldName ("x");
    _pass_filter.setFilterLimits (x_min, x_max);
    _pass_filter.setNegative(false);
    _pass_filter.filter (*cloud_out);

    _pass_filter.setInputCloud (cloud_out);
    _pass_filter.setFilterFieldName ("y");
    _pass_filter.setFilterLimits (y_min, y_max);
    _pass_filter.setNegative(false);
    _pass_filter.filter (*cloud_out);

    _pass_filter.setInputCloud (cloud_out);
    _pass_filter.setFilterFieldName ("z");
    _pass_filter.setFilterLimits (z_min, z_max);
    _pass_filter.setNegative(false);
    _pass_filter.filter (*cloud_out);
}

//TODO: Add check on axis string
void PointCloudManager::filterCloudAxis(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, double min, double max, std::string axis, bool negative){
    _pass_filter.setInputCloud (cloud_in);
    _pass_filter.setFilterFieldName (axis);
    _pass_filter.setFilterLimits (min, max);
    _pass_filter.setNegative(negative);
    _pass_filter.filter (*cloud_out);
}

void PointCloudManager::filterCloudAxis(PointCloudRGB::Ptr cloud_in, PointCloudRGB::Ptr cloud_out, double min, double max, std::string axis, bool negative){
    _pass_filter_color.setInputCloud (cloud_in);
    _pass_filter_color.setFilterFieldName (axis);
    _pass_filter_color.setFilterLimits (min, max);
    _pass_filter_color.setNegative(negative);
    _pass_filter_color.filter (*cloud_out);
}

void PointCloudManager::outlierRemoval(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out, int mean_k, double std_th){
    _sor.setInputCloud (cloud_in);
    _sor.setMeanK (mean_k);
    _sor.setStddevMulThresh (std_th);
    _sor.filter (*cloud_out);
}

void PointCloudManager::segmentRegionGrowing(PointCloud::Ptr cloud_in, PointCloudRGB::Ptr cloud_out, int min_points, int max_points, double n_neigh, double smootheness_th, double curvature_th){

    _normal_estimator.setSearchMethod (_tree);
    _normal_estimator.setInputCloud (cloud_in);
    _normal_estimator.setKSearch (100);
    _normal_estimator.compute (*_normals);

    _reg.setMinClusterSize (min_points);
    _reg.setMaxClusterSize (max_points);
    _reg.setSearchMethod (_tree);
    _reg.setNumberOfNeighbours (n_neigh);
    _reg.setInputCloud (cloud_in);
    _reg.setInputNormals (_normals);
    _reg.setSmoothnessThreshold (smootheness_th / 180.0 * M_PI);
    _reg.setCurvatureThreshold (curvature_th);

    _cluster_indices.clear();
    _reg.extract (_cluster_indices);

    cloud_out = _reg.getColoredCloud();
}

void PointCloudManager::segmentRegionGrowingRGB(PointCloudRGB::Ptr cloud_in, PointCloudRGB::Ptr cloud_out, int min_points, int max_points, float dist_th, float color_point_th, float region_point_th){

    ROS_WARN("Segment");
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::removeNaNFromPointCloud (*cloud_in, *indices);

    _color_reg.setInputCloud (cloud_in);
    _color_reg.setIndices (indices);
    _color_reg.setSearchMethod (_tree_rgb);
    _color_reg.setDistanceThreshold (dist_th); //distance to check if neighbor or not
    _color_reg.setPointColorThreshold (color_point_th);
    _color_reg.setRegionColorThreshold (region_point_th);
    _color_reg.setMinClusterSize (min_points);
    //_color_reg.setMaxClusterSize (max_points);

    ROS_WARN("Set everything");

    _cluster_indices.clear();
    _color_reg.extract (_cluster_indices);
    ROS_WARN("Extract indices");

    cloud_out = _color_reg.getColoredCloud();
    //ROS_WARN("Points: %ld", cloud_out->points.size());
}

void PointCloudManager::extractBoundingBox(PointCloud::Ptr cloud, pcl::PointXYZ *min_point_OBB, pcl::PointXYZ *max_point_OBB, pcl::PointXYZ *position_OBB, Eigen::Matrix3f *rotational_matrix_OBB){
    _feature_extractor.setInputCloud (cloud);
    _feature_extractor.compute ();

    //Eigen::Vector3d mass_center;
    _feature_extractor.getOBB (*min_point_OBB, *max_point_OBB, *position_OBB, *rotational_matrix_OBB);
    //_feature_extractor.getMassCenter (mass_center);
}

void PointCloudManager::getConvexHull(PointCloud::Ptr cloud, PointCloud::Ptr hull, int dimension){
    _conv_hull.setInputCloud (cloud);
    _conv_hull.setDimension(dimension);
    _conv_hull.reconstruct (*hull);
}

void PointCloudManager::getConcaveHull(PointCloud::Ptr cloud, PointCloud::Ptr hull, int dimension, double alpha){
    _conc_hull.setInputCloud (cloud);
    _conc_hull.setDimension(dimension);
    _conc_hull.setAlpha(alpha);
    _conc_hull.reconstruct (*hull);
}

//TODO: PLANE and AXIS
void PointCloudManager::horizontalPlaneSegmentation(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out){
    _coefficients = boost::shared_ptr<pcl::ModelCoefficients>(new pcl::ModelCoefficients());
    _inliers = boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices());

    _sac_seg.setOptimizeCoefficients (true);
    //_sac_seg.setModelType (pcl::SACMODEL_PLANE);
    _sac_seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    _sac_seg.setAxis(Vector3f(0, 0, 1));
    _sac_seg.setEpsAngle(0.15);

    _sac_seg.setMethodType (pcl::SAC_RANSAC);
    _sac_seg.setMaxIterations(500);
    _sac_seg.setDistanceThreshold (0.025);

    _sac_seg.setInputCloud (cloud_in);
    _sac_seg.segment (*_inliers, *_coefficients);

    // Project the model inliers
    _proj.setModelType (pcl::SACMODEL_PLANE);
    _proj.setInputCloud (cloud_in);
    _proj.setIndices (_inliers);
    _proj.setModelCoefficients (_coefficients);
    _proj.filter (*cloud_out);
}

void PointCloudManager::planeSegmentation(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out){
    _coefficients = boost::shared_ptr<pcl::ModelCoefficients>(new pcl::ModelCoefficients());
    _inliers = boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices());

    _sac_seg.setOptimizeCoefficients (true);
    _sac_seg.setModelType (pcl::SACMODEL_PLANE);

    _sac_seg.setMethodType (pcl::SAC_RANSAC);
    _sac_seg.setMaxIterations(500);
    _sac_seg.setDistanceThreshold (0.025);

    _sac_seg.setInputCloud (cloud_in);
    _sac_seg.segment (*_inliers, *_coefficients);

    // Project the model inliers
    _proj.setModelType (pcl::SACMODEL_PLANE);
    _proj.setInputCloud (cloud_in);
    _proj.setIndices (_inliers);
    _proj.setModelCoefficients (_coefficients);
    _proj.filter (*cloud_out);
}

std::vector<PointCloud::Ptr> PointCloudManager::euclideanClustering(PointCloud::Ptr cloud, double tolerance, int min_size, int max_size){
    _cluster_indices.clear();

    _ec.setClusterTolerance (tolerance); // 2cm
    _ec.setMinClusterSize (min_size);
    _ec.setMaxClusterSize (max_size);
    _ec.setSearchMethod (_tree);
    _ec.setInputCloud (cloud);
    _ec.extract (_cluster_indices);

    _cloud_vect.clear();

    for (const auto& cluster : _cluster_indices){
        PointCloud::Ptr cloud_cluster (new PointCloud());
        for (const auto& idx : cluster.indices) {
            if(idx >= 0 && idx < cloud->points.size())
                cloud_cluster->push_back(cloud->points[idx]);
        }

        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        _cloud_vect.push_back(cloud_cluster);
    }

    return _cloud_vect;
}

PointCloudManager::~PointCloudManager(){
    std::vector<pcl::PointIndices>().swap(_cluster_indices);
    std::vector<PointCloud::Ptr>().swap(_cloud_vect);
}
