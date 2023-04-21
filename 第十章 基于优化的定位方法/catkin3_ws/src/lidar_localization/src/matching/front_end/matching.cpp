/*
 * @Description: LIO localization frontend, implementation
 * @Author: Ge Yao
 * @Date: 2021-01-02 10:47:23
 */
#include "lidar_localization/matching/front_end/matching.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/cloud_filter/no_filter.hpp"

#include "lidar_localization/models/registration/ndt_registration.hpp"

namespace lidar_localization {

// initialize a new instance of the CloudData class with default values.
// And The global_map_ptr_(new CloudData::CLOUD()) is initialized with the new CloudData::CLOUD().
// The local_map_ptr_(new CloudData::CLOUD()) is initialized with the same as well.
// Then, current_scan_ptr_(new CloudData::CLOUD()) is also initialized with the same as well.

Matching::Matching()
    : global_map_ptr_(new CloudData::CLOUD()),
      local_map_ptr_(new CloudData::CLOUD()),
      current_scan_ptr_(new CloudData::CLOUD()) 
{
    //sets up some initial values for this class and then calls ResetLocalMap(0, 0, 0).
    InitWithConfig();

    ResetLocalMap(0.0, 0.0, 0.0);
}

//
bool Matching::InitWithConfig() {
/*
 loading the configuration file inclung parameter lists for LIO localization, frontend.
 The config_file_path is defined as WORK_SPACE_PATH + "/config/matching/matching.yaml".
*/    
    //
    // load lio localization frontend config file:
    // 
    std::string config_file_path = WORK_SPACE_PATH + "/config/matching/matching.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    // prompt:
    LOG(INFO) << std::endl << "----------------- Init LIO Localization, Frontend -------------------" << std::endl;

    // a. init point cloud map & measurement processors:
    InitPointCloudProcessors(config_node);
    // b. load global map:
    InitGlobalMap(config_node);
    // c. init lidar frontend for relative pose estimation:
    // registration for odometry estimation
    InitRegistration(config_node, registration_ptr_);
    // d. init map matcher:
    InitScanContextManager(config_node);

    return true;
}

/*
    Interface based : CloudFilterInterface
    Downsampling a PointCloud using a VoxelGrid filter with leaf size of xx mm.
    The params are passed from matching.yaml file

    remove the outliers
*/
bool Matching::InitFilter(
    const YAML::Node &config_node, std::string filter_user, 
    std::shared_ptr<CloudFilterInterface>& filter_ptr
) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();

    // prompt:
    LOG(INFO) << "\t\tFilter Method for " << filter_user << ": " << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else if (filter_mothod == "no_filter") {
        filter_ptr = std::make_shared<NoFilter>();
    } else {
        LOG(ERROR) << "Filter method " << filter_mothod << " for " << filter_user << " NOT FOUND!";
        return false;
    }

    return true;
}

bool Matching::InitLocalMapSegmenter(const YAML::Node& config_node) {
//creates a BoxFilter object, which is an implementation of std::shared_ptr that stores the passed in YAML::Node.
// Guess: create a box to segment a local map for matching to reduce resource consuming
    local_map_segmenter_ptr_ = std::make_shared<BoxFilter>(config_node);
    return true;
}

// Choose between "global_map filter", "frame_filter" or "local_map_filter (needed to call InitLocalMapSegmenter(config_node) first)" 
bool Matching::InitPointCloudProcessors(const YAML::Node& config_node) {
    // prompt:
    LOG(INFO) << "\tInit Point Cloud Processors:" << std::endl;

    // a. global map filter:
    InitFilter(config_node, "global_map", global_map_filter_ptr_);

    // b.1. local map segmenter:
    InitLocalMapSegmenter(config_node);
    // b.2. local map filter:
    InitFilter(config_node, "local_map", local_map_filter_ptr_);

    // c. scan filter -- 
    InitFilter(config_node, "frame", frame_filter_ptr_);

    return true;
}

/*
prompt the user to load a global map and then filter it with the local map.
*/
bool Matching::InitGlobalMap(const YAML::Node& config_node) {
// starts by initializing a std::string variable called map_path.
// This is the path to the global map file that will be loaded into memory.

    std::string map_path = config_node["map_path"].as<std::string>();

    // load map: 
    // pcl io loadPCDFile 
    // it loads the global map pointer from disk using pcl::io::loadPCDFile().

    pcl::io::loadPCDFile(map_path, *global_map_ptr_);

    // apply local map filter to global map for later scan-map matching:
    // applies a local filter to this new global map for later matching with scan-map queries.

    local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);

    // prompt:
    LOG(INFO) << "\tLoad Global Map, size:" << global_map_ptr_->points.size();

// prints out information about loading and whether or not there was a new global map found in memory.

    has_new_global_map_ = true;

    return true;
}

// 
bool Matching::InitRegistration(
    const YAML::Node& config_node, 
    std::shared_ptr<RegistrationInterface>& registration_ptr
) {
// load the matching.yaml registration_method into memory as string and saved into a variable registration_method

    std::string registration_method = config_node["registration_method"].as<std::string>();

    // prompt:
    LOG(INFO) << "\tLidar Frontend Estimation Method: " << registration_method << std::endl;

// make an instance of NDTRegistration class initialized with registration_method, and use a shared_ptr of RegistrationInterface point to it
// if not ndt method, prompt and return false

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else {
        LOG(ERROR) << "Estimation method " << registration_method << " NOT FOUND!";
        return false;
    }

    return true;
}

// 
bool Matching::InitScanContextManager(const YAML::Node& config_node) {
    // get loop closure config as string and stored in memory:
    std::string map_matching_method = config_node["map_matching_method"].as<std::string>();

    // prompt:
    LOG(INFO) << "\tMap Matching Method: " << map_matching_method << std::endl;

    // create instance and load the shared_ptr into memory:
// map_matching_method: scan_context      # available map matching methods: scan_context

    scan_context_manager_ptr_ = std::make_shared<ScanContextManager>(config_node[map_matching_method]);

    // load pre-built index from the instance's Load() function with scan_context_index_path passed as a parameter:
/*
# a. global map for relative pose estimation using lidar frontend:
map_path: /home/lory/catkin3_ws/src/lidar_localization/slam_data/map/filtered_map.pcd
# b. scan context index for map matching:
scan_context_path: /home/lory/catkin3_ws/src/lidar_localization/slam_data/scan_context   
*/

    std::string scan_context_index_path = config_node["scan_context_path"].as<std::string>();
    scan_context_manager_ptr_->Load(scan_context_index_path);

    return true;
}

// 
bool Matching::ResetLocalMap(float x, float y, float z) {
// starts by declaring a variable called "origin" which is of type std::vector<float>.
// then load the {x,y,z} into memory

    std::vector<float> origin = {x, y, z};

    // use ROI filtering for local map segmentation:
// pass the origin to local_map_segmenter_ptr_->SetOrigin as the origin

    local_map_segmenter_ptr_->SetOrigin(origin);

// Segment the local map from global map with calling the local_map_segmenter_ptr_->Filter() function

    local_map_segmenter_ptr_->Filter(global_map_ptr_, local_map_ptr_);

// Set the local_map_ptr_ as the standard of registration

    registration_ptr_->SetInputTarget(local_map_ptr_);

    has_new_local_map_ = true;

// Get and print the edges of local_map
//  extracts all edges from this new local map by calling GetEdge() on it's associated pointer, 
// LOGs out information about each edge and finally returns true as a result.

    std::vector<float> edge = local_map_segmenter_ptr_->GetEdge();
    LOG(INFO) << "New local map:" << edge.at(0) << ","
                                  << edge.at(1) << ","
                                  << edge.at(2) << ","
                                  << edge.at(3) << ","
                                  << edge.at(4) << ","
                                  << edge.at(5) << std::endl << std::endl;

    return true;
}
/*
 sets the pose of the object to be initialized.
 The pose is set by calling SetInitPose with a matrix4f value, which is then passed into InitPose() 
 and used as the initial position for all subsequent calculations of LocalMap.
*/

bool Matching::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    
    ResetLocalMap(init_pose(0,3), init_pose(1,3), init_pose(2,3));

    return true;
}

/*
   if gnss_cnt = 0, store the gnss_pose as init pose;
   ( If there is no previous pose, then it sets the initial pose.)
    gnss_cnt = 1,2,3, continue to increase counting;
    if gnss_cnt > 3, let the flag has_inited became true.
    (if there are more than 4 poses in the list, then has_inited is set to true )
*/

bool Matching::SetGNSSPose(const Eigen::Matrix4f& gnss_pose) {

    static int gnss_cnt = 0;

    current_gnss_pose_ = gnss_pose;

    if (gnss_cnt == 0) {
        SetInitPose(gnss_pose);
    } else if (gnss_cnt > 3) {
        has_inited_ = true;
    }
    gnss_cnt++;

    return true;
}

/**
 * @brief  get init pose using scan context matching
 * @param  init_scan, init key scan
 * @return true if success otherwise false
 */

// verify if the scan_context_manager_ptr_->DetectLoopClosure() detected whether the loopclosure's init_scan relative to the init_pose is detected

bool Matching::SetScanContextPose(const CloudData& init_scan) {
    // get init pose proposal using scan context match:

    Eigen::Matrix4f init_pose =  Eigen::Matrix4f::Identity();
    
    if (
        !scan_context_manager_ptr_->DetectLoopClosure(init_scan, init_pose)
    ) {
        return false;
    }

    // set init pose:
    // set to Identity ?? 
    SetInitPose(init_pose);
    has_inited_ = true;
    
    return true;
}

// update the pose of the laser.
bool Matching::Update(
    const CloudData& cloud_data, 
    Eigen::Matrix4f& laser_pose, Eigen::Matrix4f &map_matching_pose
) {
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;

// predict a new pose for the laser based on its current position and orientation.  

    static Eigen::Matrix4f predict_pose = init_pose_;

// verify the point cloud data are valid and reduce(filter) less important data store the data in heap without causing too many data transfer into memory 

    // remove invalid measurements:
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, indices);

    // downsample current scan:
    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(cloud_data.cloud_ptr, filtered_cloud_ptr);

// checks if the loop closure is initialized
// if not, save the current_gnss_pose_ in variable predict_pose to replace init_pose_

    if (!has_inited_) {
        predict_pose = current_gnss_pose_;
    }

    // matching: --- 时空对齐后处理模型信息
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, laser_pose);
    pcl::transformPointCloud(*cloud_data.cloud_ptr, *current_scan_ptr_, laser_pose);

    // update predicted pose:
    // laser_pose is active in 3 equations.
    // first transformed into last_pose.inverse() space, then stored in last_pose, finally step_nose project into laser_pose pspace
    
    step_pose = last_pose.inverse() * laser_pose;
    last_pose = laser_pose;
    predict_pose = laser_pose * step_pose;
    
    // shall the local map be updated:

    std::vector<float> edge = local_map_segmenter_ptr_->GetEdge();

// fabs() function in C++ returns the absolute value of the argument. It is defined in the cmath header file.
// Mathematically, fabs(num) = |num|.

    for (int i = 0; i < 3; i++) {
        if (
            fabs(laser_pose(i, 3) - edge.at(2 * i)) > 50.0 &&
            fabs(laser_pose(i, 3) - edge.at(2 * i + 1)) > 50.0
        ) {
            continue;
        }
            
        ResetLocalMap(laser_pose(0,3), laser_pose(1,3), laser_pose(2,3));
        break;
    }

    // init the map matching pose as laser pose:
    map_matching_pose = laser_pose;
    scan_context_manager_ptr_->DetectLoopClosure(cloud_data, map_matching_pose);

    return true;
}

bool Matching::HasInited(void) {
    return has_inited_;
}

bool Matching::HasNewGlobalMap(void) {
    return has_new_global_map_;
}

bool Matching::HasNewLocalMap(void) {
    return has_new_local_map_;
}

Eigen::Matrix4f Matching::GetInitPose(void) {
    return init_pose_;
}

CloudData::CLOUD_PTR& Matching::GetGlobalMap(void) {
    has_new_global_map_ = false;
    return global_map_ptr_;
}

CloudData::CLOUD_PTR& Matching::GetLocalMap(void) {
    return local_map_ptr_;
}

CloudData::CLOUD_PTR& Matching::GetCurrentScan(void) {
    return current_scan_ptr_;
}

} // namespace lidar_localization