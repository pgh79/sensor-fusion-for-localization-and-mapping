/*
 * @Description: voxel filter 模块实现
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:53:20
 */
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"

#include "glog/logging.h"

namespace lidar_localization {

// when using VoxelFilter(), the node pointer should be identified to be globla_map or local_map or frame.

VoxelFilter::VoxelFilter(const YAML::Node& node) {
    float leaf_size_x = node["leaf_size"][0].as<float>();
    float leaf_size_y = node["leaf_size"][1].as<float>();
    float leaf_size_z = node["leaf_size"][2].as<float>();
    
// store and pass leaf_size to SetFilterParam()

    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

// or VoxelFilter() can pass direct leaf_size.x(yz)

VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

// Set and print Voxel Filter params:
bool VoxelFilter::SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

    std::cout << "Voxel Filter params:" << std::endl
              << leaf_size_x << ", "
              << leaf_size_y << ", "
              << leaf_size_z 
              << std::endl << std::endl;

    return true;
}

/*
 input data and filter, output filtered_cloud
*/

bool VoxelFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
    voxel_filter_.setInputCloud(input_cloud_ptr);
    voxel_filter_.filter(*filtered_cloud_ptr);

    return true;
}
} 