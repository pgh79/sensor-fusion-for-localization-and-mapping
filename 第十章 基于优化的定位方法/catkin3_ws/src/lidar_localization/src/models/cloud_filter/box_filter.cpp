/*
 * @Description: 从点云中截取一个立方体部分
 * @Author: Ren Qian
 * @Date: 2019-03-12 23:38:31
 */
#include <vector>
#include <iostream>
#include "glog/logging.h"

#include "lidar_localization/models/cloud_filter/box_filter.hpp"

/*
class Node {
    public:
      struct bounds {
        dist_t lower[2], upper[2]; // bounds on inner/outer distances
        int child[2];
      };
      union {
        bounds data;
        int leaves[maxbucket];
      };
      int index;

      Node()
        : index(-1)
      {
        for (int i = 0; i < 2; ++i) {
          data.lower[i] = data.upper[i] = 0;
          data.child[i] = -1;
        }
      }
*/

namespace lidar_localization {
BoxFilter::BoxFilter(YAML::Node node) {
/*
size_ is an integer that stores the number of boxes in the input data, edge_ is an integer that stores the 
number of edges in the input data, and origin_ is an integer that stores the position of the box's center.
*/
// resize Edge and edge, origin variables so they have enough space to store all six rows of boxes from the input data.
    size_.resize(6);
    edge_.resize(6);
    origin_.resize(3);

/*
box_filter_size: [-150.0, 150.0, -150.0, 150.0, -150.0, 150.0]  # 参数顺序是min_x, max_x, min_y, max_y, min_z, max_z
*/
// get and store multi_dim data from yaml file 

    for (size_t i = 0; i < size_.size(); i++) {
        size_.at(i) = node["box_filter_size"][i].as<float>();
    }

// pass size_ to SetSize

    SetSize(size_);
}

/*
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lidar_localization {
class CloudData {
  public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

  public:
    CloudData()
      :cloud_ptr(new CLOUD()) {
    }

  public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
};
}
*/

bool BoxFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr,
                       CloudData::CLOUD_PTR& output_cloud_ptr) {
    output_cloud_ptr->clear();

/*
    setMin, set Max: cube的两个对角点，立方体对角线的两个点
    max，min并不代表数值上的min和max
*/
    pcl_box_filter_.setMin(Eigen::Vector4f(edge_.at(0), edge_.at(2), edge_.at(4), 1.0e-6));
    pcl_box_filter_.setMax(Eigen::Vector4f(edge_.at(1), edge_.at(3), edge_.at(5), 1.0e6));
    pcl_box_filter_.setInputCloud(input_cloud_ptr);
    pcl_box_filter_.filter(*output_cloud_ptr);

    return true;
}

// 
void BoxFilter::SetSize(std::vector<float> size) {
    size_ = size;
    std::cout << "Box Filter params:" << std::endl
              << "min_x: " << size.at(0) << ", "
              << "max_x: " << size.at(1) << ", "
              << "min_y: " << size.at(2) << ", "
              << "max_y: " << size.at(3) << ", "
              << "min_z: " << size.at(4) << ", "
              << "max_z: " << size.at(5)
              << std::endl << std::endl;
    
    CalculateEdge();
}

void BoxFilter::SetOrigin(std::vector<float> origin) {
    origin_ = origin;
    CalculateEdge();
}

// edge_.at(i) is relative to origin_.at(i)
// edge_ is a vector

void BoxFilter::CalculateEdge() {
    for (size_t i = 0; i < origin_.size(); ++i) {
        edge_.at(2 * i) = size_.at(2 * i) + origin_.at(i);
        edge_.at(2 * i + 1) = size_.at(2 * i + 1) + origin_.at(i);
    }
}

std::vector<float> BoxFilter::GetEdge() {
    return edge_;
}
} // namespace lidar_slam