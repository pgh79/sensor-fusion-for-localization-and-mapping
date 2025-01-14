/*
 * @Description: lio localization backend workflow, implementation
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#include "lidar_localization/matching/back_end/sliding_window.hpp"

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

/*This defines all of the variables that will be used throughout this project 
such as what type of data we're working with (LIDAR), how many samples per second our sensor can take, etc...
*/
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/tools/file_manager.hpp"

//starts with a namespace declaration
namespace lidar_localization {

// declares a class called SlidingWindow and initializes an object of that class with default values.
SlidingWindow::SlidingWindow() {
    InitWithConfig();
}

//  implementation of a function that loads the local configuration file.
bool SlidingWindow::InitWithConfig() {
    //
    // load lio localization backend config file: 
    // starts by getting the data path from the config node. If it is a relative path, then it will be set to ./ and if not, it will be set to WORK_SPACE_PATH.
    // create a string called config_file_path which will be used later in this function to load the configuration file for LIO localization from WORK_SPACE_.
    // opens a config file that is located at WORK_SPACE_PATH + "/config/matching/sliding_window.yaml" and then loads it as an instance of a YAML node object named config_file_path
    std::string config_file_path = WORK_SPACE_PATH + "/config/matching/sliding_window.yaml";
   // This path is then passed into YAML::LoadFile() which loads it as an instance of a YAML node object named config_.
    YAML::Node config_node = YAML::LoadFile(config_file_path);
// prints out some text indicating successful completion of initialization.
    std::cout << "-----------------Init LIO Localization, Backend-------------------" << std::endl;

/*
The InitDataPath function needs an input parameter of type YAML::Node& which means that you can pass in any valid YAML::Node object as long as its name matches "data_path".
 This is important because there are multiple nodes with different names that have the same value (e.g., "data" and "config").
*/
    // a. estimation output path:
    InitDataPath(config_node);
    // b. key frame selection config:
    InitKeyFrameSelection(config_node);
    // c. sliding window config:
    InitSlidingWindow(config_node);
    // d. IMU pre-integration:
    InitIMUPreIntegrator(config_node);

    return true;
}

bool SlidingWindow::InitDataPath(const YAML::Node& config_node) {
// data_path是string格式的路径,
//  gets the value of data path from the config node and assigns it into a variable called data_path.
    std::string data_path = config_node["data_path"].as<std::string>();
// 如果读取到的data_path是相对路径./，则修改成绝对路径 WORK_SPACE_PATH
    if (data_path == "./") {
        data_path = WORK_SPACE_PATH;
    }

// 创建data_path下的slam_data文件夹，失败则返回false
//  calls FileManager::CreateDirectory() which will create an empty folder slam_data if one doesn't exist already.
//  create a directory in the data_path, which is where all of the data files will be stored
    if (!FileManager::CreateDirectory(data_path + "/slam_data"))
        return false;

//  creates a string called "trajectory_path_"
// creates a new file called trajectory and saves it in the data_path.
//  This file contains the estimated trajectory for each key frame that was created during the course of this application's execution
    trajectory_path_ = data_path + "/slam_data/trajectory";
//  checks to see if the user has already created this directory or not, and if they haven't then it will create it for them.
    if (!FileManager::InitDirectory(trajectory_path_, "Estimated Trajectory"))
        return false;

    return true;
}

// analyzes the configuration node to determine how many key frames are needed for this slide window.
bool SlidingWindow::InitKeyFrameSelection(const YAML::Node& config_node) {
    key_frame_config_.max_distance = config_node["key_frame"]["max_distance"].as<float>();
//  sets up an interval that will be used to calculate when each frame should start and end.
    key_frame_config_.max_interval = config_node["key_frame"]["max_interval"].as<float>();
// returns true if initialization was successful or false otherwise.    
    return true;
}

//  creating an instance of the SlidingWindow class, setting its configuration parameters, and returning true.
bool SlidingWindow::InitSlidingWindow(const YAML::Node& config_node) {
    // init sliding window:
    const int sliding_window_size = config_node["sliding_window_size"].as<int>();
    // creates a shared pointer to the CeresSlidingWindow class and sets some configuration options for it.
    //  to store information about how many measurements are being taken by this window.
    sliding_window_ptr_ = std::make_shared<CeresSlidingWindow>(sliding_window_size);

// specifies that the window will be using data from the config_node["measurements"]["map_matching"] variable as its source of matching data.
 //  This means that when an input value matches with any of these values, it will be considered as a match and sent into the sliding window algorithm for processing.
    // select measurements:
   
    measurement_config_.source.map_matching = config_node["measurements"]["map_matching"].as<bool>();
    measurement_config_.source.imu_pre_integration = config_node["measurements"]["imu_pre_integration"].as<bool>();

    // get measurement noises, pose:
    /*
     gets measurement noises from two different sources: lidar_odometry and map_matching.
    The lidar_odometry noise is resized to have six values, while map_matching noise is resized to have six values as well.
    */
    measurement_config_.noise.lidar_odometry.resize(6); // 六个自由度，double类型噪声
    measurement_config_.noise.map_matching.resize(6);
// gets and hold position noises from three different sources: gnss_position, lidar_odometry, and map_matching.
    for (int i = 0; i < 6; ++i) {
        measurement_config_.noise.lidar_odometry(i) =
            config_node["lidar_odometry"]["noise"][i].as<double>();
        measurement_config_.noise.map_matching(i) =
            config_node["map_matching"]["noise"][i].as<double>();
    }

// added noise
    // get measurement noises, position:（经纬高）
    measurement_config_.noise.gnss_position.resize(3);
    for (int i = 0; i < 3; i++) {
        measurement_config_.noise.gnss_position(i) =
            config_node["gnss_position"]["noise"][i].as<double>();
    }

    return true;
}
/*
imu_pre_integration:
    earth:
        # gravity can be calculated from https://www.sensorsone.com/local-gravity-calculator/ using latitude and height:
        gravity_magnitude: 9.80943
    covariance:
        measurement:
            accel: 2.5e-3
            gyro: 1.0e-4
        random_walk:
            accel: 1.0e-4
            gyro: 1.0e-4
*/

/*
input: const YAML::Node& config_node
output: bool
function: if measurement: imu_pre_integration is true, set the imu_pre_integrator_ptr_ point to 
                     store an instance of an IMUPreIntegrator object, and The parameter for this object is
                      found from the configuration node "imu_pre_integration" that was passed into InitIMUPreIntegrator() .
param:
     const YAML::Node& config_node
     imu_pre_integrator_ptr_
     measurement_config_.source.imu_pre_integration
     config_node["imu_pre_integration"]
     IMUPreIntegrator
*/
bool SlidingWindow::InitIMUPreIntegrator(const YAML::Node& config_node) {
  // declare a nullptr 
    imu_pre_integrator_ptr_ = nullptr;

// if measurement: imu_pre_integration is true
    if (measurement_config_.source.imu_pre_integration) {
    // use  imu_pre_integrator_ptr_  to store an instance of an IMUPreIntegrator object.
    // The parameter for this object is found from the configuration node that was passed into InitIMUPreIntegrator() .
        imu_pre_integrator_ptr_ = std::make_shared<IMUPreIntegrator>(config_node["imu_pre_integration"]);
    }

    return true;
}

/*
input: const IMUData &imu_data
output: bool
function: first verifies that the measurement configuration has a source field set to "imu_pre_integration" and that the pre-integration function is not null.
                    If either of these conditions are not met, then the code returns false.
                    If both conditions are met, then the code calls the pre-integration function with the IMU data as an argument.
param: 
    const IMUData & imu_data
     imu_pre_integrator_ptr_
     measurement_config_.source.imu_pre_integration
  */ 
bool SlidingWindow::UpdateIMUPreIntegration(const IMUData &imu_data) {
   // first verifies that the measurement configuration has a source field set to "imu_pre_integration" and that the pre-integration function is not null.
    if ( !measurement_config_.source.imu_pre_integration || nullptr == imu_pre_integrator_ptr_ )
        return false;
   
    // verify the IMUPreIntegrator object's IsInited() is false and the return valuie of Update(imu_data) is true
    if (
        !imu_pre_integrator_ptr_->IsInited() ||
        imu_pre_integrator_ptr_->Update(imu_data)
    ) {
        return true;
    }

    return false;
}

/*
input: 
    const PoseData &laser_odom,
    const PoseData &map_matching_odom,
    const IMUData &imu_data, 
    const PoseData& gnss_pose:  a pointer to another array of PoseData objects called gnss_pose that contains GPS coordinates for each frame in time-space.
output: bool
function: If there is a new keyframe, then the code needs to analyze it.
 The analysis of a keyframe involves calculating various values related to the pose data.
 If there is no new keyframe, then the code simply updates the laser_odom, map_matching_odom, and imu_data variables with the latest values from their respective sources.

param: 
            laser_odom, 
            map_matching_odom,
            imu_data,
            gnss_pose
  */ 
 // The first variable is a pointer to an array of PoseData objects, which are created when the program starts and updated as needed.
 // Next, there is a pointer to an IMUData object, which contains data about the user's orientation relative to their body.

bool SlidingWindow::Update(
    const PoseData &laser_odom,
    const PoseData &map_matching_odom,
    const IMUData &imu_data, 
    const PoseData& gnss_pose
) {
    // clear param to 0??
    // start over and create new frames from scratch.
    ResetParam();

// New keyframe means that something has changed so we need to create new frames from scratch again (i.e., start over).
    if ( 
        MaybeNewKeyFrame(
            laser_odom, 
            map_matching_odom,
            imu_data,
            gnss_pose
        ) 
    ) {
// Update() will update all four arrays with new values if they have changed since last frame; otherwise it does nothing.
        Update();
        MaybeOptimized();
    }

    return true;
}

// the SlidingWindow class manages a sliding window of key frames.
// checks to see if there are any new key frames. If so, it gets the latest key frame.
//
bool SlidingWindow::HasNewKeyFrame() {
    return has_new_key_frame_;
}
//
//checks to see if the current key frame has been optimized.
// 
bool SlidingWindow::HasNewOptimized() {
    return has_new_optimized_;
}

//updates the sliding window pointer and gets the latest optimized key frame.
//
void SlidingWindow::GetLatestKeyFrame(KeyFrame& key_frame) {
    key_frame = current_key_frame_;
}

// GetLatestKeyFrame(), GetLatestKeyGNSS(), and GetLatestOptimizedOdometry() functions all get the latestkeyframe orgnization 
//from their respective data structures based on their arguments (current_key_frame for KeyFrame, current_key_gnss for KeyGnss, and current_optimal_odometry for OptimizedOdometry).
//
void SlidingWindow::GetLatestKeyGNSS(KeyFrame& key_frame) {
    key_frame = current_key_gnss_;
}

void SlidingWindow::GetLatestOptimizedOdometry(KeyFrame& key_frame) {
    sliding_window_ptr_->GetLatestOptimizedKeyFrame(key_frame);
}

/*
load all of the optimized key frames for the sliding window.
 Once it has loaded all of the optimized key frames, it will insert them at the end of the key_frames_deque list.
*/
void SlidingWindow::GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque) {
// clears the key frames deque.
    key_frames_deque.clear();
    
    // load optimized key frames:
// calls the sliding window's GetOptimizedKeyFrames method, passing in a std::deque of KeyFrame objects.
// The GetOptimizedKeyFrames method returns a std::deque of KeyFrame objects that have been optimized for performance.
    sliding_window_ptr_->GetOptimizedKeyFrames(key_frames_.optimized);


// inserts the std::deque of optimized KeyFrame objects at the end of the key frames deque.
    key_frames_deque.insert(
        key_frames_deque.end(), 
        key_frames_.optimized.begin(), key_frames_.optimized.end()
    );
}

// saves the current pose of the window as a matrix4f.
bool SlidingWindow::SavePose(std::ofstream& ofs, const Eigen::Matrix4f& pose) {
// output 3x4 matrix of Matrix4f pose
// at the last element, endl
// others are separated by " ", space
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ofs << pose(i, j);
            
            if (i == 2 && j == 3) {
                ofs << std::endl;
            } else {
                ofs << " ";
            }
        }
    }

    return true;
}

bool SlidingWindow::SaveOptimizedTrajectory() {
    static Eigen::Matrix4f current_pose = Eigen::Matrix4f::Identity();

    if ( sliding_window_ptr_->GetNumParamBlocks() == 0 )
        return false;

    // create output files:
// creating three files.
 //The first file is laser_odom_ofs, which will store the output of the optimization process.
 //The second file is optimized_ofs, which will store the optimized trajectory.
 //The third file is ground_truth_ofs, which will store the ground truth trajectory.
 //
    std::ofstream laser_odom_ofs, optimized_ofs, ground_truth_ofs;
// verify if it's created successfully
    if (
        !FileManager::CreateFile(laser_odom_ofs, trajectory_path_ + "/laser_odom.txt") ||
        !FileManager::CreateFile(optimized_ofs, trajectory_path_ + "/optimized.txt") ||
        !FileManager::CreateFile(ground_truth_ofs, trajectory_path_ + "/ground_truth.txt")  
    ) {
        return false;
    }

    // load optimized key frames: input database
    sliding_window_ptr_->GetOptimizedKeyFrames(key_frames_.optimized);

    // write 
    // 
    //     a. lidar odometry estimation
    //     b. sliding window optimized odometry estimation
    //     c. IMU/GNSS position reference
    // 
    // as trajectories for evo evaluation:
    for (size_t i = 0; i < key_frames_.optimized.size(); ++i) {
        // a. lidar odometry:
        current_pose = key_frames_.lidar.at(i).pose;
        current_pose(2, 3) = 0.0f;
        SavePose(laser_odom_ofs, current_pose);
        // b. sliding window optimized odometry:
        current_pose = key_frames_.optimized.at(i).pose;
        current_pose(2, 3) = 0.0f;
        SavePose(optimized_ofs, current_pose);
        // c. IMU/GNSS position reference as ground truth:
        current_pose = key_frames_.reference.at(i).pose;
        current_pose(2, 3) = 0.0f;
        SavePose(ground_truth_ofs, current_pose);
    }

    return true;
}

// reset the two boolean variables - "has_new_key_frame_" and "has_new_optimized_" to false.
void SlidingWindow::ResetParam() {
    has_new_key_frame_ = false;
    has_new_optimized_ = false;
}

bool SlidingWindow::MaybeNewKeyFrame( 
    const PoseData &laser_odom, 
    const PoseData &map_matching_odom,
    const IMUData &imu_data,
    const PoseData &gnss_odom
) {

// creates a new KeyFrame object called last_key_frame.
// This object will store the latest key frame in the key frame array.
//
    static KeyFrame last_key_frame;

    // 
    // key frame selection for sliding window:
    // 
// key_frames_.lidar, which stores an array of PoseData objects.
// This array will hold the key frames for the sliding window analysis.
// checks to see if there are any key frames in key_frames_.lidar.
// If there are no key frames, then the code sets up an IMU pre-integrator to help with navigation and positioning during analysis.
    if ( key_frames_.lidar.empty() ) {
        // init IMU pre-integrator:
        if ( imu_pre_integrator_ptr_ ) {
            imu_pre_integrator_ptr_->Init(imu_data);
        }
// 
        has_new_key_frame_ = true;
    } else if (
        // spatial:
//  If there are any key frames, then the code starts by analyzing each pose in turn using laser_odom and map_matching_odom as input data points.
// This is important because it will allow the code to use the IMU data in conjunction with GNSS data to create a more accurate 3D model of the environment.
        ( 
            laser_odom.pose.block<3,1>(0, 3) - 
            last_key_frame.pose.block<3,1>(0, 3) 
        ).lpNorm<1>() > key_frame_config_.max_distance || 
        // temporal:
        (
            laser_odom.time - last_key_frame.time
        ) > key_frame_config_.max_interval
    ) {
        // finish current IMU pre-integration:
        if ( imu_pre_integrator_ptr_ ) {
            imu_pre_integrator_ptr_->Reset(imu_data, imu_pre_integration_); 
        }

        has_new_key_frame_ = true;
    } else {
        has_new_key_frame_ = false;
    }

    // if so:
    if ( has_new_key_frame_ ) {
      // create key frame for lidar odometry, relative pose measurement:
// time at which the current key frame was created.  
        current_key_frame_.time = laser_odom.time;
// index of current_key_frame_ is taken from key frame's lidar point cloud's size
        current_key_frame_.index = key_frames_.lidar.size();
// Pose of current_key_frame_ is taken from laser_odom.pose
        current_key_frame_.pose = laser_odom.pose;
// vel of current_key_frame_ is taken and stored from gnss_odom.vel.v and w
        current_key_frame_.vel.v = gnss_odom.vel.v;
        current_key_frame_.vel.w = gnss_odom.vel.w;

        current_map_matching_pose_ = map_matching_odom;

        // create key frame for GNSS measurement, full LIO state:
        current_key_gnss_.time = current_key_frame_.time;
// index 
        current_key_gnss_.index = current_key_frame_.index;
        current_key_gnss_.pose = gnss_odom.pose;
        current_key_gnss_.vel.v = gnss_odom.vel.v;
        current_key_gnss_.vel.w = gnss_odom.vel.w;

        // add to cache for later evo evaluation:
        key_frames_.lidar.push_back(current_key_frame_);
        key_frames_.reference.push_back(current_key_gnss_);

        // save for next key frame selection:
        last_key_frame = current_key_frame_;
    }

    return has_new_key_frame_;
}

bool SlidingWindow::Update(void) {
// tarts by initializing a static variable called last_key_frame.
// This variable stores the most recent key frame in the sliding window.
// The code then updates the current key frame based on the value of last_key_frame.
    static KeyFrame last_key_frame_ = current_key_frame_;

    //
    // add node for new key frame pose:
    //
    // fix the pose of the first key frame for lidar only mapping:
    if ( sliding_window_ptr_->GetNumParamBlocks() == 0 ) {
        // TODO: add init key frame
        sliding_window_ptr_->AddPRVAGParam(current_key_frame_, true);
    } else {
        // TODO: add current key frame
        sliding_window_ptr_->AddPRVAGParam(current_key_frame_, false);
    }

    // get num. of vertices: 顶点的数量
    const int N = sliding_window_ptr_->GetNumParamBlocks();
    // get param block ID, current:
    const int param_index_j = N - 1;

    //
    // add unary constraints:
    //
    //
    // a. map matching / GNSS position:
    //
    if ( N > 0 && measurement_config_.source.map_matching ) {
        // get prior position measurement:
        Eigen::Matrix4d prior_pose = current_map_matching_pose_.pose.cast<double>();

        // TODO: add constraint, GNSS position:
        sliding_window_ptr_->AddPRVAGMapMatchingPoseFactor(
            param_index_j, 
            prior_pose, measurement_config_.noise.map_matching
        );
    }

    //
    // add binary constraints:
    //
    if ( N > 1 ) {
        // get param block ID, previous:
        const int param_index_i = N - 2;
        
        //
        // a. lidar frontend:
        //
        // get relative pose measurement:
        Eigen::Matrix4d relative_pose = (last_key_frame_.pose.inverse() * current_key_frame_.pose).cast<double>();
        // TODO: add constraint, lidar frontend / loop closure detection:
        sliding_window_ptr_->AddPRVAGRelativePoseFactor(
            param_index_i, param_index_j, 
            relative_pose, measurement_config_.noise.lidar_odometry
        );
        
        //
        // b. IMU pre-integration:
        //
        if ( measurement_config_.source.imu_pre_integration ) {
            // TODO: add constraint, IMU pre-integraion:
            sliding_window_ptr_->AddPRVAGIMUPreIntegrationFactor(
                param_index_i, param_index_j,
                imu_pre_integration_
            );
        }
    }

    // move forward:
    last_key_frame_ = current_key_frame_;

    return true;
}

/*
If Optimize() succeeds in improving performance, then "has_new_optimized_" is set to false and the code returns from this method.
 The code checks if the sliding window pointer has been optimized, and if so, returns true.
*/
bool SlidingWindow::MaybeOptimized() {    
    if ( sliding_window_ptr_->Optimize() ) {
        has_new_optimized_ = true;

        return true;
    }
    
    return false;
}

} // namespace lidar_localization

/*
#
# data output path:
#
data_path: ./

#
# key frame detection
#
key_frame:
    # max. distance between two key frames:
    max_distance: 0.25
    # max. time interval between two key frames:
    max_interval: 0.10

#
# sliding window size:
#
sliding_window_size: 15

#
# select measurements:
# 
measurements:
    map_matching: true
    imu_pre_integration: true

#
# measurement configs:
#
lidar_odometry:
    noise: [1.0, 1.0, 1.0, 0.01, 0.01, 0.01] # x y z yaw roll pitch

map_matching:
    noise: [1.0, 1.0, 4.0, 0.01, 0.01, 0.01] # x y z yaw roll pitch

gnss_position:
    noise: [1.0, 1.0, 4.0] # x y z

imu_pre_integration:
    earth:
        # gravity can be calculated from https://www.sensorsone.com/local-gravity-calculator/ using latitude and height:
        gravity_magnitude: 9.80943
    covariance:
        measurement:
            accel: 2.5e-3
            gyro: 1.0e-4
        random_walk:
            accel: 1.0e-4
            gyro: 1.0e-4

*/