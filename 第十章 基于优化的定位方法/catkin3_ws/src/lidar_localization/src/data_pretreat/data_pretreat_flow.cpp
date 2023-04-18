/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
// starts by creating a new DataPretreatFlow object.
// This is done with the ros::NodeHandle& nh, std::string cloud_topic parameters.
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic) {
// The subscribers are then created and assigned to the data pretreat flow object using the CloudSubscriber and IMUSubscriber objects respectively.
// The velocity subscriber is created next, followed by the GNSS subscriber.
    // subscribers:
    // a. velodyne measurement:
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    // b. OXTS IMU:
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    // c. OXTS velocity:
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
    // d. OXTS GNSS:
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
 /*
 * creating a new TFListener object.
 *This is done with the std::make_shared function, which creates a new instance of the class and returns it to the caller.
 *The first parameter passed into this function is an nh variable that will be used as a reference to the network handle.
 *The second parameter passed in is "/imu_link", which specifies where on our server we want to listen for data from our IMU sensor.
 *The third parameter passed in is "/velo_link".
 *This specifies where on our server we want to listen for data from our Velo lidar sensor.
 */   
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");

    // publishers:
// Four publishers are created: 
// cloud_publisher (which publishes information about ?position of the device's lidar sensor?) 
//imu_publisher (which publishes information about ?the position of the device's IMU sensor.?).
// pos_vel publisher (which publishes information about ?a vector containing information about velocity changes in relation to map coordinates on an interval basis.?).
// gnss_publisher (which publishes information about ??).

// These objects are then assigned references using std::make_shared .
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, "/velo_link", 100);
    imu_pub_ptr_ = std::make_shared<IMUPublisher>(nh, "/synced_imu", "/imu_link", 100);
    pos_vel_pub_ptr_ = std::make_shared<PosVelPublisher>(nh, "/synced_pos_vel", "/map", "/imu_link", 100);
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", "/map", 100);

    // motion compensation for lidar measurement: 
    // correct the distortion of point cloud
    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
}

bool DataPretreatFlow::Run() {
// starts by reading the data.If it can't read the data, then it returns false.
    if (!ReadData())
        return false;

// InitCalibration() is called to initialize the calibration process, or return false
    if (!InitCalibration()) 
        return false;
// init GNSS(), or return false
    if (!InitGNSS())
        return false;

    while(HasData()) {
// checks if there are any valid readings in the buffer before continuing on its way.
// If there are no valid readings, then this means that all of the sensor's buffers 
// have been filled up and nothing else has been received yet; therefore, it will continue on its way without doing anything else.
        if (!ValidData())
            continue;
// if hasData() and data is valid, then TransformData and publishdata()
// transforming all of the data into objects containing useful information like GPS coordinates or lidar range estimates,
        TransformData();
// publishes these objects back out onto their respective queues so other parts of your program can access them later on when needed.
        PublishData();
    }

    return true;
}


/*
*1.从ros缓冲区中传感器数据取出来，并放进XXX_data_buff_容器中
*2.传感器同步，另外三种传感器做插值
*3. 如果任意传感器没有插值，剔除该雷达点云帧，找下一个点云帧做融合

*/
bool DataPretreatFlow::ReadData() {
/*
*雷达点云作为主传感器，其他传感器做插值。包含IMU信息(unsynced_imu_)、速度信息(unsynced_velocity_)、GNSS信息(unsynced_gnss_)
*雷达点云不需要插值，直接放进cloud_data_buff_容器中
*/
//  starts by declaring static double-ended queue(compared to vector) variables as buffer to hold the input lists of unsynced streaming sensor data.
// imu, velocity (?? odometry??), gnss
    static std::deque<IMUData> unsynced_imu_;
    static std::deque<VelocityData> unsynced_velocity_;
    static std::deque<GNSSData> unsynced_gnss_;

/*
以imu的ParseData()函数为例：
ParseData()
读取数据，将新的传感器数据new_imu_data_填充进imu容器imu_data_buff中。

void IMUSubscriber::ParseData(std::deque<IMUData>& imu_data_buff) {
    if (new_imu_data_.size() > 0) {
        imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
        new_imu_data_.clear();
    }
}
*/
    // fetch lidar measurements from buffer:
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    imu_sub_ptr_->ParseData(unsynced_imu_);
    velocity_sub_ptr_->ParseData(unsynced_velocity_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_);

// If there is no data in the cloud data buffer, it will return false.
// 点云容器不能为空
    if (cloud_data_buff_.size() == 0)
        return false;

/*
* 2、传感器同步，另外三种传感器做插值
cloud_time为主传感器激光雷达的参考时间，然后后面通过SyncData（）函数对三个传感器进行插值实现同步操作。具体就是索引和插值。
插值SyncData()，具体代码以imu为例，见末尾
插值的流程是先获得主传感器时间，然后根据索引结果获得这一同步时间前后的两帧数据，根据前后两帧数据的采集时刻，以及要插入的时刻，根据比例获得权重得到另一传感器在同步时间的结果。这里以IMU数据为例，三种传感器流程类似。
找到与同步时间相邻的左右两个数据，在UnsyncedData.at(0)和.at(1)之间
根据时间差计算权重线性插值
输入原始数据，输出线性插值后的数据。
*/
    // use timestamp of lidar measurement as reference:
    double cloud_time = cloud_data_buff_.front().time;
    // sync IMU, velocity and GNSS with lidar measurement:
    // find the two closest measurement around lidar measurement time
    // then use linear interpolation to generate synced measurement:
    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
    bool valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);

    // only mark lidar as 'inited' when all the three sensors are synced:
// 如果任意传感器没有插值，剔除该雷达点云帧，找下一个点云帧做融合
    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_imu || !valid_velocity || !valid_gnss) {
// Removes the first element of the container. If there are no elements in the container, the behavior is undefined.
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}

bool DataPretreatFlow::InitCalibration() {
    // lookup imu pose in lidar frame: (Lidar database)
    static bool calibration_received = false;
    if (!calibration_received) {
// if function LokupData() in lidar_to_imu_ptr_ return true, then relative pose relation of lidar_to_imu is passed in, 
// then calibration can be initialized
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

// initialize the GNSS receiver.
bool DataPretreatFlow::InitGNSS() {
// checks to see if the GNSS receiver has been initialized.
// If it hasn't, then it will start it and set gnss_inited to be true and return the status.
    static bool gnss_inited = false;
    if (!gnss_inited) {
// Get gnss data from buffer
        GNSSData gnss_data = gnss_data_buff_.front();
/*
GNSS数据初始化位姿
使用GNSS数据（平移）和IMU（旋转）数据初始化位姿、更新位姿的两个函数为

gnss_data.InitOriginPosition(); 
gnss_data.UpdateXYZ();
源码中表现

void GNSSData::InitOriginPosition() {
    geo_converter.Reset(latitude, longitude, altitude);
    origin_position_inited = true;
}

void GNSSData::UpdateXYZ() {
    if (!origin_position_inited) {
        LOG(WARNING) << "GeoConverter has not set origin position";
    }
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}
*/ 
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}
// starts by checking if the cloud, imu, velocity, gnss data buffer is empty respectively.
// If it is empty, then the function returns false.
bool DataPretreatFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
    if (velocity_data_buff_.size() == 0)
        return false;
    if (gnss_data_buff_.size() == 0)
        return false;

    return true;
}

// validates the data in the buffer.
bool DataPretreatFlow::ValidData() {
// set the current_cloud_data get the first element of the corresponding buffer
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

// starts by checking the difference between the current lidar cloud data time and the current imu, velocity, gnss data time respectively.
// If there is a difference, then it means that something has changed in the data stream.
    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;
    //
    // this check assumes the frequency of lidar is 10Hz:
    //

// the time difference is set to between -0.05 ~ 0.05

// checks if the difference time is less than a certain range of values (in this case, -0.05).
// If so, then that means the lidar data is much later than other sensor data, remove the first point cloud element and return false.
    if (diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

//checks if the difference between the current time and the previous time is bigger than a certain range of values (in this case, 0.05).
// If so, then that means the lidar data is much ahead than other sensor data, remove the first point cloud element and return false.
    if (diff_imu_time > 0.05) {
        imu_data_buff_.pop_front();
        return false;
    }

    if (diff_velocity_time > 0.05) {
        velocity_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

// time difference is set to between -0.05 ~ 0.05, data are valid and the first element of buffer is sequncelly used and poped out with returing true.

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    velocity_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

bool DataPretreatFlow::TransformData() {
// starts by creating a matrix4f object called gnss_pose_ and initializing it with the identity matrix.
// starts by creating a reference pose matrix and then using the GNSS data to update the pose.
// The first three dimensions of the pose are from the imu data, while the fourth dimension is from the gnss.

    // a. get reference pose:
    gnss_pose_ = Eigen::Matrix4f::Identity();
    
/*
* 用GNSS数据（平移）和IMU（旋转）数据初始化位姿、更新位姿的两个函数为

gnss_data.InitOriginPosition(); 
gnss_data.UpdateXYZ();
源码中表现

void GNSSData::InitOriginPosition() {
    geo_converter.Reset(latitude, longitude, altitude);
    origin_position_inited = true;
}

void GNSSData::UpdateXYZ() {
    if (!origin_position_inited) {
        LOG(WARNING) << "GeoConverter has not set origin position";
    }
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U); // 经纬度高度，东北天
}
*/

// get position from GNSS 更新gnss位置数据
    current_gnss_data_.UpdateXYZ();  // ?? no definition of current_gnss_data_ in this function ??
// the (1~3 rows,4th column) of gnss_pose_ matrix get the current gnss position data's E, N, U projection respectively
// set the first column of our new matrix to be equal to current_gnss_data_.localE.
    gnss_pose_(0,3) = current_gnss_data_.local_E;
    gnss_pose_(1,3) = current_gnss_data_.local_N;
    gnss_pose_(2,3) = current_gnss_data_.local_U;
    // get orientation from IMU:
    // continues by getting an orientation from IMU using GetOrientationMatrix().
    // take the IMU's current orientation and convert it into a 3-dimensional matrix.
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    // this is lidar pose in GNSS/map frame:
// the pointer lidar_to_imu_ point to gnss_pose_ 
    gnss_pose_ *= lidar_to_imu_;

    // b. set synced pos vel
// the position xyz vector is gotten from current_gnss_data_ and velocity xyz vector is gotten from current_velocity_data_.linear_velocity
    pos_vel_.pos.x() = current_gnss_data_.local_E;
    pos_vel_.pos.y() = current_gnss_data_.local_N;
    pos_vel_.pos.z() = current_gnss_data_.local_U;

//current_velocity_data_, an object that contains information about the robot's linear velocity and angular velocity??.
    pos_vel_.vel.x() = current_velocity_data_.linear_velocity.x;
    pos_vel_.vel.y() = current_velocity_data_.linear_velocity.y;
    pos_vel_.vel.z() = current_velocity_data_.linear_velocity.z;

    // c. motion compensation for lidar measurements:
// 该变换感觉是反了，是否应该改为lidar_to_imu_.inverse().
// current_velocity_data_.TransformCoordinate(lidar_to_imu_);
    current_velocity_data_.TransformCoordinate(lidar_to_imu_.inverse());
// 0.1 ?? time?? with the even velocity hypothesis?? Set the motion model
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
// compensate the point cloud distortion , 转换后的参数用同一个名字
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);

    return true;
}

bool DataPretreatFlow::PublishData() {
    // take lidar measurement time as synced timestamp: (linear interplotion)
    // starts by getting the timestamp of the last time that data was synced
    const double &timestamp_synced = current_cloud_data_.time;

// publish distortion adjusted cloud data, the first passed parameter is a pointer to avoid direct data copy
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, timestamp_synced);
// current_imu_data_ is taken from imu data buffer's first element
    imu_pub_ptr_->Publish(current_imu_data_, timestamp_synced);
// gnss position and imu(or 轮速计？？) linear velocity in synced timestamp
    pos_vel_pub_ptr_->Publish(pos_vel_, timestamp_synced);
    
    //
    // this synced odometry has the following info:
    //
    // a. lidar frame's pose in map. (lidar_to_imu_??)
    // b. lidar frame's velocity 
// gnss_pose_: 3x3 imu ratation, 3x1 gnss postion (lidar pose in GNSS/map frame??)
// current_velocity_data_, an object that contains information about the robot's linear velocity and angular velocity??.
    gnss_pub_ptr_->Publish(gnss_pose_, current_velocity_data_, timestamp_synced);

    return true;
}

}

/*
插值函数 
bool IMUData::SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time) {
    // 1.保证数据大于2个，同步时间在两个传感器数据UnsyncedData.at(0)和.at(1)中间
    while (UnsyncedData.size() >= 2) {
        // a. 同步时间sync小于传感器第一个数据,失败
        if (UnsyncedData.front().time > sync_time) 
            return false;
        // b. 同步时间sync大于第二个数据，将第二个设置为.at(0)
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        // c. 距离前一个时间差值较大，说明数据有丢失，前一个不能用
        if (sync_time - UnsyncedData.front().time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        // d. 距离后一个时间差值较大，说明数据有丢失，后一个不能用
        if (UnsyncedData.at(1).time - sync_time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;
    // 同步数据synced，前后两帧数据front、back
    IMUData front_data = UnsyncedData.at(0);
    IMUData back_data = UnsyncedData.at(1);
    IMUData synced_data;
    // 2、根据时间差计算权重线性插值
    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;// 同步时间
    // 线速度
    synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
    synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
    synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
    // 角速度
    synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
    // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
    // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
    synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
    synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
    synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
    synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
    synced_data.orientation.Normlize();// 线性插值之后要归一化
    
    // 最后插入即可
    SyncedData.push_back(synced_data);
 
    return true;
}
*/

/*
bool DistortionAdjust::AdjustCloud(...)
{
    float orientation_space = 2.0 * M_PI;
    float delete_space = 5.0 * M_PI / 180.0;
    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);

    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotate_matrix = t_V.matrix();
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3,3>(0,0) = rotate_matrix.inverse();
   pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);// 这边为什么要将 首个激光点 旋转到 雷达的X轴上去呢？
    velocity_ = rotate_matrix * velocity_;// 没能明白您这里的线速度和角速度为什么还需要坐标变换，您能解释下吗，麻烦了。
    angular_rate_ = rotate_matrix * angular_rate_;

    for (size_t point_index = 1; point_index < origin_cloud_ptr->points.size(); ++point_index) {
        float orientation = atan2(origin_cloud_ptr->points[point_index].y, origin_cloud_ptr->points[point_index].x);
        if (orientation < 0.0)
            orientation += 2.0 * M_PI;
        
        if (orientation < delete_space || 2.0 * M_PI - orientation < delete_space)
            continue;

        float real_time = fabs(orientation) / orientation_space * scan_period_ - scan_period_ / 2.0;

        Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x,
                                     origin_cloud_ptr->points[point_index].y,
                                     origin_cloud_ptr->points[point_index].z);

        Eigen::Matrix3f current_matrix = UpdateMatrix(real_time);
        Eigen::Vector3f rotated_point = current_matrix * origin_point;
        Eigen::Vector3f adjusted_point = rotated_point + velocity_ * real_time;
        CloudData::POINT point;
        point.x = adjusted_point(0);
        point.y = adjusted_point(1);
        point.z = adjusted_point(2);
        output_cloud_ptr->points.push_back(point);
    }

    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());
    return true;
}
// 关于补畸变的事情，别的程序里的流程都是先找到起始点和终止点，然后检测角度是否旋转过半(halfpassed那个变量)，在一帧点云里，如果点的排列顺序是先列后行，那么没有问题，
//只需要判断一次halfpassed就可以了，但是如果是先行后列，则需要判断64次，这个数据集里就属于后者，64次判断很难保证比较准，因为某一根线的部分区域没有返回值是很正常的现象。
//如果都转到第一个点上去，那么就可以根据每个点所处的象限判断它的角度了，不需要判断halfpassed
// 大概明白您的意思了，您看我理解有没有错。 但这样可能会带来一个问题。假如当中间某个点求出来的角度是1°，没有halfpassed，也就没法知道这个点到底是1°还是361°，也就没法判断这个点相对swap第一个点，雷达是转了1°还是361°。 
// 所以您通过下面的判断，不对第一个点附近的点做处理。
if (orientation < delete_space || 2.0 * M_PI - orientation < delete_space) continue;
*/