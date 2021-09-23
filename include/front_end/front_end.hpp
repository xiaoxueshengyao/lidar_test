/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 */
#ifndef FRONT_END_FRONT_END_HPP_
#define FRONT_END_FRONT_END_HPP_

#include <deque>

#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>

#include "kitti_data/cloud_data.hpp"
#include "general_models/registration/ndt_registration.hpp"
#include "general_models/cloud_filter/voxel_filter.hpp"

namespace lidar_test {
class FrontEnd {
  public:
    struct Frame { 
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData cloud_data;
    };

  public:
    FrontEnd();

    bool InitWithConfig();
    bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
    bool SetInitPose(const Eigen::Matrix4f& init_pose);

    bool SaveMap();
    bool GetNewLocalMap(CloudData::CloudPtr& local_map_ptr);
    bool GetNewGlobalMap(CloudData::CloudPtr& global_map_ptr);
    bool GetCurrentScan(CloudData::CloudPtr& current_scan_ptr);

  private:
    bool InitParam(const YAML::Node& config_node);
    bool InitDataPath(const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
    bool UpdateWithNewFrame(const Frame& new_key_frame);

  private:
    std::string data_path_ = "";

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> display_filter_ptr_;
    std::shared_ptr<RegistrationInterface> registration_ptr_; 

    std::deque<Frame> local_map_frames_;
    std::deque<Frame> global_map_frames_;

    bool has_new_local_map_ = false;
    bool has_new_global_map_ = false;
    CloudData::CloudPtr local_map_ptr_;
    CloudData::CloudPtr global_map_ptr_;
    CloudData::CloudPtr result_cloud_ptr_;
    Frame current_frame_;

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

    float key_frame_distance_ = 2.0;
    int local_frame_num_ = 20;
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}

#endif