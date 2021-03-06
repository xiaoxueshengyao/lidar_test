/***
 * 0604参考kitti_data把数据发布也分开
 * ***/

#ifndef CLOUD_PUBLISHER_HPP_
#define CLOUD_PUBLISHER_HPP_

#include "sensor_msgs/PointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl_conversions/pcl_conversions.h>
#include "kitti_data/cloud_data.hpp"


namespace lidar_test{

//点云发布
class CloudPublisher{
    public:
      CloudPublisher(ros::NodeHandle& nh, std::string topic_name,std::string frame_id,size_t buff_size);
      CloudPublisher() = default;


      void Publish(CloudData::CloudPtr& cloud_ptr);
      void Publish(CloudData::CloudPtr& cloud_ptr, double time);
      bool HasSubscribers();

    private:
      void PublishData(CloudData::CloudPtr& cloud_ptr_input, ros::Time time);

    private:
      ros::NodeHandle nh_;
      ros::Publisher publisher_;
      std::string frame_id_;

};


}


#endif