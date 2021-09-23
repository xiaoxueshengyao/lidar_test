
#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_test/saveMap.h"

#include "front_end/front_end_flow.hpp"

using namespace lidar_test;//saveMap的命名空间和其他的不一致，因为是自己瞎改的

std::shared_ptr<FrontEndFlow> _front_end_flow_ptr;

bool save_map_callback(saveMap::Request &request, saveMap::Response &response) {
    response.succeed = _front_end_flow_ptr->SaveMap();
    _front_end_flow_ptr->PublishGlobalMap();
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "/home/jingwan/lidar_test/src/lidar_test/config/front_end/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);
    // _front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);
    _front_end_flow_ptr = std::shared_ptr<FrontEndFlow>(new FrontEndFlow(nh));

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        _front_end_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}