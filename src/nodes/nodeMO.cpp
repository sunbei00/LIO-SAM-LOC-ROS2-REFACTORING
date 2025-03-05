#include "MapOptimization.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto MO = std::make_shared<MapOptimization>(options);
    exec.add_node(MO);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Map Optimization Started.\033[0m");

    //std::thread visualizeMapThread(&MapOptimization::visualizeGlobalMapThread, MO);
    std::thread globalMatchingThread(&MapOptimization::globalMatchingThread, MO);

    exec.spin();

    rclcpp::shutdown();

    //visualizeMapThread.join();
    globalMatchingThread.join();

    return 0;
}
