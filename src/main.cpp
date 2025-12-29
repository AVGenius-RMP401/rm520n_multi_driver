#include "rm520n_multi_driver/rm520n_multi_driver_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rm520n_multi_driver::RM520NMultiDriverNode>(rclcpp::NodeOptions());

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
