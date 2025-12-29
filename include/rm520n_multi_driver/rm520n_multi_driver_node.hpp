#ifndef RM520N_MULTI_DRIVER__RM520N_MULTI_DRIVER_NODE_HPP_
#define RM520N_MULTI_DRIVER__RM520N_MULTI_DRIVER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tod_msgs/msg/network_status.hpp"
#include "rm520n_multi_driver/serial_interface.hpp"
#include "rm520n_multi_driver/nmea_parser.hpp"
#include "rm520n_multi_driver/at_interface.hpp"

#include <memory>
#include <chrono>
#include <string>

namespace rm520n_multi_driver
{

class RM520NMultiDriverNode : public rclcpp::Node
{
public:
    explicit RM520NMultiDriverNode(const rclcpp::NodeOptions &options);
    ~RM520NMultiDriverNode();

private:
    bool at_ok(const std::vector<std::string>& lines);
    void timerCallback();
    void gpsCallback();
    void atPollingWorker();
    uint8_t rsrpToQualityPercent(int rsrp_dbm);

    std::shared_ptr<SerialInterface> gps_serial_;
    std::shared_ptr<ATInterface> at_interface_;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<tod_msgs::msg::NetworkStatus>::SharedPtr status_pub_;

    rclcpp::TimerBase::SharedPtr timer_, gps_timer_;

    std::string gps_port_;
    int gps_baud_;
    std::string at_port_;
    int at_baud_;
    int polling_interval_ms_, gps_interval_ms;
    bool gps_status{false};

    std::mutex net_mtx_;
    NetworkStatus latest_status_;
    std::thread at_worker_;
    bool stop_worker_ = false;
};

}  // namespace rm520n_multi_driver

#endif  // RM520N_MULTI_DRIVER__RM520N_MULTI_DRIVER_NODE_HPP_
