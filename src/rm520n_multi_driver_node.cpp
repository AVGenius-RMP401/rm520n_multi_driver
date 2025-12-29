#include "rm520n_multi_driver/rm520n_multi_driver_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace rm520n_multi_driver
{

RM520NMultiDriverNode::RM520NMultiDriverNode(const rclcpp::NodeOptions &options)
    : Node("rm520n_multi_driver", options)
{
    gps_port_ = declare_parameter<std::string>("gps_port", "/dev/ttyUSB4");
    gps_baud_ = declare_parameter<int>("gps_baudrate", 115200);
    at_port_ = declare_parameter<std::string>("at_port", "/dev/ttyUSB5");  // 새 파라미터
    at_baud_ = declare_parameter<int>("at_baudrate", 115200);
    polling_interval_ms_ = declare_parameter<int>("polling_interval_ms", 1000);
    gps_interval_ms = declare_parameter<int>("gps_interval_ms", 1000);
    std::string gps_topic_name = declare_parameter<std::string>("publish_topics.gps_topic", "gps_fix");
    std::string status_topic_name = declare_parameter<std::string>("publish_topics.status_topic", "network_status");
    
    at_interface_ = std::make_shared<ATInterface>(at_port_, at_baud_);

    const auto reply = at_interface_->sendAT("AT+QGPS=1", gps_interval_ms);
    if (at_ok(reply)) {
        RCLCPP_INFO(get_logger(), "GPS module activated successfully.");
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to activate GPS module.");
        for (auto& l : reply) RCLCPP_DEBUG(get_logger(), "[AT] %s", l.c_str());
    }

    gps_serial_ = std::make_shared<SerialInterface>(gps_port_, gps_baud_);

    gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(gps_topic_name, 10);
    status_pub_ = create_publisher<tod_msgs::msg::NetworkStatus>(status_topic_name, 10);

    at_worker_ = std::thread([this]() { atPollingWorker(); });

    timer_ = create_wall_timer(
        std::chrono::milliseconds(polling_interval_ms_),
        std::bind(&RM520NMultiDriverNode::timerCallback, this));

    gps_timer_ = create_wall_timer(
        std::chrono::milliseconds(gps_interval_ms),
        std::bind(&RM520NMultiDriverNode::gpsCallback, this));
}

RM520NMultiDriverNode::~RM520NMultiDriverNode()
{
    stop_worker_ = true;
    if (at_worker_.joinable())
        at_worker_.join();
}

bool RM520NMultiDriverNode::at_ok(const std::vector<std::string>& lines) {
    if (lines.empty()) return false;
    if (std::find(lines.begin(), lines.end(), "ERROR") != lines.end()) return false;
    return std::find(lines.begin(), lines.end(), "OK") != lines.end();
}


void RM520NMultiDriverNode::atPollingWorker()
{
    while (!stop_worker_) {
        auto net_status = at_interface_->getNetworkStatus();
        if (net_status) {
            std::lock_guard<std::mutex> lock(net_mtx_);
            latest_status_ = *net_status;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void RM520NMultiDriverNode::timerCallback()
{
    tod_msgs::msg::NetworkStatus status_msg;
    status_msg.stamp = now();

    // 네트워크 상태(AT 명령 기반)
    {
        std::lock_guard<std::mutex> lock(net_mtx_);
        status_msg.access_technology = latest_status_.access_tech;
        status_msg.carrier           = latest_status_.carrier;
        status_msg.rsrp              = latest_status_.rsrp;
        status_msg.rsrq              = latest_status_.rsrq;
        status_msg.sinr              = latest_status_.sinr;
        status_msg.signal_quality    = rsrpToQualityPercent(latest_status_.rsrp);
        status_msg.registered        = latest_status_.registered;
        status_msg.attached          = latest_status_.attached;
        status_msg.connected         = latest_status_.connected;
        status_msg.gps_status         = gps_status;

    }
    status_pub_->publish(status_msg);
}

void RM520NMultiDriverNode::gpsCallback()
{
    try
    {
        std::string nmea_line = gps_serial_->readLine();
        if (!nmea_line.empty() && nmea_line[0] == '$' && nmea_line.find("$G") == 0)
        {
            auto gps_data = NMEAParser::parse(nmea_line);
            sensor_msgs::msg::NavSatFix gps_msg;
            gps_msg.header.stamp = now();
            gps_msg.header.frame_id = "gps";
            gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
            if (gps_data)
            {
                gps_msg.header.stamp = now();
                gps_msg.header.frame_id = "gps";
                gps_msg.latitude = std::get<0>(*gps_data);
                gps_msg.longitude = std::get<1>(*gps_data);
                if(std::get<2>(*gps_data) > 0.1)
                    gps_msg.altitude = std::get<2>(*gps_data);
                gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                gps_status = true;
            }
            else
            {
                gps_msg.latitude = 0.0;
                gps_msg.longitude = 0.0;
                gps_msg.altitude = 0.0;
                gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                gps_status = false;
            }
            gps_pub_->publish(gps_msg);
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "GPS read or parse error: %s", e.what());
        gps_status = false;
    }
}

uint8_t RM520NMultiDriverNode::rsrpToQualityPercent(int rsrp_dbm) {
    if (rsrp_dbm >= -44) return 100;
    if (rsrp_dbm <= -140) return 0;
    // 선형 변환
    double percent = std::round((rsrp_dbm + 140) / 80.0 * 100.0);
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    return static_cast<uint8_t>(percent);
}

}  // namespace rm520n_multi_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rm520n_multi_driver::RM520NMultiDriverNode)
