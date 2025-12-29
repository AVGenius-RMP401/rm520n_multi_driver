#ifndef RM520N_MULTI_DRIVER__AT_INTERFACE_HPP_
#define RM520N_MULTI_DRIVER__AT_INTERFACE_HPP_

#include <string>
#include <optional>
#include <tuple>
#include <memory>
#include <vector>

#include "rm520n_multi_driver/serial_interface.hpp"

namespace rm520n_multi_driver
{

struct NetworkStatus
{
    bool connected;
    bool registered;
    bool attached;
    std::string access_tech;  // LTE/NR5G-SA/NR5G-NSA
    std::string carrier;      // 통신사
    int rsrp;
    int rsrq;
    int sinr;
};

class ATInterface
{
public:
    ATInterface(const std::string &port, int baudrate = 115200);

    std::optional<NetworkStatus> getNetworkStatus();
    std::vector<std::string> sendAT(const std::string &cmd, int timeout_ms = 300);

    bool isRegistered();
    bool isAttached();

private:
    std::unique_ptr<SerialInterface> serial_;

    // 파싱 함수
    static std::string extractCarrier(const std::vector<std::string> &lines);
    static NetworkStatus parseQengServingcell(const std::vector<std::string> &lines);

    static bool checkCREG(const std::vector<std::string> &lines);
    static bool checkCGATT(const std::vector<std::string> &lines);

    // util
    static std::vector<std::string> split(const std::string &str, char delim);
};

}  // namespace rm520n_multi_driver

#endif  // RM520N_MULTI_DRIVER__AT_INTERFACE_HPP_
