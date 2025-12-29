#include "rm520n_multi_driver/at_interface.hpp"
#include <regex>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

namespace rm520n_multi_driver
{

ATInterface::ATInterface(const std::string &port, int baudrate)
{
    serial_ = std::make_unique<SerialInterface>(port, baudrate);
    serial_->flush();
}

std::vector<std::string> ATInterface::sendAT(const std::string &cmd, int timeout_ms)
{
    serial_->flush();
    serial_->writeLine(cmd + "\r");
    std::string raw;
    auto start = std::chrono::steady_clock::now();
    while (true)
    {
        int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > timeout_ms) break;
        raw = serial_->readRaw(256, 30); // polling 단위는 상황에 따라 조정
        if (raw.find("OK") != std::string::npos || raw.find("ERROR") != std::string::npos)
            break;
    }

    // 줄 단위로 분리해서, 공백/빈 줄 제외하고 벡터에 담기
    std::vector<std::string> lines;
    std::istringstream ss(raw);
    std::string line;
    while (std::getline(ss, line)) {
        // 앞뒤 공백/개행 제거
        size_t first = line.find_first_not_of(" \r\n");
        size_t last = line.find_last_not_of(" \r\n");
        if (first == std::string::npos) continue; // 빈 줄
        line = line.substr(first, last - first + 1);
        if (!line.empty()) lines.push_back(line);
    }
    return lines;
}


std::optional<NetworkStatus> ATInterface::getNetworkStatus()
{
    NetworkStatus status;

    // 1. 통신사명 (AT+QSPN)
    auto qspn = sendAT("AT+QSPN", 300);
    status.carrier = extractCarrier(qspn);

    // 2. 신호 및 네트워크 정보 (AT+QENG="servingcell")
    auto servingcell = sendAT("AT+QENG=\"servingcell\"", 300);
    NetworkStatus radio = parseQengServingcell(servingcell);

    status.access_tech = radio.access_tech;
    status.rsrp = radio.rsrp;
    status.rsrq = radio.rsrq;
    status.sinr = radio.sinr;

    // 3. 등록/attach
    auto creg = sendAT("AT+CEREG?", 300);
    auto cgatt = sendAT("AT+CGATT?", 300);
    status.registered = checkCREG(creg);
    status.attached = checkCGATT(cgatt);
    status.connected = status.registered && status.attached && status.rsrp > -120;

    if (status.access_tech.empty())
        status.access_tech = "UNKNOWN";
    if (status.carrier.empty())
        status.carrier = "UNKNOWN";

    return status;
}

// ----- 파싱 함수들 -----

std::string ATInterface::extractCarrier(const std::vector<std::string> &lines)
{
    // +QSPN: "SKTelecom","SKTelecom","45005",0
    std::regex reg("\\+QSPN: \"([^\"]+)\"");
    for (const auto &line : lines)
    {
        std::smatch match;
        if (std::regex_search(line, match, reg) && match.size() > 1)
            return match.str(1);
    }
    return "UNKNOWN";
}

NetworkStatus ATInterface::parseQengServingcell(const std::vector<std::string> &lines)
{
    NetworkStatus status;
    status.access_tech = "UNKNOWN";
    status.rsrp = -150;
    status.rsrq = -30;
    status.sinr = 0;

    for (const auto& line : lines)
    {
        if (line.find("+QENG:") == std::string::npos) continue;
        auto fields = split(line, ',');
        // LTE
        if (line.find("\"LTE\"") != std::string::npos && fields.size() > 16)
        {
            status.access_tech = "LTE";
            try {
                status.rsrp = std::stoi(fields[13]);
                status.rsrq = std::stoi(fields[14]);
                status.sinr = std::stoi(fields[16]);
            } catch (...) {
                status.rsrp = -150; status.rsrq = -30; status.sinr = 0;
            }
            break;
        }
        // 5G SA
        if (line.find("NR5G-SA") != std::string::npos && fields.size() > 10)
        {
            status.access_tech = "NR5G-SA";
            try {
                status.rsrp = std::stoi(fields[fields.size() - 6]);
                status.rsrq = std::stoi(fields[fields.size() - 5]);
                status.sinr = std::stoi(fields[fields.size() - 4]);
            } catch (...) {
                status.rsrp = -150; status.rsrq = -30; status.sinr = 0;
            }
            break;
        }
        // 5G NSA
        if (line.find("NR5G-NSA") != std::string::npos && fields.size() > 7)
        {
            status.access_tech = "NR5G-NSA";
            try {
                status.rsrp = std::stoi(fields[4]);
                status.sinr = std::stoi(fields[5]);
                status.rsrq = std::stoi(fields[6]);
            } catch (...) {
                status.rsrp = -150; status.rsrq = -30; status.sinr = 0;
            }
            break;
        }
    }
    return status;
}

bool ATInterface::checkCREG(const std::vector<std::string> &lines)
{
    std::regex reg("\\+CEREG:\\s*\\d,(\\d)");
    for (const auto &line : lines)
    {
        std::smatch match;
        if (std::regex_search(line, match, reg) && match.size() > 1)
        {
            int val = std::stoi(match.str(1));
            if (val == 1 || val == 5)
                return true;
        }
    }
    return false;
}

bool ATInterface::checkCGATT(const std::vector<std::string> &lines)
{
    std::regex reg("\\+CGATT:\\s*(\\d)");
    for (const auto &line : lines)
    {
        std::smatch match;
        if (std::regex_search(line, match, reg) && match.size() > 1)
            return std::stoi(match.str(1)) == 1;
    }
    return false;
}

std::vector<std::string> ATInterface::split(const std::string &str, char delim)
{
    std::vector<std::string> out;
    std::string buf;
    std::istringstream ss(str);
    while (std::getline(ss, buf, delim)) out.push_back(buf);
    return out;
}

} // namespace rm520n_multi_driver
