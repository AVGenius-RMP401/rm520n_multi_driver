#include "rm520n_multi_driver/nmea_parser.hpp"
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cstdlib>

namespace rm520n_multi_driver
{

// 문자열을 ',' 기준으로 분리
static std::vector<std::string> split(const std::string &s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

bool NMEAParser::validateChecksum(const std::string &nmea_sentence)
{
    // $로 시작, *가 반드시 포함되어야 함
    auto star = nmea_sentence.find('*');
    if (nmea_sentence.empty() || nmea_sentence[0] != '$' || star == std::string::npos)
        return false;

    // 데이터 부분 추출 (0번 인덱스 $ 제외, * 앞까지)
    std::string data = nmea_sentence.substr(1, star - 1);
    unsigned char cs = 0;
    for (auto ch : data) cs ^= ch;

    // * 이후 두 글자(16진수) 비교
    if (star + 2 >= nmea_sentence.size()) return false;
    std::string cs_str = nmea_sentence.substr(star + 1, 2);

    unsigned int cs_val = 0;
    try {
        cs_val = std::stoul(cs_str, nullptr, 16);
    } catch (...) {
        return false;
    }
    return cs == cs_val;
}

std::optional<std::tuple<double, double, double>> NMEAParser::parse(const std::string &nmea_sentence)
{
    // (1) Checksum 유효성 우선 검사 (필요 없다면 주석처리)
    if (!validateChecksum(nmea_sentence))
        return std::nullopt;

    // (2) GGA, GNGGA, RMC, GNRMC 등 모두 지원
    auto type = nmea_sentence.substr(1, 5); // GPGGA, GPRMC 등

    // RMC 문장
    if (type == "GPRMC" || type == "GNRMC")
    {
        auto fields = split(nmea_sentence, ',');
        if (fields.size() < 7) return std::nullopt;

        // 필드 2: status, "A": valid, "V": void
        std::string status = (fields.size() > 2 ? fields[2] : "");
        if (status != "A") return std::nullopt;

        std::string lat_str = fields[3];
        std::string lat_dir = fields[4];
        std::string lon_str = fields[5];
        std::string lon_dir = fields[6];

        if (lat_str.empty() || lon_str.empty()) return std::nullopt;

        double latitude = convertToDecimalDegrees(lat_str, lat_dir);
        double longitude = convertToDecimalDegrees(lon_str, lon_dir);
        double altitude = 0.0;  // RMC는 고도 정보 없음

        return std::make_tuple(latitude, longitude, altitude);
    }
    // GGA 문장
    else if (type == "GPGGA" || type == "GNGGA")
    {
        auto fields = split(nmea_sentence, ',');
        if (fields.size() < 10) return std::nullopt;

        // 필드 6: fix quality (0: invalid)
        std::string fix_quality = (fields.size() > 6 ? fields[6] : "0");
        if (fix_quality == "0") return std::nullopt;

        std::string lat_str = fields[2];
        std::string lat_dir = fields[3];
        std::string lon_str = fields[4];
        std::string lon_dir = fields[5];
        std::string alt_str = fields[9];

        if (lat_str.empty() || lon_str.empty()) return std::nullopt;

        double latitude = convertToDecimalDegrees(lat_str, lat_dir);
        double longitude = convertToDecimalDegrees(lon_str, lon_dir);

        double altitude = 0.0;
        try {
            altitude = std::stod(alt_str);
        } catch (...) {
            altitude = 0.0;
        }

        return std::make_tuple(latitude, longitude, altitude);
    }
    // 기타 포맷(필요시 추가)

    return std::nullopt;
}

double NMEAParser::convertToDecimalDegrees(const std::string &raw, const std::string &direction)
{
    if (raw.empty()) return 0.0;

    double deg = 0.0;
    double min = 0.0;

    size_t point_pos = raw.find('.');
    if (point_pos == std::string::npos || point_pos < 2) return 0.0;

    size_t deg_len = (direction == "N" || direction == "S") ? 2 : 3;
    try {
        deg = std::stod(raw.substr(0, deg_len));
        min = std::stod(raw.substr(deg_len));
    } catch (...) {
        return 0.0;
    }

    double decimal = deg + (min / 60.0);

    if (direction == "S" || direction == "W")
        decimal *= -1;

    return decimal;
}

}  // namespace rm520n_multi_driver
