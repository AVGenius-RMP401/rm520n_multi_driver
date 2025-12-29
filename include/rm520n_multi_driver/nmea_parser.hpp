#ifndef RM520N_MULTI_DRIVER__NMEA_PARSER_HPP_
#define RM520N_MULTI_DRIVER__NMEA_PARSER_HPP_

#include <string>
#include <optional>
#include <tuple>

namespace rm520n_multi_driver
{

class NMEAParser
{
public:
    // GGA 또는 RMC 문장을 파싱해서 (위도, 경도, 고도) 반환
    // 고도는 GGA에서만 추출 가능 (RMC는 고도 없음 → 0 반환)
    static std::optional<std::tuple<double, double, double>> parse(const std::string &nmea_sentence);

    static bool validateChecksum(const std::string &nmea_sentence);
private:
    static double convertToDecimalDegrees(const std::string &raw, const std::string &direction);
};

}  // namespace rm520n_multi_driver

#endif  // RM520N_MULTI_DRIVER__NMEA_PARSER_HPP_
