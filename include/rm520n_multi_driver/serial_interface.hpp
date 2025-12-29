#ifndef RM520N_MULTI_DRIVER__SERIAL_INTERFACE_HPP_
#define RM520N_MULTI_DRIVER__SERIAL_INTERFACE_HPP_

#include <string>
#include <memory>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
#include <vector>
#include <mutex>

namespace rm520n_multi_driver
{

class SerialInterface
{
public:
    SerialInterface(const std::string &port, int baudrate = 115200);
    ~SerialInterface();

    std::string readLine(int timeout_ms = 1000); // timeout 옵션 추가
    std::string readRaw(size_t nbytes = 512, int timeout_ms = 200);
    void writeLine(const std::string &cmd); // AT 명령 송신용

    void flush(); // 버퍼 플러시
    bool isOpen() const;

private:
    int fd_;
    std::mutex mtx_; // (멀티스레딩 환경 대비)
    void configurePort(int baudrate);
};

}  // namespace rm520n_multi_driver

#endif  // RM520N_MULTI_DRIVER__SERIAL_INTERFACE_HPP_
