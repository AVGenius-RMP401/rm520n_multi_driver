#include "rm520n_multi_driver/serial_interface.hpp"
#include <iostream>
#include <cstring>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <sys/ioctl.h>
#include <errno.h>

namespace rm520n_multi_driver
{

    SerialInterface::SerialInterface(const std::string &port, int baudrate)
    : fd_(-1)
    {
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0)
            throw std::runtime_error("Failed to open serial port " + port + ": " + strerror(errno));

        // [추가] DTR/RTS ON
        int modem_bits = 0;
        if (ioctl(fd_, TIOCMGET, &modem_bits) == -1)
            throw std::runtime_error("Failed to get modem bits: " + std::string(strerror(errno)));
        modem_bits |= TIOCM_DTR;
        modem_bits |= TIOCM_RTS;
        if (ioctl(fd_, TIOCMSET, &modem_bits) == -1)
            throw std::runtime_error("Failed to set modem bits: " + std::string(strerror(errno)));

        // [추가] 장치 안정화를 위한 sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        configurePort(baudrate);
        flush();
    }

    SerialInterface::~SerialInterface()
    {
        if (fd_ >= 0)
        {
            close(fd_);
        }
    }

    void SerialInterface::configurePort(int baudrate)
    {
        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(fd_, &tty) != 0)
            throw std::runtime_error("Error from tcgetattr: " + std::string(strerror(errno)));

        // Baudrate 확장
        speed_t speed;
        switch (baudrate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
    #ifdef B230400
            case 230400: speed = B230400; break;
    #endif
    #ifdef B921600
            case 921600: speed = B921600; break;
    #endif
            default:
                throw std::runtime_error("Unsupported baud rate");
        }

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit
        tty.c_iflag = 0;
        tty.c_oflag = 0;
        tty.c_lflag = 0;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 10; // 1s 단위 타임아웃

        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
            throw std::runtime_error("Error from tcsetattr: " + std::string(strerror(errno)));
    }

    void SerialInterface::writeLine(const std::string &cmd)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (fd_ < 0) throw std::runtime_error("Serial port not open");
        std::string msg = cmd;
        if (msg.back() != '\r') msg += "\r"; // AT 명령은 \r 필수
        ssize_t n = write(fd_, msg.c_str(), msg.size());
        if (n < 0)
            throw std::runtime_error("Serial write error: " + std::string(strerror(errno)));
    }

    std::string SerialInterface::readLine(int timeout_ms)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (fd_ < 0) throw std::runtime_error("Serial port not open");
        std::string line;
        char c;
        auto start = std::chrono::steady_clock::now();
        while (true)
        {
            ssize_t n = read(fd_, &c, 1);
            if (n > 0)
            {
                if (c == '\n') break;
                if (c != '\r') line += c;
            }
            else if (n == 0)
            {
                // 타임아웃 체크
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_ms)
                    break;
                // std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            else
            {
                throw std::runtime_error("Serial read error: " + std::string(strerror(errno)));
            }
        }
        return line;
    }

    // serial_interface.cpp
    std::string SerialInterface::readRaw(size_t nbytes, int timeout_ms)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (fd_ < 0) throw std::runtime_error("Serial port not open");
        std::string data;
        char buf[256];
        auto start = std::chrono::steady_clock::now();
        while (data.size() < nbytes)
        {
            ssize_t n = read(fd_, buf, std::min(nbytes - data.size(), sizeof(buf)));
            if (n > 0)
                data.append(buf, n);
                if (data.size() >= nbytes) break;
            else {
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_ms)
                    break;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        return data;
    }


    void SerialInterface::flush()
    {
        if (fd_ >= 0)
            tcflush(fd_, TCIOFLUSH);
    }

    bool SerialInterface::isOpen() const
    {
        return fd_ >= 0;
    }

} // namespace rm520n_multi_driver
