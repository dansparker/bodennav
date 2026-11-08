#include "can/socketcan.hpp"

#ifdef __linux__
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#endif

namespace bodennav::can {

#ifdef __linux__

SocketCan::~SocketCan() {
    if (fd_ >= 0) ::close(fd_);
}

bool SocketCan::open(const std::string& interface, std::string& error) {
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        error = std::string("socket: ") + std::strerror(errno);
        return false;
    }
    ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        error = interface + ": " + std::strerror(errno);
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        error = std::string("bind: ") + std::strerror(errno);
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    return true;
}

bool SocketCan::send(const Frame& frame) {
    if (fd_ < 0) return false;
    can_frame f{};
    f.can_id = frame.id & CAN_SFF_MASK;
    f.can_dlc = frame.dlc;
    std::memcpy(f.data, frame.data.data(), 8);
    return ::write(fd_, &f, sizeof(f)) == static_cast<ssize_t>(sizeof(f));
}

std::optional<Frame> SocketCan::receive(int timeout_ms) {
    if (fd_ < 0) return std::nullopt;
    pollfd p{fd_, POLLIN, 0};
    if (::poll(&p, 1, timeout_ms) <= 0) return std::nullopt;
    can_frame f{};
    if (::read(fd_, &f, sizeof(f)) != static_cast<ssize_t>(sizeof(f))) return std::nullopt;
    if ((f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0) return std::nullopt;
    Frame out;
    out.id = f.can_id & CAN_SFF_MASK;
    out.dlc = static_cast<std::uint8_t>(f.can_dlc > 8 ? 8 : f.can_dlc);
    std::memcpy(out.data.data(), f.data, 8);
    return out;
}

#else

SocketCan::~SocketCan() = default;

bool SocketCan::open(const std::string& interface, std::string& error) {
    error = interface + ": SocketCAN is only available on Linux";
    return false;
}

bool SocketCan::send(const Frame&) { return false; }

std::optional<Frame> SocketCan::receive(int) { return std::nullopt; }

#endif

}  // namespace bodennav::can
