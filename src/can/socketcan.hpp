#pragma once

// Linux SocketCAN (can0, vcan0). On other systems open() fails.

#include "can/canaerospace.hpp"

#include <optional>
#include <string>

namespace bodennav::can {

class SocketCan {
public:
    SocketCan() = default;
    ~SocketCan();
    SocketCan(const SocketCan&) = delete;
    SocketCan& operator=(const SocketCan&) = delete;

    // false with `error` set when the interface cannot be opened.
    bool open(const std::string& interface, std::string& error);
    bool send(const Frame& frame);
    // Waits up to `timeout_ms` for a standard frame.
    std::optional<Frame> receive(int timeout_ms);
    [[nodiscard]] bool is_open() const { return fd_ >= 0; }

private:
    int fd_{-1};
};

}  // namespace bodennav::can
