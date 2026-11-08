#include "can/canaerospace.hpp"

#include <cstdio>
#include <cstring>

namespace bodennav::can {

std::optional<float> Message::as_float() const {
    if (data_type != static_cast<std::uint8_t>(DataType::Float) || data_length < 4) return std::nullopt;
    const std::uint32_t bits = (static_cast<std::uint32_t>(data[0]) << 24) | (static_cast<std::uint32_t>(data[1]) << 16) |
                               (static_cast<std::uint32_t>(data[2]) << 8) | static_cast<std::uint32_t>(data[3]);
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

std::string Message::as_achar() const {
    std::string text;
    for (std::uint8_t i = 0; i < data_length; ++i) {
        if (data[i] == 0) break;
        text.push_back(static_cast<char>(data[i]));
    }
    return text;
}

std::optional<Message> decode(const Frame& frame) {
    if (frame.id > 0x7FF || frame.dlc < 4 || frame.dlc > 8) return std::nullopt;
    Message m;
    m.id = static_cast<std::uint16_t>(frame.id);
    m.node_id = frame.data[0];
    m.data_type = frame.data[1];
    m.service_code = frame.data[2];
    m.message_code = frame.data[3];
    m.data_length = static_cast<std::uint8_t>(frame.dlc - 4);
    for (std::uint8_t i = 0; i < m.data_length; ++i) m.data[i] = frame.data[4 + i];
    return m;
}

Frame Encoder::header(std::uint16_t id, DataType type, std::uint8_t service_code, std::uint8_t length) {
    Frame f;
    f.id = id;
    f.dlc = static_cast<std::uint8_t>(4 + length);
    f.data[0] = node_id_;
    f.data[1] = static_cast<std::uint8_t>(type);
    f.data[2] = service_code;
    f.data[3] = codes_[id]++;  // wraps 255 -> 0
    return f;
}

Frame Encoder::float_value(std::uint16_t id, float value, std::uint8_t service_code) {
    Frame f = header(id, DataType::Float, service_code, 4);
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    f.data[4] = static_cast<std::uint8_t>(bits >> 24);
    f.data[5] = static_cast<std::uint8_t>(bits >> 16);
    f.data[6] = static_cast<std::uint8_t>(bits >> 8);
    f.data[7] = static_cast<std::uint8_t>(bits);
    return f;
}

Frame Encoder::uchar_value(std::uint16_t id, std::uint8_t value, std::uint8_t service_code) {
    Frame f = header(id, DataType::UChar, service_code, 1);
    f.data[4] = value;
    return f;
}

Frame Encoder::achar4(std::uint16_t id, const std::string& text) {
    Frame f = header(id, DataType::AChar4, 0, 4);
    for (std::size_t i = 0; i < 4; ++i) f.data[4 + i] = i < text.size() ? static_cast<std::uint8_t>(text[i]) : 0;
    return f;
}

Frame Encoder::ids_response(std::uint8_t request_message_code, std::uint8_t hardware_revision,
                            std::uint8_t software_revision) const {
    Frame f;
    f.id = id::kIdsResponse;
    f.dlc = 8;
    f.data[0] = node_id_;
    f.data[1] = static_cast<std::uint8_t>(DataType::UChar4);
    f.data[2] = kServiceIds;
    f.data[3] = request_message_code;
    f.data[4] = hardware_revision;
    f.data[5] = software_revision;
    f.data[6] = 0;  // identifier distribution: CANaerospace standard
    f.data[7] = 0;  // header type: CANaerospace standard
    return f;
}

bool is_ids_request(const Message& message, std::uint8_t node_id) {
    return message.id == id::kIdsRequest && message.service_code == kServiceIds &&
           (message.node_id == 0 || message.node_id == node_id);
}

std::string to_candump(const Frame& frame, double timestamp_s, const std::string& interface) {
    char head[64];
    std::snprintf(head, sizeof(head), "(%.6f) %s %03X#", timestamp_s, interface.c_str(),
                  static_cast<unsigned>(frame.id));
    std::string line(head);
    for (std::uint8_t i = 0; i < frame.dlc && i < 8; ++i) {
        char byte[3];
        std::snprintf(byte, sizeof(byte), "%02X", frame.data[i]);
        line += byte;
    }
    return line;
}

}  // namespace bodennav::can
