#pragma once

// CANaerospace V1.7 frames as used by OpenEFIS (docs/adr/0002-canaerospace.md
// and 0007-nav-radio.md in dansparker/open-efis): CAN 2.0A, 500 kbit/s,
// 4 header bytes (node ID, data type, service code, message code) and up to
// 4 data bytes, big endian.

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace bodennav::can {

struct Frame {
    std::uint32_t id{0};  // 11-bit identifier
    std::uint8_t dlc{0};
    std::array<std::uint8_t, 8> data{};
};

enum class DataType : std::uint8_t {
    NoData = 0,
    Error = 1,
    Float = 2,
    Short = 6,
    UShort = 7,
    UChar = 10,
    BChar = 11,
    UChar4 = 16,
    AChar4 = 25,
};

namespace id {
inline constexpr std::uint16_t kIdsRequest = 128;     // node service channel 0
inline constexpr std::uint16_t kIdsResponse = 129;
inline constexpr std::uint16_t kAdf1Bearing = 1083;   // FLOAT deg, relative to the nose, clockwise
inline constexpr std::uint16_t kIls1Localizer = 1087; // FLOAT deg, + aircraft right of the course
inline constexpr std::uint16_t kIls1Glideslope = 1091;// FLOAT deg, + aircraft above the glide path
inline constexpr std::uint16_t kNav1Frequency = 1104; // FLOAT MHz
inline constexpr std::uint16_t kAdf1Frequency = 1108; // FLOAT kHz
inline constexpr std::uint16_t kVor1Radial = 1126;    // FLOAT deg magnetic, FROM the station
// user range 1500-1799, fixed by OpenEFIS ADR 0007
inline constexpr std::uint16_t kMarkerBeacons = 1501; // UCHAR bits 0 outer, 1 middle, 2 inner
inline constexpr std::uint16_t kNav1Ident = 1502;     // ACHAR4
inline constexpr std::uint16_t kAdf1Ident = 1503;     // ACHAR4
// proposed by bodennav (not yet in OpenEFIS): tuning from the display
inline constexpr std::uint16_t kNav1TuneRequest = 1530;  // FLOAT MHz, sent by the display node
inline constexpr std::uint16_t kAdf1TuneRequest = 1531;  // FLOAT kHz, sent by the display node
}  // namespace id

inline constexpr std::uint8_t kServiceIds = 0;

struct Message {
    std::uint16_t id{};
    std::uint8_t node_id{};
    std::uint8_t data_type{};
    std::uint8_t service_code{};
    std::uint8_t message_code{};
    std::array<std::uint8_t, 4> data{};
    std::uint8_t data_length{};

    [[nodiscard]] std::optional<float> as_float() const;
    [[nodiscard]] std::string as_achar() const;
};

// nullopt for identifiers above 0x7FF and frames shorter than the header.
[[nodiscard]] std::optional<Message> decode(const Frame& frame);

// Builds the frames of one node; the message code counts per identifier.
class Encoder {
public:
    explicit Encoder(std::uint8_t node_id) : node_id_(node_id) {}
    [[nodiscard]] Frame float_value(std::uint16_t id, float value, std::uint8_t service_code = 0);
    [[nodiscard]] Frame uchar_value(std::uint16_t id, std::uint8_t value, std::uint8_t service_code = 0);
    // Up to 4 ASCII characters, padded with 0.
    [[nodiscard]] Frame achar4(std::uint16_t id, const std::string& text);
    // Answer to an identification request (hardware and software revision).
    [[nodiscard]] Frame ids_response(std::uint8_t request_message_code, std::uint8_t hardware_revision,
                                     std::uint8_t software_revision) const;
    [[nodiscard]] std::uint8_t node_id() const { return node_id_; }

private:
    [[nodiscard]] Frame header(std::uint16_t id, DataType type, std::uint8_t service_code, std::uint8_t length);
    std::uint8_t node_id_;
    std::map<std::uint16_t, std::uint8_t> codes_;
};

// Is `message` an identification request for `node_id` (addressed or broadcast)?
[[nodiscard]] bool is_ids_request(const Message& message, std::uint8_t node_id);

// candump -L line, e.g. "(1700000000.000000) can0 466#1E02000143A00000".
[[nodiscard]] std::string to_candump(const Frame& frame, double timestamp_s, const std::string& interface);

}  // namespace bodennav::can
