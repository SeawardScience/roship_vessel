#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <cstdint>
#include <cmath>

namespace roship_vessel::nmea {

/// Find the first '$' in raw and return the NMEA sentence from there (trailing whitespace stripped).
inline bool findSentence(const std::string& raw, std::string& out)
{
    auto pos = raw.find('$');
    if (pos == std::string::npos) return false;
    out = raw.substr(pos);
    while (!out.empty() && (out.back() == '\r' || out.back() == '\n' || out.back() == ' '))
        out.pop_back();
    return !out.empty();
}

/// XOR checksum of bytes between '$' and '*'.  Returns true if the '*hh' suffix matches.
inline bool validateChecksum(const std::string& sentence)
{
    auto star = sentence.rfind('*');
    if (star == std::string::npos || star + 3 > sentence.size()) return false;
    uint8_t computed = 0;
    for (size_t i = 1; i < star; ++i)
        computed ^= static_cast<uint8_t>(sentence[i]);
    try {
        uint8_t expected = static_cast<uint8_t>(std::stoul(sentence.substr(star + 1, 2), nullptr, 16));
        return computed == expected;
    } catch (...) { return false; }
}

/// Split a NMEA sentence by commas, stripping the '*checksum' suffix from the last field.
inline std::vector<std::string> split(const std::string& sentence)
{
    auto star = sentence.rfind('*');
    std::string body = (star != std::string::npos) ? sentence.substr(0, star) : sentence;
    std::vector<std::string> fields;
    std::istringstream ss(body);
    std::string tok;
    while (std::getline(ss, tok, ','))
        fields.push_back(tok);
    return fields;
}

/// Convert NMEA DDMM.MMMM (or DDDMM.MMMM) to decimal degrees.
/// Pass the hemisphere character ('N','S','E','W') for sign.
inline double coordToDecimal(const std::string& nmea, char hemi)
{
    if (nmea.empty()) return 0.0;
    double raw = std::stod(nmea);
    double degrees = std::floor(raw / 100.0);
    double minutes = raw - degrees * 100.0;
    double decimal = degrees + minutes / 60.0;
    return (hemi == 'S' || hemi == 'W') ? -decimal : decimal;
}

/// Map GGA fix-quality integer to sensor_msgs NavSatStatus value (-1/0/1/2).
inline int8_t fixQualityToStatus(int q)
{
    switch (q) {
        case 1: return 0;   // STATUS_FIX
        case 2: return 1;   // STATUS_SBAS_FIX (DGPS)
        case 4: return 2;   // STATUS_GBAS_FIX (RTK fixed)
        case 5: return 2;   // STATUS_GBAS_FIX (RTK float)
        default: return -1; // STATUS_NO_FIX
    }
}

}  // namespace roship_vessel::nmea
