#pragma once

#include <cmath>
#include <cstddef>
#include <cstdio>

namespace netra_rid_usb {

inline bool usable_position(double lat, double lon) {
  return std::isfinite(lat) && std::isfinite(lon) && !(lat == 0.0 && lon == 0.0);
}

inline bool should_emit(bool fresh_location, double lat, double lon) {
  return fresh_location && usable_position(lat, lon);
}

inline int format_detection(char *out, std::size_t size, const char *id,
                            double lat, double lon, float altitude_hae_m,
                            double pilot_lat, double pilot_lon,
                            const char *mac, int rssi) {
  char altitude[24];
  if (!std::isfinite(altitude_hae_m) || altitude_hae_m == -1000.0f) {
    std::snprintf(altitude, sizeof(altitude), "null");
  } else {
    std::snprintf(altitude, sizeof(altitude), "%.1f", altitude_hae_m);
  }
  return std::snprintf(
      out, size,
      R"({"type":"detection","id":"%s","lat":%.6f,"lon":%.6f,"alt_hae_m":%s,"pilot_lat":%.6f,"pilot_lon":%.6f,"mac":"%s","rssi":%d})",
      id, lat, lon, altitude, pilot_lat, pilot_lon, mac, rssi);
}

}  // namespace netra_rid_usb
