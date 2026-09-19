#include <cassert>
#include <cstring>

#include "../shared/rid_usb_format.h"

int main() {
  char json[384];
  int length = netra_rid_usb::format_detection(
      json, sizeof(json), "RID-1", 32.877, -117.235, 112.5f,
      32.876, -117.236, "aa:bb:cc:dd:ee:ff", -62);
  assert(length > 0);
  assert(std::strstr(json, "\"alt_hae_m\":112.5") != nullptr);
  assert(std::strstr(json, "alt_msl") == nullptr);

  netra_rid_usb::format_detection(
      json, sizeof(json), "RID-1", 32.877, -117.235, 0.0f,
      32.876, -117.236, "aa:bb:cc:dd:ee:ff", -62);
  assert(std::strstr(json, "\"alt_hae_m\":0.0") != nullptr);

  netra_rid_usb::format_detection(
      json, sizeof(json), "RID-1", 32.877, -117.235, -1000.0f,
      32.876, -117.236, "aa:bb:cc:dd:ee:ff", -62);
  assert(std::strstr(json, "\"alt_hae_m\":null") != nullptr);

  assert(netra_rid_usb::usable_position(32.877, -117.235));
  assert(!netra_rid_usb::usable_position(0.0, 0.0));
  assert(netra_rid_usb::should_emit(true, 32.877, -117.235));
  assert(!netra_rid_usb::should_emit(false, 32.877, -117.235));
  assert(!netra_rid_usb::should_emit(true, 0.0, 0.0));
}
