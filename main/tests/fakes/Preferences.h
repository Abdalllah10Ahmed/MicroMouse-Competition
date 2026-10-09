#pragma once

// Minimal in-memory stand-in for the ESP32 Preferences (NVS) API, used only by
// the host maze-map tests. Keys are ignored; a single blob is retained for the
// lifetime of the process so save() then load() round-trip correctly.
#include <cstddef>
#include <cstring>

class Preferences {
 public:
  bool begin(const char *, bool) { return true; }

  size_t putBytes(const char *, const void *data, size_t len) {
    if (len > sizeof(store_)) return 0;
    std::memcpy(store_, data, len);
    length_ = len;
    return len;
  }

  size_t getBytesLength(const char *) const { return length_; }

  size_t getBytes(const char *, void *out, size_t len) {
    if (len > length_) len = length_;
    std::memcpy(out, store_, len);
    return len;
  }

  void remove(const char *) { length_ = 0; }
  void end() {}

 private:
  // One saved maze is magic(4) + cells(16*16*4) = 1028 bytes; keep headroom.
  static inline char store_[2048] = {};
  static inline size_t length_ = 0;
};