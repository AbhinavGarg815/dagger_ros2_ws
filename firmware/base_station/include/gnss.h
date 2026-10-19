// The F9P side: UBX configuration (CFG-VALSET), and splitting its UART stream into
// UBX frames (read here) and RTCM3 frames (sent on to the rover). Keys and layouts:
// u-blox ZED-F9P Interface Description (HPG 1.32).
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

namespace gnss {

// CFG keys (the size is in bits 28-30 of the key: 1 = bit, 2 = 1 byte, 3 = 2, 4 = 4, 5 = 8)
const uint32_t TMODE_MODE = 0x20030001, TMODE_POS_TYPE = 0x20030002, TMODE_LAT = 0x40030009,
               TMODE_LON = 0x4003000A, TMODE_HEIGHT = 0x4003000B, TMODE_LAT_HP = 0x2003000C,
               TMODE_LON_HP = 0x2003000D, TMODE_HEIGHT_HP = 0x2003000E, TMODE_FIXED_POS_ACC = 0x4003000F,
               TMODE_SVIN_MIN_DUR = 0x40030010, TMODE_SVIN_ACC_LIMIT = 0x40030011;
// The port and the receiver itself, so nothing has to be saved in the F9P (owner, 2026-10-16:
// a u-center save did not stick): sent in RAM at every power-up.
const uint32_t UART1_BAUDRATE = 0x40520001, UART1_IN_UBX = 0x10730001, UART1_IN_NMEA = 0x10730002,
               UART1_IN_RTCM3 = 0x10730004, UART1_OUT_UBX = 0x10740001, UART1_OUT_NMEA = 0x10740002,
               UART1_OUT_RTCM3 = 0x10740004, NAVSPG_DYNMODEL = 0x20110021, RATE_MEAS = 0x30210001;
const uint32_t OUT_RTCM_1005 = 0x209102BE, OUT_RTCM_1074 = 0x2091035F, OUT_RTCM_1084 = 0x20910364,
               OUT_RTCM_1094 = 0x20910369, OUT_RTCM_1124 = 0x2091036E, OUT_RTCM_1230 = 0x20910304,
               OUT_UBX_NAV_PVT = 0x20910007, OUT_UBX_NAV_SVIN = 0x20910089;

// Builds one UBX-CFG-VALSET (RAM layer) with up to 16 key/values, then writes it.
class ValSet {
 public:
  ValSet() { memset(buf_, 0, sizeof buf_); n_ = 4; buf_[1] = 0x01; }  // version 0, layer RAM
  ValSet &set(uint32_t key, int32_t value) {
    int size = 1;
    switch ((key >> 28) & 0x7) { case 3: size = 2; break; case 4: size = 4; break; default: size = 1; }
    memcpy(buf_ + n_, &key, 4);
    memcpy(buf_ + n_ + 4, &value, size);
    n_ += 4 + size;
    return *this;
  }
  template <typename Out>
  void send(Out out) const {
    uint8_t h[6] = {0xB5, 0x62, 0x06, 0x8A, (uint8_t)(n_ & 0xFF), (uint8_t)(n_ >> 8)};
    uint8_t a = 0, b = 0;
    for (int i = 2; i < 6; i++) { a += h[i]; b += a; }
    for (int i = 0; i < n_; i++) { a += buf_[i]; b += a; }
    uint8_t c[2] = {a, b};
    out(h, 6);
    out(buf_, n_);
    out(c, 2);
  }

 private:
  uint8_t buf_[4 + 16 * 8];
  int n_;
};

// What the F9P last said about itself.
struct State {
  bool svinActive = false, svinValid = false;
  uint32_t svinDur = 0;
  float svinAccM = 0;
  double svinLat = 0, svinLon = 0, svinH = 0;   // the survey's result, when valid
  uint8_t fixType = 0;                           // NAV-PVT: 3 = 3D, 5 = time only (a base)
  uint8_t numSV = 0;                             // NAV-PVT: satellites used in the solution
  uint32_t heardMs = 0;
};

const double kPi = 3.14159265358979323846;   // not PI: Arduino defines that

// ECEF (m) -> WGS84 latitude, longitude (deg), height above the ellipsoid (m).
inline void ecefToLlh(double x, double y, double z, double &lat, double &lon, double &h) {
  const double a = 6378137.0, e2 = 6.69437999014e-3, b = a * sqrt(1 - e2), ep2 = (a * a - b * b) / (b * b);
  double p = sqrt(x * x + y * y), th = atan2(z * a, p * b);
  lon = atan2(y, x);
  double phi = atan2(z + ep2 * b * pow(sin(th), 3), p - e2 * a * pow(cos(th), 3));
  for (int i = 0; i < 3; i++) {                  // a few rounds make it exact to well under a mm
    double n = a / sqrt(1 - e2 * sin(phi) * sin(phi));
    h = p / cos(phi) - n;
    phi = atan2(z, p * (1 - e2 * n / (n + h)));
  }
  double n = a / sqrt(1 - e2 * sin(phi) * sin(phi));
  h = p / cos(phi) - n;
  lat = phi * 180.0 / kPi;
  lon = lon * 180.0 / kPi;
}

// One byte at a time: UBX frames are read into `st`; whole RTCM3 frames (checked
// with their CRC-24Q) are handed to `rtcm(frame, n)`.
class Splitter {
 public:
  template <typename Rtcm>
  void feed(uint8_t c, State &st, uint32_t nowMs, Rtcm rtcm) {
    switch (mode_) {
      case 0:
        if (c == 0xB5) { mode_ = 1; }
        else if (c == 0xD3) { mode_ = 10; buf_[0] = c; n_ = 1; }
        return;
      case 1:
        if (c == 0x62) { mode_ = 2; n_ = 0; return; }
        mode_ = 0;                                 // not UBX after all: this byte may start a frame
        feed(c, st, nowMs, rtcm);
        return;
      case 2:                                      // class, id, length (4 bytes)
        buf_[n_++] = c;
        if (n_ == 4) {
          len_ = buf_[2] | (buf_[3] << 8);
          if (len_ > sizeof(buf_) - 6) { mode_ = 0; return; }
          mode_ = 3;
        }
        return;
      case 3:                                      // payload, then 2 checksum bytes
        buf_[n_++] = c;
        if (n_ == 4 + len_ + 2) { mode_ = 0; ubx(st, nowMs); }
        return;
      case 10:                                     // RTCM3: 2 more header bytes
        buf_[n_++] = c;
        if (n_ == 3) {
          len_ = ((buf_[1] & 0x03) << 8) | buf_[2];
          if (len_ + 6u > sizeof(buf_)) { mode_ = 0; return; }
          mode_ = 11;
        }
        return;
      case 11:
        buf_[n_++] = c;
        if (n_ == len_ + 6) {
          mode_ = 0;
          if (crc24q(buf_, len_ + 3) == ((uint32_t)buf_[len_ + 3] << 16 | (uint32_t)buf_[len_ + 4] << 8 | buf_[len_ + 5]))
            rtcm(buf_, n_);
        }
        return;
    }
  }

 private:
  void ubx(State &st, uint32_t nowMs) {
    uint8_t a = 0, b = 0;
    for (int i = 0; i < 4 + len_; i++) { a += buf_[i]; b += a; }
    if (a != buf_[4 + len_] || b != buf_[5 + len_]) return;
    const uint8_t *p = buf_ + 4;
    st.heardMs = nowMs;
    if (buf_[0] == 0x01 && buf_[1] == 0x3B && len_ >= 40) {          // NAV-SVIN
      uint32_t dur, acc;
      int32_t x, y, z;
      memcpy(&dur, p + 8, 4);
      memcpy(&x, p + 12, 4);
      memcpy(&y, p + 16, 4);
      memcpy(&z, p + 20, 4);
      memcpy(&acc, p + 28, 4);
      st.svinDur = dur;
      st.svinAccM = acc / 10000.0f;                                  // 0.1 mm -> m
      st.svinValid = p[36] != 0;
      st.svinActive = p[37] != 0;
      if (st.svinValid) {
        double X = x / 100.0 + (int8_t)p[24] / 10000.0, Y = y / 100.0 + (int8_t)p[25] / 10000.0,
               Z = z / 100.0 + (int8_t)p[26] / 10000.0;               // cm + 0.1 mm parts
        ecefToLlh(X, Y, Z, st.svinLat, st.svinLon, st.svinH);
      }
    } else if (buf_[0] == 0x01 && buf_[1] == 0x07 && len_ >= 92) {   // NAV-PVT
      st.fixType = p[20];
      st.numSV = p[23];
    }
  }

  static uint32_t crc24q(const uint8_t *d, int n) {
    uint32_t crc = 0;
    for (int i = 0; i < n; i++) {
      crc ^= (uint32_t)d[i] << 16;
      for (int k = 0; k < 8; k++) {
        crc <<= 1;
        if (crc & 0x1000000) crc ^= 0x1864CFB;
      }
    }
    return crc & 0xFFFFFF;
  }

  uint8_t mode_ = 0;
  uint16_t n_ = 0, len_ = 0;
  uint8_t buf_[1100];          // RTCM3 frames are at most 1029 bytes
};

}  // namespace gnss
