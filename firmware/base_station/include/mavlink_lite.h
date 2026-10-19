// The few MAVLink 2 messages the base needs, packed by hand (no generated library).
// Same layouts and checksums as the rover's dagger_bringup/base_link.py, which is
// checked byte for byte against pymavlink (tools/test_base_link.py).
#pragma once
#include <stdint.h>
#include <string.h>

namespace mav {

const uint8_t BASE_SYSID = 71, BASE_COMPID = 220;
const uint16_t CMD_BASE = 31010;                       // MAV_CMD_USER_1

// msgid, CRC_EXTRA, full payload length
const uint32_t HEARTBEAT = 0, COMMAND_INT = 75, COMMAND_ACK = 77, GPS_RTCM_DATA = 233,
               NAMED_VALUE_FLOAT = 251, NAMED_VALUE_INT = 252;

inline uint8_t crcExtra(uint32_t id) {
  switch (id) {
    case HEARTBEAT: return 50;
    case COMMAND_INT: return 158;
    case COMMAND_ACK: return 143;
    case GPS_RTCM_DATA: return 35;
    case NAMED_VALUE_FLOAT: return 170;
    case NAMED_VALUE_INT: return 44;
  }
  return 0;
}

inline void crcAdd(uint16_t &crc, uint8_t b) {
  uint8_t t = b ^ (uint8_t)(crc & 0xFF);
  t ^= (uint8_t)(t << 4);
  crc = (crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ (t >> 4);
}

// Little-endian packing into a payload buffer.
inline void put8(uint8_t *p, int at, uint8_t v) { p[at] = v; }
inline void put16(uint8_t *p, int at, uint16_t v) { memcpy(p + at, &v, 2); }
inline void put32(uint8_t *p, int at, uint32_t v) { memcpy(p + at, &v, 4); }
inline void putF(uint8_t *p, int at, float v) { memcpy(p + at, &v, 4); }

// Writes one frame through `out(bytes, n)`. Trailing zero bytes of the payload are
// trimmed, as MAVLink 2 does; the checksum covers what is sent.
template <typename Out>
void send(Out out, uint8_t &seq, uint32_t msgid, const uint8_t *payload, uint8_t len) {
  while (len > 1 && payload[len - 1] == 0) len--;
  uint8_t h[10] = {0xFD, len, 0, 0, seq++, BASE_SYSID, BASE_COMPID,
                   (uint8_t)(msgid & 0xFF), (uint8_t)((msgid >> 8) & 0xFF), (uint8_t)((msgid >> 16) & 0xFF)};
  uint16_t crc = 0xFFFF;
  for (int i = 1; i < 10; i++) crcAdd(crc, h[i]);
  for (int i = 0; i < len; i++) crcAdd(crc, payload[i]);
  crcAdd(crc, crcExtra(msgid));
  uint8_t c[2] = {(uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8)};
  out(h, 10);
  out(payload, len);
  out(c, 2);
}

// One RTCM3 message in GPS_RTCM_DATA: whole if it fits in 180 bytes, else in up to 4
// pieces sharing a 5-bit sequence number. ArduPilot (AP_GPS::handle_gps_rtcm_fragment)
// knows a split message is complete when a piece is shorter than 180 bytes, or when there
// are 4 full ones: a message filling 2 or 3 pieces exactly gets an empty last piece, or
// the Cube would never pass it on. Longer than 4 x 180: not carried (false).
template <typename Out>
bool sendRtcm(Out out, uint8_t &seq, uint8_t &rtcmSeq, const uint8_t *msg, int n) {
  const int piece = 180;
  if (n > 4 * piece) return false;
  int pieces = (n + piece - 1) / piece;
  if (n > piece && n % piece == 0 && pieces < 4) pieces++;
  uint8_t s = rtcmSeq++ & 0x1F;
  for (int i = 0; i < pieces; i++) {
    int len = n - i * piece < piece ? n - i * piece : piece;
    uint8_t p[2 + piece] = {0};
    p[0] = (uint8_t)((pieces > 1 ? 1 : 0) | (i << 1) | (s << 3));
    p[1] = (uint8_t)len;
    memcpy(p + 2, msg + i * piece, len);
    send(out, seq, GPS_RTCM_DATA, p, 2 + piece);
  }
  return true;
}

// A COMMAND_INT 31010 addressed to the base, as received.
struct Command {
  float p1, p2, p3;
  int32_t x, y;
  float z;
  uint8_t fromSys, fromComp;
};

// Reads MAVLink 2 frames one byte at a time; keeps only COMMAND_INT 31010 for the
// base with a good checksum. Everything else the Cube forwards (its telemetry) is
// passed over.
class Parser {
 public:
  // True when `b` completed a command for us; it is then in `cmd`.
  bool feed(uint8_t b, Command &cmd) {
    switch (state_) {
      case 0:
        if (b == 0xFD) { state_ = 1; n_ = 0; }
        return false;
      case 1:                                          // header after the magic byte
        h_[n_++] = b;
        if (n_ == 9) { len_ = h_[0]; signed_ = h_[1] & 0x01; n_ = 0; state_ = len_ ? 2 : 3; }
        return false;
      case 2:                                          // payload
        p_[n_++] = b;
        if (n_ == len_) { n_ = 0; state_ = 3; }
        return false;
      case 3:                                          // checksum
        c_[n_++] = b;
        if (n_ < 2) return false;
        if (signed_) { state_ = 4; n_ = 0; return false; }
        state_ = 0;
        return finish(cmd);
      case 4:                                          // a signature: not checked, skipped
        if (++n_ == 13) { state_ = 0; }
        return false;
    }
    state_ = 0;
    return false;
  }

 private:
  bool finish(Command &cmd) {
    uint32_t id = h_[6] | ((uint32_t)h_[7] << 8) | ((uint32_t)h_[8] << 16);
    if (id != COMMAND_INT) return false;
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < 9; i++) crcAdd(crc, h_[i]);
    for (int i = 0; i < len_; i++) crcAdd(crc, p_[i]);
    crcAdd(crc, crcExtra(id));
    if (c_[0] != (crc & 0xFF) || c_[1] != (crc >> 8)) return false;
    uint8_t q[35] = {0};                               // trimmed payload: the rest is zero
    memcpy(q, p_, len_ < 35 ? len_ : 35);
    uint16_t command;
    memcpy(&command, q + 28, 2);
    if (command != CMD_BASE || q[30] != BASE_SYSID || (q[31] != BASE_COMPID && q[31] != 0)) return false;
    memcpy(&cmd.p1, q + 0, 4);
    memcpy(&cmd.p2, q + 4, 4);
    memcpy(&cmd.p3, q + 8, 4);
    memcpy(&cmd.x, q + 16, 4);
    memcpy(&cmd.y, q + 20, 4);
    memcpy(&cmd.z, q + 24, 4);
    cmd.fromSys = h_[4];
    cmd.fromComp = h_[5];
    return true;
  }

  uint8_t state_ = 0, n_ = 0, len_ = 0;
  bool signed_ = false;
  uint8_t h_[9], p_[255], c_[2];
};

}  // namespace mav
