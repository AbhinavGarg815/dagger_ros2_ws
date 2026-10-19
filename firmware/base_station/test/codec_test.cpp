// The firmware's MAVLink, UBX and RTCM handling, run on a PC (no Blue Pill):
//
//   g++ -std=c++17 -Wall -I include test/codec_test.cpp -o codec_test && ./codec_test
//
// The expected frames were made by pymavlink (MAVLink), by the rover's own encoder
// (dagger_bringup/base_link.py, itself checked against pymavlink), and by a Python
// UBX/RTCM writer -- so the Blue Pill and the rover agree byte for byte.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "gnss.h"
#include "mavlink_lite.h"

static int failures = 0;
static std::vector<uint8_t> sent;
static void collect(const uint8_t *b, int n) { sent.insert(sent.end(), b, b + n); }

static std::string hex(const std::vector<uint8_t> &v) {
  std::string s;
  char t[3];
  for (uint8_t b : v) { snprintf(t, sizeof t, "%02x", b); s += t; }
  return s;
}
static std::vector<uint8_t> unhex(const char *h) {
  std::vector<uint8_t> v;
  for (; h[0] && h[1]; h += 2) { unsigned x; sscanf(h, "%2x", &x); v.push_back((uint8_t)x); }
  return v;
}
static void check(bool ok, const char *what) {
  printf("%s  %s\n", ok ? "ok  " : "FAIL", what);
  if (!ok) failures++;
}
static void frame(uint32_t id, const uint8_t *p, uint8_t len, const char *want, const char *what) {
  sent.clear();
  uint8_t seq = 0;
  mav::send(collect, seq, id, p, len);
  bool ok = hex(sent) == want;
  if (!ok) printf("      got  %s\n      want %s\n", hex(sent).c_str(), want);
  check(ok, what);
}

int main() {
  // --- MAVLink out: as pymavlink packs them (system 71, component 220, seq 0) ---
  uint8_t hb[9] = {3, 0, 0, 0, 0, 8, 0, 4, 3};
  frame(mav::HEARTBEAT, hb, 9, "fd0900000047dc00000003000000000800040390be", "HEARTBEAT, mode fixed");
  uint8_t nvi[18] = {0};
  mav::put32(nvi, 0, 1000);
  int32_t lat = 267478123;
  memcpy(nvi + 4, &lat, 4);
  memcpy(nvi + 8, "BS_LAT", 6);
  frame(mav::NAMED_VALUE_INT, nvi, 18, "fd0e00000047dcfc0000e80300006b64f10f42535f4c4154280c", "NAMED_VALUE_INT BS_LAT");
  uint8_t nvf[18] = {0};
  mav::put32(nvf, 0, 1000);
  mav::putF(nvf, 4, 2.5f);
  memcpy(nvf + 8, "BS_ACC", 6);
  frame(mav::NAMED_VALUE_FLOAT, nvf, 18, "fd0e00000047dcfb0000e80300000000204042535f4143437595", "NAMED_VALUE_FLOAT BS_ACC");
  uint8_t ack[10] = {0};
  mav::put16(ack, 0, 31010);
  ack[8] = 1;
  ack[9] = 191;
  frame(mav::COMMAND_ACK, ack, 10, "fd0a00000047dc4d0000227900000000000001bf4da7", "COMMAND_ACK to the rover");
  uint8_t rtcm[182] = {0};
  rtcm[0] = 3 << 3;
  rtcm[1] = 4;
  rtcm[2] = 0xD3; rtcm[3] = 0x00; rtcm[4] = 0x01; rtcm[5] = 0xAA;
  frame(mav::GPS_RTCM_DATA, rtcm, 182, "fd0600000047dce900001804d30001aad7ed", "GPS_RTCM_DATA, one piece");

  // --- GPS_RTCM_DATA pieces, put back together as the Cube does ---
  // ArduPilot's AP_GPS::handle_gps_rtcm_fragment, line for line, fed with what we send.
  {
    std::vector<uint8_t> injected, want;
    uint8_t buf[720], apSeq = 0, received = 0, count = 0;
    uint16_t total = 0;
    int discarded = 0;
    auto fragment = [&](uint8_t flags, const uint8_t *data, uint8_t len) {
      if ((flags & 1) == 0) { injected.insert(injected.end(), data, data + len); return; }
      uint8_t frag = (flags >> 1) & 3, s = (flags >> 3) & 0x1F;
      if (received && (apSeq != s || (received & (1 << frag)))) {
        discarded += __builtin_popcount(received);
        count = 0;
        received = 0;
      }
      apSeq = s;
      received |= 1 << frag;
      memcpy(buf + 180 * frag, data, len);
      if (len < 180) { count = frag + 1; total = 180 * frag + len; }
      else if (received == 0x0F) { count = 4; total = 720; }
      if (count && received == (1 << count) - 1) {
        injected.insert(injected.end(), buf, buf + total);
        count = 0;
        received = 0;
      }
    };
    sent.clear();
    uint8_t seq = 0, rtcmSeq = 0;
    bool refused = false;
    for (int n : {6, 179, 180, 181, 359, 360, 361, 539, 540, 541, 719, 720, 721}) {
      std::vector<uint8_t> m(n);
      for (int i = 0; i < n; i++) m[i] = (uint8_t)(n + i * 7);
      if (mav::sendRtcm(collect, seq, rtcmSeq, m.data(), n)) want.insert(want.end(), m.begin(), m.end());
      else refused = n == 721;
    }
    for (size_t i = 0; i + 12 <= sent.size();) {           // our frames: unpacked, zeros put back
      uint8_t len = sent[i + 1];
      uint8_t p[182] = {0};
      memcpy(p, &sent[i + 10], len);
      fragment(p[0], p + 2, p[1]);
      i += 12 + len;
    }
    check(injected == want && discarded == 0 && received == 0,
          "RTCM of 6-720 bytes (360 and 540 too) whole through ArduPilot's reassembly");
    check(refused, "RTCM longer than 720 bytes not sent");
  }

  // --- MAVLink in: the rover's commands, as dagger_bringup/base_link sends them ---
  mav::Parser parser;
  mav::Command c{};
  bool got = false;
  for (uint8_t b : unhex("fd2000000901bf4b0000000000400000000000000000000000006b64f10f9c9d362d561ed143227947dc5879"))
    got |= parser.feed(b, c);
  check(got && lroundf(c.p1) == 2 && c.x == 267478123 && c.y == 758554012 && fabsf(c.z - 418.237f) < 1e-3f &&
            c.fromSys == 1 && c.fromComp == 191, "COMMAND_INT fixed on the Yard base, parsed");
  got = false;
  for (uint8_t b : unhex("fd2000000a01bf4b00000000803f000096430000004000000000000000000000000000000000227947dc6b69"))
    got |= parser.feed(b, c);
  check(got && lroundf(c.p1) == 1 && lroundf(c.p2) == 300 && fabsf(c.p3 - 2.0f) < 1e-6f, "COMMAND_INT survey, parsed");
  auto bad = unhex("fd2000000a01bf4b00000000803f000096430000004000000000000000000000000000000000227947dc6b6a");
  got = false;
  for (uint8_t b : bad) got |= parser.feed(b, c);
  check(!got, "a frame with a bad checksum is ignored");

  // --- UBX out: CFG-VALSET (RAM) ---
  sent.clear();
  gnss::ValSet().set(gnss::TMODE_MODE, 2).set(gnss::TMODE_LAT, 267478123).send(collect);
  check(hex(sent) == "b562068a1100000100000100032002090003406b64f10fe37c", "UBX-CFG-VALSET TMODE fixed + latitude");
  // As pyubx2 builds them: the port set-up sent at power-up, and a survey-in (300 s, 2.0 m).
  sent.clear();
  gnss::ValSet()
      .set(gnss::UART1_IN_UBX, 1).set(gnss::UART1_IN_NMEA, 0).set(gnss::UART1_IN_RTCM3, 0)
      .set(gnss::UART1_OUT_UBX, 1).set(gnss::UART1_OUT_NMEA, 0).set(gnss::UART1_OUT_RTCM3, 1)
      .set(gnss::NAVSPG_DYNMODEL, 2).set(gnss::RATE_MEAS, 1000).set(gnss::UART1_BAUDRATE, 115200)
      .send(collect);
  check(hex(sent) == "b562068a350000010000010073100102007310000400731000010074100102007410000400741001"
                     "210011200201002130e8030100524000c20100d33e", "UBX-CFG-VALSET port set-up, as pyubx2");
  sent.clear();
  gnss::ValSet().set(gnss::TMODE_MODE, 1).set(gnss::TMODE_SVIN_MIN_DUR, 300).set(gnss::TMODE_SVIN_ACC_LIMIT, 20000)
      .send(collect);
  check(hex(sent) == "b562068a1900000100000100032001100003402c01000011000340204e000011e9",
        "UBX-CFG-VALSET survey-in, as pyubx2");

  // --- RTCM3 in: a whole frame with a good CRC-24Q is passed on; a bad one is not ---
  gnss::Splitter split;
  gnss::State st;
  auto r3 = unhex("d300133e3f404142434445464748494a4b4c4d4e4f50a5666b");
  std::vector<uint8_t> out;
  int frames = 0;
  auto onRtcm = [&](const uint8_t *f, int n) { out.assign(f, f + n); frames++; };
  for (uint8_t b : unhex("0102b5")) split.feed(b, st, 0, onRtcm);     // noise before it
  for (uint8_t b : r3) split.feed(b, st, 0, onRtcm);
  check(frames == 1 && out == r3, "RTCM3 frame found in the stream and passed on whole");
  r3[10] ^= 0x01;
  for (uint8_t b : r3) split.feed(b, st, 0, onRtcm);
  check(frames == 1, "RTCM3 frame with a bad CRC is dropped");

  // --- the survey's result: ECEF -> latitude, longitude, height ---
  double la, lo, h;
  gnss::ecefToLlh(1392881.6234, 5527081.9566, 2853478.7681, la, lo, h);
  check(fabs(la - 26.7478123) < 1e-8 && fabs(lo - 75.8554012) < 1e-8 && fabs(h - 418.237) < 1e-3,
        "ECEF to latitude/longitude/height, within 1 mm");

  printf(failures ? "\n%d FAILED\n" : "\nall passed\n", failures);
  return failures ? 1 : 0;
}
