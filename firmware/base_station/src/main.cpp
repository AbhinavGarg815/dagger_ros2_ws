// The RTK base station's Blue Pill (Step R1): it sets the F9P up (survey-in, or fixed on
// a site's coordinates), sends its RTCM3 corrections to the rover wrapped in MAVLink
// GPS_RTCM_DATA, says what it is doing (HEARTBEAT, NAMED_VALUE_*), and takes the
// rover's commands (COMMAND_INT 31010). The conversation: DaggerGCS
// docs/proposals/base-station.md. The rover side: dagger_bringup/base_station.
//
// The last command is kept in flash, so after a power cycle the base comes back as
// it was left (on its site). With no command ever: survey-in, 300 s / 2.0 m.
// LED (PC13): steady = fixed (sending corrections), slow blink = surveying,
// fast blink = no fix yet.
#include <Arduino.h>
#include <EEPROM.h>

#include "gnss.h"
#include "mavlink_lite.h"

static const uint32_t GNSS_BAUD = 115200;   // F9P UART1: set by us at power-up, whatever it was
static const uint32_t RADIO_BAUD = 57600;   // the RFD868's serial speed
static const uint8_t MODE_NO_FIX = 1, MODE_SURVEYING = 2, MODE_FIXED = 3;
static const uint8_t ACTION_SURVEY = 1, ACTION_FIXED = 2;
static const uint32_t SETTINGS_MAGIC = 0xBA5E0001;

HardwareSerial gnssPort(PA10, PA9);         // USART1: rx, tx
HardwareSerial radio(PA3, PA2);             // USART2: rx, tx

// What the base was last told, kept in flash.
struct Settings {
  uint32_t magic;
  uint8_t action;                           // ACTION_SURVEY or ACTION_FIXED
  int32_t lat, lon;                         // fixed: degrees x 1e7
  float height;                             // fixed: m above the ellipsoid
  float minS, accM;                         // survey: at least this long, until this good
};

static Settings cfg;
static gnss::State st;
static gnss::Splitter splitter;
static mav::Parser parser;
static uint8_t seq = 0, rtcmSeq = 0;

static void toGnss(const uint8_t *b, int n) { gnssPort.write(b, n); }
static void toRadio(const uint8_t *b, int n) { radio.write(b, n); }

static void sendMav(uint32_t id, const uint8_t *payload, uint8_t len) {
  mav::send(toRadio, seq, id, payload, len);
}

// The F9P's UART1 and receiver settings (RAM). Its speed is unknown (factory 38400, or
// whatever was saved), so the settings go out at each usual speed; at a wrong one the F9P
// sees only bytes with bad checksums and ignores them. The last setting moves it to
// GNSS_BAUD, and we follow.
static void configurePort() {
  static const uint32_t speeds[] = {38400, 9600, 57600, 115200, 230400, 460800};
  for (uint32_t s : speeds) {
    gnssPort.begin(s);
    delay(20);
    gnss::ValSet()
        .set(gnss::UART1_IN_UBX, 1).set(gnss::UART1_IN_NMEA, 0).set(gnss::UART1_IN_RTCM3, 0)
        .set(gnss::UART1_OUT_UBX, 1).set(gnss::UART1_OUT_NMEA, 0).set(gnss::UART1_OUT_RTCM3, 1)
        .set(gnss::NAVSPG_DYNMODEL, 2).set(gnss::RATE_MEAS, 1000)
        .set(gnss::UART1_BAUDRATE, (int32_t)GNSS_BAUD)
        .send(toGnss);
    gnssPort.flush();
    delay(60);
  }
  gnssPort.begin(GNSS_BAUD);
  delay(50);
}

// The F9P's outputs on UART1 (RAM): corrections for the rover, and what we read here.
static void configureOutputs() {
  gnss::ValSet()
      .set(gnss::OUT_RTCM_1005, 5).set(gnss::OUT_RTCM_1074, 1).set(gnss::OUT_RTCM_1084, 1)
      .set(gnss::OUT_RTCM_1094, 1).set(gnss::OUT_RTCM_1124, 1).set(gnss::OUT_RTCM_1230, 5)
      .set(gnss::OUT_UBX_NAV_PVT, 1).set(gnss::OUT_UBX_NAV_SVIN, 1)
      .send(toGnss);
}

// Applies `cfg` to the F9P (RAM). A survey restarts: off first, then survey-in.
static void apply() {
  if (cfg.action == ACTION_FIXED) {
    double hCm = cfg.height * 100.0;
    int32_t cm = (int32_t)floor(hCm);
    int32_t hp = (int32_t)lround((hCm - cm) * 100.0);          // 0.1 mm
    if (hp > 99) { cm += 1; hp -= 100; }
    gnss::ValSet()
        .set(gnss::TMODE_MODE, 2).set(gnss::TMODE_POS_TYPE, 1)
        .set(gnss::TMODE_LAT, cfg.lat).set(gnss::TMODE_LAT_HP, 0)
        .set(gnss::TMODE_LON, cfg.lon).set(gnss::TMODE_LON_HP, 0)
        .set(gnss::TMODE_HEIGHT, cm).set(gnss::TMODE_HEIGHT_HP, hp)
        .set(gnss::TMODE_FIXED_POS_ACC, 100)                   // 1 cm (0.1 mm units)
        .send(toGnss);
  } else {
    gnss::ValSet().set(gnss::TMODE_MODE, 0).send(toGnss);
    delay(100);
    gnss::ValSet()
        .set(gnss::TMODE_MODE, 1)
        .set(gnss::TMODE_SVIN_MIN_DUR, (int32_t)cfg.minS)
        .set(gnss::TMODE_SVIN_ACC_LIMIT, (int32_t)(cfg.accM * 10000.0f))   // m -> 0.1 mm
        .send(toGnss);
  }
}

static bool gnssHeard(uint32_t now) { return st.heardMs != 0 && now - st.heardMs < 5000; }

// The F9P keeps our settings only in RAM: when it restarts on its own (power, a loose
// plug), it falls silent at our speed. Set it up again, every 5 s until it answers.
static void reconfigureIfSilent(uint32_t now) {
  static uint32_t lastTry = 0;
  if (gnssHeard(now) || now - lastTry < 5000) return;
  lastTry = now;
  configurePort();
  configureOutputs();
  delay(100);
  apply();
}

static uint8_t mode() {
  if (!gnssHeard(millis())) return MODE_NO_FIX;              // not heard: it does nothing for us
  if (cfg.action == ACTION_FIXED) return MODE_FIXED;
  if (st.svinValid && !st.svinActive) return MODE_FIXED;      // survey done: fixed on its result
  if (st.svinActive) return MODE_SURVEYING;
  return MODE_NO_FIX;                                         // waiting for a fix to start
}

static void sendInt(const char *name, int32_t v) {
  uint8_t p[18] = {0};
  mav::put32(p, 0, millis());
  memcpy(p + 4, &v, 4);
  strncpy((char *)p + 8, name, 10);
  sendMav(mav::NAMED_VALUE_INT, p, 18);
}

static void sendFloat(const char *name, float v) {
  uint8_t p[18] = {0};
  mav::put32(p, 0, millis());
  mav::putF(p, 4, v);
  strncpy((char *)p + 8, name, 10);
  sendMav(mav::NAMED_VALUE_FLOAT, p, 18);
}

// Once a second: who we are and what we do.
static void report() {
  uint8_t m = mode();
  uint8_t hb[9] = {0};
  mav::put32(hb, 0, m);                                       // custom_mode = our mode
  hb[4] = 0;                                                  // MAV_TYPE_GENERIC
  hb[5] = 8;                                                  // MAV_AUTOPILOT_INVALID
  hb[7] = 4;                                                  // MAV_STATE_ACTIVE
  hb[8] = 3;
  sendMav(mav::HEARTBEAT, hb, 9);
  if (m == MODE_SURVEYING) {
    sendFloat("BS_ACC", st.svinAccM);
    sendInt("BS_DUR", (int32_t)st.svinDur);
    if (st.svinObs > 0) {                                     // the mean so far: "save now" fixes on it
      sendInt("BS_MLAT", (int32_t)lround(st.svinLat * 1e7));
      sendInt("BS_MLON", (int32_t)lround(st.svinLon * 1e7));
      sendInt("BS_MHGT", (int32_t)lround(st.svinH * 1000.0));
    }
  }
  bool done = cfg.action == ACTION_SURVEY && m == MODE_FIXED;
  sendInt("BS_DONE", done ? 1 : 0);
  sendInt("BS_SATS", gnssHeard(millis()) ? st.numSV : -1);    // -1: the F9P is not heard
  if (m == MODE_FIXED) {
    if (done) {
      sendInt("BS_LAT", (int32_t)lround(st.svinLat * 1e7));
      sendInt("BS_LON", (int32_t)lround(st.svinLon * 1e7));
      sendInt("BS_HGT", (int32_t)lround(st.svinH * 1000.0));
    } else {
      sendInt("BS_LAT", cfg.lat);
      sendInt("BS_LON", cfg.lon);
      sendInt("BS_HGT", (int32_t)lround(cfg.height * 1000.0));
    }
  }
}

// An RTCM3 frame from the F9P: on to the rover in GPS_RTCM_DATA (too big to carry: dropped).
static void onRtcm(const uint8_t *frame, int n) {
  mav::sendRtcm(toRadio, seq, rtcmSeq, frame, n);
}

static void ack(const mav::Command &c, uint8_t result) {
  uint8_t p[10] = {0};
  mav::put16(p, 0, mav::CMD_BASE);
  p[2] = result;
  p[8] = c.fromSys;
  p[9] = c.fromComp;
  sendMav(mav::COMMAND_ACK, p, 10);
}

static void onCommand(const mav::Command &c) {
  Settings next = cfg;
  int action = (int)lroundf(c.p1);
  if (action == ACTION_SURVEY) {
    next.action = ACTION_SURVEY;
    next.minS = c.p2 > 0 ? c.p2 : 300;
    next.accM = c.p3 > 0 ? c.p3 : 2.0f;
  } else if (action == ACTION_FIXED) {
    next.action = ACTION_FIXED;
    next.lat = c.x;
    next.lon = c.y;
    next.height = c.z;
  } else {
    ack(c, 2);                                                // MAV_RESULT_DENIED
    return;
  }
  // A resend (its first ACK was lost) changes nothing; it carries the same request number.
  // A new request with the same settings does: Survey again starts the survey over (the
  // base was moved). Without a number (an older rover), the same settings mean a resend.
  static uint32_t lastRequest = 0;
  uint32_t request = c.p4 > 0 ? (uint32_t)lroundf(c.p4) : 0;
  bool same = next.action == cfg.action && (next.action == ACTION_SURVEY
      ? (next.minS == cfg.minS && next.accM == cfg.accM)
      : (next.lat == cfg.lat && next.lon == cfg.lon && next.height == cfg.height));
  bool resend = request ? request == lastRequest : same;
  if (!resend) {
    if (!same) {
      cfg = next;
      EEPROM.put(0, cfg);                                     // flash: only when it changes
    }
    apply();
  }
  if (request) lastRequest = request;
  ack(c, 0);                                                  // MAV_RESULT_ACCEPTED
}

void setup() {
  pinMode(PC13, OUTPUT);
  radio.begin(RADIO_BAUD);
  EEPROM.get(0, cfg);
  if (cfg.magic != SETTINGS_MAGIC) {
    cfg = Settings{SETTINGS_MAGIC, ACTION_SURVEY, 0, 0, 0.0f, 300.0f, 2.0f};
  }
  delay(1500);                                                // let the F9P boot
  configurePort();
  configureOutputs();
  delay(100);
  apply();
}

void loop() {
  uint32_t now = millis();
  while (gnssPort.available()) splitter.feed((uint8_t)gnssPort.read(), st, now, onRtcm);
  mav::Command c;
  while (radio.available()) {
    if (parser.feed((uint8_t)radio.read(), c)) onCommand(c);
  }
  if (now > 8000) reconfigureIfSilent(now);                  // after the first set-up has had its chance
  static uint32_t lastReport = 0;
  if (now - lastReport >= 1000) {
    lastReport = now;
    report();
  }
  uint8_t m = mode();
  bool on = m == MODE_FIXED || (m == MODE_SURVEYING ? (now / 500) % 2 : (now / 125) % 2);
  digitalWrite(PC13, on ? LOW : HIGH);                        // PC13 LED is on when low
}
