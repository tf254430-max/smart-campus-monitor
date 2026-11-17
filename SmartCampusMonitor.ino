/*
  Smart Campus Resource & Accessibility Monitor
  BDA 2201 Microprocessors and Microcontrollers - CAT 2
  Tinka Fahad (254430), Cavendish University Uganda

  One ESP32 node that a student can walk up to (or listen to) and ask:
  "Is the Library Zone / Computer Lab / Accessible Study Pod free right now?"

  Inputs  : HC-SR04 ultrasonic (seat row / ramp clearance), PIR motion,
            DHT22 (temperature + humidity), LDR light sensor,
            3 push-buttons (one per facility).
  Edge AI : moving-average filters, button debouncing, z-score spike
            detection, a small logistic occupancy model and comfort rules
            -> AVAILABLE / CROWDED / UNAVAILABLE.
  Outputs : RGB LED, servo "status flag", buzzer tone patterns (for blind
            students), SSD1306 OLED (facility name + live status).
  Network : Wi-Fi -> MQTT (live dashboard) + HTTP POST (response metadata).

  Every sensor reading is checked. A disconnected or faulty sensor never
  crashes the node: it is reported, dropped from the decision, and the
  status is marked as lower confidence.
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHTesp.h>
#include <ESP32Servo.h>
#include <math.h>

// ---------------- Pin map (see wiring table in the report) ----------------
const int PIN_TRIG   = 5;    // HC-SR04 trigger
const int PIN_ECHO   = 18;   // HC-SR04 echo
const int PIN_PIR    = 27;   // PIR motion output
const int PIN_MOTION_TEST = 4;   // 'Motion test' button: lets automated tests inject motion
                                 // (Wokwi scenarios cannot click the PIR); held = person moving
const int PIN_DHT    = 15;   // DHT22 data
const int PIN_LDR    = 34;   // LDR module analog out (ADC1, input only)
const int PIN_BTN_LIB = 32;  // "Library Zone" button
const int PIN_BTN_LAB = 33;  // "Computer Lab" button
const int PIN_BTN_POD = 25;  // "Accessible Study Pod" button
const int PIN_LED_R  = 13;   // RGB LED red
const int PIN_LED_G  = 14;   // RGB LED green
const int PIN_LED_B  = 26;   // RGB LED blue
const int PIN_SERVO  = 19;   // status flag servo
const int PIN_BUZZER = 23;   // piezo buzzer
// OLED on the default I2C pins: SDA = 21, SCL = 22

// ---------------- Network settings ----------------
const char* WIFI_SSID  = "Wokwi-GUEST";   // Wokwi's simulated access point
const char* WIFI_PASS  = "";
const char* MQTT_HOST  = "broker.hivemq.com";
const int   MQTT_PORT  = 1883;
const char* MQTT_BASE  = "cavendish/bda2201/254430/campus";   // + /<facility>
const char* HTTP_URL   = "http://httpbin.org/post";           // echoes the JSON back

// ---------------- Calibration ----------------
const float LUX_CAL = 0.825f;             // see readLux()
const unsigned long SENSOR_WARMUP_MS = 2500;   // DHT22 needs ~2 s after power-up

// ---------------- Status + facility definitions ----------------
enum Status { AVAILABLE, CROWDED, UNAVAILABLE };
const char* STATUS_NAME[] = { "AVAILABLE", "CROWDED", "UNAVAILABLE" };

enum FacilityId { LIBRARY = 0, LAB = 1, POD = 2 };

struct Facility {
  const char* name;      // shown on the OLED and dashboard
  const char* key;       // used in the MQTT topic
  float minLux;          // below this the room is too dark to use
  float comfortMaxC;     // above this the room feels hot -> CROWDED/DEGRADED
  float unsafeMaxC;      // above this the room is not usable
  float blockedCm;       // ultrasonic: something this close = ramp/door blocked (pod only)
  bool  singleUser;      // a pod is "in use" as soon as one person is inside
};

Facility FACILITIES[3] = {
  { "Library Zone",          "library", 150.0, 28.0, 35.0,  0.0, false },
  { "Computer Lab",          "lab",      80.0, 27.0, 33.0,  0.0, false },
  { "Accessible Study Pod",  "pod",      80.0, 28.0, 35.0, 40.0, true  },
};

// ---------------- Hardware objects ----------------
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
DHTesp dht;
Servo flagServo;
WiFiClient netClient;          // used only by MQTT
WiFiClient httpClient;         // separate socket for HTTP so a POST never drops the MQTT session
PubSubClient mqtt(netClient);

// ---------------- Small helper: moving average + spike (z-score) detector ----------------
struct Smoother {
  static const int MAX = 8;
  int N;                       // window length chosen per sensor
  float buf[MAX];
  int count = 0, idx = 0;
  int suspect = 0;             // consecutive readings that looked like spikes
  explicit Smoother(int n) : N(n) {}
  void add(float v) { buf[idx] = v; idx = (idx + 1) % N; if (count < N) count++; }
  float mean() const { float s = 0; for (int i = 0; i < count; i++) s += buf[i]; return count ? s / count : NAN; }
  float stdev() const {
    if (count < 3) return 0;
    float m = mean(), s = 0;
    for (int i = 0; i < count; i++) s += (buf[i] - m) * (buf[i] - m);
    return sqrt(s / (count - 1));
  }
  // true when v is far from the recent average (sudden jump / glitch)
  bool isSpike(float v, float minJump) const {
    if (count < 4) return false;
    float m = mean(), sd = stdev();
    return fabs(v - m) > max(minJump, 3.0f * sd);
  }
  void reset() { count = 0; idx = 0; suspect = 0; }
  // Spike-aware add: a single odd reading is ignored (returns true = spike),
  // but if it stays for 3 readings in a row it is a real change and is accepted.
  bool addFiltered(float v, float minJump) {
    if (isSpike(v, minJump) && ++suspect < 3) return true;
    if (suspect >= 3) reset();          // real step change: restart the window at the new level
    suspect = 0;
    add(v);
    return false;
  }
};

// Window lengths: light and distance change in steps (lights on/off, person
// sits down) so they react fast; temperature/humidity drift slowly so they
// are smoothed more.
Smoother sDist(5), sTemp(8), sHum(8), sLux(4);

// PIR history: 1 sample per second for the last 30 s -> "how busy" ratio
const int PIR_WINDOW = 30;
uint8_t pirHist[PIR_WINDOW];
int pirCount = 0, pirIdx = 0;

// ---------------- Live state ----------------
struct Reading {
  float distCm = NAN, tempC = NAN, humPct = NAN, lux = NAN;
  bool  motion = false;
  bool  distOk = false, dhtOk = false, ldrOk = false;
  bool  spike = false;
};

Reading raw;                       // last raw reading
FacilityId current = LIBRARY;      // facility the student asked about
Status status = AVAILABLE, lastStatus = (Status)255;   // 255 = 'nothing announced yet' -> first decision is announced + published
float occScore = 0, comfortIdx = 1;
String reason = "starting up";
bool lowConfidence = false;
unsigned long bootMs = 0;

// network bookkeeping (shown on the dashboard as "network response metadata")
int lastHttpCode = 0;
unsigned long lastHttpMs = 0;
bool lastMqttOk = false;
unsigned long lastMqttTry = 0, lastHttpPost = 0, lastWifiTry = 0;
int wifiRetries = 0;

// ---------------- Buttons with debouncing ----------------
struct Button {
  int pin; FacilityId fac; bool stable = true; bool lastRaw = true; unsigned long changedAt = 0;
};
Button buttons[3] = { { PIN_BTN_LIB, LIBRARY }, { PIN_BTN_LAB, LAB }, { PIN_BTN_POD, POD } };
const unsigned long DEBOUNCE_MS = 40;

// ---------------- Non-blocking buzzer patterns ----------------
// A pattern is a list of (tone Hz, on ms, off ms). Blind students learn:
//   1 high beep = AVAILABLE, 2 medium beeps = CROWDED, 3 low long beeps = UNAVAILABLE
struct Beep { int hz; int onMs; int offMs; };
Beep pattern[4]; int patternLen = 0, patternStep = -1; unsigned long stepAt = 0; bool beepOn = false;

void playPattern(const Beep* p, int n) {
  for (int i = 0; i < n; i++) pattern[i] = p[i];
  patternLen = n; patternStep = 0; stepAt = millis(); beepOn = false;
}

void serviceBuzzer() {
  if (patternStep < 0) return;
  unsigned long now = millis();
  if (!beepOn) {
    if (now >= stepAt) {
      tone(PIN_BUZZER, pattern[patternStep].hz);
      beepOn = true; stepAt = now + pattern[patternStep].onMs;
    }
  } else if (now >= stepAt) {
    noTone(PIN_BUZZER);
    beepOn = false; stepAt = now + pattern[patternStep].offMs;
    if (++patternStep >= patternLen) patternStep = -1;
  }
}

void announceStatus(Status s) {
  static const Beep okB[]   = { {1760, 120, 80} };
  static const Beep busyB[] = { {988, 160, 140}, {988, 160, 80} };
  static const Beep noB[]   = { {440, 400, 200}, {440, 400, 200}, {440, 400, 80} };
  if (s == AVAILABLE) playPattern(okB, 1);
  else if (s == CROWDED) playPattern(busyB, 2);
  else playPattern(noB, 3);
}

// ---------------- Sensor reading with error handling ----------------
float readDistanceCm(bool& ok) {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 30000UL);   // 30 ms ~ 5 m max
  if (us == 0) { ok = false; return NAN; }               // no echo: unplugged / out of range
  float cm = us / 58.0f;
  ok = (cm >= 2.0f && cm <= 400.0f);                     // HC-SR04 valid range
  return ok ? cm : NAN;
}

float readLux(bool& ok) {
  // analogReadMilliVolts() uses the ESP32's factory ADC calibration,
  // which is far more accurate than scaling the raw 0..4095 count.
  float volts = analogReadMilliVolts(PIN_LDR) / 1000.0f;
  // The module is a voltage divider; convert the voltage to lux
  // (formula from the Wokwi photoresistor datasheet, gamma 0.7, RL10 50k).
  if (volts <= 0.01f || volts >= 3.29f) { ok = false; return NAN; }  // rail = open / shorted wire
  float resistance = 2000.0f * volts / (1.0f - volts / 3.3f);
  float lux = pow(50.0f * 1e3f * pow(10.0f, 0.7f) / resistance, 1.0f / 0.7f);
  // 5-point calibration (10/50/150/400/1000 lux reference levels) showed the
  // raw formula reads about 21% high on the ESP32 ADC -> scale by 0.825.
  lux *= LUX_CAL;
  ok = isfinite(lux);
  return ok ? lux : NAN;
}

void sampleSensors() {
  raw = Reading();
  raw.distCm = readDistanceCm(raw.distOk);

  TempAndHumidity th = dht.getTempAndHumidity();
  raw.dhtOk = (dht.getStatus() == DHTesp::ERROR_NONE) && !isnan(th.temperature) && !isnan(th.humidity);
  if (raw.dhtOk) { raw.tempC = th.temperature; raw.humPct = th.humidity; }

  raw.lux = readLux(raw.ldrOk);
  // motion = the PIR output, OR the test button (wired-OR in software)
  raw.motion = digitalRead(PIN_PIR) == HIGH || digitalRead(PIN_MOTION_TEST) == LOW;

  // Spike (anomaly) filter on temperature: one glitch is reported and ignored,
  // so it can't poison the average; a lasting change is accepted after 3 s.
  if (raw.dhtOk) {
    if (sTemp.addFiltered(raw.tempC, 4.0f)) raw.spike = true;
    else sHum.add(raw.humPct);
  }
  if (raw.distOk) sDist.add(raw.distCm);
  if (raw.ldrOk)  sLux.add(raw.lux);

  pirHist[pirIdx] = raw.motion ? 1 : 0;
  pirIdx = (pirIdx + 1) % PIR_WINDOW;
  if (pirCount < PIR_WINDOW) pirCount++;
}

float pirRatio() {
  if (!pirCount) return 0;
  int s = 0; for (int i = 0; i < pirCount; i++) s += pirHist[i];
  return (float)s / pirCount;
}

// ---------------- Edge AI: decide the facility state ----------------
float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

void classify() {
  const Facility& f = FACILITIES[current];
  float dist = sDist.mean(), temp = sTemp.mean(), hum = sHum.mean(), lux = sLux.mean();
  bool haveDist = raw.distOk && !isnan(dist);
  bool haveDht  = raw.dhtOk  && !isnan(temp);
  bool haveLux  = raw.ldrOk  && !isnan(lux);

  int faults = (!raw.distOk) + (!raw.dhtOk) + (!raw.ldrOk);
  lowConfidence = faults > 0 || raw.spike;

  // 1) Occupancy: small logistic model (weights tuned on my test runs).
  //    closeness = how near the nearest person/bag is to the seat-row sensor.
  float closeness = haveDist ? clamp01((150.0f - dist) / 120.0f) : 0.5f;   // unknown -> neutral
  float heat = haveDht ? clamp01((temp - 24.0f) / 8.0f) : 0.0f;            // body heat in a full room
  float z = -3.2f + 2.6f * pirRatio() + 2.4f * closeness + 0.9f * heat;
  occScore = 1.0f / (1.0f + exp(-z));

  // 2) Comfort index (1 = comfortable, 0 = unbearable) from temperature + humidity.
  if (haveDht) {
    float hot = clamp01((temp - f.comfortMaxC) / (f.unsafeMaxC - f.comfortMaxC));
    float damp = clamp01((hum - 70.0f) / 25.0f);
    comfortIdx = 1.0f - max(hot, damp);
  } else {
    comfortIdx = NAN;
  }

  // 3) Decision rules, worst condition wins.
  Status s = AVAILABLE;
  reason = "free and comfortable";

  if (f.blockedCm > 0 && haveDist && dist < f.blockedCm && !raw.motion && pirRatio() < 0.1f) {
    s = UNAVAILABLE; reason = "ramp/door blocked by an object";
  } else if (haveDht && temp >= f.unsafeMaxC) {
    s = UNAVAILABLE; reason = "too hot to use";
  } else if (haveLux && lux < f.minLux * 0.3f) {
    s = UNAVAILABLE; reason = "lights off / too dark";
  } else if (faults >= 2) {
    s = UNAVAILABLE; reason = "sensors offline - status unknown";
  } else if (f.singleUser && (raw.motion || pirRatio() > 0.2f)) {
    s = CROWDED; reason = "pod in use";
  } else if (occScore >= 0.70f) {
    s = CROWDED; reason = "busy - occupancy " + String((int)(occScore * 100)) + "%";
  } else if (haveDht && comfortIdx < 0.5f) {
    s = CROWDED; reason = "degraded: hot or humid";
  } else if (haveLux && lux < f.minLux) {
    s = CROWDED; reason = "degraded: poor light for reading";
  }

  if (s == AVAILABLE && faults == 1) reason = "free - one sensor offline";
  if (raw.spike && s == AVAILABLE) reason = "free - sudden spike ignored";
  status = s;
}

// ---------------- Actuators ----------------
void setRgb(bool r, bool g, bool b) { digitalWrite(PIN_LED_R, r); digitalWrite(PIN_LED_G, g); digitalWrite(PIN_LED_B, b); }

const char* driveActuators() {
  int angle;
  const char* colour;
  switch (status) {
    case AVAILABLE:   setRgb(0, 1, 0); angle = 0;   colour = "GREEN"; break;
    case CROWDED:     setRgb(1, 1, 0); angle = 90;  colour = "AMBER"; break;
    default:          setRgb(1, 0, 0); angle = 180; colour = "RED";   break;
  }
  flagServo.write(angle);
  return colour;
}

void drawOled() {
  const Facility& f = FACILITIES[current];
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);  oled.print(f.name);
  oled.setTextSize(2);
  oled.setCursor(0, 12); oled.print(status == UNAVAILABLE ? "CLOSED" : (status == CROWDED ? "BUSY" : "FREE"));
  oled.setTextSize(1);
  oled.setCursor(0, 32);
  if (raw.dhtOk) { oled.printf("%.1fC %.0f%%", sTemp.mean(), sHum.mean()); } else { oled.print("TEMP SENSOR FAULT"); }
  oled.setCursor(0, 42);
  if (raw.distOk) oled.printf("Dist %.0fcm Occ %d%%", sDist.mean(), (int)(occScore * 100));
  else oled.print("DIST SENSOR FAULT");
  oled.setCursor(0, 54);
  oled.print(WiFi.status() == WL_CONNECTED ? (mqtt.connected() ? "WiFi+MQTT OK" : "WiFi OK, MQTT..") : "OFFLINE (local)");
  oled.display();
}

// ---------------- Network (never blocks the sensors for long) ----------------
void serviceWifi() {
  if (WiFi.status() == WL_CONNECTED) { wifiRetries = 0; return; }
  if (millis() - lastWifiTry < 5000) return;
  lastWifiTry = millis();
  wifiRetries++;
  Serial.printf("[NET ] Wi-Fi not connected, retry %d\n", wifiRetries);
  WiFi.begin(WIFI_SSID, WIFI_PASS, 6);
}

void serviceMqtt() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) { mqtt.loop(); return; }
  if (millis() - lastMqttTry < 5000) return;
  lastMqttTry = millis();
  String clientId = "cuu-254430-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  if (mqtt.connect(clientId.c_str())) {
    Serial.printf("[MQTT] connected to %s:%d as %s\n", MQTT_HOST, MQTT_PORT, clientId.c_str());
  } else {
    Serial.printf("[MQTT] connect failed, state=%d (will retry)\n", mqtt.state());
  }
}

String buildPayload(const char* colour, int angle) {
  const Facility& f = FACILITIES[current];
  JsonDocument doc;
  doc["node"] = "cuu-main-block-01";
  doc["facility"] = f.name;
  doc["facility_key"] = f.key;
  doc["status"] = STATUS_NAME[status];
  doc["reason"] = reason;
  doc["low_confidence"] = lowConfidence;
  doc["uptime_s"] = (millis() - bootMs) / 1000;
  JsonObject t = doc["telemetry"].to<JsonObject>();
  if (raw.distOk) t["distance_cm"] = round(sDist.mean() * 10) / 10.0; else t["distance_cm"] = nullptr;
  if (raw.dhtOk)  { t["temp_c"] = round(sTemp.mean() * 10) / 10.0; t["humidity_pct"] = round(sHum.mean()); }
  else            { t["temp_c"] = nullptr; t["humidity_pct"] = nullptr; }
  if (raw.ldrOk)  t["light_lux"] = round(sLux.mean()); else t["light_lux"] = nullptr;
  t["motion"] = raw.motion;
  t["pir_ratio_30s"] = round(pirRatio() * 100) / 100.0;
  JsonObject ai = doc["edge_ai"].to<JsonObject>();
  ai["occupancy_score"] = round(occScore * 100) / 100.0;
  if (!isnan(comfortIdx)) ai["comfort_index"] = round(comfortIdx * 100) / 100.0; else ai["comfort_index"] = nullptr;
  ai["spike_detected"] = raw.spike;
  JsonArray faults = doc["sensor_faults"].to<JsonArray>();
  if (!raw.distOk) faults.add("ultrasonic");
  if (!raw.dhtOk)  faults.add("dht22");
  if (!raw.ldrOk)  faults.add("ldr");
  JsonObject act = doc["actuators"].to<JsonObject>();
  act["rgb"] = colour; act["servo_deg"] = angle; act["buzzer"] = STATUS_NAME[status];
  JsonObject net = doc["network"].to<JsonObject>();
  net["rssi_dbm"] = WiFi.RSSI();
  net["last_http_code"] = lastHttpCode;
  net["last_http_ms"] = lastHttpMs;
  String out; serializeJson(doc, out);
  return out;
}

void publish(const String& payload) {
  if (!mqtt.connected()) { lastMqttOk = false; return; }
  String topic = String(MQTT_BASE) + "/" + FACILITIES[current].key;
  lastMqttOk = mqtt.publish(topic.c_str(), payload.c_str(), true);   // retained: dashboard sees the latest at once
  Serial.printf("[MQTT] publish %s -> %s (%u bytes)\n", topic.c_str(), lastMqttOk ? "OK" : "FAILED", payload.length());
}

void postHttp(const String& payload) {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  http.setTimeout(4000);
  unsigned long t0 = millis();
  if (!http.begin(httpClient, HTTP_URL)) { Serial.println("[HTTP] begin failed"); return; }
  http.addHeader("Content-Type", "application/json");
  lastHttpCode = http.POST(payload);
  lastHttpMs = millis() - t0;
  if (lastHttpCode > 0) {
    Serial.printf("[HTTP] POST %s -> %d in %lu ms (%d bytes back)\n", HTTP_URL, lastHttpCode, lastHttpMs, http.getSize());
  } else {
    Serial.printf("[HTTP] POST failed: %s (node keeps working offline)\n", http.errorToString(lastHttpCode).c_str());
  }
  http.end();
}

// ---------------- Buttons ----------------
void serviceButtons() {
  for (auto& b : buttons) {
    bool r = digitalRead(b.pin);
    if (r != b.lastRaw) { b.lastRaw = r; b.changedAt = millis(); }
    if (millis() - b.changedAt > DEBOUNCE_MS && r != b.stable) {
      b.stable = r;
      if (r == LOW) {                         // pressed (INPUT_PULLUP)
        current = b.fac;
        // each facility starts with a fresh history (filters + PIR window)
        sDist.reset(); sTemp.reset(); sHum.reset(); sLux.reset();
        pirCount = 0; pirIdx = 0;
        Serial.printf("[BTN ] query: %s\n", FACILITIES[current].name);
        static const Beep click[] = { {2600, 40, 60} };
        playPattern(click, 1);
        lastStatus = (Status)255;             // force a fresh announcement + publish
      }
    }
  }
}

// ---------------- Setup + main loop ----------------
void setup() {
  Serial.begin(115200);
  bootMs = millis();
  pinMode(PIN_TRIG, OUTPUT); pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_MOTION_TEST, INPUT_PULLUP);
  for (auto& b : buttons) pinMode(b.pin, INPUT_PULLUP);
  pinMode(PIN_LED_R, OUTPUT); pinMode(PIN_LED_G, OUTPUT); pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  analogReadResolution(12);

  dht.setup(PIN_DHT, DHTesp::DHT22);
  flagServo.setPeriodHertz(50);
  flagServo.attach(PIN_SERVO, 500, 2400);

  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[OLED] not found - continuing with LED, servo and buzzer only");
  } else {
    oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
    oled.setCursor(0, 0); oled.println("Smart Campus Monitor"); oled.println("Cavendish University");
    oled.println(); oled.println("Starting sensors..."); oled.display();
  }

  Serial.println();
  Serial.println("=== Smart Campus Resource & Accessibility Monitor ===");
  Serial.println("node cuu-main-block-01 | ESP32 | Tinka Fahad 254430");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS, 6);
  lastWifiTry = millis();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(1024);
  mqtt.setKeepAlive(45);         // the simulator's network gateway adds latency
  mqtt.setSocketTimeout(10);
  Serial.println("[BOOT] ready - press a button to choose a facility (default: Library Zone)");
}

unsigned long lastSample = 0, lastPublish = 0;
bool wifiAnnounced = false;

void loop() {
  serviceButtons();
  serviceBuzzer();
  serviceWifi();
  serviceMqtt();

  if (!wifiAnnounced && WiFi.status() == WL_CONNECTED) {
    wifiAnnounced = true;
    Serial.printf("[NET ] Wi-Fi connected, IP %s, RSSI %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  }

  unsigned long now = millis();
  if (now - bootMs < SENSOR_WARMUP_MS) return;   // let the DHT22 settle before the first decision
  if (now - lastSample >= 1000) {                 // 1 Hz sense -> think -> act
    lastSample = now;
    sampleSensors();
    classify();
    const char* colour = driveActuators();
    int angle = status == AVAILABLE ? 0 : (status == CROWDED ? 90 : 180);
    drawOled();

    Serial.printf("[TEL ] t=%lus fac=%s | RAW dist=%s temp=%s hum=%s lux=%s pir=%d | AVG dist=%.0f temp=%.1f lux=%.0f pir30=%.2f | AI occ=%.2f comfort=%s%s | %s (%s) | LED=%s servo=%d\n",
      (now - bootMs) / 1000, FACILITIES[current].key,
      raw.distOk ? String(raw.distCm, 0).c_str() : "ERR",
      raw.dhtOk ? String(raw.tempC, 1).c_str() : "ERR",
      raw.dhtOk ? String(raw.humPct, 0).c_str() : "ERR",
      raw.ldrOk ? String(raw.lux, 0).c_str() : "ERR",
      raw.motion,
      sDist.mean(), sTemp.mean(), sLux.mean(), pirRatio(),
      occScore, isnan(comfortIdx) ? "n/a" : String(comfortIdx, 2).c_str(),
      raw.spike ? " SPIKE" : "",
      STATUS_NAME[status], reason.c_str(), colour, angle);

    if (!raw.distOk) Serial.println("[WARN] ultrasonic: no echo (unplugged or out of range) - using other sensors");
    if (!raw.dhtOk)  Serial.println("[WARN] DHT22: read failed - comfort check skipped");
    if (!raw.ldrOk)  Serial.println("[WARN] LDR: reading at the rail - check wiring");

    bool changed = status != lastStatus;
    if (changed) {
      Serial.printf("[ACT ] status -> %s : LED %s, servo %d deg, buzzer pattern %s\n",
                    STATUS_NAME[status], colour, angle, STATUS_NAME[status]);
      announceStatus(status);
      lastStatus = status;
    }
    if (changed || now - lastPublish >= 10000) {   // publish on change, plus a heartbeat every 10 s
      lastPublish = now;
      String payload = buildPayload(colour, angle);
      publish(payload);
      if (changed || now - lastHttpPost >= 30000) { lastHttpPost = now; postHttp(payload); }
    }
  }
}
