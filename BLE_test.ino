#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEAdvertisedDevice.h>
#include "NimBLEEddystoneTLM.h"
#include "NimBLEBeacon.h"
#include <M5Unified.h>

#define ENDIAN_CHANGE_U16(x) ((((x)&0xFF00) >> 8) + (((x)&0xFF) << 8))

// --- CONFIG ---
static const uint16_t MFID_APPLE = 0x004C;
static const uint16_t MFID_TELTONIKA = 0x089A;  // Teltonika EYE
static const char* TARGET_NAME = "MP1_D3BB42";  // <- ชื่ออุปกรณ์เป้าหมาย
// ---------------

int scanTime = 5 * 1000;  // scan 5s
NimBLEScan* pBLEScan;

// --------- ค่าเซนเซอร์ที่จะแสดงบนจอ ----------
struct SensorData {
  bool hasEddystone = false;
  bool decoded      = false;
  int batt_mV = -1;    // จาก TLM
  float temp_C = NAN;  // จาก TLM
  int advCount = -1;   // จาก TLM
  int uptime_s = -1;   // จาก TLM
  int rssi = 0;
  int humidity_pct = -1;
  String name = "-";
  String mac = "-";
} g;

uint32_t g_lastSensorMs = 0;
// ------------------------------------------------

static void printHex(const uint8_t* p, size_t n) {
  for (size_t i = 0; i < n; ++i) Serial.printf("[%02X]", p[i]);
}

// TLV generic (โชว์เป็นชิ้น ๆ ใน Serial ให้เราดู pattern ก่อน)
static void tryParseTLVGeneric(const uint8_t* p, size_t n) {
  size_t i = 0;
  while (i + 2 <= n) {
    uint8_t t = p[i++], L = p[i++];
    if (i + L > n) {
      Serial.printf(" TLV break t=0x%02X len=%u\n", t, L);
      break;
    }
    Serial.printf(" TLV t=0x%02X len=%u val=", t, L);
    printHex(&p[i], L);
    Serial.println();

    // ตัวอย่าง map ง่าย ๆ (เดา tag ชนิด sensor – ถ้าไม่ตรง spec จะเห็นได้จาก log)
    if (t == 0x01 && L == 2) {  // ตัวอย่าง: แบต (สมมุติ)
      int mv = (p[i] << 8) | p[i + 1];
      Serial.printf(" -> (guess) battery ~ %dmV\n", mv);
    } else if (t == 0x02 && L == 2) {  // ตัวอย่าง: อุณหภูมิ (สมมุติ)
      int raw = (p[i] << 8) | p[i + 1];
      float tc = raw / 256.0f;
      Serial.printf(" -> (guess) temp ~ %.2f C\n", tc);
    }
    i += L;
  }
}

// ✅ Teltonika MFID=0x089A Decode (เพิ่มตรงนี้)
static void parseTeltonika089A(const uint8_t* p, size_t n) {
  // โครงสร้าง: [ver(1)][flags(1)][extended...]
  if (n < 2) return;

  uint8_t ver   = p[0];
  uint8_t flags = p[1];
  const uint8_t* v = p + 2;
  size_t   r = n - 2;

  Serial.printf(" TELTO ver=0x%02X flags=0x%02X\n", ver, flags);

  // ตามสเปก: เรียงข้อมูลตามลิสต์นี้เมื่อ flag ถูกตั้ง
  auto need = [&](int bit){ return (flags >> bit) & 0x01; };

  // ตัวแปรชั่วคราว
  bool got = false;
  int   batt_mV = -1;
  float tempC = NAN;
  int   hum = -1;
  uint16_t moveCnt = 0;
  int8_t  pitch = 0;
  int16_t roll  = 0;
  bool magnet_present = need(2);
  bool magnet_state   = need(3); // 1=มีแม่เหล็ก, 0=ไม่มีแม่เหล็ก (ตามสเปก)

  // 0: Temperature (2B, °C/100)
  if (need(0)) {
    if (r < 2) return;
    int16_t t = (int16_t)((v[0] << 8) | v[1]);
    tempC = t / 100.0f;
    v += 2; r -= 2;
  }

  // 1: Humidity (1B, %)
  if (need(1)) {
    if (r < 1) return;
    hum = v[0];
    v += 1; r -= 1;
  }

  // 4: Movement counter (2B), MSB = movement state อยู่ในเอกสารเป็นอีกบิตหนึ่ง (bit4 เป็น presence)
  if (need(4)) {
    if (r < 2) return;
    moveCnt = (uint16_t)((v[0] << 8) | v[1]);
    v += 2; r -= 2;
  }

  // 5: Movement angle (3B): pitch(int8), roll(int16)
  if (need(5)) {
    if (r < 3) return;
    pitch = (int8_t)v[0];
    roll  = (int16_t)((v[1] << 8) | v[2]);
    v += 3; r -= 3;
  }

  // 7: Battery (1B): mV = 2000 + N*10
  if (need(7)) {
    if (r < 1) return;
    batt_mV = 2000 + ((int)v[0]) * 10;
    v += 1; r -= 1;
  }

  // อัปเดตค่าแสดงผล (เฉพาะเมื่อ decode สำเร็จจริง)
  g.batt_mV      = batt_mV;
  g.temp_C       = tempC;
  g.humidity_pct = hum;
  g.advCount     = moveCnt;          // ใช้เก็บ movement count ชั่วคราว (หรือเพิ่มฟิลด์ใหม่ก็ได้)
  g.uptime_s     = (pitch << 16) | (uint16_t)roll; // ถ้ายังไม่ใช้ uptime, เก็บ angle ไว้ชั่วคราว (หรือเพิ่มฟิลด์ใหม่)
  g.name         = "MP1_D3BB42";     // ตอกย้ำชื่อเป้าหมาย
  g_lastSensorMs = millis();
  g.decoded      = true;
  got = true;

  Serial.printf(" DECODED -> Batt=%d mV Temp=%.2f C Hum=%d%% MoveCnt=%u Pitch=%d Roll=%d Magnet=%s\n",
                g.batt_mV, g.temp_C, g.humidity_pct, moveCnt, pitch, roll,
                (magnet_present ? (magnet_state ? "DETECTED" : "NOT DETECTED") : "N/A"));
}


// iBeacon ตรวจให้ชัด: 0x004C + 0x02 0x15
static bool isIBeacon(const std::string& md) {
  if (md.size() < 25) return false;
  const uint8_t* p = (const uint8_t*)md.data();
  uint16_t mfid = (uint16_t)p[1] << 8 | p[0];
  return (mfid == MFID_APPLE && p[2] == 0x02 && p[3] == 0x15);
}

static bool isTargetDevice(const NimBLEAdvertisedDevice* d) {
  if (d->haveName() && String(d->getName().c_str()) == TARGET_NAME) return true;
  // ถ้ารู้ MAC ชัดเจน สามารถเพิ่มกรอง MAC ตรงนี้ได้
  return false;
}

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* d) override {
    // กรองเฉพาะอุปกรณ์เป้าหมาย
    if (!isTargetDevice(d)) return;

    g.rssi = d->getRSSI();
    g.name = d->haveName() ? String(d->getName().c_str()) : "-";
    g.mac = String(d->getAddress().toString().c_str());

    Serial.printf("---- TARGET %s (%s) RSSI=%d ----\n",
                  g.name.c_str(), g.mac.c_str(), g.rssi);

    // Manufacturer Data (Teltonika / iBeacon)
    if (d->haveManufacturerData()) {
      std::string md = d->getManufacturerData();
      const uint8_t* p = (const uint8_t*)md.data();
      size_t n = md.size();

      if (n >= 2) {
        uint16_t mfid = (uint16_t)p[1] << 8 | p[0];  // Company ID (LE)
        Serial.printf("ManufacturerData len=%d mfid=0x%04X ", (int)n, mfid);
        printHex(p, n);
        Serial.println();

        // 1) ถ้าเซนเซอร์คอนฟิกเป็น iBeacon เท่านั้น → ไม่มีค่า temp/hum ในเฟรม (จะไม่ decoded)
        if (isIBeacon(md)) {
          Serial.println(" -> iBeacon frame (no sensor values in this frame)");
          NimBLEBeacon b;
          b.setData(p, n);
          Serial.printf(" UUID:%s Major:%d Minor:%d Tx:%d\n",
                        b.getProximityUUID().toString().c_str(),
                        ENDIAN_CHANGE_U16(b.getMajor()),
                        ENDIAN_CHANGE_U16(b.getMinor()),
                        b.getSignalPower());
          // ไม่ตั้ง g.decoded ที่นี่
        }
        // 2) ถ้าเป็น Teltonika (ต้องส่ง custom frame) → parse TLV ภายใน
        else if (mfid == MFID_TELTONIKA) {
          Serial.println(" -> Teltonika (0x089A) payload:");
          if (n > 2) {
            const uint8_t* payload = p + 2;  // ข้าม company ID 2 ไบต์
            size_t plen = n - 2;

            // log เพิ่มเติมช่วยจับรูปแบบจริง
            Serial.print(" RAW payload: ");
            printHex(payload, plen);
            Serial.println();

            // (ถ้ามี header 2 ไบต์ของ frameType/flags ตามโค้ดเดิม)
            if (plen >= 2) {
              Serial.printf(" frameType=0x%02X flags/mask=0x%02X\n", payload[0], payload[1]);
            }

            // เรียก TLV parser ของเรา (จะตั้ง g.decoded และ g_lastSensorMs เมื่อถอดสำเร็จ)
            const uint8_t* tlv = (plen > 2) ? (payload + 2) : payload;
            size_t tln = (plen > 2) ? (plen - 2) : plen;
            parseTeltonika089A(tlv, tln);
          }
        }
        // 3) Vendor อื่น ๆ
        else {
          Serial.println(" -> Other vendor frame");
        }
      }
    }

    // ================= ไล่ Service Data ทั้งหมด แล้วจับ FEAA/TLM =================
    if (d->getServiceDataCount() > 0) {
      for (int i = 0; i < d->getServiceDataCount(); ++i) {
        std::string sdu = d->getServiceData(i);  // AD type 0x16 (Service Data - 16-bit UUID)
        const uint8_t* s = (const uint8_t*)sdu.data();
        size_t sn = sdu.size();
        if (sn < 3) continue;

        // โครง Service Data: [UUID16 (LE)] [payload...]
        uint16_t uuid16 = (uint16_t)s[1] << 8 | s[0];
        const uint8_t* payload = s + 2;
        size_t plen = sn - 2;

        if (uuid16 == 0xFEAA && plen >= 1) {
          Serial.printf("ServiceData(FEAA) len=%d : ", (int)plen);
          printHex(payload, plen);
          Serial.println();

          // TLM (unencrypted) = frameType 0x20
          if (payload[0] == 0x20) {
            if (plen >= 14) {
              // TLM layout:
              // 0:type(0x20) 1:ver 2-3:mV 4-5:Temp*256(signed) 6-9:AdvCount(BE) 10-13:Uptime(BE)
              uint8_t ver = payload[1];
              int batt_mV = (payload[2] << 8) | payload[3];
              int16_t traw = (int16_t)((payload[4] << 8) | payload[5]);
              float tempC = traw / 256.0f;
              uint32_t adv = (payload[6] << 24) | (payload[7] << 16) | (payload[8] << 8) | payload[9];
              uint32_t sec = (payload[10] << 24) | (payload[11] << 16) | (payload[12] << 8) | payload[13];

              g.hasEddystone = true;
              g.batt_mV = batt_mV;
              g.temp_C = tempC;
              g.advCount = (int)adv;
              g.uptime_s = (int)sec;

              Serial.printf(" TLM v%d Volt=%dmV Temp=%.2fC Adv=%u Uptime=%us\n",
                            ver, batt_mV, tempC, adv, sec);
            } else {
              Serial.println(" TLM payload too short");
            }
          }
        }
      }
    }

    // Eddystone TLM (แบต/อุณหภูมิ/ตัวนับ/uptime) : UUID 0xFEAA
    if (d->haveServiceUUID()) {
      NimBLEUUID uuid = d->getServiceUUID();
      if (uuid.equals((uint16_t)0xFEAA)) {
        std::string sd = d->getServiceData(uuid);
        if (!sd.empty()) {
          const uint8_t* s = (const uint8_t*)sd.data();
          Serial.printf("Eddystone (FEAA) serviceData len=%d ", (int)sd.size());
          printHex(s, sd.size());
          Serial.println();

          if (sd.size() > 0 && s[0] == 0x20) {  // TLM (unencrypted)
            NimBLEEddystoneTLM tlm;
            tlm.setData(s, sd.size());

            g.hasEddystone = true;
            g.batt_mV = tlm.getVolt();
            g.temp_C = (double)tlm.getTemp();
            g.advCount = tlm.getCount();
            g.uptime_s = tlm.getTime();

            // cross-check manual temp: (s[4:6] เป็น temp*256 signed)
            if (sd.size() >= 6) {
              int raw = (int)s[5] + ((int)s[4] << 8);
              float calc = raw / 256.0f;
              Serial.printf(" TLM Volt=%dmV Temp=%.2fC (calc=%.2fC) Adv=%d Uptime=%ds\n",
                            g.batt_mV, g.temp_C, calc, g.advCount, g.uptime_s);
            }
          }
        }
      }
    }

    Serial.println();
  }
} scanCallbacks;

void drawScreen() {
  M5.Display.clear();
  M5.Display.setCursor(10, 10);
  M5.Display.setTextSize(2);

  M5.Display.println("EYE Sensor (Target)");
  M5.Display.printf("Name: %s\n", g.name.c_str());
  M5.Display.printf("MAC : %s\n", g.mac.c_str());
  M5.Display.printf("RSSI: %d dBm\n", g.rssi);
  M5.Display.println("--------------------");

  bool hasSensor = (millis() - g_lastSensorMs) <= 15000;  // 15s ล่าสุดมีเฟรม
  if (hasSensor) {
    if (g.batt_mV >= 0) M5.Display.printf("Battery: %d mV\n", g.batt_mV);
    if (!isnan(g.temp_C)) M5.Display.printf("Temp : %.2f C\n", g.temp_C);
    if (g.humidity_pct >= 0) M5.Display.printf("Humid : %d %%\n", g.humidity_pct);
    if (g.advCount >= 0) M5.Display.printf("AdvCnt : %d\n", g.advCount);
    if (g.uptime_s >= 0) M5.Display.printf("Uptime : %d s\n", g.uptime_s);
  } else {
    M5.Display.println("No sensor frame yet...");
    M5.Display.println("(Teltonika TLV / FEAA UID)");
  }
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setBrightness(200);
  M5.Display.clear();
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 10);
  M5.Display.println("BLE Scanner Init...");

  Serial.begin(115200);

  NimBLEDevice::init("Beacon-scanner");
  pBLEScan = NimBLEDevice::getScan();  // ✅ ใช้ NimBLEDevice
  pBLEScan->setScanCallbacks(&scanCallbacks);
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(120);
  pBLEScan->setWindow(120);
  pBLEScan->setDuplicateFilter(false);  // ✅ เก็บเฟรมซ้ำ (เผื่อ interleave)
}

void loop() {
  // แจ้งสถานะบนจอ: กำลังสแกน
  M5.Display.clear();
  M5.Display.setCursor(10, 10);
  M5.Display.setTextSize(2);
  M5.Display.println("Scanning target...");
  M5.Display.println("(5 seconds)");

  // สแกน 5 วินาที
  NimBLEScanResults rs = pBLEScan->getResults(scanTime, false);
  Serial.printf("Devices found: %d\n", rs.getCount());
  Serial.println("Scan done!");
  pBLEScan->clearResults();

  // วาดค่าที่มีอยู่ล่าสุด
  drawScreen();

  // รอ 5 นาที
  delay(20000);
}
