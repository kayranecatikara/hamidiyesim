#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

/*
 * AVCI ELRS Kare Yer Istasyonu - ESP32 DevKit V1
 *
 * Harici kutuphane YOKTUR. Arduino-ESP32 cekirdeginin WiFi, WebServer ve
 * HardwareSerial siniflari kullanilir.
 *
 * UART BAGLANTISI (Ranger Micro modul-bay/CRSF pin adlarini kilavuzdan teyit et):
 *   ESP32 GPIO17 (TX2) -> Ranger CRSF RX / module input
 *   ESP32 GPIO16 (RX2) <- Ranger CRSF TX / telemetry output
 *   ESP32 GND          -- Ranger GND (ortak toprak zorunlu)
 *
 * Ranger'i ESP32'nin 5V/3V3 pininden BESLEME. Anteni tak, modulu ureticinin
 * belirttigi 6-16.8 V XT30 girisinden uygun batarya/BEC ile besle.
 * Mantik seviyesi modul bayinizde 3.3 V TTL degilse uygun seviye donusturucu kullan.
 *
 * GPIO27 -> GND: fiziksel E-STOP (INPUT_PULLUP). Acikken aninda notr+DISARM.
 * GPIO26 -> GND: ARM izin anahtari. ARM HTTP komutu ancak pin LOW ise kabul edilir.
 *
 * Ucus karti kanal sozlesmesi (Betaflight/INAV Receiver sekmesinde AETR):
 *   CH1 Roll, CH2 Pitch, CH3 Throttle, CH4 Yaw,
 *   CH5 ARM (AUX1), CH6 ANGLE/ALT-HOLD tercihi (AUX2).
 * Bu kod kalkis yapmaz. Arac havada ve irtifa tutan uygun moddayken acik-dongu,
 * zamana dayali kare cizer. GPS'siz konum geri beslemesi olmadigindan geometrik
 * hassas kare garanti edilemez.
 */

// ---------- Kullanici ayarlari ----------
static const char *AP_SSID = "AVCI-ELRS-GCS";
static const char *AP_PASSWORD = "AvciGuvenli2026"; // En az 8 karakter; sahada degistirin.

// ---------- Istasyon (STA) modu - AG TOPOLOJISI ----------
// NEDEN: ESP32 yalniz softAP iken operator dizustunu AVCI-ELRS-GCS agina
// baglamak zorundaydi. O anda dizustu BASKA bir agda olamadigi icin,
// gcs_server baska bir makinede kosuyorsa video ve telemetri kesiliyordu.
// (Ayni makinede kosuyorsa localhost calismaya devam ediyordu.)
//
// COZUM: WIFI_AP_STA. ESP32 hem kendi AP'sini yayinlamayi SURDURUR (saha
// yedegi, altyapisiz calisma) hem de mevcut aga istemci olarak katilir.
// Boylece dizustu tek agda kalir; hem gcs_server'a hem ESP32'ye erisir.
//
// GERIYE DONUK UYUM: STA_SSID bos birakilirsa davranis ESKISININ AYNISIDIR
// (yalniz AP). Bos degilse AP + STA birlikte kosar; AP hicbir kosulda
// kapatilmaz, cunku sahada altyapi agi olmayabilir.
static const char *STA_SSID = "";          // bos = yalniz AP (eski davranis)
static const char *STA_PASSWORD = "";
constexpr uint32_t STA_RETRY_MS = 15000;   // baglanti koparsa yeniden deneme araligi

constexpr int CRSF_RX_PIN = 16;
constexpr int CRSF_TX_PIN = 17;
constexpr int ESTOP_PIN = 27;
constexpr int ARM_PERMIT_PIN = 26;
constexpr uint32_t CRSF_BAUD = 400000;
constexpr uint32_t FRAME_PERIOD_US = 20000; // 50 Hz; guvenli ve yaygin handset hizi

constexpr uint16_t PWM_MIN = 1000;
constexpr uint16_t PWM_MID = 1500;
constexpr uint16_t PWM_MAX = 2000;
constexpr uint16_t HOVER_THROTTLE_US = 1500; // Araciniza pervaneler SOKUKEN kalibre edin.
constexpr uint16_t MOVE_DEFLECTION_US = 220; // 1500 +/- 220
constexpr uint32_t LEG_DURATION_MS = 3000;
constexpr uint32_t CORNER_PAUSE_MS = 500;
constexpr uint32_t COMMAND_WATCHDOG_MS = 1500;

HardwareSerial CrsfSerial(2);
WebServer server(80);

enum class MissionState : uint8_t {
  DISARMED, ARMED_IDLE, FORWARD, PAUSE_1, RIGHT, PAUSE_2,
  BACKWARD, PAUSE_3, LEFT, COMPLETE, ABORTED
};

MissionState mission = MissionState::DISARMED;
uint32_t stateStartedMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastFrameUs = 0;
bool armed = false;
uint16_t channelsUs[16];

// Telemetri: CRSF LINK_STATISTICS (0x14) icinden son gorulen uplink LQ.
int linkQuality = -1;
int8_t uplinkRssiDbm = 0;
uint32_t lastTelemetryMs = 0;
uint8_t rxFrame[64];
size_t rxCount = 0;

// ---------- GERCEK KARE HIZI OLCUMU ----------
// Onceki surum /api/status icinde "frame_hz":50 SABITINI donduruyordu. Bu bir
// olcum degil, niyet beyaniydi: loop() HTTP istegiyle veya UART okumasiyla
// gecikirse gercek hiz 50'nin altina duser ve arayuz bunu ASLA goremezdi.
// Simdi gonderilen kareler sayilir ve 1 sn'lik pencerede gercek hiz cikarilir.
uint32_t framesSent = 0;          // pencere icindeki kare sayisi
uint32_t frameWindowMs = 0;       // pencere baslangici
uint16_t measuredFrameHz = 0;     // son tamamlanan pencerenin olcumu
uint32_t frameSkips = 0;          // zamaninda gonderilemeyen kare sayisi (teshis)

// STA baglanti durumu
uint32_t lastStaTryMs = 0;
bool staEnabled = false;

const char PAGE[] PROGMEM = R"HTML(<!doctype html><html lang="tr"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AVCI ELRS · Kare Gorevi</title><style>
:root{color-scheme:dark;--z:#0a151c;--p:#0f212b;--h:#1f4451;--y:#3be0a0;--u:#f5b03c;--k:#f04a3f;--m:#dde9e8;--s:#9bb4b8}
*{box-sizing:border-box}body{margin:0;background:var(--z);color:var(--m);font:15px system-ui;padding:24px}.k{max-width:760px;margin:auto}.panel{background:var(--p);border:1px solid var(--h);padding:18px;margin:12px 0;border-radius:8px}h1{font-size:22px;margin:0}.ust{display:flex;justify-content:space-between;gap:12px;align-items:center}.rozet{border:1px solid var(--h);padding:6px 10px}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}.v{font:24px ui-monospace,monospace;color:var(--y)}label{display:block;color:var(--s);font-size:12px}button{width:100%;min-height:54px;border:1px solid var(--h);background:#153039;color:var(--m);font-weight:700;margin-top:10px}button.arm{border-color:var(--u)}button.go{border-color:var(--y)}button.stop{background:var(--k);border-color:var(--k)}button:disabled{opacity:.35}.not{color:var(--u);line-height:1.5}@media(max-width:560px){.grid{grid-template-columns:1fr}.ust{align-items:flex-start;flex-direction:column}}
</style></head><body><main class="k"><section class="panel ust"><div><h1>AVCI ELRS · KARE GOREVI</h1><label>ESP32 / CRSF 400 kbaud / 50 Hz</label></div><div class="rozet" id="net">BAGLANIYOR</div></section>
<section class="panel grid"><div><label>DURUM</label><div class="v" id="state">-</div></div><div><label>ELRS LINK</label><div class="v" id="lq">-</div></div><div><label>ARM IZNI</label><div class="v" id="permit">-</div></div>
<div><label>CRSF KARE HIZI (OLCULEN)</label><div class="v" id="hz">-</div></div><div><label>E-STOP</label><div class="v" id="estop">-</div></div><div><label>AG</label><div class="v" id="net2" style="font-size:15px">-</div></div></section>
<section class="panel"><label>GOREV AKISI</label><p>ILERI → SAG → GERI → SOL → NOTR</p><button class="arm" id="arm" onclick="cmd('arm')">ARM (fiziksel izin gerekir)</button><button class="go" id="go" onclick="cmd('start')">KAREYI BASLAT</button><button onclick="cmd('abort')">GOREVI DURDUR · HAVADA NOTR</button><button class="stop" onclick="cmd('disarm')">ACIL DISARM</button><p class="not" id="msg">Pervaneler sokukken masa testi yapmadan ucus denemesi yapmayin.</p></section>
</main><script>
async function cmd(c){try{let r=await fetch('/api/'+c,{method:'POST'}),j=await r.json();document.querySelector('#msg').textContent=j.message}catch(e){document.querySelector('#msg').textContent='Komut baglantisi kesildi'}}
async function tick(){try{let r=await fetch('/api/status',{cache:'no-store'}),j=await r.json();net.textContent='ESP32 BAGLI';state.textContent=j.state;lq.textContent=j.lq<0?'BEKLENIYOR':j.lq+'%';permit.textContent=j.arm_permit?'ACIK':'KAPALI';
hz.textContent=j.frame_hz?j.frame_hz+' Hz':'OLCULUYOR';hz.style.color=(j.frame_hz&&j.frame_hz<45)?'var(--k)':'var(--y)';
estop.textContent=j.estop?'BASILI':'SERBEST';estop.style.color=j.estop?'var(--k)':'var(--y)';
net2.textContent='AP '+j.ap_ip+' ('+j.ap_clients+')'+(j.sta_enabled?(j.sta_connected?' · STA '+j.sta_ip:' · STA YOK'):'');
arm.disabled=j.armed||!j.arm_permit||j.estop;go.disabled=!j.armed||j.mission_active||j.estop}catch(e){net.textContent='BAGLANTI YOK'}setTimeout(tick,300)}tick();
</script></body></html>)HTML";

const char *stateName(MissionState s) {
  switch (s) {
    case MissionState::DISARMED: return "DISARM";
    case MissionState::ARMED_IDLE: return "ARM / BEKLE";
    case MissionState::FORWARD: return "1/4 ILERI";
    case MissionState::PAUSE_1: return "KOSE 1";
    case MissionState::RIGHT: return "2/4 SAG";
    case MissionState::PAUSE_2: return "KOSE 2";
    case MissionState::BACKWARD: return "3/4 GERI";
    case MissionState::PAUSE_3: return "KOSE 3";
    case MissionState::LEFT: return "4/4 SOL";
    case MissionState::COMPLETE: return "TAMAMLANDI";
    default: return "IPTAL";
  }
}

const char *stateCode(MissionState s) {
  switch (s) {
    case MissionState::FORWARD: return "FORWARD";
    case MissionState::RIGHT: return "RIGHT";
    case MissionState::BACKWARD: return "BACKWARD";
    case MissionState::LEFT: return "LEFT";
    case MissionState::COMPLETE: return "COMPLETE";
    default: return "IDLE";
  }
}

bool missionActive() {
  return mission >= MissionState::FORWARD && mission <= MissionState::LEFT;
}

uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : crc << 1;
  }
  return crc;
}

uint16_t usToCrsf(uint16_t us) {
  us = constrain(us, PWM_MIN, PWM_MAX);
  return map(us, PWM_MIN, PWM_MAX, 172, 1811);
}

void sendRcFrame() {
  uint16_t ch[16];
  for (int i = 0; i < 16; ++i) ch[i] = usToCrsf(channelsUs[i]);
  uint8_t frame[26] = {0xEE, 24, 0x16}; // handset/module address, len, RC_CHANNELS_PACKED
  uint32_t bitBuffer = 0;
  uint8_t bits = 0, out = 3;
  for (int i = 0; i < 16; ++i) {
    bitBuffer |= (uint32_t)(ch[i] & 0x07FF) << bits;
    bits += 11;
    while (bits >= 8) {
      frame[out++] = bitBuffer & 0xFF;
      bitBuffer >>= 8;
      bits -= 8;
    }
  }
  frame[25] = crc8(&frame[2], 23); // type + 22-byte payload
  CrsfSerial.write(frame, sizeof(frame));

  // Gercek hiz olcumu: pencere dolunca sayaci hiza cevir.
  ++framesSent;
  const uint32_t now = millis();
  if (now - frameWindowMs >= 1000) {
    measuredFrameHz = (uint16_t)((framesSent * 1000UL) / (now - frameWindowMs));
    framesSent = 0;
    frameWindowMs = now;
  }
}

void neutralControls(bool keepArmed) {
  for (auto &c : channelsUs) c = PWM_MID;
  channelsUs[2] = keepArmed ? HOVER_THROTTLE_US : PWM_MIN;
  channelsUs[4] = keepArmed ? PWM_MAX : PWM_MIN; // AUX1 ARM
  channelsUs[5] = PWM_MAX;                       // AUX2 stabilised/alt-hold mode
}

void abortAndDisarm(const char *reason) {
  armed = false;
  mission = MissionState::ABORTED;
  neutralControls(false);
  Serial.printf("[EMNIYET] %s\n", reason);
}

void enterState(MissionState next) {
  mission = next;
  stateStartedMs = millis();
  neutralControls(armed);
  switch (next) {
    case MissionState::FORWARD:  channelsUs[1] = PWM_MID - MOVE_DEFLECTION_US; break;
    case MissionState::RIGHT:    channelsUs[0] = PWM_MID + MOVE_DEFLECTION_US; break;
    case MissionState::BACKWARD: channelsUs[1] = PWM_MID + MOVE_DEFLECTION_US; break;
    case MissionState::LEFT:     channelsUs[0] = PWM_MID - MOVE_DEFLECTION_US; break;
    default: break;
  }
  Serial.printf("[GOREV] %s\n", stateName(next));
}

void updateMission() {
  if (!armed || !missionActive()) return;
  const uint32_t elapsed = millis() - stateStartedMs;
  switch (mission) {
    case MissionState::FORWARD:  if (elapsed >= LEG_DURATION_MS) enterState(MissionState::PAUSE_1); break;
    case MissionState::PAUSE_1:  if (elapsed >= CORNER_PAUSE_MS) enterState(MissionState::RIGHT); break;
    case MissionState::RIGHT:    if (elapsed >= LEG_DURATION_MS) enterState(MissionState::PAUSE_2); break;
    case MissionState::PAUSE_2:  if (elapsed >= CORNER_PAUSE_MS) enterState(MissionState::BACKWARD); break;
    case MissionState::BACKWARD: if (elapsed >= LEG_DURATION_MS) enterState(MissionState::PAUSE_3); break;
    case MissionState::PAUSE_3:  if (elapsed >= CORNER_PAUSE_MS) enterState(MissionState::LEFT); break;
    case MissionState::LEFT:     if (elapsed >= LEG_DURATION_MS) enterState(MissionState::COMPLETE); break;
    default: break;
  }
}

void parseTelemetry() {
  while (CrsfSerial.available()) {
    uint8_t b = CrsfSerial.read();
    if (rxCount == 0 && b != 0xEA && b != 0xEE && b != 0xC8) continue;
    if (rxCount < sizeof(rxFrame)) rxFrame[rxCount++] = b; else rxCount = 0;
    if (rxCount >= 2) {
      const size_t total = rxFrame[1] + 2;
      if (total < 4 || total > sizeof(rxFrame)) { rxCount = 0; continue; }
      if (rxCount == total) {
        if (crc8(&rxFrame[2], total - 3) == rxFrame[total - 1] && rxFrame[2] == 0x14 && total >= 14) {
          uplinkRssiDbm = -(int8_t)rxFrame[3];
          linkQuality = rxFrame[5];
          lastTelemetryMs = millis();
        }
        rxCount = 0;
      }
    }
  }
  if (millis() - lastTelemetryMs > 2000) linkQuality = -1;
}

void jsonReply(bool ok, const String &message) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(ok ? 200 : 409, "application/json",
              String("{\"ok\":") + (ok ? "true" : "false") + ",\"message\":\"" + message + "\"}");
}

void setupHttp() {
  server.on("/", HTTP_GET, [] { server.send_P(200, "text/html; charset=utf-8", PAGE); });
  server.on("/api/status", HTTP_GET, [] {
    if (missionActive()) lastControlMs = millis(); // tarayici heartbeat'i
    String j = "{\"state\":\"" + String(stateName(mission)) + "\",\"state_code\":\"" + String(stateCode(mission)) + "\",\"armed\":" + (armed ? "true" : "false") +
      ",\"mission_active\":" + (missionActive() ? "true" : "false") + ",\"arm_permit\":" +
      (digitalRead(ARM_PERMIT_PIN) == LOW ? "true" : "false") + ",\"lq\":" + String(linkQuality) +
      ",\"rssi_dbm\":" + String(uplinkRssiDbm) + ",\"telemetry_age_ms\":" +
      String(lastTelemetryMs ? millis() - lastTelemetryMs : 4294967295UL) +
      // OLCULEN deger (sabit degil). Ilk pencere dolana kadar 0 doner;
      // arayuz 0'i "henuz olculmedi" olarak gosterir, 50 uydurmaz.
      ",\"frame_hz\":" + String(measuredFrameHz) +
      ",\"frame_skips\":" + String(frameSkips) +
      ",\"estop\":" + (digitalRead(ESTOP_PIN) == LOW ? "true" : "false") +
      ",\"uptime_s\":" + String(millis() / 1000) +
      ",\"ap_ip\":\"" + WiFi.softAPIP().toString() + "\"" +
      ",\"ap_clients\":" + String(WiFi.softAPgetStationNum()) +
      ",\"sta_enabled\":" + (staEnabled ? "true" : "false") +
      ",\"sta_connected\":" + (staEnabled && WiFi.status() == WL_CONNECTED ? "true" : "false") +
      ",\"sta_ip\":\"" + (staEnabled && WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("")) + "\"" +
      ",\"sta_rssi\":" + String(staEnabled && WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0) +
      ",\"channels_us\":[";
    for (int i = 0; i < 16; ++i) { if (i) j += ','; j += String(channelsUs[i]); }
    j += "]}";
    server.sendHeader("Access-Control-Allow-Origin", "*"); server.send(200, "application/json", j);
  });
  // CORS on-kontrolu: arayuz basit istek gonderdigi surece tarayici preflight
  // yapmaz, ama ileride baslik eklenirse 404 yerine duzgun cevap donsun.
  server.on("/api/status", HTTP_OPTIONS, [] {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    server.send(204);
  });
  server.on("/api/arm", HTTP_POST, [] {
    lastControlMs = millis();
    if (digitalRead(ESTOP_PIN) == LOW) return jsonReply(false, "E-STOP aktif");
    if (digitalRead(ARM_PERMIT_PIN) != LOW) return jsonReply(false, "Fiziksel ARM izin anahtari kapali");
    armed = true; mission = MissionState::ARMED_IDLE; neutralControls(true);
    jsonReply(true, "ARM komutu etkin; arac durumunu gozle dogrulayin");
  });
  server.on("/api/start", HTTP_POST, [] {
    lastControlMs = millis();
    if (!armed) return jsonReply(false, "Once ARM gerekli");
    if (digitalRead(ARM_PERMIT_PIN) != LOW) return jsonReply(false, "ARM izni kapali");
    enterState(MissionState::FORWARD); jsonReply(true, "Kare gorevi basladi");
  });
  server.on("/api/abort", HTTP_POST, [] {
    lastControlMs = millis(); mission = MissionState::ARMED_IDLE; neutralControls(armed);
    Serial.println("[GOREV] Operator durdurdu; havada notr/armed bekleme");
    jsonReply(true, "Gorev durdu; kontrol notr, arac ARM durumunda");
  });
  server.on("/api/disarm", HTTP_POST, [] {
    lastControlMs = millis(); abortAndDisarm("Operator acil DISARM"); jsonReply(true, "Acil DISARM gonderiliyor");
  });
  server.onNotFound([] { server.send(404, "application/json", "{\"ok\":false,\"message\":\"Bulunamadi\"}"); });
  server.begin();
}

void setup() {
  Serial.begin(115200);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(ARM_PERMIT_PIN, INPUT_PULLUP);
  neutralControls(false);
  CrsfSerial.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN, false);
  frameWindowMs = millis();

  // AG: AP her zaman acik kalir (saha yedegi). STA_SSID doluysa ayni anda
  // mevcut aga da katilinir -> operator dizustu tek agda kalip hem
  // gcs_server'a hem ESP32'ye erisebilir. Bkz. dosya basindaki gerekce.
  staEnabled = (STA_SSID != nullptr && STA_SSID[0] != '\0');
  WiFi.mode(staEnabled ? WIFI_AP_STA : WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("[AG] AP hazir: http://%s (SSID %s)\n",
                WiFi.softAPIP().toString().c_str(), AP_SSID);

  if (staEnabled) {
    WiFi.begin(STA_SSID, STA_PASSWORD);
    lastStaTryMs = millis();
    Serial.printf("[AG] STA baglaniyor: %s\n", STA_SSID);
    // Kisa, BLOKLAYICI OLMAYAN bekleme: baglanamazsa kurulum yine de surer,
    // loop() icindeki yeniden deneme devrede kalir. CRSF uretimi gecikmesin.
    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; ++i) delay(100);
    if (WiFi.status() == WL_CONNECTED)
      Serial.printf("[AG] STA bagli: http://%s (RSSI %d dBm)\n",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
    else
      Serial.println("[AG] STA henuz baglanmadi - AP uzerinden erisim surer, yeniden denenecek");
  }

  setupHttp();
  Serial.println("AVCI GCS hazir.");
}

void loop() {
  server.handleClient();
  parseTelemetry();
  updateMission();

  if (digitalRead(ESTOP_PIN) == LOW && (armed || mission != MissionState::ABORTED)) abortAndDisarm("Fiziksel E-STOP");
  if (armed && digitalRead(ARM_PERMIT_PIN) != LOW) abortAndDisarm("ARM izin anahtari acildi");
  if (missionActive() && millis() - lastControlMs > COMMAND_WATCHDOG_MS) {
    mission = MissionState::ARMED_IDLE;
    neutralControls(true);
    Serial.println("[EMNIYET] Operator heartbeat kaybi; havada notr/armed bekleme");
  }

  // STA baglantisi koptuysa periyodik yeniden dene. AP hicbir kosulda
  // kapanmaz; bu yalnizca altyapi agina yeniden katilma denemesidir.
  if (staEnabled && WiFi.status() != WL_CONNECTED &&
      millis() - lastStaTryMs > STA_RETRY_MS) {
    lastStaTryMs = millis();
    WiFi.begin(STA_SSID, STA_PASSWORD);
    Serial.println("[AG] STA yeniden baglanma denemesi");
  }

  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastFrameUs) >= FRAME_PERIOD_US) {
    // ZAMANLAMA DUZELTMESI: eskiden kosulsuz "lastFrameUs += FRAME_PERIOD_US"
    // yapiliyordu. loop() bir HTTP istegi yuzunden ornegin 200 ms takilirsa
    // birikmis 10 kare arka arkaya PATLIYORDU (link uzerinde ani yigilma,
    // 50 Hz sozlesmesinin ihlali). Artik iki periyottan fazla geride
    // kalinmissa faz yeniden senkronlanir ve kacan kareler SAYILIR; boylece
    // gecikme gizlenmek yerine measuredFrameHz uzerinden gorunur olur.
    const uint32_t behind = nowUs - lastFrameUs;
    if (behind > FRAME_PERIOD_US * 2) {
      frameSkips += behind / FRAME_PERIOD_US;
      lastFrameUs = nowUs;                  // faz sifirla, patlama yok
    } else {
      lastFrameUs += FRAME_PERIOD_US;       // normal seyir: faz korunur
    }
    sendRcFrame();
  }
  delay(1);
}
