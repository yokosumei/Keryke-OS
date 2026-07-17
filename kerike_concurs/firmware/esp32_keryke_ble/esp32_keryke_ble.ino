/**
 * KERYKE v3 — Nod de achiziție ESP32-S3 cu protocol binar peste BLE SECURIZAT
 * ============================================================================
 * Variantă Bluetooth a `esp32_keryke_tcp`: ACELAȘI protocol binar, dar transportul
 * este BLE (Bluetooth Low Energy) în locul TCP/WiFi. ESP32-S3 NU are Bluetooth
 * Classic (fără SPP/RFCOMM) — de aceea se folosește BLE GATT.
 *
 * SECURITATE (cerința „conexiune sigură cu parolă"):
 *   - Pairing cu PASSKEY FIX de 6 cifre (LE Secure Connections + MITM + Bonding).
 *     Centralul (Pi/telefon) TREBUIE să introducă PIN-ul corect; altfel pairing-ul
 *     eșuează și legătura se închide. => „nu poate fi împerecheat decât cu anumite
 *     dispozitive" (cele care cunosc PIN-ul).
 *   - Caracteristicile GATT cer link CRIPTAT + AUTENTIFICAT (ENC_MITM): fără pairing
 *     reușit nu curge niciun octet de protocol.
 *   - BONDING: cheile se salvează în NVS; dispozitivele împerecheate rămân
 *     recunoscute între reset-uri (nu mai cer PIN a doua oară). Un dispozitiv nou tot
 *     trebuie să treacă prin pairing cu PIN => „doar cine știe parola" se poate lega.
 *
 * Arhitectură (identică cu varianta TCP — decuplarea achiziției de rețea):
 *   - Core 1 (loop Arduino): achiziție senzori (MPU6050 + VL53L1X), integrare yaw,
 *     detecție fază de balans (gait-sync), control servo + motor vibrații. NU
 *     depinde de starea Bluetooth.
 *   - Stiva BLE (Bluedroid, tasks proprii pe core 0): advertising, pairing/bonding,
 *     GATT server, parser de protocol în callback-ul de WRITE, transmisie prin NOTIFY.
 *   - Task „ble_push" (core 0): emisie periodică nesolicitată (RSP_TELEMETRY_PUSH, 5 s).
 *   - Sincronizare: snapshot telemetrie protejat de portMUX (secțiune critică scurtă),
 *     comenzi actuator (servo/vibrații) prin coadă FreeRTOS.
 *
 * Pornire NERESTRICȚIONATĂ, dar SEMNALIZATĂ (senzori):
 *   - Eșecul de inițializare al MPU6050/VL53L1X NU blochează boot-ul: aplicația
 *     pornește degradat (fail-safe: „fără date ≠ liber"), BLE rămâne disponibil
 *     pentru diagnoză, iar senzorul e reîncercat periodic (auto-recovery).
 *   - Semnalizare pe LED-ul integrat al plăcii (LED_BUILTIN — pe DevKitC-1 este
 *     WS2812 RGB, expus de core drept RGB_BUILTIN), non-blocant, din loop():
 *       albastru continuu        = setup() în curs
 *       verde, puls scurt / 3 s  = ambii senzori OK (heartbeat discret)
 *       roșu, clipire 300 ms     = VL53L1X (ToF) indisponibil — critic pt. siguranță
 *       galben, clipire 300 ms   = MPU6050 (IMU) indisponibil
 *       roșu/galben alternat     = ambii senzori indisponibili
 *   - Semnal haptic la boot (utilizatorul final e nevăzător, nu vede LED-ul):
 *     3 pulsuri lungi = ToF picat; 2 pulsuri lungi = IMU picat (grupuri cu pauză).
 *
 * Protocol (cadru, IDENTIC cu varianta TCP): STX | OPCODE | LEN(uint16 LE) | PAYLOAD | ETX
 *   STX = 0x02, ETX = 0x03
 *   Un cadru complet încape într-o singură notificare BLE (MTU negociat 517).
 *
 * Opcode-uri (identice):
 *   0x10 CMD_GET_TELEMETRY   (Central -> ESP, LEN=0)
 *   0x11 RSP_TELEMETRY       (ESP -> Central, LEN=50) răspuns SOLICITAT
 *   0x12 RSP_TELEMETRY_PUSH  (ESP -> Central, LEN=50) emisie NESOLICITATĂ periodică (5 s)
 *   0x20 CMD_MOTOR_ROTATE    (Central -> ESP, LEN=2, int16 LE grade relative servo)
 *   0x22 CMD_SERVO_POSE      (Central -> ESP, LEN=2: poza[0/1/2] + mod[0=gait,1=imediat])
 *   0x23 CMD_VIBRATE         (Central -> ESP, LEN=3: intensitate[0..255] + durata_ms[uint16 LE])
 *   0x06 RSP_ACK             (ESP -> Central, LEN=3: opcode ecou + int16 valoare)
 *   0x15 RSP_NACK            (ESP -> Central, LEN=2: opcode ecou + cod eroare)
 *
 * GATT:
 *   Serviciu KERYKE                5f6d0001-9b5a-4c3d-8e2f-1a2b3c4d5e6f
 *     Caracteristică TX (NOTIFY)   5f6d0002-9b5a-4c3d-8e2f-1a2b3c4d5e6f  (ESP -> Central)
 *     Caracteristică RX (WRITE)    5f6d0003-9b5a-4c3d-8e2f-1a2b3c4d5e6f  (Central -> ESP)
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLESecurity.h>

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_VL53L1X.h"
#include <math.h>

// ============== ↓ CONFIGURARE BLUETOOTH ↓ ==============
static const char*    BLE_DEVICE_NAME = "KERYKE-ESP32";   // nume la advertising (vizibil la scanare)
// PAROLA (passkey de împerechere): fix, 6 cifre (000000..999999). Centralul TREBUIE
// să introducă exact această valoare la pairing. Schimb-o pentru dispozitivul tău.
static const uint32_t BLE_PASSKEY     = 123456;
// ============== ↑ CONFIGURARE BLUETOOTH ↑ ==============

// UUID-uri GATT (128-bit, fixe)
#define KERYKE_SERVICE_UUID "5f6d0001-9b5a-4c3d-8e2f-1a2b3c4d5e6f"
#define KERYKE_TX_UUID      "5f6d0002-9b5a-4c3d-8e2f-1a2b3c4d5e6f"  // NOTIFY: ESP -> Central
#define KERYKE_RX_UUID      "5f6d0003-9b5a-4c3d-8e2f-1a2b3c4d5e6f"  // WRITE : Central -> ESP

// ---------------- Hardware ----------------
TwoWire I2CMPU = TwoWire(0);
TwoWire I2CTOF = TwoWire(1);

#define MPU_SDA   41
#define MPU_SCL   42
#define TOF_SDA   39
#define TOF_SCL   40
#define IRQ_PIN   -1
#define XSHUT_PIN  1

// Servo (LEDC) — adaptați la actuatorul real (stepper: vezi applyMotorRotation)
#define SERVO_PIN      21
#define SERVO_LEDC_CH  0
#define SERVO_FREQ_HZ  50
#define SERVO_RES_BITS 14
#define SERVO_US_MIN   500
#define SERVO_US_MAX   2500

// Motor vibrații (LEDC PWM) — GPIO18 (non-strapping; poarta MOSFET)
#define VIBRATION_PIN      18
#define VIBRATION_LEDC_CH  1
#define VIBRATION_FREQ_HZ  5000
#define VIBRATION_RES_BITS 8         // 0..255

// LED de stare — LED-ul INTEGRAT al plăcii (pe ESP32-S3 DevKitC-1, LED_BUILTIN este
// un WS2812 RGB adresabil pe GPIO48; core-ul îl expune și ca RGB_BUILTIN, comandat
// cu rgbLedWrite() — culori). Semnalizare NON-BLOCANTĂ a stării senzorilor.
#ifdef RGB_BUILTIN
  #define STATUS_LED_PIN  RGB_BUILTIN   // WS2812 integrat (culori)
#else
  #define STATUS_LED_PIN  LED_BUILTIN   // fallback: LED simplu (doar on/off)
#endif
#define STATUS_LED_BRIGHTNESS 40        // 0..255 (WS2812 e orbitor la maxim)

// ============== ↓ CONFIGURARE LOGGING (la compilare) ↓ ==============
// LOG_ENABLED 0 => nicio ieșire pe consolă (toate macro-urile se compilează în gol).
// LOG_ENABLED 1 => logurile emise din contextul de TIMP REAL (core 1: loop/acqTask)
//   NU scriu direct pe UART (o linie [TELE] de ~230 caractere blochează ~20 ms la
//   115200); ele se pun într-o coadă și sunt tipărite de un task dedicat pe core 0
//   (logTask). Logurile din setup() și din task-urile de pe core 0 scriu direct
//   (acolo blocarea pe UART nu afectează bucla de timp real).
// LOG_HEXDUMP 1 => dump hexa [TX]/[RX] al cadrelor de protocol (diagnostic;
//   rulează în context BLE pe core 0). Cere LOG_ENABLED 1.
#define LOG_ENABLED  0
#define LOG_HEXDUMP  0
// ============== ↑ CONFIGURARE LOGGING (la compilare) ↑ ==============

// Poziții servo (grade) + temporizare gait-sync
#define SERVO_CENTER   90
#define SERVO_LEFT     45
#define SERVO_RIGHT    135
#define SERVO_HOLD_MS  2000          // menține poza, apoi revine la centru

// Detecție fază de balans (gait) din IMU
#define GAIT_WINDOW_SIZE      10
#define GAIT_ACCEL_DIP        0.5f   // scădere accel sub media ferestrei
#define GAIT_GYRO_PEAK        0.3f   // vârf |gyroZ| (rad/s)
#define MIN_SWING_INTERVAL_MS 500

// Vibrații: rampă lină (pentru CMD_VIBRATE)
#define VIBRATION_RAMP_STEP  8

// Ritm cardiac haptic la obstacol — perioada scade cu distanța (mai aproape = mai rapid)
#define HB_DIST_FAST_MM      500    // < 0.5 m  -> ritm alert (rapid)
#define HB_DIST_MEDIUM_MM    1000   // < 1.0 m  -> ritm mediu
#define HB_DIST_SLOW_MM      1500   // < 1.5 m  -> ritm lent  (>= 1.5 m => fără vibrații)
#define HB_PERIOD_FAST_MS    400    // ~150 bpm
#define HB_PERIOD_MEDIUM_MS  750    // ~80 bpm
#define HB_PERIOD_SLOW_MS    1100   // ~55 bpm
#define HB_LUB_MS    60             // durata bătăii principale "lub"
#define HB_GAP_MS    100            // pauza între lub și dub
#define HB_DUB_MS    50             // durata celei de-a doua bătăi "dub"
#define HB_LUB_LEVEL 230            // intensitate PWM "lub"
#define HB_DUB_LEVEL 160            // intensitate PWM "dub"

// VL53L1X — config + filtrare (aplicație critică: ghidare nevăzător)
#define TOF_DISTANCE_MODE     2     // 1 = SHORT (<=1.3m, robust la lumina), 2 = LONG (<=~4m)
#define TOF_TIMING_BUDGET_MS  50    // LONG cere buget mai mare (50-100ms) pt. rază/acuratețe
#define TOF_INTERMEAS_MS      55    // perioadă între măsurători (>= timing budget)
#define TOF_MEDIAN_N          5     // fereastra filtrului median (probe VALIDE)
#define TOF_STALE_MS          300   // fără probă validă atâta timp => distanță NECUNOSCUTĂ
#define TOF_RECOVER_MS        2000  // dacă senzorul e mut, reîncearcă reinițializarea la acest interval
#define TOF_POLL_MS           10    // interogarea dataReady() e o tranzacție I2C reală —
                                    // limitată la 10 ms (măsurătoare nouă apare oricum la TOF_INTERMEAS_MS)

// MPU6050 — dacă IMU e absent/mut, reîncearcă reinițializarea la acest interval
#define MPU_RECOVER_MS        5000

Adafruit_MPU6050  mpu;
Adafruit_VL53L1X  vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

// ---------------- Protocol (IDENTIC cu varianta TCP) ----------------
static constexpr uint8_t  P_STX = 0x02;
static constexpr uint8_t  P_ETX = 0x03;
static constexpr uint16_t P_MAX_PAYLOAD = 256;
static constexpr uint32_t P_INTERBYTE_TIMEOUT_MS = 500;
static constexpr uint32_t P_TELEMETRY_PUSH_MS    = 300;//5000;  // emisie periodică nesolicitată catre Raspberry pi

enum Opcode : uint8_t {
  CMD_GET_TELEMETRY  = 0x10,
  RSP_TELEMETRY      = 0x11,   // răspuns solicitat
  RSP_TELEMETRY_PUSH = 0x12,   // emisie periodică nesolicitată (payload identic)
  CMD_MOTOR_ROTATE   = 0x20,   // rotație relativă servo (int16 grade)
  CMD_SERVO_POSE     = 0x22,   // poză servo (centru/stânga/dreapta) + mod gait/imediat
  CMD_VIBRATE        = 0x23,   // impuls motor vibrații (intensitate + durată)
  RSP_ACK            = 0x06,
  RSP_NACK           = 0x15,
};

enum NackCode : uint8_t {
  NACK_BAD_LENGTH     = 0x01,
  NACK_UNKNOWN_OPCODE = 0x02,
  NACK_MOTOR_BUSY     = 0x03,
};

#pragma pack(push, 1)
struct TelemetryPacket {          // 50 octeți, little-endian nativ
  uint32_t counter;               //  4
  uint8_t  angle;                 //  1  [0..180]  (unghi derivat din yaw)
  int16_t  distance_mm;           //  2
  float    pitch, roll, yaw;      // 12  [rad]
  float    ax, ay, az;            // 12  [m/s^2]
  float    gx, gy, gz;            // 12  [rad/s]
  float    temp_c;                //  4  [°C]
  uint8_t  swing;                 //  1  fază de balans detectată (0/1)
  uint8_t  servo;                 //  1  unghi servo curent [0..180]
  uint8_t  vibration;             //  1  nivel PWM vibrații curent [0..255]
};
#pragma pack(pop)
static_assert(sizeof(TelemetryPacket) == 50, "TelemetryPacket trebuie sa aiba 50 de octeti");

// Comandă actuator (BLE -> core 1). Tip + doi parametri generici.
enum ActCmdType : uint8_t { ACT_ROTATE_REL = 0, ACT_SERVO_POSE = 1, ACT_VIBRATE = 2 };
struct ActuatorCommand {
  ActCmdType type;
  int16_t    a;   // ROTATE: grade;  POSE: poză(0/1/2);  VIBRATE: intensitate
  int16_t    b;   // POSE: mod(0=gait,1=imediat);  VIBRATE: durată ms
};

// Zone de distanță pentru ritmul cardiac haptic (declarat aici ca tipul să fie
// cunoscut de prototipurile auto-generate ale funcțiilor heartbeat).
enum HbZone : uint8_t { ZONE_NONE = 0, ZONE_SLOW, ZONE_MEDIUM, ZONE_FAST };

// ---------------- Stare partajată ----------------
static portMUX_TYPE     g_snapMux = portMUX_INITIALIZER_UNLOCKED;
static TelemetryPacket  g_snapshot = {};        // scris de core 1, citit de stiva BLE
static QueueHandle_t    g_actQueue = nullptr;   // comenzi actuator, BLE -> core 1

// Stare senzori: false = absent/mut (la init sau la runtime). Disciplina de acces
// pe magistralele I2C (fără lock): core 1 atinge senzorul DOAR cât flag-ul lui e
// true; task-ul de întreținere (maintTask, core 0) îl atinge DOAR cât e false și
// ridică flag-ul ULTIMUL, după reconfigurarea completă — de aici volatile.
static volatile bool g_mpuOk = false;
static volatile bool g_tofOk = false;
// Ultimul moment în care ToF a semnalat dataReady() ("viu" pe I2C); folosit la
// detecția senzorului mut. Scriere de 32 de biți aliniată => atomică pe Xtensa.
static volatile unsigned long g_lastTofEventMs = 0;

// ---------------- Stare BLE ----------------
// Securitatea (pairing cu PIN + criptare) este IMPUSĂ de permisiunile GATT ENC_MITM
// pe caracteristici: fără pairing reușit, stiva respinge scrierile pe RX și abonarea
// la notificările TX. Aici urmărim doar starea conexiunii pentru gating-ul emisiei.
static BLECharacteristic* g_txChar    = nullptr;   // NOTIFY: ESP -> Central
static BLE2902*           g_cccd      = nullptr;    // descriptorul CCCD al TX (stare abonare)
static SemaphoreHandle_t  g_bleTxMux  = nullptr;    // serializează notificările
static volatile bool      g_connected = false;      // sesiune BLE activă
static volatile bool      g_subscribed = false;     // clientul s-a abonat la TX (cere link criptat)

// ---------------- Parser cadru (automat finit) — IDENTIC cu varianta TCP ----------------
class FrameParser {
public:
  uint8_t  opcode = 0;
  uint16_t length = 0;
  uint8_t  payload[P_MAX_PAYLOAD];
  uint8_t  raw[4 + P_MAX_PAYLOAD + 1];   // cadrul brut (STX..ETX) pentru dump hexa
  uint16_t rawLen = 0;

  void reset() { state_ = S_STX; }

  // Returnează true când un cadru complet și valid a fost recepționat.
  bool feed(uint8_t b) {
    const uint32_t now = millis();
    if (state_ != S_STX && (now - lastByteMs_) > P_INTERBYTE_TIMEOUT_MS) {
      state_ = S_STX;               // resincronizare pe timeout inter-octet
    }
    lastByteMs_ = now;

    if (state_ == S_STX) {
      if (b == P_STX) { rawLen = 0; raw[rawLen++] = b; }
    } else if (rawLen < sizeof(raw)) {
      raw[rawLen++] = b;
    }

    switch (state_) {
      case S_STX:
        if (b == P_STX) state_ = S_OPCODE;
        break;
      case S_OPCODE:
        opcode = b;
        state_ = S_LEN_L;
        break;
      case S_LEN_L:
        length = b;
        state_ = S_LEN_H;
        break;
      case S_LEN_H:
        length |= static_cast<uint16_t>(b) << 8;
        if (length > P_MAX_PAYLOAD) { state_ = S_STX; break; }
        idx_   = 0;
        state_ = (length > 0) ? S_PAYLOAD : S_ETX;
        break;
      case S_PAYLOAD:
        payload[idx_++] = b;
        if (idx_ >= length) state_ = S_ETX;
        break;
      case S_ETX:
        state_ = S_STX;
        return (b == P_ETX);        // ETX invalid => cadru respins, resincronizare
    }
    return false;
  }

private:
  enum State : uint8_t { S_STX, S_OPCODE, S_LEN_L, S_LEN_H, S_PAYLOAD, S_ETX };
  State    state_      = S_STX;
  uint16_t idx_        = 0;
  uint32_t lastByteMs_ = 0;
};

// ---------------- Logging (configurabil la compilare, vezi LOG_ENABLED) ----------------
// ATENȚIE (gotcha .ino): blocul stă DUPĂ clasa FrameParser — logRealtime/logTask
// sunt primele funcții din sketch, iar prototipurile auto-generate de Arduino se
// inserează chiar înaintea lor; orice tip folosit în semnături trebuie să fie
// deja declarat mai sus.
#if LOG_ENABLED
#define LOG_MSG_MAX   240   // o linie [TELE] completă încape
#define LOG_QUEUE_LEN 16

struct LogMsg { char text[LOG_MSG_MAX]; };
static QueueHandle_t g_logQueue = nullptr;

// Emitere din contextul de timp real (core 1): formatare + coadă, fără UART.
// Coadă plină => mesajul se PIERDE (preferăm pierderea unui log în locul
// blocării buclei de timp real pe UART).
static void logRealtime(const char* fmt, ...) {
  if (!g_logQueue) return;
  LogMsg m;
  va_list args;
  va_start(args, fmt);
  vsnprintf(m.text, sizeof(m.text), fmt, args);
  va_end(args);
  xQueueSend(g_logQueue, &m, 0);
}

// Task pe core 0: golește coada și scrie pe UART (aici blocarea e inofensivă).
static void logTask(void*) {
  LogMsg m;
  for (;;) {
    if (xQueueReceive(g_logQueue, &m, portMAX_DELAY) == pdTRUE) Serial.print(m.text);
  }
}

#define LOG_RT(...)     logRealtime(__VA_ARGS__)     // din core 1 (loop / acqTask)
#define LOG_DIRECT(...) Serial.printf(__VA_ARGS__)   // din setup() sau core 0
#else
#define LOG_RT(...)     do {} while (0)
#define LOG_DIRECT(...) do {} while (0)
#endif

// ---------------- Depanare: dump hexazecimal cadre (doar cu LOG_HEXDUMP) ----------------
// Apelat exclusiv din contexte de pe core 0 (callback BLE / task push).
static void hexDumpFrame(const char* dir, const uint8_t* data, uint16_t len) {
#if LOG_ENABLED && LOG_HEXDUMP
  Serial.printf("%s cadru [%u octeti]:", dir, len);
  for (uint16_t i = 0; i < len; ++i) Serial.printf(" %02X", data[i]);
  Serial.println();
#else
  (void)dir; (void)data; (void)len;
#endif
}

// ---------------- Serializare / transmisie prin NOTIFY ----------------
// Trimite un cadru complet (STX..ETX) ca o singură notificare BLE. Fără sesiune
// activă (sau link necriptat) => nu trimite nimic. MTU 517 => cadrul (max 55 la
// telemetrie) încape într-o singură notificare.
static bool bleSendFrame(uint8_t opcode, const uint8_t* payload, uint16_t len) {
  if (!g_connected || !g_txChar || len > P_MAX_PAYLOAD) return false;
  uint8_t buf[1 + 1 + 2 + P_MAX_PAYLOAD + 1];
  uint16_t i = 0;
  buf[i++] = P_STX;
  buf[i++] = opcode;
  buf[i++] = static_cast<uint8_t>(len & 0xFF);
  buf[i++] = static_cast<uint8_t>(len >> 8);
  if (len) { memcpy(&buf[i], payload, len); i += len; }
  buf[i++] = P_ETX;

  if (g_bleTxMux) xSemaphoreTake(g_bleTxMux, portMAX_DELAY);
  g_txChar->setValue(buf, i);
  g_txChar->notify();
  if (g_bleTxMux) xSemaphoreGive(g_bleTxMux);

  hexDumpFrame("[TX]", buf, i);
  return true;
}

static void sendAck(uint8_t echoedOpcode, int16_t value) {
  uint8_t p[3] = { echoedOpcode,
                   static_cast<uint8_t>(value & 0xFF),
                   static_cast<uint8_t>((value >> 8) & 0xFF) };
  bleSendFrame(RSP_ACK, p, sizeof(p));
}

static void sendNack(uint8_t echoedOpcode, uint8_t code) {
  uint8_t p[2] = { echoedOpcode, code };
  bleSendFrame(RSP_NACK, p, sizeof(p));
}

// Trimite un cadru de telemetrie cu instantaneul curent. Opcode-ul distinge sursa:
// RSP_TELEMETRY (răspuns solicitat) vs RSP_TELEMETRY_PUSH (push periodic).
static bool sendTelemetry(uint8_t opcode = RSP_TELEMETRY) {
  TelemetryPacket snap;
  taskENTER_CRITICAL(&g_snapMux);
  snap = g_snapshot;                         // copie atomică a instantaneului
  taskEXIT_CRITICAL(&g_snapMux);
  return bleSendFrame(opcode,
                      reinterpret_cast<const uint8_t*>(&snap), sizeof(snap));
}

// ---------------- Dispecer de comenzi ----------------
static void handleFrame(const FrameParser& f) {
  switch (f.opcode) {

    case CMD_GET_TELEMETRY: {
      if (f.length != 0) { sendNack(f.opcode, NACK_BAD_LENGTH); return; }
      sendTelemetry();
      break;
    }

    case CMD_MOTOR_ROTATE: {
      if (f.length != 2) { sendNack(f.opcode, NACK_BAD_LENGTH); return; }
      int16_t deg;
      memcpy(&deg, f.payload, sizeof(deg));    // int16 little-endian
      ActuatorCommand cmd{ ACT_ROTATE_REL, deg, 0 };
      if (xQueueSend(g_actQueue, &cmd, 0) == pdTRUE) sendAck(f.opcode, deg);
      else                                           sendNack(f.opcode, NACK_MOTOR_BUSY);
      break;
    }

    case CMD_SERVO_POSE: {
      if (f.length != 2) { sendNack(f.opcode, NACK_BAD_LENGTH); return; }
      const uint8_t pose = f.payload[0];       // 0=centru, 1=stânga, 2=dreapta
      const uint8_t mode = f.payload[1];       // 0=gait-sync, 1=imediat
      ActuatorCommand cmd{ ACT_SERVO_POSE, (int16_t)pose, (int16_t)mode };
      if (xQueueSend(g_actQueue, &cmd, 0) == pdTRUE) sendAck(f.opcode, pose);
      else                                           sendNack(f.opcode, NACK_MOTOR_BUSY);
      break;
    }

    case CMD_VIBRATE: {
      if (f.length != 3) { sendNack(f.opcode, NACK_BAD_LENGTH); return; }
      const uint8_t intensity = f.payload[0];  // 0..255
      uint16_t durationMs;
      memcpy(&durationMs, &f.payload[1], sizeof(durationMs));  // uint16 little-endian
      ActuatorCommand cmd{ ACT_VIBRATE, (int16_t)intensity, (int16_t)durationMs };
      if (xQueueSend(g_actQueue, &cmd, 0) == pdTRUE) sendAck(f.opcode, intensity);
      else                                           sendNack(f.opcode, NACK_MOTOR_BUSY);
      break;
    }

    default:
      sendNack(f.opcode, NACK_UNKNOWN_OPCODE);
  }
}

// ---------------- Callback-uri BLE: server (conectare/deconectare) ----------------
// Semnături portabile (1-argument): librăria le apelează pe acestea (BLEServer.cpp
// invocă atât varianta 1-arg cât și cea Bluedroid), deci nu depindem de tipuri IDF.
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* srv) override {
    g_connected  = true;
    g_subscribed = false;             // se abonează la TX abia după pairing (PIN + criptare)
    LOG_DIRECT("[BLE] Conectat (conn_id=%u) — pairing cu PIN + criptare necesare "
               "inainte de orice schimb.\n", srv->getConnId());
    // NU reporni advertising: o singură sesiune activă (ca serverul TCP listen(1)).
  }

  void onDisconnect(BLEServer* srv) override {
    g_connected  = false;
    g_subscribed = false;
    LOG_DIRECT("[BLE] Deconectat — repornesc advertising.\n");
    srv->getAdvertising()->start();   // redevine descoperibil pentru reconectare
  }
};

// ---------------- Callback-uri BLE: RX (comenzi de la central) ----------------
// Permisiunea GATT ENC_MITM pe caracteristica RX garantează că onWrite se declanșează
// DOAR pe un link criptat + autentificat (pairing cu PIN reușit). Fără pairing, stiva
// respinge scrierea înainte de a ajunge aici — deci nu mai e nevoie de verificări.
class RxCallbacks : public BLECharacteristicCallbacks {
public:
  void onWrite(BLECharacteristic* chr) override {
    const uint8_t* data = chr->getData();
    const size_t   n    = chr->getLength();
    for (size_t i = 0; i < n; ++i) {
      if (parser_.feed(data[i])) {
        hexDumpFrame("[RX]", parser_.raw, parser_.rawLen);
        handleFrame(parser_);
      }
    }
  }
private:
  FrameParser parser_;
};

// ---------------- Task push telemetrie (nesolicitat, 5 s) ----------------
// Emisia se face doar când clientul e abonat la notificări (CCCD are permisiune
// ENC_MITM => abonarea implică pairing reușit). Astfel push-ul confirmă și securizarea.
static void blePushTask(void*) {
  uint32_t seq = 0;
  bool mtuWarned = false;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(P_TELEMETRY_PUSH_MS));
    // Valoarea CCCD persistă în stiva locală și după deconectare — abonarea e
    // relevantă doar pe o conexiune activă (altfel logul ar arăta fals "abonat=DA").
    g_subscribed = (g_connected && g_cccd && g_cccd->getNotifications());

    // Gardă MTU: notify() TRUNCHIAZĂ tăcut la MTU-3 al peer-ului (BLECharacteristic.cpp).
    // Cadrul de telemetrie are 55 octeți => centralul trebuie să fi negociat MTU >= 58,
    // altfel TOATE cadrele ar sosi ciuntite. Semnalăm explicit ca defectul să fie vizibil.
    if (g_connected) {
      BLEServer* srv = BLEDevice::getServer();
      const uint16_t peerMtu = srv ? srv->getPeerMTU(srv->getConnId()) : 0;
      if (peerMtu > 0 && peerMtu < 58 && !mtuWarned) {
        mtuWarned = true;
        LOG_DIRECT("[BLE] AVERTISMENT: MTU negociat=%u (<58) — notificarile de "
                   "telemetrie vor fi TRUNCHIATE. Clientul trebuie sa ceara MTU mai mare "
                   "(bleak/BlueZ si Windows o fac implicit).\n", peerMtu);
      }
    } else {
      mtuWarned = false;                        // re-verifică la următoarea conexiune
    }
    const bool ok = (g_connected && g_subscribed);
    const char* emitStare = !ok ? "OMIS (fara abonare securizata)"
                                : (sendTelemetry(RSP_TELEMETRY_PUSH) ? "OK (55 octeti)"
                                                                     : "ESUAT (notify)");
    LOG_DIRECT("[TELE] tentativa #%lu | BLE=%s | abonat(securizat)=%s | emisie=%s\n",
               (unsigned long)seq,
               g_connected ? "CONECTAT" : "DECONECTAT",
               g_subscribed ? "DA" : "NU", emitStare);
    seq++;
  }
}

// ---------------- Inițializare BLE securizat ----------------
static void setupBLE() {
  g_bleTxMux = xSemaphoreCreateMutex();

  LOG_DIRECT("[BOOT] pornesc stiva BLE (BLEDevice::init)...\n");
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(517);            // cadrul de telemetrie (55B) încape într-o notificare

  // --- Parametri de securitate: LE Secure Connections + MITM + Bonding, passkey static ---
  // IO capability „DisplayOnly": ESP-ul „afișează" (fix), centralul INTRODUCE PIN-ul.
  // Passkey-ul static + IO DisplayOnly + MITM sunt suficiente ca stiva să ceară PIN-ul;
  // enforcement-ul efectiv vine din permisiunile ENC_MITM ale caracteristicilor.
  BLESecurity* sec = new BLESecurity();
  sec->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);  // Secure Connections + MITM + Bond
  sec->setCapability(ESP_IO_CAP_OUT);                        // DisplayOnly => centralul tastează PIN-ul
  sec->setKeySize(16);
  sec->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  sec->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  sec->setPassKey(true, BLE_PASSKEY);                        // passkey STATIC (parola fixă)

  // --- Server + serviciu GATT ---
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(KERYKE_SERVICE_UUID);

  // TX (NOTIFY): ESP -> Central. Acces doar pe link criptat+autentificat.
  g_txChar = service->createCharacteristic(
      KERYKE_TX_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  g_txChar->setAccessPermissions(ESP_GATT_PERM_READ_ENC_MITM);
  g_cccd = new BLE2902();            // CCCD: abonarea cere link criptat (semnal de securizare)
  g_cccd->setAccessPermissions(ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM);
  g_txChar->addDescriptor(g_cccd);

  // RX (WRITE): Central -> ESP. Scrierea cere link criptat+autentificat (forțează pairing).
  BLECharacteristic* rxChar = service->createCharacteristic(
      KERYKE_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxChar->setAccessPermissions(ESP_GATT_PERM_WRITE_ENC_MITM);
  rxChar->setCallbacks(new RxCallbacks());

  service->start();

  // --- Advertising ---
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(KERYKE_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);       // intervale prietenoase cu iOS
  adv->setMinPreferred(0x12);

  BLEDevice::startAdvertising();
  LOG_DIRECT("[BLE] Advertising ca \"%s\". Passkey de imperechere: %06u\n",
             BLE_DEVICE_NAME, BLE_PASSKEY);
  LOG_DIRECT("[BLE] Conexiunea cere pairing cu PIN (MITM) + criptare inainte de "
             "orice schimb de date. Dispozitivele imperecheate raman bonded.\n");
}

// ==================================================================================
// De aici în jos: achiziție / actuatori / algoritmi — IDENTIC cu varianta TCP.
// ==================================================================================

// ---------------- Actuator (rulează pe core 1) ----------------
static int currentServoAngle = SERVO_CENTER;

// --- stare servo (poze gait-sync) ---
static int  targetServoAngle    = SERVO_CENTER;
static bool servoActionPending  = false;
static unsigned long servoReturnTime = 0;
static bool inSwingPhase        = false;

// --- stare motor vibrații ---
static int vibrationTarget      = 0;
static int vibrationCurrent     = 0;
static unsigned long vibrationEndTime    = 0;
static unsigned long lastVibrationUpdate = 0;

// --- stare detecție gait (fereastră glisantă) ---
static float accelMagHistory[GAIT_WINDOW_SIZE];
static float gyroZHistory[GAIT_WINDOW_SIZE];
static int  gaitHistoryIdx      = 0;
static bool gaitHistoryFilled   = false;
static unsigned long lastSwingDetected = 0;

static void servoWriteAngle(int angle) {
  angle = constrain(angle, 0, 180);
  const uint32_t us = SERVO_US_MIN +
      (uint32_t)((SERVO_US_MAX - SERVO_US_MIN) * (angle / 180.0f));
  const uint32_t duty =
      (uint32_t)(((uint64_t)us << SERVO_RES_BITS) * SERVO_FREQ_HZ / 1000000ULL);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(SERVO_PIN, duty);        // 3.x: identificat prin pin
#else
  ledcWrite(SERVO_LEDC_CH, duty);    // 2.x: identificat prin canal
#endif
}

static int poseToAngle(uint8_t pose) {
  switch (pose) {
    case 1:  return SERVO_LEFT;
    case 2:  return SERVO_RIGHT;
    default: return SERVO_CENTER;
  }
}

static void moveServoTo(int angle, bool holdThenReturn) {
  currentServoAngle = constrain(angle, 0, 180);
  servoWriteAngle(currentServoAngle);
  servoReturnTime = holdThenReturn ? (millis() + SERVO_HOLD_MS) : 0;
}

static void commandServoPose(uint8_t pose, bool immediate) {
  targetServoAngle = poseToAngle(pose);
  if (immediate) {
    servoActionPending = false;
    moveServoTo(targetServoAngle, false);
    LOG_RT("[SERVO] pozitie imediata %d°\n", targetServoAngle);
  } else {
    servoActionPending = true;
    LOG_RT("[GAIT-SYNC] poza %d° programata (asteapta swing)\n", targetServoAngle);
  }
}

static void applyServoCommand() {
  if (!servoActionPending) {
    if (servoReturnTime && millis() > servoReturnTime && currentServoAngle != SERVO_CENTER) {
      moveServoTo(SERVO_CENTER, false);
      LOG_RT("[GAIT-SYNC] Servo revenit la centru\n");
    }
    return;
  }
  // Fără IMU nu există detecție de balans: degradare grațioasă — aplicare imediată,
  // altfel poza programată ar rămâne agățată la nesfârșit.
  if (inSwingPhase || !g_mpuOk) {
    moveServoTo(targetServoAngle, true);
    servoActionPending = false;
    LOG_RT("[GAIT-SYNC] Servo aplicat %s: %d°\n",
           g_mpuOk ? "in swing" : "imediat (IMU absent)", targetServoAngle);
  }
}

static void applyMotorRotation(int16_t deg) {
  moveServoTo(currentServoAngle + deg, false);
  LOG_RT("[MOTOR] Rotatie %+d° -> pozitie %d°\n", deg, currentServoAngle);
}

// --- Motor vibrații: scriere PWM + impuls cu durată + rampă lină ---
static void vibrationWrite(int level) {
  level = constrain(level, 0, 255);
  vibrationCurrent = level;
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(VIBRATION_PIN, level);
#else
  ledcWrite(VIBRATION_LEDC_CH, level);
#endif
}

static void triggerVibration(int intensity, int durationMs) {
  vibrationTarget  = constrain(intensity, 0, 255);
  vibrationEndTime = millis() + durationMs;
}

static void updateVibration() {
  const unsigned long now = millis();
  if (now - lastVibrationUpdate < 4) return;
  lastVibrationUpdate = now;

  if (vibrationEndTime && now > vibrationEndTime) vibrationTarget = 0;

  if (vibrationCurrent < vibrationTarget)
    vibrationCurrent = min(vibrationCurrent + VIBRATION_RAMP_STEP, vibrationTarget);
  else if (vibrationCurrent > vibrationTarget)
    vibrationCurrent = max(vibrationCurrent - VIBRATION_RAMP_STEP, vibrationTarget);
  vibrationWrite(vibrationCurrent);
}

// --- Ritm cardiac haptic la obstacol (mașină de stări non-blocantă "lub-dub") ---
static HbZone        hbZone       = ZONE_NONE;
static uint8_t       hbPhase      = 0;
static unsigned long hbPhaseStart = 0;
static uint32_t      hbPeriodMs   = 0;

static HbZone heartbeatZone(int16_t dist) {
  if (dist <= 0)                return ZONE_NONE;
  if (dist < HB_DIST_FAST_MM)   return ZONE_FAST;
  if (dist < HB_DIST_MEDIUM_MM) return ZONE_MEDIUM;
  if (dist < HB_DIST_SLOW_MM)   return ZONE_SLOW;
  return ZONE_NONE;
}

static uint32_t heartbeatPeriod(HbZone z) {
  switch (z) {
    case ZONE_FAST:   return HB_PERIOD_FAST_MS;
    case ZONE_MEDIUM: return HB_PERIOD_MEDIUM_MS;
    case ZONE_SLOW:   return HB_PERIOD_SLOW_MS;
    default:          return 0;
  }
}

static void updateHeartbeat(uint32_t periodMs, unsigned long now) {
  if (periodMs == 0) {
    if (hbPeriodMs != 0) { vibrationWrite(0); hbPeriodMs = 0; }
    return;
  }
  if (hbPeriodMs == 0) { hbPhase = 0; hbPhaseStart = now; }
  hbPeriodMs = periodMs;

  const unsigned long elapsed = now - hbPhaseStart;
  switch (hbPhase) {
    case 0:  vibrationWrite(HB_LUB_LEVEL); if (elapsed >= HB_LUB_MS) { hbPhase = 1; hbPhaseStart = now; } break;
    case 1:  vibrationWrite(0);            if (elapsed >= HB_GAP_MS) { hbPhase = 2; hbPhaseStart = now; } break;
    case 2:  vibrationWrite(HB_DUB_LEVEL); if (elapsed >= HB_DUB_MS) { hbPhase = 3; hbPhaseStart = now; } break;
    default: {
      vibrationWrite(0);
      const unsigned long beatSpan = HB_LUB_MS + HB_GAP_MS + HB_DUB_MS;
      const unsigned long restMs   = (periodMs > beatSpan) ? (periodMs - beatSpan) : 0;
      if (elapsed >= restMs) { hbPhase = 0; hbPhaseStart = now; }
      break;
    }
  }
}

// --- Detecție fază de balans (gait) din IMU ---
static bool detectSwingPhase(float accelMag, float gyroZ) {
  accelMagHistory[gaitHistoryIdx] = accelMag;
  gyroZHistory[gaitHistoryIdx]    = gyroZ;
  gaitHistoryIdx = (gaitHistoryIdx + 1) % GAIT_WINDOW_SIZE;
  if (gaitHistoryIdx == 0) gaitHistoryFilled = true;
  if (!gaitHistoryFilled) return false;

  float sum = 0;
  for (int i = 0; i < GAIT_WINDOW_SIZE; i++) sum += accelMagHistory[i];
  const float mean = sum / GAIT_WINDOW_SIZE;

  const bool accelDip = (accelMag < mean - GAIT_ACCEL_DIP);
  const bool gyroPeak = (fabsf(gyroZ) > GAIT_GYRO_PEAK);
  if (accelDip && gyroPeak) {
    const unsigned long now = millis();
    if (now - lastSwingDetected > MIN_SWING_INTERVAL_MS) {
      lastSwingDetected = now;
      return true;
    }
  }
  return false;
}

// ---------------- LED de stare (semnalizare non-blocantă a senzorilor) ----------------
static unsigned long ledLastToggle = 0;
static bool          ledPhaseOn    = false;
static bool          ledAltColor   = false;  // alternare roșu/galben când ambii senzori au picat

static void statusLedWrite(uint8_t r, uint8_t g, uint8_t b) {
#ifdef RGB_BUILTIN
  // WS2812 integrat: scalare cu luminozitatea configurată
  rgbLedWrite(STATUS_LED_PIN,
              (uint16_t)r * STATUS_LED_BRIGHTNESS / 255,
              (uint16_t)g * STATUS_LED_BRIGHTNESS / 255,
              (uint16_t)b * STATUS_LED_BRIGHTNESS / 255);
#else
  digitalWrite(STATUS_LED_PIN, (r || g || b) ? HIGH : LOW);
#endif
}

static void statusLedInit() {
#ifndef RGB_BUILTIN
  pinMode(STATUS_LED_PIN, OUTPUT);
#endif
  statusLedWrite(0, 0, 0);
}

// Mașină de stări pe millis() (același stil ca updateHeartbeat) — apelată la FIECARE
// iterație loop(). Coduri: verde puls scurt/3s = ambii OK; roșu clipind = ToF picat
// (critic pt. siguranță); galben clipind = IMU picat; roșu/galben alternat = ambii.
static void updateStatusLed(unsigned long now) {
  if (g_mpuOk && g_tofOk) {
    if (!ledPhaseOn) {
      if (now - ledLastToggle >= 3000) {
        statusLedWrite(0, 255, 0);
        ledPhaseOn = true;
        ledLastToggle = now;
      }
    } else if (now - ledLastToggle >= 50) {
      statusLedWrite(0, 0, 0);
      ledPhaseOn = false;
      ledLastToggle = now;
    }
    return;
  }
  if (now - ledLastToggle < 300) return;
  ledLastToggle = now;
  ledPhaseOn = !ledPhaseOn;
  if (!ledPhaseOn) {
    statusLedWrite(0, 0, 0);
  } else if (!g_tofOk && !g_mpuOk) {
    ledAltColor = !ledAltColor;
    if (ledAltColor) statusLedWrite(255, 0, 0);
    else             statusLedWrite(255, 160, 0);
  } else if (!g_tofOk) {
    statusLedWrite(255, 0, 0);          // roșu: ToF indisponibil
  } else {
    statusLedWrite(255, 160, 0);        // galben: IMU indisponibil
  }
}

// Semnal haptic de diagnostic la boot (utilizatorul final e nevăzător, nu vede LED-ul):
// 3 pulsuri lungi = ToF picat; 2 pulsuri lungi = IMU picat (grupuri separate de pauză).
// Blocant, dar scurt și executat O SINGURĂ DATĂ, înainte de pornirea achiziției.
static void signalSensorFaultHaptic() {
  if (g_tofOk && g_mpuOk) return;
  delay(600);                            // separare de self-test-ul de vibrații
  if (!g_tofOk) {
    for (int i = 0; i < 3; i++) { vibrationWrite(255); delay(400); vibrationWrite(0); delay(250); }
  }
  if (!g_mpuOk) {
    delay(600);                          // pauză între grupuri
    for (int i = 0; i < 2; i++) { vibrationWrite(255); delay(400); vibrationWrite(0); delay(250); }
  }
  vibrationWrite(0);
  vibrationCurrent = 0;
  vibrationTarget  = 0;
}

// ---------------- Self-test de pornire ----------------
static bool selfTestMPU() {
  sensors_event_t a, g, t;
  if (!mpu.getEvent(&a, &g, &t)) {
    LOG_DIRECT("[SELFTEST] MPU6050: citire esuata\n");
    return false;
  }
  const float amag = sqrtf(a.acceleration.x * a.acceleration.x +
                           a.acceleration.y * a.acceleration.y +
                           a.acceleration.z * a.acceleration.z);
  LOG_DIRECT("[SELFTEST] MPU6050: |a|=%.2f m/s^2  T=%.1f°C  gyro=(%.2f,%.2f,%.2f) rad/s\n",
             amag, t.temperature, g.gyro.x, g.gyro.y, g.gyro.z);
  return (amag > 6.0f && amag < 14.0f);
}

static bool selfTestToF() {
  const uint32_t t0 = millis();
  while (!vl53.dataReady() && (millis() - t0) < 500) delay(10);
  if (!vl53.dataReady()) {
    LOG_DIRECT("[SELFTEST] VL53L1X: fara masuratoare (timeout)\n");
    return false;
  }
  const int16_t d = vl53.distance();
  vl53.clearInterrupt();
  LOG_DIRECT("[SELFTEST] VL53L1X: distanta=%d mm\n", d);
  return (d >= 0);
}

static void selfTestServoSweep() {
  LOG_DIRECT("[SELFTEST] Servo SG90: check miscare 90->45->135->90 ...\n");
  const int seq[] = { SERVO_CENTER, SERVO_LEFT, SERVO_RIGHT, SERVO_CENTER };
  for (unsigned i = 0; i < sizeof(seq) / sizeof(seq[0]); ++i) {
    servoWriteAngle(seq[i]);
    delay(450);
  }
  currentServoAngle = SERVO_CENTER;
}

static void selfTestVibration() {
  LOG_DIRECT("[SELFTEST] Motor vibratii (GPIO%d): PLIN 255 pentru 1.5s ...\n", VIBRATION_PIN);
  vibrationWrite(255);
  delay(1500);
  LOG_DIRECT("[SELFTEST] Motor vibratii: 3 pulsuri ...\n");
  for (int i = 0; i < 3; i++) {
    vibrationWrite(255); delay(200);
    vibrationWrite(0);   delay(200);
  }
  vibrationWrite(0);
  vibrationCurrent = 0;
  vibrationTarget  = 0;
}

static void runStartupSelfTest() {
  LOG_DIRECT("\n[SELFTEST] ===== Verificare de pornire =====\n");
  LOG_DIRECT("[SELFTEST] Senzor MPU6050 : %s\n",
             g_mpuOk ? (selfTestMPU() ? "OK" : "ESUAT") : "SARIT (absent la init)");
  LOG_DIRECT("[SELFTEST] Senzor VL53L1X : %s\n",
             g_tofOk ? (selfTestToF() ? "OK" : "ESUAT") : "SARIT (absent la init)");
  selfTestServoSweep();
  selfTestVibration();
  LOG_DIRECT("[SELFTEST] ===== Gata =====\n\n");
}

// ---------------- Achiziție ----------------
static unsigned long counter        = 0;
static int16_t       lastDistance   = 0;
static bool          distanceValid  = false;
static unsigned long lastValidTofMs = 0;
static int           simAngle       = 90;
static float         yawIntegrated  = 0.0f;
static unsigned long lastYawUpdate  = 0;
static const unsigned long SAMPLE_INTERVAL_MS = 50;   // perioada acqTask (vTaskDelayUntil)

// --- Filtru median glisant pentru distanța ToF ---
static int16_t tofBuf[TOF_MEDIAN_N];
static uint8_t tofBufIdx   = 0;
static uint8_t tofBufCount = 0;
static int16_t  tofLastRaw    = 0;
static uint8_t  tofLastStatus = 255;
static uint16_t tofLastSignal = 0;
static unsigned long lastTofPollMs = 0;   // limitator interogare dataReady() (TOF_POLL_MS)

static void tofPush(int16_t d) {
  tofBuf[tofBufIdx] = d;
  tofBufIdx = (tofBufIdx + 1) % TOF_MEDIAN_N;
  if (tofBufCount < TOF_MEDIAN_N) tofBufCount++;
}

static int16_t tofMedian() {
  int16_t t[TOF_MEDIAN_N];
  const uint8_t n = tofBufCount;
  for (uint8_t i = 0; i < n; i++) t[i] = tofBuf[i];
  for (uint8_t i = 1; i < n; i++) {
    const int16_t key = t[i];
    int j = i - 1;
    while (j >= 0 && t[j] > key) { t[j + 1] = t[j]; j--; }
    t[j + 1] = key;
  }
  return (n > 0) ? t[n / 2] : 0;
}

// Inițializare + configurare + pornire ranging VL53L1X — comună pentru setup()
// și tofRecover(). Actualizează g_tofOk prin valoarea de retur.
static bool tofConfigureAndStart() {
  if (!vl53.begin(0x29, &I2CTOF)) return false;
  vl53.VL53L1X_SetDistanceMode(TOF_DISTANCE_MODE);
  vl53.setTimingBudget(TOF_TIMING_BUDGET_MS);
  vl53.VL53L1X_SetInterMeasurementInMs(TOF_INTERMEAS_MS);
  return vl53.startRanging();
}

// Rulează în setup() (core 1, înainte de pornirea achiziției) și apoi EXCLUSIV
// în maintTask (core 0). Flag-ul g_tofOk se publică ULTIMUL, după reconfigurarea
// completă — cât e false, core 1 nu atinge magistrala ToF.
static void tofRecover() {
  LOG_DIRECT("[TOF] senzor absent/mut pe I2C -> reinitializare...\n");
  vl53.VL53L1X_StopRanging();
  const bool ok = tofConfigureAndStart();
  if (ok) {
    tofBufCount      = 0;
    g_lastTofEventMs = millis();
    LOG_DIRECT("[TOF] reinitializat OK\n");
  } else {
    LOG_DIRECT("[TOF] reinit ESUAT (vl_status=%d) -> verifica cablajul/alimentarea VL53L1X!\n",
               vl53.vl_status);
  }
  g_tofOk = ok;
}

// Configurare MPU6050 — comună pentru setup() și mpuRecover().
static void mpuConfigure() {
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

// Rulează exclusiv în maintTask (core 0); flag-ul se ridică după configurare.
static void mpuRecover() {
  LOG_DIRECT("[MPU] senzor absent/mut -> reinitializare...\n");
  if (mpu.begin(0x68, &I2CMPU)) {
    mpuConfigure();
    g_mpuOk = true;
    LOG_DIRECT("[MPU] reinitializat OK\n");
  } else {
    LOG_DIRECT("[MPU] reinit ESUAT -> verifica cablajul/alimentarea MPU6050!\n");
  }
}

// ---------------- Task întreținere senzori (core 0) ----------------
// begin()-urile Adafruit conțin delay-uri interne (~100 ms la MPU6050, zeci de ms
// la VL53L1X prin XSHUT). Rulate pe core 0, aceste blocări NU mai produc sughițuri
// în bucla de timp real (ritm haptic/servo) tocmai când un senzor e picat.
static void maintTask(void*) {
  unsigned long lastTofTry = 0, lastMpuTry = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(250));
    const unsigned long now = millis();
    if (!g_tofOk && (now - lastTofTry) >= TOF_RECOVER_MS) { lastTofTry = now; tofRecover(); }
    if (!g_mpuOk && (now - lastMpuTry) >= MPU_RECOVER_MS) { lastMpuTry = now; mpuRecover(); }
  }
}

static void updateSimAngle() {
  const float yawDeg = yawIntegrated * 180.0f / M_PI;
  simAngle = constrain(90 + (int)yawDeg, 0, 180);
}

// ---------------- Task achiziție IMU + publicare telemetrie (core 1) ----------------
// vTaskDelayUntil => perioadă STRICT fixă de SAMPLE_INTERVAL_MS (fără drift), la
// prioritate peste loopTask => eșantioane echidistante pentru fereastra gait,
// indiferent de durata iterațiilor din loop(). Rulează pe ACELAȘI core cu loop()
// (disciplina „core 1 = achiziție + control" rămâne intactă); starea partajată cu
// loop() e formată doar din scalari aliniați (bool/int16/int) => acces atomic.
static void acqTask(void*) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    const unsigned long now = millis();

    // Fără IMU: telemetria continuă (counter/distanță/actuatori), câmpurile
    // inerțiale rămân 0, gait dezactivat (poza servo se aplică imediat).
    sensors_event_t a = {}, g = {}, temp = {};
    float pitch = 0.0f, roll = 0.0f;
    if (g_mpuOk && mpu.getEvent(&a, &g, &temp)) {
      pitch = atan2f(a.acceleration.x,
                     sqrtf(a.acceleration.y * a.acceleration.y +
                           a.acceleration.z * a.acceleration.z));
      roll  = atan2f(a.acceleration.y, a.acceleration.z);

      const float dt = (now - lastYawUpdate) / 1000.0f;
      yawIntegrated += g.gyro.z * dt;
      while (yawIntegrated >  M_PI) yawIntegrated -= 2 * M_PI;
      while (yawIntegrated < -M_PI) yawIntegrated += 2 * M_PI;

      updateSimAngle();

      const float accelMag = sqrtf(a.acceleration.x * a.acceleration.x +
                                   a.acceleration.y * a.acceleration.y +
                                   a.acceleration.z * a.acceleration.z);
      inSwingPhase = detectSwingPhase(accelMag, g.gyro.z);
    } else {
      if (g_mpuOk) {                     // citire eșuată pe un senzor considerat OK
        g_mpuOk = false;
        LOG_RT("[MPU] citire esuata -> senzor marcat absent (recuperare pe core 0)\n");
      }
      inSwingPhase = false;
    }
    // Actualizat în ambele ramuri: la revenirea IMU, dt nu acoperă golul de absență.
    lastYawUpdate = now;

    // Publicarea instantaneului (secțiune critică minimală: doar copiere)
    TelemetryPacket s;
    s.counter     = counter;
    s.angle       = static_cast<uint8_t>(simAngle);
    s.distance_mm = lastDistance;
    s.pitch = pitch;              s.roll = roll;   s.yaw = yawIntegrated;
    s.ax = a.acceleration.x;      s.ay = a.acceleration.y;  s.az = a.acceleration.z;
    s.gx = g.gyro.x;              s.gy = g.gyro.y;          s.gz = g.gyro.z;
    s.temp_c = temp.temperature;
    s.swing     = inSwingPhase ? 1 : 0;
    s.servo     = static_cast<uint8_t>(currentServoAngle);
    s.vibration = static_cast<uint8_t>(vibrationCurrent);

    taskENTER_CRITICAL(&g_snapMux);
    g_snapshot = s;
    taskEXIT_CRITICAL(&g_snapMux);

    if (counter % 40 == 0) {
      LOG_RT(
        "[TELE] #%lu angle=%d° dist=%dmm(%s) | pitch=%+.2f roll=%+.2f yaw=%+.2f rad | "
        "accel=(%+.2f,%+.2f,%+.2f)m/s2 gyro=(%+.2f,%+.2f,%+.2f)rad/s | T=%.1f°C | "
        "swing=%s servo=%d° vib=%d | tof[raw=%d st=%d sig=%u] ble=%s\n",
        counter, s.angle, s.distance_mm, distanceValid ? "ok" : "?",
        s.pitch, s.roll, s.yaw,
        s.ax, s.ay, s.az, s.gx, s.gy, s.gz,
        s.temp_c,
        s.swing ? "Y" : "N", s.servo, s.vibration,
        tofLastRaw, tofLastStatus, tofLastSignal,
        (g_connected && g_subscribed) ? "SECURIZAT" : (g_connected ? "CONECTAT" : "DECONECTAT"));
    }
    counter++;
  }
}

void setup() {
  Serial.begin(115200);
#if LOG_ENABLED
  // Coada + task-ul de logging (core 0) — primele, ca LOG_RT să fie funcțional
  // din momentul în care pornesc task-urile de timp real.
  g_logQueue = xQueueCreate(LOG_QUEUE_LEN, sizeof(LogMsg));
  xTaskCreatePinnedToCore(logTask, "log", 3072, nullptr, tskIDLE_PRIORITY + 1, nullptr, 0);
#endif
  // Marker IMEDIAT (inaintea delay-ului): daca acest mesaj nu apare pe monitor,
  // aplicatia nu porneste deloc (crash pre-app / setari gresite de placa).
  LOG_DIRECT("\n[BOOT] app pornit — setup() a inceput\n");
  statusLedInit();
  statusLedWrite(0, 0, 255);           // albastru continuu = setup() în curs
  delay(1000);
  LOG_DIRECT("[KERYKE v3 BLE] Initializare...\n");

  // Senzori — pornire NERESTRICȚIONATĂ: eșecul unui senzor NU blochează boot-ul,
  // ci este semnalizat (LED + haptic) și reîncercat periodic pe core 0 (maintTask).
  LOG_DIRECT("[BOOT] init I2C...\n");
  I2CMPU.begin(MPU_SDA, MPU_SCL);
  I2CTOF.begin(TOF_SDA, TOF_SCL);

  g_mpuOk = mpu.begin(0x68, &I2CMPU);
  if (g_mpuOk) {
    mpuConfigure();
    LOG_DIRECT("[KERYKE] MPU6050 OK\n");
  } else {
    LOG_DIRECT("[KERYKE] MPU6050 ABSENT -> pornesc degradat (fara IMU/gait); reincerc periodic\n");
  }

  g_tofOk = tofConfigureAndStart();
  g_lastTofEventMs = millis();
  if (g_tofOk) {
    LOG_DIRECT("[KERYKE] VL53L1X OK (mod %s, %dms, filtru median)\n",
               TOF_DISTANCE_MODE == 2 ? "LONG" : "SHORT", TOF_TIMING_BUDGET_MS);
  } else {
    LOG_DIRECT("[KERYKE] VL53L1X ABSENT/eroare (vl_status=%d) -> pornesc degradat "
               "(distanta NECUNOSCUTA, motor oprit); reincerc periodic\n", vl53.vl_status);
  }

  // Actuator — API LEDC diferit intre core 2.x si 3.x
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  const bool servoAttached = ledcAttach(SERVO_PIN, SERVO_FREQ_HZ, SERVO_RES_BITS);
  LOG_DIRECT("[SETUP] LEDC servo    (GPIO%d, %dHz/%d-bit): %s\n",
             SERVO_PIN, SERVO_FREQ_HZ, SERVO_RES_BITS,
             servoAttached ? "ATASAT OK" : "ESUAT !!!");
#else
  ledcSetup(SERVO_LEDC_CH, SERVO_FREQ_HZ, SERVO_RES_BITS);
  ledcAttachPin(SERVO_PIN, SERVO_LEDC_CH);
#endif
  servoWriteAngle(currentServoAngle);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  const bool vibAttached = ledcAttach(VIBRATION_PIN, VIBRATION_FREQ_HZ, VIBRATION_RES_BITS);
  LOG_DIRECT("[SETUP] LEDC vibratii (GPIO%d, %dHz/%d-bit): %s\n",
             VIBRATION_PIN, VIBRATION_FREQ_HZ, VIBRATION_RES_BITS,
             vibAttached ? "ATASAT OK" : "ESUAT !!!");
#else
  ledcSetup(VIBRATION_LEDC_CH, VIBRATION_FREQ_HZ, VIBRATION_RES_BITS);
  ledcAttachPin(VIBRATION_PIN, VIBRATION_LEDC_CH);
#endif
  vibrationWrite(0);

  // Self-test de pornire (senzorii absenți sunt săriți) + diagnostic haptic:
  // utilizatorul nevăzător află de la boot dacă dispozitivul e degradat.
  runStartupSelfTest();
  signalSensorFaultHaptic();

  // Infrastructură concurentă
  g_actQueue = xQueueCreate(16, sizeof(ActuatorCommand));

  // Bluetooth securizat (advertising + pairing cu passkey)
  setupBLE();

  // Task de push telemetrie (nesolicitat, 5 s) pe core 0
  xTaskCreatePinnedToCore(blePushTask, "ble_push", 6144, nullptr,
                          tskIDLE_PRIORITY + 1, nullptr, 0);

  // Task de întreținere senzori (core 0): reinițializările cu delay-uri interne
  // nu mai ating bucla de timp real.
  xTaskCreatePinnedToCore(maintTask, "maint", 4096, nullptr,
                          tskIDLE_PRIORITY + 1, nullptr, 0);

  lastYawUpdate = millis();

  // Task de achiziție IMU + telemetrie (core 1), prioritate peste loopTask (=1):
  // vTaskDelayUntil îi dă cadență strict periodică de 50 ms.
  xTaskCreatePinnedToCore(acqTask, "acq", 6144, nullptr,
                          tskIDLE_PRIORITY + 2, nullptr, 1);

  LOG_DIRECT("[KERYKE v3 BLE] Achizitia porneste independent de starea Bluetooth "
             "si a senzorilor (stare senzori pe LED-ul integrat).\n\n");
}

void loop() {                                  // core 1: ToF + actuatori + LED
  const unsigned long now = millis();          // (eșantionarea IMU: acqTask, tot core 1)

  // 1) Comenzi actuator sosite din stiva BLE (dispecerizare pe tip)
  ActuatorCommand cmd;
  while (xQueueReceive(g_actQueue, &cmd, 0) == pdTRUE) {
    switch (cmd.type) {
      case ACT_ROTATE_REL: applyMotorRotation(cmd.a);                  break;
      case ACT_SERVO_POSE: commandServoPose((uint8_t)cmd.a, cmd.b != 0); break;
      case ACT_VIBRATE:    triggerVibration(cmd.a, cmd.b);             break;
    }
  }

  // 2) Distanță ToF: validare status + filtru median + fail-safe (prospețime).
  // Interogarea dataReady() e o tranzacție I2C reală (~0,5 ms) — limitată la
  // TOF_POLL_MS; măsurătoare nouă apare oricum doar la TOF_INTERMEAS_MS (55 ms).
  // Cu g_tofOk=false, core 1 nu atinge deloc magistrala (recuperarea rulează pe
  // core 0, în maintTask).
  if (g_tofOk && (now - lastTofPollMs) >= TOF_POLL_MS) {
    lastTofPollMs = now;
    if (vl53.dataReady()) {
      g_lastTofEventMs = now;                // senzorul e viu pe I2C
      uint8_t  st  = 255;
      uint16_t sig = 0;
      vl53.VL53L1X_GetRangeStatus(&st);
      const int16_t d = vl53.distance();
      vl53.VL53L1X_GetSignalPerSpad(&sig);
      vl53.clearInterrupt();
      tofLastRaw = d; tofLastStatus = st; tofLastSignal = sig;
      if (st == 0 && d > 0) {
        tofPush(d);
        lastDistance   = tofMedian();
        distanceValid  = true;
        lastValidTofMs = now;
      }
    }
  }
  if (distanceValid && (now - lastValidTofMs) > TOF_STALE_MS) {
    distanceValid = false;
    tofBufCount   = 0;
    LOG_RT("[TOF] invalid: raw=%d st=%d sig=%u -> NECUNOSCUT (fail-safe)\n",
           tofLastRaw, tofLastStatus, tofLastSignal);
  }
  // Senzor MUT pe I2C (nici măcar dataReady de TOF_RECOVER_MS): marcat absent —
  // maintTask (core 0) preia reinițializarea, fără să blocheze bucla. Un senzor
  // VIU dar fără țintă validă (status != 0, ex. spațiu liber > 4 m) NU se
  // reinițializează — fail-safe-ul de prospețime acoperă deja cazul.
  if (g_tofOk && (now - g_lastTofEventMs) > TOF_RECOVER_MS) {
    g_tofOk = false;
    LOG_RT("[TOF] mut pe I2C -> marcat absent (recuperare pe core 0)\n");
  }

  // 2b) Zona de ritm cardiac.
  if (distanceValid) {
    const HbZone z = heartbeatZone(lastDistance);
    if (z != hbZone) {
      hbZone = z;
      const char* nume = (z == ZONE_FAST)   ? "ALERT (< 0.5 m)"
                       : (z == ZONE_MEDIUM) ? "MEDIU (< 1 m)"
                       : (z == ZONE_SLOW)   ? "LENT  (< 1.5 m)"
                       :                      "OPRIT (> 1.5 m)";
      LOG_RT("[RITM] %s | distanta=%d mm\n", nume, lastDistance);
    }
  } else if (hbZone != ZONE_NONE) {
    hbZone = ZONE_NONE;
    LOG_RT("[RITM] senzor invalid -> motor OPRIT\n");
  }

  // 3) Actuatori + LED de stare la FIECARE iterație
  applyServoCommand();
  const uint32_t hbPeriod = heartbeatPeriod(hbZone);
  if (hbPeriod != 0) {
    updateHeartbeat(hbPeriod, now);
  } else {
    updateHeartbeat(0, now);
    updateVibration();
  }
  updateStatusLed(now);

  // 4) Eșantionarea IMU + publicarea telemetriei rulează în task-ul dedicat
  //    acqTask (vTaskDelayUntil, perioadă strict fixă) — nu mai sunt în loop().

  // Cedează 1 ms: toate mașinile de stări de aici au granularitate de milisecunde,
  // deci busy-spin-ul nu aducea nimic — doar 100% CPU pe core 1 (dispozitiv pe
  // baterie). Tick-ul FreeRTOS e 1 kHz => bucla rulează în continuare la ~1 kHz.
  vTaskDelay(pdMS_TO_TICKS(1));
}
