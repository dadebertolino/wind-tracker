/*
 * Sonda per pallone pilota - RAK3172 (RUI3 >= 4.1) + ATGM336H + BME280 - LoRa P2P
 * Autore: Davide Bertolino
 * Firmware 0.1 (aggiornare insieme a FW_VERSION)
 *
 * Sale con il pallone e trasmette ogni ~2 s posizione, quota, velocità e rotta
 * dal GPS, temperatura, umidità e pressione dal BME280 e la tensione della LiPo.
 * La base station le riceve; il vento a ogni quota si ricava sul PC.
 *
 * All'avvio: autodiagnosi con correzione automatica di modalità e parametri P2P,
 * verifica di GPS, sensore e batteria, configurazione del GPS.
 *
 * Collegamenti (hardware/sonda, schema 0.1):
 *   GPS ATGM336H su UART1, 9600 baud: PB6 (TX) -> RX del GPS, PB7 (RX) <- TX del GPS
 *   BME280 su I2C: SDA PA11, SCL PA12 (SDO a massa = 0x76, a 3V3 = 0x77)
 *   LiPo: partitore R8/R9 da 1 MOhm, metà della tensione su PB3 (ADC)
 *   LED di stato su PA8
 *
 * Pacchetto, formato v1 (32 byte, big endian):
 *   [0]     versione formato (4 bit alti) | ID sonda (4 bit bassi, 1-15)
 *   [1]     contatore: 0 = primo pacchetto dopo un riavvio, poi 1…255, 1…
 *   [2]     tipo (4 bit alti, 1 = sonda) | causa del reset (4 bit bassi,
 *           solo nel pacchetto con contatore 0, altrimenti 0)
 *   [3-5]   ora GPS, secondi dalla mezzanotte UTC (0-86399)
 *   [6-9]   latitudine, gradi x 10^7 (int32)
 *   [10-13] longitudine, gradi x 10^7 (int32)
 *   [14-15] quota GPS sul livello del mare, m (int16)
 *   [16-17] velocità al suolo, cm/s       [18-19] rotta, gradi x 100 (0-35999)
 *   [20-21] T °C x100 (int16)             [22-23] UR % x100
 *   [24-25] P hPa x10                     [26-27] tensione della LiPo, mV
 *   [28]    satelliti usati (4 bit alti, max 15) | qualità del fix GGA (4 bit bassi, 0 = nessun fix)
 *   [29]    HDOP x10 (max 254)
 *   [30-31] CRC-16/CCITT-FALSE dei byte 0-29
 * Misure non disponibili: int16 0x8000, uint16 0xFFFF, ora 0xFFFFFF,
 * latitudine e longitudine 0x7FFFFFFF, HDOP 0xFF. Senza fix posizione, quota,
 * velocità e rotta non sono disponibili; ora, satelliti e HDOP lo sono se il
 * GPS li fornisce.
 */

#include <Wire.h>

// ---------- Configurazione ----------
#define PROBE_ID     1             // 1-15
#define INTERVAL_MS  2000UL        // 2 s: un punto ogni 6-10 m di quota
#define PROBE_DEBUG  1             // 0 = nessuna stampa

// 869,525 MHz: con BW 125 kHz resta tutta in 869,40-869,65 MHz (500 mW ERP, duty cycle 10 %)
#define P2P_FREQ     869525000UL
#define P2P_SF       8
#define P2P_BW       125
#define P2P_CR       0             // 0 = 4/5
#define P2P_PL       8
#define P2P_TXP      20            // dBm, all'uscita del modulo

#define GPS_BAUD     9600
#define GPS_NAV_MODE 5             // $PCAS11: 5 = aereo, accelerazioni < 1 g (da verificare sul manuale CASIC)

#define VBAT_DIV     2             // partitore R8/R9, 1 MOhm / 1 MOhm
#define VBAT_MIN_MV  3500          // LiPo quasi scarica: il volo dura poco, meglio non partire
#define VBAT_MAX_MV  4250          // LiPo carica: 4,20 V
// ------------------------------------

#define FW_VERSION   "0.1"

// Formato del pacchetto
#define FMT_VERSION  1
#define PROBE_TYPE   1             // 1 = sonda
#define PKT_LEN      32

// Valori "non disponibile" nel payload: fuori da qualsiasi misura reale
#define ND_I16       ((int16_t)0x8000)
#define ND_U16       0xFFFF
#define ND_TOD       0xFFFFFFUL
#define ND_COORD     0x7FFFFFFFL
#define ND_U8        0xFF

// Cause del reset (byte [2], 4 bit bassi)
#define RST_NESSUNO     0          // pacchetto normale
#define RST_ALIM        1          // accensione o calo di tensione (BOR)
#define RST_PIN         2          // pin NRST
#define RST_SOFTWARE    3          // riavvio da firmware (es. passaggio a P2P)
#define RST_WATCHDOG    4          // watchdog indipendente (IWDG)
#define RST_WWDG        5          // watchdog a finestra
#define RST_LOWPOWER    6          // errore di gestione del basso consumo
#define RST_ALTRO       7          // option byte o causa non riconosciuta

// Ritardo casuale sull'intervallo (±10 %): due sonde in volo insieme non
// trasmettono sempre nello stesso istante
#define JITTER_MS    (INTERVAL_MS / 10)

// Tempo in aria di un pacchetto LoRa (header esplicito, CRC radio attivo), in µs.
// Tsym = 2^SF / BW; preambolo P2P_PL + 4,25 simboli; payload
// 8 + ceil((8 L - 4 SF + 44) / (4 (SF - 2 DE))) x (CR + 4) simboli,
// con DE (low data rate optimize) a SF 11 e 12 con 125 kHz.
constexpr uint32_t SYM_US = (1UL << P2P_SF) * 1000UL / P2P_BW;
constexpr int32_t  LDRO   = (P2P_SF >= 11 && P2P_BW == 125) ? 1 : 0;
constexpr int32_t  PL_NUM = 8 * PKT_LEN - 4 * P2P_SF + 44;
constexpr int32_t  PL_DEN = 4 * (P2P_SF - 2 * LDRO);
constexpr uint32_t PL_SYM = 8 + (PL_NUM > 0 ? (PL_NUM + PL_DEN - 1) / PL_DEN : 0) * (P2P_CR + 5);
constexpr uint32_t AIRTIME_US = (4UL * P2P_PL + 17) * SYM_US / 4 + PL_SYM * SYM_US;

// Duty cycle massimo della sottobanda: 10 %, calcolato sull'intervallo più breve
static_assert(AIRTIME_US * 10 <= (INTERVAL_MS - JITTER_MS) * 1000UL, "INTERVAL_MS troppo breve per il duty cycle del 10 %");
static_assert(PROBE_ID >= 1 && PROBE_ID <= 15, "PROBE_ID deve stare fra 1 e 15");

#if PROBE_DEBUG
  #define DBG(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG(...) do {} while (0)
#endif

// Watchdog: massimo 8 s in RUI3
#define WDT_MS        8000
#define GPS_TIMEOUT_MS 3000UL      // nessuna frase NMEA valida da 3 s: GPS muto
#define GPS_WAIT_MS   1500UL       // attesa massima di un'epoca GPS dopo l'intervallo
#define LED_PIN       PA8

// ---------- Dati del GPS ----------
// Prima di ogni funzione: l'IDE Arduino inserisce lì i prototipi, che la usano
struct GpsData {
  uint32_t tod;                    // secondi dalla mezzanotte UTC, ND_TOD se non nota
  int32_t  lat, lon;               // gradi x 10^7
  int16_t  alt;                    // m sul livello del mare
  uint16_t speed;                  // cm/s
  uint16_t course;                 // gradi x 100
  uint8_t  sats, quality, hdop;    // satelliti usati, qualità GGA, HDOP x10
};

uint8_t addr = 0;
uint8_t seq = 0;
uint8_t resetCause = RST_ALTRO;
volatile bool txDone = true;

uint16_t T1, P1; int16_t T2, T3, P2, P3, P4, P5, P6, P7, P8, P9;
uint8_t H1, H3; int16_t H2, H4, H5; int8_t H6;
int32_t t_fine;

void sendDone(void) { txDone = true; }

// ---------- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) ----------
uint16_t crc16(const uint8_t *p, uint8_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= (uint16_t)(*p++) << 8;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// ---------- Dati del GPS ----------
void gpsClear(GpsData &g) {
  g.tod = ND_TOD; g.lat = ND_COORD; g.lon = ND_COORD; g.alt = ND_I16;
  g.speed = ND_U16; g.course = ND_U16; g.sats = 0; g.quality = 0; g.hdop = ND_U8;
}

// Pacchetto formato v1 (vedi intestazione); la causa del reset va solo con seq 0
void buildPacket(uint8_t *pl, uint8_t sq, uint8_t rst, const GpsData &g,
                 int16_t t, uint16_t rh, uint16_t p, uint16_t vbat) {
  uint8_t i = 0;
  pl[i++] = FMT_VERSION << 4 | PROBE_ID;
  pl[i++] = sq;
  pl[i++] = PROBE_TYPE << 4 | (sq == 0 ? rst : RST_NESSUNO);
  pl[i++] = g.tod >> 16;       pl[i++] = g.tod >> 8;  pl[i++] = (uint8_t)g.tod;
  pl[i++] = (uint32_t)g.lat >> 24; pl[i++] = (uint32_t)g.lat >> 16;
  pl[i++] = (uint32_t)g.lat >> 8;  pl[i++] = (uint8_t)g.lat;
  pl[i++] = (uint32_t)g.lon >> 24; pl[i++] = (uint32_t)g.lon >> 16;
  pl[i++] = (uint32_t)g.lon >> 8;  pl[i++] = (uint8_t)g.lon;
  pl[i++] = (uint16_t)g.alt >> 8;  pl[i++] = (uint8_t)g.alt;
  pl[i++] = g.speed >> 8;      pl[i++] = (uint8_t)g.speed;
  pl[i++] = g.course >> 8;     pl[i++] = (uint8_t)g.course;
  pl[i++] = (uint16_t)t >> 8;  pl[i++] = (uint8_t)t;
  pl[i++] = rh >> 8;           pl[i++] = (uint8_t)rh;
  pl[i++] = p >> 8;            pl[i++] = (uint8_t)p;
  pl[i++] = vbat >> 8;         pl[i++] = (uint8_t)vbat;
  pl[i++] = (g.sats > 15 ? 15 : g.sats) << 4 | (g.quality & 0x0F);
  pl[i++] = g.hdop;
  uint16_t crc = crc16(pl, i);
  pl[i++] = crc >> 8;          pl[i++] = (uint8_t)crc;
}

// ---------- NMEA ----------
// Si usano solo GGA (ora, posizione, quota, qualità, satelliti, HDOP) e RMC
// (velocità al suolo e rotta), con qualunque talker (GP, BD, GN). Un'epoca è
// completa quando sono arrivate GGA e RMC con la stessa ora.
#define NMEA_MAX     100           // 82 caratteri da standard, con margine
#define NMEA_FIELDS  20

GpsData gps;                       // ultima epoca completa
GpsData gpsNext;                   // epoca in costruzione
uint32_t gpsEpochs = 0;            // epoche complete ricevute
uint32_t nmeaBad = 0;              // frasi con checksum errato
char nmeaLine[NMEA_MAX + 1];
uint8_t nmeaLen = 0;
bool haveGga = false, haveRmc = false;
uint32_t ggaTod = ND_TOD, rmcTod = ND_TOD;

static uint8_t hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0xFF;
}

// Frase completa "$...*HH" (senza CR/LF): checksum XOR fra '$' e '*'
bool nmeaChecksumOk(const char *s) {
  if (*s++ != '$') return false;
  uint8_t x = 0;
  while (*s && *s != '*') x ^= (uint8_t)*s++;
  if (*s != '*') return false;
  uint8_t h = hexVal(s[1]), l = hexVal(s[2]);
  return h != 0xFF && l != 0xFF && s[3] == '\0' && (uint8_t)(h << 4 | l) == x;
}

// Divide la frase sui separatori, tenendo i campi vuoti; toglie il checksum
uint8_t nmeaSplit(char *s, char **f, uint8_t max) {
  uint8_t n = 0;
  char *star = strchr(s, '*');
  if (star) *star = '\0';
  f[n++] = s;
  for (; *s && n < max; s++)
    if (*s == ',') { *s = '\0'; f[n++] = s + 1; }
  return n;
}

// "hhmmss" o "hhmmss.sss" -> secondi dalla mezzanotte
bool parseTime(const char *f, uint32_t &tod) {
  for (uint8_t i = 0; i < 6; i++)
    if (f[i] < '0' || f[i] > '9') return false;
  uint32_t h = (f[0] - '0') * 10 + (f[1] - '0');
  uint32_t m = (f[2] - '0') * 10 + (f[3] - '0');
  uint32_t s = (f[4] - '0') * 10 + (f[5] - '0');
  if (h > 23 || m > 59 || s > 60) return false;
  tod = h * 3600 + m * 60 + (s > 59 ? 59 : s);
  return true;
}

// "ddmm.mmmm" (latitudine) o "dddmm.mmmm" (longitudine) -> gradi x 10^7.
// I gradi sono le cifre prima degli ultimi due interi dei minuti.
bool parseCoord(const char *f, const char *hemi, int32_t &out) {
  const char *dot = strchr(f, '.');
  if (!dot || dot - f < 3 || dot - f > 5 || !hemi[0]) return false;
  uint32_t deg = 0;
  for (const char *c = f; c < dot - 2; c++) {
    if (*c < '0' || *c > '9') return false;
    deg = deg * 10 + (*c - '0');
  }
  double min = strtod(dot - 2, nullptr);
  if (min < 0 || min >= 60 || deg > 180) return false;
  double v = ((double)deg + min / 60.0) * 1e7;
  int32_t r = (int32_t)(v + 0.5);
  if (hemi[0] == 'S' || hemi[0] == 'W') r = -r;
  else if (hemi[0] != 'N' && hemi[0] != 'E') return false;
  out = r;
  return true;
}

static void nmeaEpochCheck() {
  if (haveGga && haveRmc && ggaTod == rmcTod) {
    gps = gpsNext;
    gpsEpochs++;
    haveGga = haveRmc = false;
  }
}

static void nmeaGga(char **f, uint8_t n) {
  if (n < 10) return;
  GpsData &g = gpsNext;
  uint32_t tod;
  g.tod = parseTime(f[1], tod) ? tod : ND_TOD;
  g.quality = (uint8_t)atoi(f[6]);
  g.sats = (uint8_t)atoi(f[7]);
  if (f[8][0]) {
    double h = strtod(f[8], nullptr) * 10.0 + 0.5;
    g.hdop = h >= 254 ? 254 : (uint8_t)h;
  } else {
    g.hdop = ND_U8;
  }
  int32_t lat, lon;
  if (g.quality > 0 && parseCoord(f[2], f[3], lat) && parseCoord(f[4], f[5], lon) && f[9][0]) {
    double a = strtod(f[9], nullptr);
    g.lat = lat; g.lon = lon;
    g.alt = a > 32767 ? 32767 : a < -32767 ? -32767 : (int16_t)(a + (a < 0 ? -0.5 : 0.5));
  } else {
    g.quality = 0; g.lat = ND_COORD; g.lon = ND_COORD; g.alt = ND_I16;
  }
  ggaTod = g.tod;
  haveGga = true;
  nmeaEpochCheck();
}

static void nmeaRmc(char **f, uint8_t n) {
  if (n < 9) return;
  GpsData &g = gpsNext;
  uint32_t tod;
  rmcTod = parseTime(f[1], tod) ? tod : ND_TOD;
  // Velocità in nodi -> cm/s; la rotta manca quando si è fermi
  if (f[2][0] == 'A' && f[7][0]) {
    double v = strtod(f[7], nullptr) * 51.4444 + 0.5;
    g.speed = v >= 65534 ? 65534 : (uint16_t)v;
  } else {
    g.speed = ND_U16;
  }
  if (f[2][0] == 'A' && f[8][0]) {
    double c = strtod(f[8], nullptr) * 100.0 + 0.5;
    g.course = (uint16_t)c % 36000;
  } else {
    g.course = ND_U16;
  }
  haveRmc = true;
  nmeaEpochCheck();
}

// Un carattere dalla seriale del GPS; true quando completa una frase valida
bool nmeaFeed(char c) {
  if (c == '$') { nmeaLen = 0; }
  if (c == '\r' || c == '\n') {
    if (nmeaLen == 0) return false;
    nmeaLine[nmeaLen] = '\0';
    nmeaLen = 0;
    if (!nmeaChecksumOk(nmeaLine)) { nmeaBad++; return false; }
    char *f[NMEA_FIELDS];
    uint8_t n = nmeaSplit(nmeaLine, f, NMEA_FIELDS);
    if (strlen(f[0]) != 6) return false;
    const char *type = f[0] + 3;   // dopo "$" e il talker
    if (strcmp(type, "GGA") == 0) nmeaGga(f, n);
    else if (strcmp(type, "RMC") == 0) nmeaRmc(f, n);
    return true;
  }
  if (nmeaLen < NMEA_MAX && (nmeaLen > 0 || c == '$')) nmeaLine[nmeaLen++] = c;
  else nmeaLen = 0;                // frase troppo lunga o senza '$': si scarta
  return false;
}

// ---------- Bus I2C ----------
// Se il sensore si è fermato a metà di una trasmissione può tenere SDA basso e
// bloccare il bus: 9 impulsi di clock gli fanno finire il byte, poi una
// condizione di STOP lo riporta a riposo.
void i2cRecover() {
  Wire.end();
  pinMode(PA11, INPUT_PULLUP);       // SDA
  pinMode(PA12, OUTPUT);             // SCL
  digitalWrite(PA12, HIGH);
  delayMicroseconds(5);
  for (uint8_t i = 0; i < 9 && digitalRead(PA11) == LOW; i++) {
    digitalWrite(PA12, LOW);  delayMicroseconds(5);
    digitalWrite(PA12, HIGH); delayMicroseconds(5);
  }
  pinMode(PA11, OUTPUT);             // STOP: SDA da basso ad alto con SCL alto
  digitalWrite(PA11, LOW);  delayMicroseconds(5);
  digitalWrite(PA12, HIGH); delayMicroseconds(5);
  digitalWrite(PA11, HIGH); delayMicroseconds(5);
  Wire.begin();
}

// ---------- BME280 ----------
void wr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

bool rd(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr); Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool bmeInit() {
  uint8_t id = 0;
  const uint8_t addrs[] = {0x76, 0x77};
  for (uint8_t i = 0; i < 2; i++) {
    addr = addrs[i];
    if (rd(0xD0, &id, 1)) break;
    addr = 0;
  }
  if (!addr)       { DBG("  BME280: non trovato su 0x76/0x77 (collegamenti? pull-up?)\r\n"); return false; }
  if (id == 0x58)  { DBG("  BME280: è un BMP280 (niente umidità)\r\n"); addr = 0; return false; }
  if (id != 0x60)  { DBG("  BME280: chip ID inatteso 0x%02X\r\n", id); addr = 0; return false; }

  wr(0xE0, 0xB6); delay(5);

  uint8_t c[26], h[7];
  if (!rd(0x88, c, 26) || !rd(0xE1, h, 7)) {
    DBG("  BME280: errore lettura calibrazione\r\n"); addr = 0; return false;
  }
  T1 = c[1] << 8 | c[0];  T2 = c[3] << 8 | c[2];  T3 = c[5] << 8 | c[4];
  P1 = c[7] << 8 | c[6];  P2 = c[9] << 8 | c[8];  P3 = c[11] << 8 | c[10];
  P4 = c[13] << 8 | c[12]; P5 = c[15] << 8 | c[14]; P6 = c[17] << 8 | c[16];
  P7 = c[19] << 8 | c[18]; P8 = c[21] << 8 | c[20]; P9 = c[23] << 8 | c[22];
  H1 = c[25];
  H2 = h[1] << 8 | h[0];  H3 = h[2];
  H4 = (int16_t)((int8_t)h[3] * 16) | (h[4] & 0x0F);
  H5 = (int16_t)((int8_t)h[5] * 16) | (h[4] >> 4);
  H6 = (int8_t)h[6];

  wr(0xF5, 0x00);
  DBG("  BME280: OK a 0x%02X\r\n", addr);
  return true;
}

int32_t compT(int32_t adc) {
  int32_t v1 = ((((adc >> 3) - ((int32_t)T1 << 1))) * T2) >> 11;
  int32_t v2 = (((((adc >> 4) - (int32_t)T1) * ((adc >> 4) - (int32_t)T1)) >> 12) * T3) >> 14;
  t_fine = v1 + v2;
  return (t_fine * 5 + 128) >> 8;
}

uint32_t compP(int32_t adc) {
  int64_t v1 = (int64_t)t_fine - 128000;
  int64_t v2 = v1 * v1 * P6;
  v2 += (v1 * P5) << 17;
  v2 += ((int64_t)P4) << 35;
  v1 = ((v1 * v1 * P3) >> 8) + ((v1 * P2) << 12);
  v1 = ((((int64_t)1) << 47) + v1) * P1 >> 33;
  if (v1 == 0) return 0;
  int64_t p = 1048576 - adc;
  p = (((p << 31) - v2) * 3125) / v1;
  v1 = ((int64_t)P9 * (p >> 13) * (p >> 13)) >> 25;
  v2 = ((int64_t)P8 * p) >> 19;
  return (uint32_t)(((p + v1 + v2) >> 8) + ((int64_t)P7 << 4));
}

uint32_t compH(int32_t adc) {
  int32_t v = t_fine - 76800;
  v = (((((adc << 14) - ((int32_t)H4 << 20) - ((int32_t)H5 * v)) + 16384) >> 15) *
       (((((((v * (int32_t)H6) >> 10) * (((v * (int32_t)H3) >> 11) + 32768)) >> 10) + 2097152) *
         (int32_t)H2 + 8192) >> 14));
  v -= ((((v >> 15) * (v >> 15)) >> 7) * (int32_t)H1) >> 4;
  if (v < 0) v = 0;
  if (v > 419430400) v = 419430400;
  return (uint32_t)(v >> 12);
}

bool bmeRead(int16_t &t100, uint16_t &rh100, uint16_t &p10) {
  wr(0xF2, 0x01);
  wr(0xF4, 0x25);
  uint8_t st = 0; uint32_t t0 = millis();
  do {
    delay(2);
    if (!rd(0xF3, &st, 1)) return false;
  } while ((st & 0x08) && millis() - t0 < 100);
  if (st & 0x08) return false;               // misura non conclusa: dati vecchi

  uint8_t d[8];
  if (!rd(0xF7, d, 8)) return false;
  int32_t adcP = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4);
  int32_t adcT = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4);
  int32_t adcH = (d[6] << 8) | d[7];
  // Valori di reset dei registri: il sensore si è riavviato e non ha misurato
  if (adcT == 0x80000 || adcP == 0x80000 || adcH == 0x8000) return false;

  int32_t  t = compT(adcT);
  uint32_t p = compP(adcP) / 2560;
  uint32_t h = (compH(adcH) * 100) >> 10;
  // Fuori dal campo di misura del BME280: lettura corrotta, non una misura
  if (t < -4000 || t > 8500 || p < 3000 || p > 11000 || h > 10000) return false;

  t100 = (int16_t)t; p10 = (uint16_t)p; rh100 = (uint16_t)h;
  return true;
}

// Se la lettura fallisce il bus I2C viene sbloccato, il sensore reinizializzato
// e riletto subito; se non risponde ancora, ci si riprova al pacchetto successivo.
bool sensorRead(int16_t &t100, uint16_t &rh100, uint16_t &p10) {
  if (addr && bmeRead(t100, rh100, p10)) return true;
  if (addr) DBG("BME280: lettura fallita, reinizializzo\r\n");
  addr = 0;
  i2cRecover();
  return bmeInit() && bmeRead(t100, rh100, p10);
}

// ---------- Tensione della LiPo ----------
// Il riferimento dell'ADC è VDDA, cioè il 3,3 V del regolatore. Lo si misura con
// il riferimento interno VREFINT confrontato con la sua taratura di fabbrica
// (VREFINT_CAL, acquisita a 3,3 V), così la misura sul partitore resta giusta
// anche se il 3,3 V non è esatto.
uint16_t readAdcMv(uint32_t channel) {
  UDRV_ADC_RESOLUTION prima = udrv_adc_get_resolution();
  udrv_adc_set_resolution(UDRV_ADC_RESOLUTION_12BIT);
  uint32_t sRef = 0, sPin = 0;
  uint8_t n = 0;
  for (uint8_t i = 0; i < 5; i++) {
    int16_t ref = 0, pin = 0;        // udrv_adc_read vuole int16_t; a 12 bit fra 0 e 4095
    if (udrv_adc_read(UDRV_ADC_CHANNEL_VREFINT, &ref) != UDRV_RETURN_OK || ref <= 0) continue;
    if (udrv_adc_read(channel, &pin) != UDRV_RETURN_OK || pin < 0) continue;
    if (i == 0) continue;            // la prima conversione dopo l'accensione del riferimento è meno precisa
    sRef += (uint32_t)ref;
    sPin += (uint32_t)pin;
    n++;
  }
  udrv_adc_set_resolution(prima);
  if (n == 0) return ND_U16;
  uint32_t vdda = __LL_ADC_CALC_VREFANALOG_VOLTAGE(sRef / n, LL_ADC_RESOLUTION_12B);
  return (uint16_t)(sPin / n * vdda / 4095);
}

uint16_t readVbat() {
  uint16_t mv = readAdcMv(PB3);
  return mv == ND_U16 ? ND_U16 : (uint16_t)(mv * VBAT_DIV);
}

// ---------- Causa del reset ----------
// Va letta prima di abilitare il watchdog: RUI3 azzera i flag nell'abilitarlo.
uint8_t readResetCause() {
  uint32_t csr = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;          // i flag si accumulano fino a che non si azzerano
  // Ogni reset interno attiva anche PINRSTF: l'ordine dei controlli conta
  if (csr & RCC_CSR_IWDGRSTF) return RST_WATCHDOG;
  if (csr & RCC_CSR_WWDGRSTF) return RST_WWDG;
  if (csr & RCC_CSR_LPWRRSTF) return RST_LOWPOWER;
  if (csr & RCC_CSR_SFTRSTF)  return RST_SOFTWARE;
  if (csr & RCC_CSR_BORRSTF)  return RST_ALIM;
  if (csr & RCC_CSR_PINRSTF)  return RST_PIN;
  return RST_ALTRO;
}

const char *resetName(uint8_t c) {
  switch (c) {
    case RST_ALIM:     return "alimentazione";
    case RST_PIN:      return "pin NRST";
    case RST_SOFTWARE: return "software";
    case RST_WATCHDOG: return "watchdog";
    case RST_WWDG:     return "watchdog a finestra";
    case RST_LOWPOWER: return "basso consumo";
    default:           return "altro";
  }
}

// ---------- GPS ----------
uint32_t lastNmea = 0;               // millis() dell'ultima frase valida

void gpsPoll() {
  while (Serial1.available())
    if (nmeaFeed((char)Serial1.read())) lastNmea = millis();
}

// Comando proprietario CASIC: "$" + corpo + "*" + checksum
void gpsSend(const char *body) {
  uint8_t x = 0;
  for (const char *c = body; *c; c++) x ^= (uint8_t)*c;
  Serial1.printf("$%s*%02X\r\n", body, x);
}

// Solo GGA e RMC, una volta al secondo: meno caratteri da leggere, nessuna
// frase persa mentre si legge il sensore. Modello dinamico per il volo.
// Le impostazioni non si salvano: si ripetono a ogni avvio.
void gpsConfigure() {
  gpsSend("PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0,,,,0");
  delay(100);
  char cmd[16];
  snprintf(cmd, sizeof(cmd), "PCAS11,%u", GPS_NAV_MODE);
  gpsSend(cmd);
}

// ---------- Autodiagnosi ----------
uint8_t errors = 0, fixes = 0;

#define CHECK(name, cur, setter, want)                                            \
  do {                                                                            \
    unsigned long c_ = (unsigned long)(cur), w_ = (unsigned long)(want);          \
    if (c_ == w_) { DBG("  %-10s %lu OK\r\n", name, w_); }                        \
    else if (setter) { DBG("  %-10s %lu -> %lu CORRETTO\r\n", name, c_, w_); fixes++; } \
    else { DBG("  %-10s ERRORE impostazione (%lu)\r\n", name, c_); errors++; }     \
  } while (0)

// pbw.get() di RUI3 4.3.0 restituisce l'indice della banda (0 = 125 kHz,
// 1 = 250, 2 = 500), pbw.set() accetta i kHz: si confronta in kHz
uint32_t bwKhz(uint32_t v) {
  switch (v) {
    case 0:  return 125;
    case 1:  return 250;
    case 2:  return 500;
    default: return v;
  }
}

void selfTest() {
  DBG("\r\n=== Autodiagnosi sonda %u, firmware %s ===\r\n", PROBE_ID, FW_VERSION);
  DBG("[Avvio]\r\n  Causa reset: %s\r\n", resetName(resetCause));

  // 1. Modalità radio
  DBG("[Radio]\r\n");
  if (api.lorawan.nwm.get() != 0) {
    if (api.lora.nwm.set()) {
      DBG("  Modalità LoRaWAN: passo a P2P e riavvio...\r\n");
      delay(200);
      api.system.reboot();
    }
    // Senza P2P non si riavvia in continuazione: segnala e prosegue
    DBG("  Modalità   ERRORE passaggio a P2P\r\n"); errors++;
  } else {
    DBG("  Modalità   P2P OK\r\n");
  }

  // 2. Parametri P2P (salvati in flash, corretti solo se diversi)
  CHECK("Frequenza", api.lora.pfreq.get(), api.lora.pfreq.set(P2P_FREQ), P2P_FREQ);
  CHECK("SF",        api.lora.psf.get(),   api.lora.psf.set(P2P_SF),     P2P_SF);
  CHECK("Banda",     bwKhz(api.lora.pbw.get()), api.lora.pbw.set(P2P_BW), P2P_BW);
  CHECK("CR",        api.lora.pcr.get(),   api.lora.pcr.set(P2P_CR),     P2P_CR);
  CHECK("Preambolo", api.lora.ppl.get(),   api.lora.ppl.set(P2P_PL),     P2P_PL);
  CHECK("Potenza",   api.lora.ptp.get(),   api.lora.ptp.set(P2P_TXP),    P2P_TXP);
  DBG("  Tempo in aria %lu ms, duty cycle %lu,%lu %%\r\n", AIRTIME_US / 1000,
      AIRTIME_US / (INTERVAL_MS * 10), AIRTIME_US / INTERVAL_MS % 10);

  // 3. GPS: deve mandare frasi NMEA valide entro qualche secondo
  DBG("[GPS]\r\n");
  uint32_t t0 = millis();
  while (lastNmea == 0 && millis() - t0 < GPS_TIMEOUT_MS) { gpsPoll(); delay(5); }
  if (lastNmea == 0) {
    DBG("  GPS        ERRORE nessuna frase NMEA in %lu s (collegamenti? alimentazione? JP2?)\r\n",
        GPS_TIMEOUT_MS / 1000);
    errors++;
  } else {
    DBG("  GPS        OK, configuro GGA + RMC e modello dinamico %u\r\n", GPS_NAV_MODE);
  }
  gpsConfigure();

  // 4. Sensore
  DBG("[Sensore]\r\n");
  if (!bmeInit()) errors++;

  // 5. Batteria
  DBG("[Batteria]\r\n");
  uint16_t vbat = readVbat();
  if (vbat == ND_U16)          { DBG("  LiPo ERRORE lettura ADC\r\n"); errors++; }
  else if (vbat < VBAT_MIN_MV) { DBG("  LiPo %u mV BASSA: ricaricare prima del lancio\r\n", vbat); errors++; }
  else if (vbat > VBAT_MAX_MV) { DBG("  LiPo %u mV ALTA (partitore? USB collegata?)\r\n", vbat); errors++; }
  else                           DBG("  LiPo %u mV OK\r\n", vbat);

  DBG("=== Esito: %u errori, %u correzioni. Intervallo %lu ms ±%lu ms ===\r\n\r\n",
      errors, fixes, INTERVAL_MS, JITTER_MS);
}

// Seme del generatore casuale diverso per ogni modulo (DevEUI) e per ogni avvio
void seedRandom() {
  uint8_t eui[8] = {0};
  api.lorawan.deui.get(eui, 8);
  uint32_t s = PROBE_ID;
  for (uint8_t i = 0; i < 8; i++) s = s * 31 + eui[i];
  randomSeed(s ^ readVbat() ^ millis());
}

// ---------- LED ----------
// A ogni pacchetto: un lampo senza fix, due lampi con il fix. Serve a terra,
// prima del lancio; in volo consuma qualche decina di µA in media.
uint32_t ledStart = 0;
bool ledDouble = false;

void ledUpdate() {
  uint32_t e = millis() - ledStart;
  bool on = e < 40 || (ledDouble && e >= 160 && e < 200);
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

// ---------- Trasmissione ----------
uint32_t lastTx = 0, nextWait = 0;
uint32_t usedEpoch = 0;
uint32_t nTxBusy = 0;

void transmit() {
  // GPS muto: niente dati vecchi nel pacchetto
  GpsData g = gps;
  if (millis() - lastNmea > GPS_TIMEOUT_MS) gpsClear(g);
  usedEpoch = gpsEpochs;

  int16_t t; uint16_t rh, p;
  bool ok = sensorRead(t, rh, p);
  if (!ok) { t = ND_I16; rh = ND_U16; p = ND_U16; }
  uint16_t vbat = readVbat();

  uint8_t pl[PKT_LEN];
  buildPacket(pl, seq, resetCause, g, t, rh, p, vbat);

  DBG("seq=%u ", seq);
  if (g.quality > 0)
    DBG("fix=%u sat=%u lat=%ld lon=%ld alt=%d m v=%u cm/s rotta=%u ", g.quality, g.sats,
        (long)g.lat, (long)g.lon, g.alt, g.speed, g.course);
  else
    DBG("NESSUN FIX sat=%u ", g.sats);
  if (ok) DBG("T=%.2f C UR=%.2f %% P=%.1f hPa ", t / 100.0f, rh / 100.0f, p / 10.0f);
  else    DBG("BME280 NON RISPONDE ");
  DBG("LiPo=%u mV ", vbat);
  if (seq == 0) DBG("(avvio: %s) ", resetName(resetCause));

  // Il contatore avanza anche se l'invio fallisce: il salto segnala il pacchetto mancato
  seq = (seq == 255) ? 1 : seq + 1;

  if (!txDone) {
    // La trasmissione precedente non è finita: non succede con l'intervallo
    // verificato dallo static_assert, se succede lo si conta
    nTxBusy++;
    DBG("RADIO OCCUPATA (%lu)\r\n", nTxBusy);
  } else {
    txDone = false;
    if (api.lora.psend(sizeof(pl), pl)) DBG("TX\r\n");
    else { txDone = true; DBG("ERRORE psend\r\n"); }
  }

  ledStart = millis();
  ledDouble = g.quality > 0;
}

// ---------- Main ----------
void setup() {
  resetCause = readResetCause();     // prima di tutto, prima del watchdog
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);       // acceso durante l'autodiagnosi
  Serial.begin(115200);
  Serial1.begin(GPS_BAUD, RAK_CUSTOM_MODE);
  gpsClear(gps);
  gpsClear(gpsNext);
  delay(1500);                       // tempo per aprire il monitor seriale
  Wire.begin();
  api.lora.registerPSendCallback(sendDone);
  selfTest();
  seedRandom();
  digitalWrite(LED_PIN, LOW);
  api.system.wdt.enable(WDT_MS);     // da qui la sonda si riavvia se resta bloccata più di 8 s
}

void loop() {
  api.system.wdt.reset();
  gpsPoll();

  // Si trasmette alla prima epoca GPS completa dopo l'intervallo, così i dati
  // sono freschi; se il GPS tace, comunque entro GPS_WAIT_MS
  uint32_t since = millis() - lastTx;
  if (since >= nextWait && (gpsEpochs != usedEpoch || since >= nextWait + GPS_WAIT_MS)) {
    lastTx = millis();
    nextWait = INTERVAL_MS - JITTER_MS + (uint32_t)random(2 * JITTER_MS + 1);
    transmit();
  }

  ledUpdate();
  delay(5);
}
