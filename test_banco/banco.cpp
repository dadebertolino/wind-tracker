#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <string>
// ---- stub minimi al posto di RUI3 ----
std::string out;
struct SerialStub {
  int printf(const char *f, ...) { char b[512]; va_list a; va_start(a, f); int n = vsnprintf(b, sizeof b, f, a); va_end(a); out += b; return n; }
  void println(const char *s) { out += s; out += "\r\n"; }
} Serial;
struct { uint32_t CSR; } RCCs;
#define RCC (&RCCs)
#define RCC_CSR_RMVF (1u<<23)
#define RCC_CSR_IWDGRSTF (1u<<29)
#define RCC_CSR_WWDGRSTF (1u<<30)
#define RCC_CSR_LPWRRSTF (1u<<31)
#define RCC_CSR_SFTRSTF (1u<<28)
#define RCC_CSR_BORRSTF (1u<<27)
#define RCC_CSR_PINRSTF (1u<<26)
enum { LORA_P2P_RXDONE = 0, LORA_P2P_RXTIMEOUT, LORA_P2P_RXERROR };
struct rui_lora_p2p_recv_t { uint8_t *Buffer; uint16_t BufferSize; int16_t Rssi; int8_t Snr; int Status; };
#include "estratto.inc"
// ---- test ----
int fails = 0;
#define EXPECT(cond, msg) do { if (cond) printf("ok    %s\n", msg); else { printf("FALLITO %s\n", msg); fails++; } } while (0)

// Passa un pacchetto per la callback e la decodifica della base; restituisce l'uscita
std::string run(const uint8_t *b, uint8_t n, int status = LORA_P2P_RXDONE) {
  out.clear();
  rui_lora_p2p_recv_t d{(uint8_t *)b, n, -72, 9, status};
  base::recvCallback(d);
  while (base::qTail != base::qHead) { base::handlePacket(base::queue[base::qTail]); base::qTail = (base::qTail + 1) & (QUEUE_LEN - 1); }
  return out;
}
bool has(const std::string &s, const std::string &x) { return s.find(x) != std::string::npos; }

// Frase NMEA con checksum, inviata alla sonda carattere per carattere
std::string nmea(const char *body) {
  uint8_t x = 0;
  for (const char *c = body; *c; c++) x ^= (uint8_t)*c;
  char cs[8]; snprintf(cs, sizeof cs, "*%02X\r\n", x);
  return std::string("$") + body + cs;
}
int feed(const std::string &s) {
  int n = 0;
  for (char c : s) n += sonda::nmeaFeed(c);
  return n;
}

int main() {
  const uint8_t v[] = "123456789";
  EXPECT(sonda::crc16(v, 9) == 0x29B1, "CRC sonda: vettore standard 0x29B1");
  EXPECT(base::crc16(v, 9) == 0x29B1, "CRC base: vettore standard 0x29B1");
  EXPECT(AIRTIME_US == 133632, "tempo in aria: 32 byte a SF8/125 kHz = 133,6 ms");

  // ---------- parser NMEA ----------
  using sonda::gps;
  sonda::gpsClear(sonda::gps); sonda::gpsClear(sonda::gpsNext);
  uint32_t ep = sonda::gpsEpochs;
  feed(nmea("GNGGA,123519.000,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"));
  EXPECT(sonda::gpsEpochs == ep, "solo GGA: epoca non ancora completa");
  feed(nmea("GNRMC,123519.000,A,4807.038,N,01131.000,E,022.4,084.4,230394,,,A"));
  EXPECT(sonda::gpsEpochs == ep + 1, "GGA + RMC con la stessa ora: epoca completa");
  EXPECT(gps.tod == 12 * 3600 + 35 * 60 + 19, "ora 12:35:19");
  EXPECT(gps.lat == 481173000 && gps.lon == 115166667, "lat 48.1173000, lon 11.5166667");
  EXPECT(gps.alt == 545 && gps.quality == 1 && gps.sats == 8 && gps.hdop == 9, "quota 545 m, fix 1, 8 satelliti, HDOP 0.9");
  EXPECT(gps.speed == 1152 && gps.course == 8440, "22.4 nodi = 1152 cm/s, rotta 84.40");

  feed(nmea("GPGGA,000001,3352.1234,S,15112.5678,W,2,12,1.2,-5.6,M,,M,,"));
  feed(nmea("GPRMC,000001,A,3352.1234,S,15112.5678,W,0.0,,010125,,,A"));
  EXPECT(gps.lat == -338687233 && gps.lon == -1512094633 && gps.alt == -6,
         "emisferi S e W: coordinate negative; quota -5.6 -> -6");
  EXPECT(gps.speed == 0 && gps.course == ND_U16, "fermo: velocità 0, rotta non disponibile");

  feed(nmea("GNGGA,101010.000,,,,,0,03,,,,,,,"));
  feed(nmea("GNRMC,101010.000,V,,,,,,,,,,N"));
  EXPECT(gps.tod == 36610 && gps.quality == 0 && gps.sats == 3 && gps.lat == ND_COORD &&
         gps.alt == ND_I16 && gps.speed == ND_U16 && gps.hdop == ND_U8,
         "ora nota ma senza fix: posizione, quota, velocità non disponibili");

  ep = sonda::gpsEpochs;
  feed(nmea("GNGGA,,,,,,0,00,25.5,,,,,,"));
  feed(nmea("GNRMC,,V,,,,,,,,,,N"));
  EXPECT(sonda::gpsEpochs == ep + 1 && gps.tod == ND_TOD && gps.hdop == 254,
         "GPS appena acceso: epoca completa, ora non disponibile, HDOP 25.5 -> 254");

  ep = sonda::gpsEpochs;
  uint32_t bad = sonda::nmeaBad;
  std::string s = nmea("GNGGA,123520.000,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
  s[10] = '9';                                   // un carattere alterato
  feed(s);
  feed(nmea("GNRMC,123520.000,A,4807.038,N,01131.000,E,022.4,084.4,230394,,,A"));
  EXPECT(sonda::nmeaBad == bad + 1 && sonda::gpsEpochs == ep, "checksum errato: frase scartata, epoca incompleta");

  feed(nmea("GNGGA,123521.000,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"));
  EXPECT(sonda::gpsEpochs == ep, "GGA e RMC di secondi diversi: nessuna epoca");
  feed(nmea("GNRMC,123521.000,A,4807.038,N,01131.000,E,022.4,084.4,230394,,,A"));
  EXPECT(sonda::gpsEpochs == ep + 1 && gps.tod == 45321, "poi GGA e RMC dello stesso secondo: epoca completa");

  ep = sonda::gpsEpochs;
  feed(nmea("GNGSV,3,1,12,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45"));
  feed(nmea("GNZDA,123522.000,27,10,2026,00,00"));
  feed(std::string(300, 'x') + "\r\n$GNGGA,12\r\n" + "$$$,,,*ZZ\r\n" + "\r\n\r\n");
  EXPECT(sonda::gpsEpochs == ep, "frasi di altri tipi, troppo lunghe o spezzate: ignorate");
  feed(nmea("GNGGA,123523.000,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"));
  feed(nmea("GNRMC,123523.000,A,4807.038,N,01131.000,E,022.4,084.4,230394,,,A"));
  EXPECT(sonda::gpsEpochs == ep + 1, "e il parser riparte con le frasi successive");

  // ---------- pacchetto: sonda -> base ----------
  sonda::GpsData g = gps;                         // 12:35:23, 48.1173 N 11.5166667 E
  uint8_t p[PKT_LEN];
  sonda::buildPacket(p, 17, RST_WATCHDOG, g, 1843, 6120, 9653, 3912);
  std::string o = run(p, PKT_LEN);
  EXPECT(PKT_LEN == 32 && has(o, "# sonda 1: fix GPS acquisito (8 satelliti)\r\n"), "primo fix: segnalato");
  EXPECT(has(o, "1,17,,12:35:23,48.1173000,11.5166667,545,11.52,84.40,18.43,61.20,965.3,3912,8,1,0.9,-72,9\r\n"),
         "pacchetto completo; reset vuoto fuori da seq 0");
  o = run(p, PKT_LEN);
  EXPECT(!has(o, "fix GPS"), "fix invariato: nessun messaggio");

  g.lat = -338687233; g.lon = -1512094633; g.alt = -6; g.sats = 20; g.quality = 2;
  sonda::buildPacket(p, 18, 0, g, -1234, 9999, 6012, 3700);
  o = run(p, PKT_LEN);
  EXPECT(has(o, "1,18,,12:35:23,-33.8687233,-151.2094633,-6,11.52,84.40,-12.34,99.99,601.2,3700,15,2,0.9,-72,9\r\n"),
         "coordinate e quota negative, T negativa, satelliti oltre 15 -> 15");

  sonda::GpsData nd; sonda::gpsClear(nd);
  sonda::buildPacket(p, 0, RST_ALIM, nd, ND_I16, ND_U16, ND_U16, ND_U16);
  o = run(p, PKT_LEN);
  EXPECT(has(o, "# sonda 1: riavvio (causa: alimentazione)") && has(o, "il sensore non risponde") &&
         has(o, "# sonda 1: fix GPS perso (0 satelliti)"), "avvio senza GPS e sensore: messaggi");
  EXPECT(has(o, std::string("1,0,1") + ",,,,,,,,,," + ",0,0,,-72,9\r\n"), "tutto non disponibile: campi vuoti");

  nd.tod = 0;
  sonda::buildPacket(p, 255, 0, nd, 2000, 5000, 9600, 3800);
  o = run(p, PKT_LEN);
  EXPECT(has(o, "1,255,,00:00:00,"), "contatore 255, mezzanotte");

  for (int i = 0; i < PKT_LEN; i++) {             // ogni singolo byte alterato va scartato
    sonda::buildPacket(p, 9, 0, g, 2000, 5000, 9600, 3800);
    p[i] ^= 0x10;
    o = run(p, PKT_LEN);
    if (!has(o, "CRC errato")) { printf("FALLITO byte %d alterato non rilevato\n", i); fails++; }
  }
  printf("ok    alterazione di ciascuno dei 32 byte rilevata dal CRC\n");

  sonda::buildPacket(p, 9, 0, g, 2000, 5000, 9600, 3800);
  o = run(p, PKT_LEN - 1);
  EXPECT(has(o, "CRC errato"), "pacchetto troncato scartato");

  sonda::buildPacket(p, 9, 0, g, 2000, 5000, 9600, 3800);
  p[0] = 2 << 4 | 1; { uint16_t c = base::crc16(p, 30); p[30] = c >> 8; p[31] = c; }
  o = run(p, PKT_LEN);
  EXPECT(has(o, "formato v2"), "versione sconosciuta scartata anche con CRC valido");

  uint8_t meteo[15] = {1 << 4 | 2, 7, 1 << 4, 0x07, 0x33, 0x17, 0xE8, 0x25, 0xB5, 0x0B, 0xB8, 0x0B, 0x54};
  { uint16_t c = base::crc16(meteo, 13); meteo[13] = c >> 8; meteo[14] = c; }
  o = run(meteo, 15);
  EXPECT(has(o, "tipo 1, 15 byte"), "pacchetto della stazione meteo (15 byte, CRC valido) scartato");

  sonda::buildPacket(p, 1, 0, g, 2000, 5000, 9600, 3800);
  uint32_t before = base::nRadioErr;
  o = run(p, PKT_LEN, LORA_P2P_RXERROR);
  EXPECT(base::nRadioErr == before + 1 && o.empty(), "callback con Status errore: ignorata, nessuna riga");

  out.clear(); uint32_t ov = base::nOverflow;
  for (int i = 0; i < 9; i++) { rui_lora_p2p_recv_t d{p, PKT_LEN, -80, 5, LORA_P2P_RXDONE}; base::recvCallback(d); }
  EXPECT(base::nOverflow == ov + 2, "coda: 7 pacchetti accodati, 2 contati come coda piena");
  base::qTail = base::qHead;

  RCCs.CSR = RCC_CSR_IWDGRSTF | RCC_CSR_PINRSTF;  EXPECT(base::readResetCause() == 4, "reset: watchdog prevale sul pin");
  RCCs.CSR = RCC_CSR_BORRSTF | RCC_CSR_PINRSTF;   EXPECT(base::readResetCause() == 1, "reset: accensione/BOR");

  printf(fails ? "\n%d TEST FALLITI\n" : "\nTutti i test superati\n", fails);
  return fails != 0;
}
