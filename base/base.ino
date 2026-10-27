/*
 * Base station LoRa P2P - RAK3172 (RUI3 >= 4.1)
 * Autore: Davide Bertolino
 * Firmware 0.1 (aggiornare insieme a FW_VERSION)
 *
 * Riceve i pacchetti delle sonde (formato v1, 32 byte), li verifica e li stampa
 * in CSV su seriale per il programma sul PC. Deriva dal ricevitore della
 * stazione meteo LoRa (firmware 1.2), con altri parametri radio e altro pacchetto.
 *
 * All'avvio: autodiagnosi con correzione automatica di modalità e parametri P2P
 * (devono essere IDENTICI a quelli delle sonde).
 *
 * Uscita CSV:
 *   id,seq,reset,ora_utc,lat,lon,alt_m,vel_ms,rotta_deg,temp_C,ur_pct,press_hPa,vbat_mV,sat,fix,hdop,rssi,snr
 * Le misure non disponibili escono come campi vuoti; "reset" è valorizzato solo
 * nel primo pacchetto dopo un riavvio della sonda (seq 0). Tutte le altre righe
 * iniziano con '#' e vanno ignorate da chi legge i dati.
 *
 * La callback di ricezione gira nell'interrupt della radio: lì il pacchetto viene
 * solo copiato in una coda, verifica e stampa avvengono in loop().
 * LED D2 su PA8: un lampo a ogni pacchetto valido.
 */

// ---------- Configurazione (uguale alle sonde) ----------
#define P2P_FREQ  869525000UL
#define P2P_SF    8
#define P2P_BW    125
#define P2P_CR    0
#define P2P_PL    8
#define P2P_TXP   20
// --------------------------------------------------------

#define FW_VERSION    "0.1"

// Formato del pacchetto (uguale alle sonde)
#define FMT_VERSION   1
#define TYPE_SONDA    1
#define LEN_SONDA     32

// Valori "non disponibile" nel payload (uguali alle sonde)
#define ND_I16    ((int16_t)0x8000)
#define ND_U16    0xFFFF
#define ND_TOD    0xFFFFFFUL
#define ND_COORD  0x7FFFFFFFL
#define ND_U8     0xFF

#define WDT_MS        8000
#define STATS_MS      600000UL      // statistiche e riavvio della ricezione ogni 10 min
#define START_WAIT_MS 10000UL       // attesa dopo il messaggio di benvenuto, prima di partire

// ---------- Coda fra interrupt e loop ----------
#define QUEUE_LEN     8             // potenza di 2
#define MAX_PKT       32

struct RxPacket { uint8_t len; int16_t rssi; int8_t snr; uint8_t buf[MAX_PKT]; };
RxPacket queue[QUEUE_LEN];
volatile uint8_t qHead = 0, qTail = 0;

// Contatori (aggiornati anche dall'interrupt)
volatile uint32_t nRadioErr = 0, nOverflow = 0, nTooLong = 0;
uint32_t nValid = 0, nCrc = 0, nFormat = 0;

// Ultima qualità del fix per sonda: si segnala quando il fix arriva o si perde
uint8_t lastFix[16];

uint8_t errors = 0, fixes = 0;

#define CHECK(name, cur, setter, want)                                                     \
  do {                                                                                     \
    unsigned long c_ = (unsigned long)(cur), w_ = (unsigned long)(want);                   \
    if (c_ == w_) { Serial.printf("#  %-10s %lu OK\r\n", name, w_); }                      \
    else if (setter) { Serial.printf("#  %-10s %lu -> %lu CORRETTO\r\n", name, c_, w_); fixes++; } \
    else { Serial.printf("#  %-10s ERRORE impostazione (%lu)\r\n", name, c_); errors++; }  \
  } while (0)

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

// ---------- Causa del reset ----------
const char *resetName(uint8_t c) {
  switch (c) {
    case 1:  return "alimentazione";
    case 2:  return "pin NRST";
    case 3:  return "software";
    case 4:  return "watchdog";
    case 5:  return "watchdog a finestra";
    case 6:  return "basso consumo";
    default: return "altro";
  }
}

// Va letta prima di abilitare il watchdog, che azzera i flag
uint8_t readResetCause() {
  uint32_t csr = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;
  if (csr & RCC_CSR_IWDGRSTF) return 4;
  if (csr & RCC_CSR_WWDGRSTF) return 5;
  if (csr & RCC_CSR_LPWRRSTF) return 6;
  if (csr & RCC_CSR_SFTRSTF)  return 3;
  if (csr & RCC_CSR_BORRSTF)  return 1;
  if (csr & RCC_CSR_PINRSTF)  return 2;
  return 7;
}

// ---------- Ricezione (interrupt) ----------
void recvCallback(rui_lora_p2p_recv_t data) {
  // Su errore CRC radio RUI3 richiama la callback con il buffer del pacchetto
  // precedente: va scartata, altrimenti ricompare un duplicato
  if (data.Status != LORA_P2P_RXDONE) { nRadioErr++; return; }
  if (data.BufferSize > MAX_PKT)      { nTooLong++;  return; }

  uint8_t next = (qHead + 1) & (QUEUE_LEN - 1);
  if (next == qTail) { nOverflow++; return; }
  RxPacket &p = queue[qHead];
  p.len = data.BufferSize;
  p.rssi = data.Rssi;
  p.snr = data.Snr;
  memcpy(p.buf, data.Buffer, data.BufferSize);
  qHead = next;
}

// ---------- Decodifica (loop) ----------
static inline uint16_t be16(const uint8_t *b) { return (uint16_t)(b[0] << 8 | b[1]); }
static inline uint32_t be24(const uint8_t *b) { return (uint32_t)b[0] << 16 | (uint32_t)b[1] << 8 | b[2]; }
static inline uint32_t be32(const uint8_t *b) { return (uint32_t)b[0] << 24 | be24(b + 1); }

// Gradi x 10^7 -> "45.1234567", senza passare per float (non basta la precisione)
void fmtDeg(char *s, size_t n, int32_t v) {
  uint32_t a = v < 0 ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
  snprintf(s, n, "%s%lu.%07lu", v < 0 ? "-" : "", (unsigned long)(a / 10000000UL),
           (unsigned long)(a % 10000000UL));
}

// true se il pacchetto è valido (stampato come riga CSV)
bool handlePacket(const RxPacket &p) {
  const uint8_t *b = p.buf;
  uint8_t n = p.len;

  // Il CRC protegge anche dai pacchetti estranei con la stessa configurazione radio
  if (n < 5 || crc16(b, n - 2) != be16(b + n - 2)) {
    nCrc++;
    Serial.printf("# pacchetto scartato: CRC errato (%u byte, RSSI %d)\r\n", n, p.rssi);
    return false;
  }

  uint8_t ver = b[0] >> 4, id = b[0] & 0x0F, seq = b[1];
  uint8_t type = b[2] >> 4, rst = b[2] & 0x0F;
  if (ver != FMT_VERSION || id == 0 || type != TYPE_SONDA || n != LEN_SONDA) {
    nFormat++;
    Serial.printf("# pacchetto scartato: formato v%u, sonda %u, tipo %u, %u byte\r\n", ver, id, type, n);
    return false;
  }
  nValid++;

  uint32_t tod  = be24(b + 3);
  int32_t  lat  = (int32_t)be32(b + 6);
  int32_t  lon  = (int32_t)be32(b + 10);
  int16_t  alt  = (int16_t)be16(b + 14);
  uint16_t spd  = be16(b + 16);
  uint16_t crs  = be16(b + 18);
  int16_t  t    = (int16_t)be16(b + 20);
  uint16_t rh   = be16(b + 22);
  uint16_t p10  = be16(b + 24);
  uint16_t vbat = be16(b + 26);
  uint8_t  sats = b[28] >> 4, fix = b[28] & 0x0F, hdop = b[29];

  // Campi vuoti per i valori non disponibili
  char sTod[10] = "", sLat[14] = "", sLon[14] = "", sAlt[8] = "", sSpd[10] = "", sCrs[10] = "";
  char sT[10] = "", sRh[10] = "", sP[10] = "", sV[8] = "", sHdop[8] = "", sRst[4] = "";
  if (tod  < 86400)    snprintf(sTod, sizeof(sTod), "%02lu:%02lu:%02lu", (unsigned long)(tod / 3600),
                                (unsigned long)(tod / 60 % 60), (unsigned long)(tod % 60));
  if (lat  != ND_COORD) fmtDeg(sLat, sizeof(sLat), lat);
  if (lon  != ND_COORD) fmtDeg(sLon, sizeof(sLon), lon);
  if (alt  != ND_I16)  snprintf(sAlt,  sizeof(sAlt),  "%d", alt);
  if (spd  != ND_U16)  snprintf(sSpd,  sizeof(sSpd),  "%.2f", spd / 100.0f);
  if (crs  != ND_U16)  snprintf(sCrs,  sizeof(sCrs),  "%.2f", crs / 100.0f);
  if (t    != ND_I16)  snprintf(sT,    sizeof(sT),    "%.2f", t / 100.0f);
  if (rh   != ND_U16)  snprintf(sRh,   sizeof(sRh),   "%.2f", rh / 100.0f);
  if (p10  != ND_U16)  snprintf(sP,    sizeof(sP),    "%.1f", p10 / 10.0f);
  if (vbat != ND_U16)  snprintf(sV,    sizeof(sV),    "%u", vbat);
  if (hdop != ND_U8)   snprintf(sHdop, sizeof(sHdop), "%.1f", hdop / 10.0f);
  if (seq == 0)        snprintf(sRst,  sizeof(sRst),  "%u", rst);

  if (seq == 0)
    Serial.printf("# sonda %u: riavvio (causa: %s)\r\n", id, resetName(rst));
  if (t == ND_I16 && rh == ND_U16)
    Serial.printf("# sonda %u: il sensore non risponde\r\n", id);
  if ((fix > 0) != (lastFix[id] > 0))
    Serial.printf(fix > 0 ? "# sonda %u: fix GPS acquisito (%u satelliti)\r\n"
                          : "# sonda %u: fix GPS perso (%u satelliti)\r\n", id, sats);
  lastFix[id] = fix;

  Serial.printf("%u,%u,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%u,%u,%s,%d,%d\r\n",
                id, seq, sRst, sTod, sLat, sLon, sAlt, sSpd, sCrs,
                sT, sRh, sP, sV, sats, fix, sHdop, p.rssi, p.snr);
  return true;
}

// ---------- Avvio ----------
// Messaggio di benvenuto, poi attesa con conto alla rovescia: tempo per aprire
// il monitor seriale e vedere l'avvio dall'inizio
void welcome() {
  Serial.println("#");
  Serial.printf("# Base station wind tracker, firmware %s\r\n", FW_VERSION);
  for (uint32_t s = START_WAIT_MS / 1000; s > 0; s -= 5) {
    Serial.printf("# avvio fra %lu s\r\n", (unsigned long)s);
    delay(5000);
  }
}

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

void selfTest(uint8_t rst) {
  Serial.println("#");
  Serial.printf("# === Autodiagnosi base station, firmware %s ===\r\n", FW_VERSION);
  Serial.printf("#  Causa reset: %s\r\n", resetName(rst));

  if (api.lorawan.nwm.get() != 0) {
    if (api.lora.nwm.set()) {
      Serial.println("#  Modalità LoRaWAN: passo a P2P e riavvio...");
      delay(200);
      api.system.reboot();
    }
    Serial.println("#  Modalità   ERRORE passaggio a P2P");
    errors++;
  } else {
    Serial.println("#  Modalità   P2P OK");
  }

  CHECK("Frequenza", api.lora.pfreq.get(), api.lora.pfreq.set(P2P_FREQ), P2P_FREQ);
  CHECK("SF",        api.lora.psf.get(),   api.lora.psf.set(P2P_SF),     P2P_SF);
  CHECK("Banda",     bwKhz(api.lora.pbw.get()), api.lora.pbw.set(P2P_BW), P2P_BW);
  CHECK("CR",        api.lora.pcr.get(),   api.lora.pcr.set(P2P_CR),     P2P_CR);
  CHECK("Preambolo", api.lora.ppl.get(),   api.lora.ppl.set(P2P_PL),     P2P_PL);
  CHECK("Potenza",   api.lora.ptp.get(),   api.lora.ptp.set(P2P_TXP),    P2P_TXP);

  Serial.printf("# === Esito: %u errori, %u correzioni ===\r\n", errors, fixes);
}

void startRx() {
  // 65534 = ricezione continua (non si ferma dopo il primo pacchetto)
  if (api.lora.precv(65534)) {
    Serial.println("# In ascolto...");
  } else {
    Serial.println("# ERRORE avvio ricezione");
  }
}

#define LED_PIN   PA8
#define LED_MS    50

void setup() {
  uint8_t rst = readResetCause();  // prima del watchdog
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  welcome();                       // e 10 s di attesa, prima del watchdog
  api.lora.registerPRecvCallback(recvCallback);
  selfTest(rst);                   // parametri corretti PRIMA di avviare la ricezione
  api.system.wdt.enable(WDT_MS);
  startRx();
  Serial.println("# id,seq,reset,ora_utc,lat,lon,alt_m,vel_ms,rotta_deg,temp_C,ur_pct,press_hPa,vbat_mV,sat,fix,hdop,rssi,snr");
}

void loop() {
  api.system.wdt.reset();

  static uint32_t ledOn = 0;
  while (qTail != qHead) {
    if (handlePacket(queue[qTail])) {
      digitalWrite(LED_PIN, HIGH);
      ledOn = millis();
    }
    qTail = (qTail + 1) & (QUEUE_LEN - 1);
  }
  if (ledOn && millis() - ledOn >= LED_MS) {
    digitalWrite(LED_PIN, LOW);
    ledOn = 0;
  }

  // Ogni 10 minuti: statistiche e riavvio della ricezione per sicurezza
  static uint32_t last = 0;
  if (millis() - last >= STATS_MS) {
    last = millis();
    Serial.printf("# stat: %lu validi, %lu CRC errato, %lu errori radio, %lu formato, %lu coda piena, %lu troppo lunghi\r\n",
                  nValid, nCrc, (unsigned long)nRadioErr, nFormat, (unsigned long)nOverflow, (unsigned long)nTooLong);
    api.lora.precv(0);             // ferma
    delay(50);
    startRx();                     // riparte
  }
  delay(10);
}
