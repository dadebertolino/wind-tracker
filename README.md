# Wind tracker

Profilo del vento per i voli in mongolfiera con un **pallone pilota** a elio.
Una sonda leggera con GPS, sensore BME280 e radio LoRa sale con il pallone e
trasmette posizione, velocità e misure ogni pochi secondi; una base station a
terra le riceve e il PC ricava velocità e direzione del vento a ogni quota.

**Autore:** [Davide Bertolino](https://www.davidebertolino.it) ·
**Licenze:** software GPL v2 o successive ([`LICENSE`](LICENSE)), hardware
CERN-OHL-S v2 ([`hardware/LICENSE`](hardware/LICENSE))

La base station è lo stesso circuito del ricevitore USB di
[stazione-meteo-lora](https://github.com/dadebertolino/stazione-meteo-lora);
cambia il firmware (frequenza, SF, formato dei pacchetti).

```
sonda sul pallone ──── LoRa P2P 869,525 MHz ────► base station ── USB ──► PC
RAK3172, ATGM336H,                                 RAK3172, CH340C           profilo del vento,
BME280, LiPo                                                                 traccia, file del lancio
```

| Componente | Ruolo | Stato |
|---|---|---|
| [`hardware/sonda/`](hardware/sonda/) | scheda in volo | schema 0.1 (ERC senza errori), circuito stampato da disegnare |
| [`hardware/base/`](hardware/base/) | base station USB | schema 0.1, copia del ricevitore della stazione meteo |
| [`sonda/`](sonda/sonda.ino) | firmware della sonda | 0.1, compila; da provare sulla scheda |
| [`base/`](base/base.ino) | firmware della base: verifica i pacchetti e li stampa in CSV | 0.1, compila; da provare sulla scheda |
| [`test_banco/`](test_banco/esegui.sh) | prove sul PC, senza schede: parser NMEA, pacchetto, decodifica | 33 test |
| `analisi/` | profilo del vento sul PC | da scrivere |

Gli sketch si compilano con il pacchetto di schede RAKwireless RUI3 (verificati
con la versione 4.3.0, scheda *WisDuo RAK3172 Evaluation Board*), senza
librerie esterne. **Sonda e base vanno aggiornate insieme.**

## Obiettivi

| | |
|---|---|
| Quota | fino a ~3 km (gli strati che interessano le mongolfiere) |
| Distanza | 10-20 km dalla base |
| Durata del volo | 15-30 minuti, salita a 3-5 m/s |
| Peso della sonda | 15-20 g con la batteria |
| Un punto ogni | 2-5 s, cioè 8-20 m di quota |

## Radio

La stazione meteo usa 868,1 MHz con duty cycle 1 %: troppo poco per un
pacchetto ogni 2 s. La sonda usa la sottobanda **869,40-869,65 MHz**, che
ammette 500 mW ERP e duty cycle 10 % (ERC/REC 70-03; da verificare il numero
della banda e l'uso consentito in Italia).

| Parametro | Proposta |
|---|---|
| Frequenza | 869,525 MHz, banda 125 kHz |
| Spreading factor | SF8 (in alternativa SF9) |
| Potenza | 20 dBm (100 mW) |
| Pacchetto | 32 byte: 134 ms a SF8, 247 ms a SF9 |
| Intervallo | 2 s a SF8 (6,7 %), 5 s a SF9 (4,9 %) |

A 20 km in vista diretta la perdita è ~117 dB: con 20 dBm arrivano ~−97 dBm,
circa 29 dB sopra la sensibilità a SF8.

## Pacchetto (formato v1, 32 byte)

| Campo | Byte |
|---|---|
| versione · ID, contatore, tipo | 3 |
| ora GPS, secondi del giorno | 3 |
| latitudine, longitudine, gradi × 10⁷ | 4 + 4 |
| quota GPS, m | 2 |
| velocità al suolo e rotta (Doppler del GPS) | 2 + 2 |
| temperatura, umidità, pressione | 2 + 2 + 2 |
| tensione della LiPo | 2 |
| satelliti (4 bit) · qualità del fix GGA (4 bit) | 1 |
| HDOP × 10 | 1 |
| CRC-16/CCITT-FALSE, come nella stazione meteo | 2 |

Interi big endian; il dettaglio dei byte è nell'intestazione di
[`sonda.ino`](sonda/sonda.ino). Il byte dell'HDOP porta il pacchetto da 31 a 32
byte senza allungare il tempo in aria (stesso gradino di simboli a SF8).

La velocità e la rotta del GPS danno direttamente il vento, con più
precisione della differenza fra due posizioni; l'ora GPS permette di calcolare
le velocità anche quando si perdono pacchetti.

## Sonda

| Costante | Valore | Significato |
|---|---|---|
| `PROBE_ID` | 1-15 | diverso per ogni sonda |
| `INTERVAL_MS` | `2000UL` | un pacchetto ogni 2 s ± 10 % a caso; lo `static_assert` calcola il tempo in aria e non compila oltre il 10 % di duty cycle |
| `PROBE_DEBUG` | `1` | stampe su seriale (UART2, 115200 baud); `0` per il volo |
| `GPS_NAV_MODE` | 5 | modello dinamico dell'ATGM336H (`$PCAS11`), aereo sotto 1 g |

**Ciclo.** Il GPS resta sempre acceso. La sonda legge le frasi NMEA (solo GGA
e RMC: le altre si spengono all'avvio con `$PCAS03`) e, passato l'intervallo,
trasmette alla prima epoca completa, cioè GGA e RMC dello stesso secondo, così
i dati sono freschi. Se il GPS tace trasmette lo stesso, con la posizione non
disponibile: dalla base si vede che la sonda è viva.

**Prima del lancio.** Il LED fa un lampo a ogni pacchetto senza fix e due lampi
con il fix. All'avvio l'autodiagnosi controlla radio, GPS, sensore e tensione
della LiPo (errore sotto 3,5 V).

Riprese dal nodo della stazione meteo: autodiagnosi e correzione dei parametri
radio, lettura del BME280 con sblocco del bus I2C, contatore e causa del reset,
watchdog a 8 s.

## Base station

Seriale a 115200 baud, una riga per pacchetto valido:

```
# id,seq,reset,ora_utc,lat,lon,alt_m,vel_ms,rotta_deg,temp_C,ur_pct,press_hPa,vbat_mV,sat,fix,hdop,rssi,snr
# sonda 1: fix GPS acquisito (8 satelliti)
1,17,,12:35:23,48.1173000,11.5166667,545,11.52,84.40,18.43,61.20,965.3,3912,8,1,0.9,-72,9
```

- Misure non disponibili come campi vuoti; `reset` solo con `seq` 0.
- Le righe che iniziano con `#` sono messaggi: avvio, riavvio di una sonda,
  fix acquisito o perso, sensore guasto, pacchetti scartati, statistiche ogni
  10 minuti.
- Latitudine e longitudine con 7 decimali, stampate da interi: nessun
  arrotondamento del float.
- `vel_ms` e `rotta_deg` sono velocità al suolo e direzione verso cui si muove
  il pallone (dal GPS): il vento **viene** dalla direzione opposta.
- Il LED D2 fa un lampo a ogni pacchetto valido.

Il resto è come il ricevitore della stazione meteo: coda fra interrupt e loop,
scarto dei richiami con `Status` di errore e dei pacchetti con CRC errato,
watchdog, riavvio della ricezione ogni 10 minuti.

## Prove

```bash
sh test_banco/esegui.sh
```

Estrae dagli sketch il parser NMEA e la costruzione del pacchetto della sonda e
la decodifica della base, li compila sul PC e verifica: frasi GGA/RMC vere,
emisferi S e W, GPS senza fix e appena acceso, checksum errato, GGA e RMC di
secondi diversi, frasi spezzate o troppo lunghe; pacchetto completo e con
valori negativi, tutto non disponibile, alterazione di ciascuno dei 32 byte,
pacchetto troncato, versione sconosciuta, pacchetto della stazione meteo,
callback con errore, coda piena; tempo in aria.

Da provare sulle schede:

| Prova | Cosa guardare |
|---|---|
| Avvio della sonda con monitor seriale | autodiagnosi; il GPS deve rispondere entro 3 s |
| GPS all'aperto | fix in meno di un minuto a freddo, due lampi del LED; righe con posizione sulla base |
| Comandi `$PCAS03` e `$PCAS11` | dopo l'avvio il GPS deve mandare solo GGA e RMC; se arrivano ancora GSV il comando non è stato accettato (innocuo) |
| Tensione della LiPo, multimetro sulla batteria | `vbat_mV` entro qualche decina di mV; il partitore da 1 MΩ e l'ADC non sono ancora stati provati insieme |
| Portata | a terra e poi in volo: RSSI, SNR, salti nel contatore |

## Da verificare

- Limiti della sottobanda 869,40-869,65 MHz.
- Regole per i palloni liberi leggeri: ENAC e organizzazione del raduno o del
  campo di volo.
- La breakout ATGM336H: regolatore, tensione minima, backup per la partenza
  a caldo.
