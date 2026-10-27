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
| `sonda/` | firmware della sonda | da scrivere |
| `base/` | firmware della base | da scrivere, partendo dal ricevitore della stazione meteo |
| `analisi/` | profilo del vento sul PC | da scrivere |

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
| Pacchetto | ~31 byte: 134 ms a SF8, 247 ms a SF9 |
| Intervallo | 2 s a SF8 (6,7 %), 5 s a SF9 (4,9 %) |

A 20 km in vista diretta la perdita è ~117 dB: con 20 dBm arrivano ~−97 dBm,
circa 29 dB sopra la sensibilità a SF8.

## Pacchetto (proposta)

| Campo | Byte |
|---|---|
| versione · ID, contatore, tipo | 3 |
| ora GPS, secondi del giorno | 3 |
| latitudine, longitudine, gradi × 10⁷ | 4 + 4 |
| quota GPS, m | 2 |
| velocità al suolo e rotta (Doppler del GPS) | 2 + 2 |
| temperatura, umidità, pressione | 2 + 2 + 2 |
| tensione della LiPo | 2 |
| satelliti · stato del fix | 1 |
| CRC-16/CCITT-FALSE, come nella stazione meteo | 2 |

La velocità e la rotta del GPS danno direttamente il vento, con più
precisione della differenza fra due posizioni; l'ora GPS permette di calcolare
le velocità anche quando si perdono pacchetti.

## Da verificare

- Limiti della sottobanda 869,40-869,65 MHz.
- Regole per i palloni liberi leggeri: ENAC e organizzazione del raduno o del
  campo di volo.
- La breakout ATGM336H: regolatore, tensione minima, backup per la partenza
  a caldo.
