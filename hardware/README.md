# Hardware

Progetti KiCad del wind tracker. Licenza CERN-OHL-S v2
([`LICENSE`](LICENSE)): chi modifica e distribuisce questi progetti deve
pubblicarne i sorgenti con la stessa licenza.

| Scheda | Cartella | Stato |
|---|---|---|
| Sonda per il pallone pilota: RAK3172, ATGM336H, BME280, LiPo con ricarica USB-C | [`sonda/`](sonda/) | schema 0.1 (ERC senza errori), circuito stampato da disegnare |
| Base station USB, per PC | [`base/`](base/) | schema 0.1, uguale al ricevitore della stazione meteo (ERC: una segnalazione esclusa e motivata) |

## Struttura

```
hardware/
├── librerie/
│   ├── wind-tracker.kicad_sym    simboli nostri, condivisi dalle schede
│   └── wind-tracker.pretty/      impronte nostre
├── sonda/
│   ├── sonda.kicad_pro
│   ├── sonda.kicad_sch
│   ├── sym-lib-table             → librerie/wind-tracker.kicad_sym
│   └── fp-lib-table              → librerie/wind-tracker.pretty
└── base/                         stessa struttura
```

File in formato **KiCad 9**, si aprono anche con le versioni successive. Il
circuito stampato (`.kicad_pcb`) lo crea KiCad la prima volta che si apre
l'editor PCB dal progetto. Tutti i simboli e le impronte usati sono nella
libreria ufficiale di KiCad.

## Sonda, schema 0.1

| Parte | Soluzione |
|---|---|
| Modulo radio | U1 RAK3172 **senza IPEX**: uscita RF sul pad 12, rete a pi greco R11 (0 Ω) / C10 / C11 (non montati, da tarare se serve), pista a 50 Ω corta fino a J7 |
| Antenna | J7: filo da λ/4 (~8,6 cm a 869,5 MHz, da accorciare in prova) appeso verso il basso, e contrappeso da λ/4 su J7.2: niente connettori, pochi decimi di grammo |
| GPS | breakout ATGM336H saldata su J6 (VCC, GND, TX, RX, PPS); UART1 del RAK3172: PB6 → RX del GPS, PB7 ← TX del GPS; PPS su PB5 |
| Alimentazione del GPS | JP2: 1-2 (di serie) da +3V3; 2-3 da VSYS (3,0-4,7 V) **solo** se la breakout ha il suo regolatore e accetta 5 V |
| Sensore | BME280 a 6 pin senza regolatore su J1; I²C su PA12 (SCL) e PA11 (SDA); CSB a 3V3, SDO a massa (0x76) |
| Batteria | LiPo 1S da 150-250 mAh su J5 (JST PH 2 mm) |
| Inversione della LiPo | Q1 DMG2305UX sul positivo, come nel nodo BME280 della stazione meteo: i connettori JST PH delle LiPo non hanno una polarità standard |
| Ricarica | USB-C J4 solo alimentazione (R3, R4 da 5,1 kΩ su CC); U2 **MCP73831-2** a 4,20 V; R5 = 10 kΩ → 100 mA (1000 V / R5), adatta a celle da 100 mAh in su; D1 acceso durante la carica |
| Commutazione USB / batteria | D3 (Schottky B5819WS) da VBUS a VSYS, Q2 (DMG2305UX) da batteria a VSYS con il gate su VBUS e R7 a massa: con la USB il circuito si alimenta da lì e la carica termina regolarmente; senza USB, Q2 conduce |
| Regolatore | U3 AP2112K-3.3, lo stesso della base: 3,3 V per RAK3172, GPS e sensore; SW1 sul pin EN accende e spegne la sonda (da spenta la ricarica continua) |
| Tensione della LiPo | R8/R9 da 1 MΩ: metà della tensione su PB3 (ADC), C7 da 100 nF. Il partitore consuma ~2 µA anche da spento: oltre 5 anni per scaricare 100 mAh |
| Programmazione | J2 (UART2, BOOT0, NRST) e J3 (SWD) come nel nodo BME280, stesso adattatore. JP1 aperto: alimenta la sonda; chiuderlo solo per alimentare dall'adattatore senza LiPo e senza USB |
| LED | D1 carica (rosso), D2 su PA8 (verde): stato del fix a terra, spento in volo |

Componenti passivi in 0805 per la saldatura a mano, come nella stazione meteo:
il peso in più rispetto allo 0402 è trascurabile.

**Sul circuito stampato:** J1 (sensore) sul bordo, lontano da U2, U3 e dal
GPS, che scaldano; antenna patch del GPS rivolta verso l'alto; circuito
stampato da 0,8 mm.

Da verificare:
- codice d'ordine della variante RAK3172 senza IPEX;
- sulla breakout ATGM336H: regolatore, tensione minima, presenza della batteria
  di backup;
- polarità del connettore della LiPo acquistata, prima di collegarla;
- la LiPo non si carica sotto 0 °C: in inverno caricarla prima di uscire.

## Base station, schema 0.1

Copia dello schema 0.1 del ricevitore USB di
[stazione-meteo-lora](https://github.com/dadebertolino/stazione-meteo-lora/tree/main/hardware/ricevitore):
stessa netlist, cambiano solo cartiglio e note. RAK3172 con IPEX e antenna
esterna su u.FL: qui conta un'antenna verticale buona, montata in alto.

**Esclusione ERC** (ripresa dal ricevitore): il pin V3 del CH340C è definito
come uscita di alimentazione; a 3,3 V il datasheet lo vuole collegato a VCC,
quindi sulla stessa rete dell'uscita di U3. La segnalazione è esclusa, con la
motivazione, nel `.kicad_pro`.
