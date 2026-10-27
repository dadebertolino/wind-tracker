# Estrae dagli sketch le parti da provare al banco (vedi esegui.sh)
import sys
sonda, base, out = sys.argv[1:4]
def cut(path, start, end):
    s = open(path, encoding="utf-8").read()
    i = s.index(start); j = s.index(end, i)
    return s[i:j]
parts = [
  "// ===== estratto dalla sonda =====\n",
  cut(sonda, "#define PROBE_ID", "#if PROBE_DEBUG"),
  "namespace sonda {\n",
  cut(sonda, "// ---------- Dati del GPS", "uint8_t addr = 0;"),
  cut(sonda, "// ---------- CRC-16", "// ---------- Bus I2C"),
  "}\n",
  "// ===== estratto dalla base =====\n",
  "#undef FW_VERSION\n",  # sonda e base hanno versioni diverse
  "namespace base {\n", cut(base, "#define FW_VERSION", "// ---------- Avvio"), "}\n",
]
open(out, "w", encoding="utf-8").write("".join(parts))
