#!/bin/sh
# Prove al banco senza schede: estrae dagli sketch la costruzione del pacchetto
# e il parser NMEA (sonda) e la verifica/decodifica (base), le compila sul PC
# con stub minimi al posto di RUI3 ed esegue i test.
# Uso: sh esegui.sh   (serve un compilatore C++17 e python3)
set -e
cd "$(dirname "$0")"
python3 estrai.py ../sonda/sonda.ino ../base/base.ino estratto.inc
c++ -std=c++17 -Wall -Wextra -o banco banco.cpp
./banco
