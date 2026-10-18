# Test recordings

- `tetra_synthetic.wav` – **synthetic** TETRA downlink, generated with
  [`../gen/tetra_synth.cpp`](../gen/tetra_synth.cpp). Fictitious amateur radio
  cell: MCC 901, MNC 9999, colour code 1, downlink 438.2375 MHz, uplink 7.6 MHz
  below (duplex field 7). Stations ISSI 10000001…10000006 (8 digits, so no
  DMR ID can match), groups 9 and 91. Contains no data of any real network.
  32 kS/s, 2 × 16 bit, carrier at −3 kHz, 19 s.
- `dmr_sample.wav` – from SDR++Brown (DMR decoder tests).
