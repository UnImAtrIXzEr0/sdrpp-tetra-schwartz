# SDR++ TETRA-SCHWARTZ

**A TETRA receiver and measurement program for amateur radio** – based on
[SDR++](https://github.com/AlexandreRouma/SDRPlusPlus) and the
[SDR++Brown](https://github.com/sannysanoff/SDRPlusPlusBrown) fork, extended
for amateur radio TETRA cells by UnImAtrIXzEr0.

> **Experimental beta.** Not an official release of SDR++ or SDR++Brown and
> not endorsed by their authors. Please report problems with this edition
> only [here](https://github.com/UnImAtrIXzEr0/sdrpp-tetra-schwartz/issues).

[![Download the latest version](https://img.shields.io/github/v/release/UnImAtrIXzEr0/sdrpp-tetra-schwartz?label=download&style=for-the-badge)](https://github.com/UnImAtrIXzEr0/sdrpp-tetra-schwartz/releases/latest)
[![Build](https://img.shields.io/github/actions/workflow/status/UnImAtrIXzEr0/sdrpp-tetra-schwartz/build_tetra_custom.yml?branch=main&style=for-the-badge)](https://github.com/UnImAtrIXzEr0/sdrpp-tetra-schwartz/actions)
[![Licence GPL-3.0](https://img.shields.io/badge/licence-GPL--3.0-blue?style=for-the-badge)](license)

![Measuring station layout: spectrum and waterfall, tile column with constellation, measurements and trends, stations below](docs/readme/overview_tiles.png)

*All pictures show the synthetic test cell of this repository (MCC 901, MNC 9999) – no real network.*

## What it does

- **Decoding** of TETRA downlinks (TMO): cell, network and neighbour cell
  information, calls, talkers, SDS, status, group attachments, speech audio.
  Station names from the RadioID database and talkgroup names, both on request.
- **Measurements following ETSI EN 300 394-1** (most recent editions):
  modulation accuracy (vector error, residual carrier, 200 bursts), adjacent
  channel power with the limit tables, power and time mask of bursts,
  switching transients, carrier frequency with reference selection, MER and
  decoder EVM, I/Q offset and imbalance.
- **TX analysis**: spectrum through the TETRA filter, occupied bandwidth,
  shoulders, carrier leakage, 25 kHz channel raster.
- **Burst analysis of handhelds** (uplink alone, linearity check by an
  attenuator step), **multipath** (echo delay and level from the training
  sequences), **frequency stability**, **trends**, **alarms**, **HTML report**.
- **Drive tests**: GPS, CSV log, coverage maps for Google Earth (KML) and
  OpenStreetMap, propagation diagram around the base station.
- **Your own layout**: a tile column next to the waterfall, the content of
  every tile selectable, sizes adjustable, kept across restarts; touch view
  for tablets.
- Automatic RF gain that looks for the best signal-to-noise ratio, dBm
  calibration, snap to carrier, lock VFO.
- **No decryption.** Encrypted traffic stays encrypted.

| Constellation | TX analysis | Burst analysis |
|---|---|---|
| ![Constellation](docs/readme/win_const.png) | ![TX analysis](docs/readme/win_txa.png) | ![Burst analysis](docs/readme/win_bursts.png) |

| Network & cell | Trends | Coverage map |
|---|---|---|
| ![Network](docs/readme/win_network.png) | ![Trends](docs/readme/win_trends.png) | ![Map](docs/readme/map_level.png) |

## Download

On the [**Releases page**](https://github.com/UnImAtrIXzEr0/sdrpp-tetra-schwartz/releases/latest):

| File | For |
|---|---|
| `…_windows_x64.zip` | Windows 10 / 11 (64 bit) |
| `…_x86_64.AppImage` | Linux x86-64 (built on Ubuntu 24.04) |
| `…_Bedienungsanleitung_….pdf` | Manual, German |
| `…_User_Manual_….pdf` | Manual, English |
| `SHA256SUMS.txt` | Checksums of the downloads |

## Quick start

1. **Windows:** unpack the zip into a folder of its own and start `sdrpp.exe`.
   SDR drivers (e.g. for the USRP, PlutoSDR, RTL-SDR) as described in the manual.
   **Linux:** `chmod +x SDRPP-TETRA-SCHWARTZ_*.AppImage`, then start it.
2. Choose your SDR under **Source** and a sample rate (2–4 MS/s is enough for
   one carrier; uplink and downlink together need more than the duplex spacing).
3. Press **Play**, click on the TETRA carrier in the spectrum (**Snap to
   carrier** puts the VFO on its centre), switch on **TETRA Demodulator**.
4. Choose **ETSI** or **Amateur radio** (own duplex tables) at the top of the
   TETRA menu, **Layout…** for a tile layout (measuring station, drive test).
5. For comparable values: **Level and hold** for the gain, a dBm calibration
   and a frequency reference – see the manual, chapter “Setting up the SDR”.

Tips:
- Hover over a value: almost every value explains itself and names its standard.
- Values marked *indicative* are not a test per the standard (e.g. over the air
  instead of at the antenna connector); the manual says what is missing.
- Drag the grip (six dots) of a section header to reorder the menu.

## Feedback

- **Bugs:** [open an issue](https://github.com/UnImAtrIXzEr0/sdrpp-tetra-schwartz/issues/new/choose)
  with version, operating system, SDR and sample rate.
- **Questions, measurement experiences, ideas:** [Discussions](https://github.com/UnImAtrIXzEr0/sdrpp-tetra-schwartz/discussions).
- Please do not post data of real networks or other people (ISSIs, names,
  positions) in screenshots or logs.
- Pull requests are welcome; they are reviewed and merged by the maintainer.

## Legal notes

- For **experimental amateur radio purposes**: receive your own or amateur
  radio cells. Listening to other radio services may be prohibited where you
  live (Germany: § 5 TDDDG).
- Origin, authors, licences and the list of changes: [NOTICE.md](NOTICE.md).
  SDR++, SDR++Brown and the TETRA plugin are GPL-3.0, the osmo-tetra decoder
  parts AGPL-3.0. The TETRA speech codec is ETSI reference code (© ETSI),
  downloaded from etsi.org during the build.
- No warranty. Measured values of an SDR are no substitute for a calibrated
  test system.

## Kurz auf Deutsch

SDR++ TETRA-SCHWARTZ ist ein TETRA-Empfangs- und Messprogramm für den
Amateurfunk (experimentelle Beta). Download für Windows und Linux sowie die
**deutsche Bedienungsanleitung (PDF)** auf der
[Releases-Seite](https://github.com/UnImAtrIXzEr0/sdrpp-tetra-schwartz/releases/latest).
Fehler bitte als [Issue](https://github.com/UnImAtrIXzEr0/sdrpp-tetra-schwartz/issues/new/choose)
melden, Fragen und Erfahrungen in den
[Discussions](https://github.com/UnImAtrIXzEr0/sdrpp-tetra-schwartz/discussions).
Keine Entschlüsselung; nur für eigene bzw. Amateurfunk-Zellen.

## Building from source

The CI workflow [`build_tetra_custom.yml`](.github/workflows/build_tetra_custom.yml)
builds both packages and runs the headless tests; it is the reference for the
dependencies. Linux in short: the packages listed in [AGENTS.md](AGENTS.md),
then `cmake -B build -DOPT_BUILD_USRP_SOURCE=ON … && cmake --build build`.
The test recording `e2e/recordings/tetra_synthetic.wav` is generated by
[`e2e/gen/tetra_synth.cpp`](e2e/gen/tetra_synth.cpp).

The original readme of SDR++Brown: [docs/README_SDRPPBROWN.md](docs/README_SDRPPBROWN.md).
