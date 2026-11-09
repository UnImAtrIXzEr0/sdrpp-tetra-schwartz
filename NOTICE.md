# SDR++ Air Interface Analyzer – notice of origin, modifications and licences

SDR++ Air Interface Analyzer is a **modified version (fork)** of SDR++ for amateur
radio TETRA, created and maintained by **UnImAtrIXzEr0**.
It is an experimental beta, built and adapted for the maintainer's own needs
and published as it is: **no support** and no promise of answers, fixes or
new functions; suggestions are welcome, their implementation is reserved. It is **not an official release** of SDR++ or
SDR++Brown and is **not endorsed by their authors**. Please report problems
with this edition only here:
<https://github.com/UnImAtrIXzEr0/sdrpp-air-interface-analyzer/issues>,
not to the authors of SDR++ or SDR++Brown.

## Origin

| Part | Authors / copyright holders | Licence | Source |
|---|---|---|---|
| SDR++ (original software) | Alexandre Rouma (ON5RYZ) and the SDR++ contributors | GPL-3.0 | <https://github.com/AlexandreRouma/SDRPlusPlus> |
| SDR++Brown v1.2.1 (code base of this edition) | sannysanoff and contributors | GPL-3.0 | <https://github.com/sannysanoff/SDRPlusPlusBrown> |
| TETRA demodulator plugin | cropinghigh | GPL-3.0 | <https://github.com/cropinghigh/sdrpp-tetra-demodulator> |
| osmo-tetra (TETRA lower/upper MAC decoder) | Harald Welte, Sylvain Munaut, Holger Hans Peter Freyther and contributors (Osmocom) | AGPL-3.0 | <https://gitea.osmocom.org/tetra/osmo-tetra> |
| TETRA speech codec | ETSI, reference code of ETSI EN 300 395-2 (downloaded from ETSI at build time) | ETSI terms | <https://www.etsi.org> |
| Modifications of this edition | © 2026 UnImAtrIXzEr0 | GPL-3.0 | <https://github.com/UnImAtrIXzEr0/sdrpp-air-interface-analyzer> |

The full list of SDR++ contributors, libraries, hardware donators and
patrons is kept unchanged in the program (Info / credits screen) and in
`core/src/credits.cpp`. All original copyright notices and licence files are
kept: `license` (GPL-3.0) and
`decoder_modules/ch_tetra_demodulator/src/decoder/COPYING` (AGPL-3.0).

## Modifications (GPL-3.0 section 5a)

This edition was modified by UnImAtrIXzEr0 starting 2026-10-14,
based on SDR++Brown v1.2.1. Main changes (details in the git history):

- TETRA demodulator: 8-point π/4-DQPSK constellation with ETSI limits,
  measurements (MER/EVM, I/Q offset, carrier and symbol clock, BER, CRC,
  input rate), optional AFC and automatic RF gain, editable duplex tables and
  an amateur radio mode, layer-3 decoding (calls, talkers, SDS, LIP
  positions, scan lists), network and neighbour cell information, TX analysis (spectrum,
  adjacent channel power, occupied bandwidth, shoulders, carrier leakage,
  I/Q imbalance), GPS
  coverage logging, a map of the drive test (track of up to 24 h recorded in
  the background, points coloured by level / MER / BER / quality, cell site
  with distance rings, scale bar, legend, drives loaded from CSV logs) with
  optional map tiles (disk cache, saved maps for offline use) and a PNG image
  of the map, map export as KMZ/KML for Google Earth (folders per value,
  route, time stamps, legends) and as HTML, CSV logs with ISO 8601 time
  stamps including the UTC offset, an HTTP user agent of the edition for all
  downloads, trends over 24 h and
  from CSV logs, timeslot occupancy, monitoring of further carriers, burst
  power analysis (ramps, length, ripple, time mask, timing against the
  downlink, ramp spectrum, uplink noise), multipath analysis, frequency
  stability (Allan deviation), reception quality versus level, limit alarms
  (including lost samples), an HTML measurement report, an automatic RF gain
  that looks for the best signal-to-noise ratio and watches the ADC peak of
  the whole band, and menu sections the user can reorder.
- Core: VFO selection, click-through and audio fixes, audio sinks that no
  longer stall the receive chain, start-up defaults, hidden built-in menus,
  instrument-style themes, new start screen and recoloured icons, RGBA
  textures for images drawn by modules (`core/src/gui/widgets/texture.cpp`,
  used for the map tiles).
- Source modules (RTL-SDR, PlutoSDR, Airspy, USRP, HackRF, BladeRF, LimeSDR,
  SDRplay): interfaces for the automatic RF gain and the SDR frequency
  calibration of the TETRA demodulator, gain controls locked while the
  automatic gain is active; PlutoSDR: overflow (lost sample) counter, fixed
  device address, full scale of PlutoSDR and BladeRF samples set to the
  12-bit ADC range; sources not supported by this edition are hidden from the
  source list by default (Module Manager).

- Decoder: the encryption code of osmo-tetra (TEA1/TEA2/TEA3 keystream
  generators, TAA1, HURDLE, key handling) is **removed**. This edition does
  not decrypt anything; encrypted elements are shown as encrypted and not
  decoded further.
- Tests: the TETRA test recording is **synthetic** (fictitious amateur radio
  cell MCC 901 / MNC 9999, see `e2e/recordings/README.md` and
  `e2e/gen/tetra_synth.cpp`); it contains no data of any real network.

The TETRA speech codec is the reference code of ETSI EN 300 395-2
(© ETSI). It is not part of this repository: the build downloads it from
etsi.org and applies the patches in
`decoder_modules/ch_tetra_demodulator/src/decoder/etsi_codec-patches`. The
binaries contain it compiled. ETSI's copyright and terms apply to it.

The SDRplay source module needs the SDRplay API installed on the computer
(Linux: the API library is not bundled in the AppImage).

## Data sources and services used at run time

Nothing is downloaded unless the user asks for it. Their own terms of use
apply. All downloads use the system's `curl` and send the user agent
`SDRPP-AIA/<version> (+https://github.com/UnImAtrIXzEr0/sdrpp-air-interface-analyzer)`;
it contains no personal data.

- RadioID.net – DMR ID database (callsigns for ISSIs), on request.
- BrandMeister – talkgroup names, on request.

**Map tiles under the drive map.** They are only loaded when the user selects
a map in the drive test (default: no map). Only the tiles in view are
fetched (no prefetching), cached on disk for 30 days (at most 500 MB) and
shown with the attribution the provider requires. Maps can be saved for use
without network only from providers whose terms allow it.

| Service | Provider | Licence / terms | Saved maps |
|---|---|---|---|
| OpenStreetMap standard tiles | OpenStreetMap Foundation; data © OpenStreetMap contributors | Data: [ODbL](https://www.openstreetmap.org/copyright); [tile usage policy](https://operations.osmfoundation.org/policies/tiles/) | no (not permitted by the tile usage policy) |
| TopPlusOpen (worldwide, more detailed for Europe and Germany) | Bundesamt für Kartographie und Geodäsie (BKG), <https://gdz.bkg.bund.de> | [Datenlizenz Deutschland – Namensnennung – 2.0 (dl-de/by-2-0)](https://www.govdata.de/dl-de/by-2-0); attribution “© BKG (year of the last data retrieval) dl-de/by-2-0” with the data sources | yes |
| basemap.de Web Raster (Germany) | GeoBasis-DE / BKG, <https://basemap.de> | [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/); attribution “© GeoBasis-DE / BKG (year) CC BY 4.0” | yes |
| Sentinel-2 cloudless 2024 (satellite imagery) | EOX IT Services GmbH, <https://s2maps.eu> | [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/) for the 2018 and later editions (**non-commercial use only**); contains modified Copernicus Sentinel data 2024 | yes |

Saved maps are stored in the settings folder (`maps/<name>/`) together with
the provider, the attribution and the date of saving; the cache is in
`map_cache/`. Images saved from the map carry the attribution of the map.

**HTML map export.** The exported `_map.html` is opened in the user's
browser, which loads OpenStreetMap tiles (© OpenStreetMap contributors,
ODbL) and the Leaflet library (BSD-2-Clause) with the leaflet.heat plugin
from unpkg.com. The KMZ contains its point symbol and legends; the plain
KML refers to the point symbol of Google (maps.google.com). Google Earth
shows its own imagery under its own terms.

## Fonts

- Roboto Regular and Roboto Medium (Google), Apache License 2.0 – see
  `res/fonts/LICENSE-Roboto.txt` (in the program folder: `share/sdrpp/fonts` or `res/fonts`).

## Bundled single-file libraries

- stb_image_write v1.16 (Sean Barrett), public domain or MIT licence (licence
  text at the end of `core/src/imgui/stb_image_write.h`), added in v0.25.0:
  the PNG image of the drive map, the legend images and the compression of
  the KMZ files of the map export.
- stb_image v2.26 (Sean Barrett, already part of the SDR++ code base,
  `core/src/imgui/stb_image.h`, public domain or MIT licence) decodes the map
  tiles.

## Trademarks and names

- **About the name:** “Air Interface Analyzer” describes what the program
  does: it analyses radio air interfaces (currently TETRA). Up to v0.23 the
  edition was called “SDR++ TETRA-SCHWARTZ”.
- Theme names and the program name are invented names. This project is not
  affiliated with, sponsored or endorsed by any manufacturer or rights holder
  and uses no third-party logos.
- TETRA is a standard of ETSI.

## No warranty

This program is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License version 3 (parts derived
from osmo-tetra: GNU Affero General Public License version 3).
It is distributed in the hope that it will be useful, but **WITHOUT ANY
WARRANTY**; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the licence files for details.
