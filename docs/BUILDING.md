# Building SDR++ TETRA-SCHWARTZ from source

The ready-made downloads on the
[Releases page](https://github.com/UnImAtrIXzEr0/sdrpp-tetra-schwartz/releases/latest)
cover most users:

| Package | Runs on | Notes |
|---|---|---|
| `…_windows_x64.zip` | Windows 10 and 11, 64 bit | built with Visual Studio 2022 |
| `…_x86_64.AppImage` | Linux x86-64 with **glibc 2.39 or newer**: Ubuntu 24.04 and newer, Debian 13, Fedora 40 and newer, current Arch / openSUSE Tumbleweed | built on Ubuntu 24.04 |

Build it yourself for anything else, for example **Ubuntu 22.04, Debian 12,
Linux Mint 21** (glibc too old for the AppImage), **ARM boards** (Raspberry Pi
and similar), or to get the newest state of `main` before a release.

| Platform | Status |
|---|---|
| Linux x86-64 / ARM64 | supported, built and tested in CI on x86-64 |
| Windows 10 / 11 x64 | supported (the zip); building yourself is possible but laborious |
| macOS | **not tested.** The SDR++ base builds on macOS; the TETRA parts have never been built there |
| Android | **not supported.** The SDR++ base has an Android build, this edition is not maintained or tested for it |
| Windows 7 / 8, 32-bit systems | not supported |

You need an internet connection while configuring: the build downloads the
TETRA speech codec (reference code of ETSI EN 300 395-2, © ETSI) from etsi.org.

---

## 1. SDR drivers

Install the driver / library of your SDR **first**, as its makers describe it.
This project does not maintain these packages; their pages are the reference:

| SDR | Where to get driver and library |
|---|---|
| RTL-SDR | [Osmocom rtl-sdr](https://osmocom.org/projects/rtl-sdr/wiki) · Windows: WinUSB driver with [Zadig](https://zadig.akeo.ie/) |
| PlutoSDR / AD9363 boards | [libiio](https://github.com/analogdevicesinc/libiio) and [libad9361-iio](https://github.com/analogdevicesinc/libad9361-iio) · [Pluto drivers for Windows](https://wiki.analog.com/university/tools/pluto/drivers/windows) |
| USRP (B2xx and others) | [UHD installation guide](https://files.ettus.com/manual/page_install.html) – afterwards load the FPGA images with `uhd_images_downloader` |
| HackRF | [HackRF documentation](https://hackrf.readthedocs.io/) |
| Airspy / Airspy HF+ | [airspyone_host](https://github.com/airspy/airspyone_host), [airspyhf](https://github.com/airspy/airspyhf) |
| BladeRF | [bladeRF wiki](https://github.com/Nuand/bladeRF/wiki) |
| LimeSDR | [LimeSuite](https://github.com/myriadrf/LimeSuite) |
| SDRplay | [SDRplay API](https://www.sdrplay.com/api/) (the API service must be running) |
| anything with a SoapySDR driver | [SoapySDR wiki](https://github.com/pothosware/SoapySDR/wiki) – build with `-DOPT_BUILD_SOAPY_SOURCE=ON` |
| Windows driver bundle used by the CI | [PothosSDR](https://downloads.myriadrf.org/builds/PothosSDR/) (2020.01.26, installed to `C:\Program Files\PothosSDR`) |

On Linux the user needs access to the USB device: the driver packages bring
udev rules (for the PlutoSDR see the manual, chapter 2).

---

## 2. Linux

### 2.1 Debian / Ubuntu / Mint / Raspberry Pi OS

The exact package list of the CI build (Ubuntu 24.04). On older releases a
package may be missing or named differently (e.g. `libfuse2` instead of
`libfuse2t64`, `libvolk2-dev` instead of `libvolk-dev`); simply leave out the
SDR libraries you do not need.

```sh
sudo apt update
sudo apt install build-essential cmake ninja-build git unzip file xxd autoconf libtool \
    libfftw3-dev libglfw3-dev libvolk-dev libzstd-dev librtaudio-dev libpulse-dev portaudio19-dev \
    libspdlog-dev liborc-0.4-dev libcodec2-dev \
    librtlsdr-dev libhackrf-dev libairspy-dev libairspyhf-dev libiio-dev libad9361-dev \
    libbladerf-dev liblimesuite-dev libuhd-dev libboost-dev libboost-thread-dev
```

### 2.2 Fedora (not tested in CI)

```sh
sudo dnf install gcc-c++ cmake ninja-build git unzip xxd autoconf libtool \
    fftw-devel glfw-devel volk-devel libzstd-devel rtaudio-devel pulseaudio-libs-devel portaudio-devel \
    spdlog-devel orc-devel codec2-devel \
    rtl-sdr-devel hackrf-devel airspyone_host-devel libiio-devel libad9361-devel uhd-devel boost-devel
```

### 2.3 Arch Linux / Manjaro (not tested in CI)

```sh
sudo pacman -S --needed base-devel cmake ninja git unzip xxd fftw glfw volk zstd rtaudio libpulse portaudio \
    spdlog orc codec2 rtl-sdr hackrf airspy libiio libad9361 libuhd boost
```

Package names on other distributions differ; look for the development
packages of the libraries named above.

### 2.4 Configure, build, start

```sh
git clone https://github.com/UnImAtrIXzEr0/sdrpp-tetra-schwartz.git
cd sdrpp-tetra-schwartz
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr \
    -DOPT_BUILD_CH_TETRA_DEMODULATOR=ON -DOPT_BUILD_NEW_PORTAUDIO_SINK=ON \
    -DOPT_BUILD_USRP_SOURCE=ON -DOPT_BUILD_BLADERF_SOURCE=ON -DOPT_BUILD_LIMESDR_SOURCE=ON
cmake --build build -j$(nproc)
sudo cmake --install build
sdrpp
```

- **Leave out what you do not have:** every SDR module is a CMake option
  (`-DOPT_BUILD_<NAME>_SOURCE=ON/OFF`, list at the top of `CMakeLists.txt`).
  If configuring stops because a library is missing, either install it or
  switch its module off, e.g. `-DOPT_BUILD_USRP_SOURCE=OFF`,
  `-DOPT_BUILD_PLUTOSDR_SOURCE=OFF`, `-DOPT_BUILD_HACKRF_SOURCE=OFF`.
- SDRplay: install the SDRplay API first, then add `-DOPT_BUILD_SDRPLAY_SOURCE=ON`.
- An own AppImage: `./make_appimage.sh build ./linuxdeploy-x86_64.AppImage`
  (see the CI workflow).
- Update later: `git pull`, then the `cmake --build` and `--install` lines again.

---

## 3. Windows (advanced)

The Windows build needs several hand-built libraries. The CI workflow
[`.github/workflows/build_tetra_custom.yml`](../.github/workflows/build_tetra_custom.yml),
job `build_windows`, is the exact, working recipe. In short:

1. **Visual Studio 2022** (Community is enough) with “Desktop development with
   C++”, **CMake**, **Git**, **7-Zip**.
2. **vcpkg**, then
   `vcpkg install fftw3:x64-windows glfw3:x64-windows portaudio:x64-windows zstd:x64-windows libusb:x64-windows itpp:x64-windows spdlog:x64-windows boost:x64-windows`.
3. **PothosSDR 2020.01.26** to `C:\Program Files\PothosSDR` (VOLK and most SDR
   libraries), libusb 1.0.23 and a current `librtlsdr` as in the workflow.
4. **rtaudio** (built with CMake, commit as in the workflow).
5. Optional: SDRplay API, **codec2** (MSYS2 / MinGW, only for the M17
   decoder), **UHD** (for the USRP; `.github/actions/build-uhd`), libperseus,
   librfnm, libfobos, HydraSDR – or switch these modules off.
6. Configure and build:
   ```bat
   cmake -B build -DCOPY_MSVC_REDISTRIBUTABLES=ON -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake ^
         -DOPT_BUILD_CH_TETRA_DEMODULATOR=ON -DOPT_BUILD_NEW_PORTAUDIO_SINK=ON -DOPT_BUILD_MPEG_ADTS_SINK=OFF
   cmake --build build --config RelWithDebInfo
   ```
7. Package: `make_windows_package.ps1 .\build .\root` (expects the modules of
   the CI build; remove the lines of modules you switched off).

Unless you want to change the program, the ready-made zip is the easier way.

---

## 4. macOS (not tested)

SDR++ builds on macOS with Homebrew (`brew install cmake fftw glfw volk zstd
rtaudio portaudio spdlog librtlsdr hackrf airspy libiio libad9361 uhd`) and
`make_macos_bundle.sh`. This edition has **never been built on macOS**;
expect to adapt things. Reports and fixes are welcome (Discussions / pull
requests).

## 5. Android

Not supported. The SDR++ base contains an Android build (`android/`), but
this edition is neither built nor tested for Android.

---

## 6. Tests (optional)

The headless tests need Python 3, `numpy` and an X server (or `xvfb-run`):

```sh
xvfb-run -a -s "-screen 0 1600x1000x24" python3 e2e/test_tetra_schwartz.py
```

The test recording `e2e/recordings/tetra_synthetic.wav` is a synthetic cell
made with [`e2e/gen/tetra_synth.cpp`](../e2e/gen/tetra_synth.cpp).
