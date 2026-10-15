#!/usr/bin/env python3
"""
E2E tests for the SDR++ TETRA-SCHWARTZ edition (headless, e.g. under xvfb).

1. Fresh start: an almost empty config, so all edition defaults and config
   migrations run with every module loaded (themes, hidden menus and sources,
   start screen). The program must start and keep running.
2. TETRA chain with the bundled recording (File source): decoder, measurements,
   TX analysis, all four own windows open (tables, spectrum, network view are
   drawn), AFC on. Checks lock, MER, input rate and the TX analysis results,
   and that nothing crashes or blocks.
"""

import os
import shutil
import sys
import tempfile
import time
from e2e_common import (
    SDRPPTestContext, get_base_config, stats, STATS_MODE, http_get
)


def recording():
    here = os.path.dirname(os.path.abspath(__file__))
    return os.environ.get("E2E_TEST_FILE", os.path.join(here, "recordings", "tetra_sample.wav"))


def upsample_recording(src, dst, factor=8):
    """Write the 32 kS/s recording at factor x the rate (band-limited FFT
    interpolation). A real receiver delivers far more than 32 kS/s; at 32 kS/s
    neither the 25 kHz decoder VFO nor the 250 kHz TX analysis tap fit, so the
    VFO would be pushed to the band edge. Returns False if numpy is missing."""
    try:
        import numpy as np
        import wave
    except ImportError:
        return False
    with wave.open(src, "rb") as w:
        ch, sw, fs, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        raw = w.readframes(n)
    if ch != 2 or sw != 2:
        return False
    a = np.frombuffer(raw, dtype="<i2").astype(np.float64).reshape(-1, 2)
    x = a[:, 0] + 1j * a[:, 1]
    X = np.fft.fftshift(np.fft.fft(x))
    m = len(x) * factor
    pad = (m - len(x)) // 2
    Y = np.concatenate([np.zeros(pad, complex), X, np.zeros(m - len(x) - pad, complex)])
    y = np.fft.ifft(np.fft.ifftshift(Y)) * factor
    out = np.empty((m, 2), dtype="<i2")
    out[:, 0] = np.clip(np.round(y.real), -32768, 32767)
    out[:, 1] = np.clip(np.round(y.imag), -32768, 32767)
    with wave.open(dst, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(fs * factor)
        w.writeframes(out.tobytes())
    return True


def log_has_crash(ctx):
    try:
        with open(ctx.log_path, "r", errors="replace") as f:
            txt = f.read()
    except Exception:
        return None
    for bad in ("Segmentation fault", "terminate called", "Aborted", "AddressSanitizer", "std::bad_alloc"):
        if bad in txt:
            return bad
    return None


def alive(ctx):
    return ctx.proc is not None and ctx.proc.poll() is None and "error" not in http_get(ctx.base_url, "/status")


def test_fresh_start():
    stats.test_start("test_fresh_start")
    base = get_base_config()
    cfg = {"modulesDirectory": base["modulesDirectory"], "resourcesDirectory": base["resourcesDirectory"]}
    with SDRPPTestContext() as ctx:
        ctx.write_configs(cfg)
        if not ctx.start():
            stats.test_fail("test_fresh_start", "SDR++ did not start with a fresh config")
            return False
        ctx.sleep(6.0)
        if not alive(ctx):
            stats.test_fail("test_fresh_start", "SDR++ stopped after start (fresh config)")
            return False
        crash = log_has_crash(ctx)
        if crash:
            stats.test_fail("test_fresh_start", f"log shows '{crash}'")
            return False
        # every SDR supported by the edition must load (SDRplay only where its
        # API library is installed, as on the build machine)
        try:
            with open(ctx.log_path, "r", errors="replace") as f:
                txt = f.read()
        except Exception:
            txt = ""
        bad = [l for l in txt.splitlines() if "Couldn't load" in l or "isn't a loadable module" in l or "Symbol resolution failed" in l]
        if bad:
            stats.test_fail("test_fresh_start", "module load error: " + bad[0][:200])
            return False
    stats.test_pass("test_fresh_start", "fresh config: defaults and migrations applied, all modules loaded, running")
    return True


def test_tetra_chain():
    stats.test_start("test_tetra_chain")
    src = recording()
    if not os.path.exists(src):
        stats.test_fail("test_tetra_chain", f"recording not found: {src}")
        return False
    # The file source takes the centre frequency from the file name; give the
    # recording one (the TETRA carrier is then at 468.125 MHz = centre - 3 kHz)
    tmpdir = tempfile.mkdtemp(prefix="tetra_rec_")
    rec = os.path.join(tmpdir, "baseband_468128000Hz_12-00-00_01-01-2026.wav")
    upsampled = upsample_recording(src, rec)
    if not upsampled:
        shutil.copyfile(src, rec)
    stats.info(f"recording: {'upsampled to 256 kS/s' if upsampled else 'original 32 kS/s (numpy missing)'}")

    cfg = get_base_config()
    cfg["moduleInstances"]["TETRA Demodulator"] = {"module": "ch_tetra_demodulator", "enabled": True}
    cfg["moduleInstances"]["File Source"] = {"module": "file_source", "enabled": True}
    cfg["source"] = "File"
    cfg["File Source"] = {"path": rec, "filename": rec}
    cfg["streams"]["TETRA Demodulator"] = {"muted": False, "sink": "None", "volume": 1.0}
    cfg["theme"] = "RoSc 2"

    with SDRPPTestContext() as ctx:
        ctx.write_configs(cfg)
        if not ctx.start():
            stats.test_fail("test_tetra_chain", "SDR++ did not start")
            return False
        ctx.sleep(1.0)
        ctx.module_cmd("File Source", "set_filename", rec)
        http_get(ctx.base_url, "/sdr/start")
        # signal centre of the recording is about -3 kHz
        ctx.module_cmd("TETRA Demodulator", "set_offset", "-3000")
        ctx.module_cmd("TETRA Demodulator", "set_afc", "1")
        ctx.module_cmd("TETRA Demodulator", "set_txa", "1")
        ctx.module_cmd("TETRA Demodulator", "set_windows", "1")

        best = None
        locked = False
        t0 = time.time()
        while time.time() - t0 < 25.0:
            ctx.sleep(1.0)
            if not alive(ctx):
                stats.test_fail("test_tetra_chain", "SDR++ stopped while decoding")
                return False
            m = ctx.module_cmd("TETRA Demodulator", "get_measurements", timeout=8.0)
            if "error" in m:
                stats.test_fail("test_tetra_chain", f"module not responding: {m['error']}")
                return False
            best = m
            if m.get("rx_state") == 2 and m.get("sync"):
                locked = True
            if locked and m.get("txa_valid") and m.get("input_rate", 0) > 0:
                break
        stats.info(f"last measurements: {best}")

        # windows off again and analysis off: must not block either
        ctx.module_cmd("TETRA Demodulator", "set_windows", "0")
        ctx.module_cmd("TETRA Demodulator", "set_txa", "0")
        ctx.sleep(1.5)
        if not alive(ctx):
            stats.test_fail("test_tetra_chain", "SDR++ stopped after closing windows / analysis")
            return False
        crash = log_has_crash(ctx)
        if crash:
            stats.test_fail("test_tetra_chain", f"log shows '{crash}'")
            return False

        problems = []
        if not locked:
            problems.append("decoder never locked")
        if best and best.get("mer_db", 0) < 10:
            problems.append(f"MER low ({best.get('mer_db')})")
        if best and not best.get("txa_valid"):
            problems.append("TX analysis produced no result")
        if upsampled and best and best.get("txa_valid") and not (15000 < best.get("txa_obw_hz", 0) < 27000):
            problems.append(f"occupied bandwidth implausible ({best.get('txa_obw_hz')})")
        if problems:
            stats.test_fail("test_tetra_chain", "; ".join(problems))
            return False

    stats.test_pass("test_tetra_chain", "lock, MER, TX analysis and own windows OK, no crash")
    return True


if __name__ == "__main__":
    if not STATS_MODE:
        stats.verbose = True
        stats.section("SDR++ TETRA-SCHWARTZ edition tests")
    r1 = test_fresh_start()
    r2 = test_tetra_chain()
    ok = r1 and r2
    stats.final_summary(2, int(r1) + int(r2), int(not r1) + int(not r2))
    sys.exit(0 if ok else 1)
