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
        # burst analysis on the (continuous) downlink and a second carrier
        ctx.module_cmd("TETRA Demodulator", "set_bursts", "1")
        ctx.module_cmd("TETRA Demodulator", "set_multipath", "1")
        ctx.module_cmd("TETRA Demodulator", "set_alarm_test", "1")
        m0 = ctx.module_cmd("TETRA Demodulator", "get_measurements", timeout=8.0)
        if m0.get("center_hz"):
            ctx.module_cmd("TETRA Demodulator", "add_carrier", str(int(m0["center_hz"]) + 60000))

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
            if locked and m.get("txa_valid") and m.get("input_rate", 0) > 0 and m.get("trend_points", 0) >= 6 \
                    and m.get("carriers_running", 0) >= 1 and m.get("aach_count", 0) > 0 and m.get("mp_slots", 0) >= 50 \
                    and m.get("alarms_total", 0) >= 1:
                break
        stats.info(f"last measurements: {best}")

        # windows off again and analysis off: must not block either
        ctx.module_cmd("TETRA Demodulator", "set_windows", "0")
        ctx.module_cmd("TETRA Demodulator", "set_txa", "0")
        ctx.module_cmd("TETRA Demodulator", "set_bursts", "0")
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
        if best and best.get("trend_points", 0) < 3:
            problems.append(f"trend history not filled ({best.get('trend_points')})")
        if best and best.get("carriers_running", 0) < 1:
            problems.append(f"second carrier not running ({best.get('carriers')} carriers)")
        if best and locked and best.get("aach_count", 0) <= 0:
            problems.append("no timeslot occupancy counted (AACH)")
        if best and best.get("burst_state", 0) < 1:
            problems.append(f"burst analysis not running ({best.get('burst_status')})")
        if best and locked and (best.get("mp_slots", 0) < 20 or best.get("mp_quality", 0) < 3.5):
            problems.append(f"multipath: training sequences not found in the real signal (slots {best.get('mp_slots')}, quality {best.get('mp_quality')})")
        if best and best.get("alarms_total", 0) < 1:
            problems.append("test alarm was not raised")
        # report
        ctx.module_cmd("TETRA Demodulator", "make_report", "1")
        ctx.sleep(1.5)
        rp = ctx.module_cmd("TETRA Demodulator", "get_report", timeout=8.0).get("path", "")
        if not rp or rp.startswith("!") or not os.path.exists(rp):
            problems.append(f"report not written ({rp})")
        else:
            with open(rp, "r", errors="replace") as f:
                html = f.read()
            if "Summary" not in html or "<svg" not in html:
                problems.append("report incomplete")
            stats.info(f"report: {rp} ({len(html)} bytes)")
        if upsampled and best and best.get("txa_valid") and not (15000 < best.get("txa_obw_hz", 0) < 27000):
            problems.append(f"occupied bandwidth implausible ({best.get('txa_obw_hz')})")
        if problems:
            stats.test_fail("test_tetra_chain", "; ".join(problems))
            return False

    stats.test_pass("test_tetra_chain", "lock, MER, TX analysis, trends, carriers, bursts and own windows OK, no crash")
    return True


def make_burst_recording(path, fs=256000, seconds=6.0, ramp_sym=3.0, len_sym=231.0, noise_db=-55.0):
    """Synthetic uplink: 231-symbol bursts every 4 timeslots with cosine ramps
    of ramp_sym symbols, 8-PSK-like symbols, noise noise_db below full scale."""
    import numpy as np
    import wave
    n = int(fs * seconds)
    t_sym = np.arange(n) * 18000.0 / fs
    ph = np.mod(t_sym, 4 * 255.0) - 20.0
    env = np.zeros(n)
    up = (ph >= -ramp_sym / 2) & (ph < ramp_sym / 2)
    env[up] = 0.5 - 0.5 * np.cos(np.pi * (ph[up] + ramp_sym / 2) / ramp_sym)
    flat = (ph >= ramp_sym / 2) & (ph <= len_sym - ramp_sym / 2)
    env[flat] = 1.0
    dn = (ph > len_sym - ramp_sym / 2) & (ph < len_sym + ramp_sym / 2)
    env[dn] = 0.5 + 0.5 * np.cos(np.pi * (ph[dn] - (len_sym - ramp_sym / 2)) / ramp_sym)
    rng = np.random.default_rng(1)
    syms = np.exp(1j * np.pi / 4 * rng.integers(0, 8, int(t_sym[-1]) + 2))
    k = np.floor(t_sym).astype(int)
    f = t_sym - k
    sig = syms[k] * (1 - f) + syms[k + 1] * f
    namp = 10 ** (noise_db / 20) / np.sqrt(2)
    x = 0.3 * env * sig + namp * (rng.standard_normal(n) + 1j * rng.standard_normal(n))
    out = np.empty((n, 2), dtype="<i2")
    out[:, 0] = np.clip(np.round(x.real * 32767), -32768, 32767)
    out[:, 1] = np.clip(np.round(x.imag * 32767), -32768, 32767)
    with wave.open(path, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(fs)
        w.writeframes(out.tobytes())


NTS1 = [1,1, 0,1, 0,0, 0,0, 1,1, 1,0, 1,0, 0,1, 1,1, 0,1, 0,0]
NTS2 = [0,1, 1,1, 1,0, 1,0, 0,1, 0,0, 0,0, 1,1, 0,1, 1,1, 1,0]


def pi4_waveform(nsym, sps, rng, training=True):
    """pi/4-DQPSK with RRC (alpha 0.35); the centre of symbol k is at sample
    k*sps. With training=True every slot carries NTS1/NTS2 at symbols 122..132."""
    import numpy as np
    dph = {(0, 0): np.pi / 4, (0, 1): 3 * np.pi / 4, (1, 1): -3 * np.pi / 4, (1, 0): -np.pi / 4}
    bits = rng.integers(0, 2, (nsym, 2))
    if training:
        for s0 in range(0, nsym, 255):
            t = NTS1 if (s0 // 255) % 2 == 0 else NTS2
            for k in range(11):
                if s0 + 122 + k < nsym:
                    bits[s0 + 122 + k] = (t[2 * k], t[2 * k + 1])
    ph = np.cumsum([dph[(int(a), int(b))] for a, b in bits])
    sym = np.exp(1j * ph)
    span, a = 6, 0.35
    t = np.arange(-span * sps, span * sps + 1) / sps
    with np.errstate(divide="ignore", invalid="ignore"):
        h = (np.sin(np.pi * t * (1 - a)) + 4 * a * t * np.cos(np.pi * t * (1 + a))) / (np.pi * t * (1 - (4 * a * t) ** 2))
    h[span * sps] = 1 - a + 4 * a / np.pi
    sing = np.isclose(np.abs(t), 1 / (4 * a))
    h[sing] = a / np.sqrt(2) * ((1 + 2 / np.pi) * np.sin(np.pi / (4 * a)) + (1 - 2 / np.pi) * np.cos(np.pi / (4 * a)))
    up = np.zeros(nsym * sps, complex)
    up[::sps] = sym
    y = np.convolve(up, h)[span * sps: span * sps + nsym * sps]
    return y / np.sqrt(np.mean(np.abs(y) ** 2))


def make_two_freq_recording(path, fs=576000, seconds=5.0, off=100000.0, echo_us=111.1, echo_gain=0.5, ramp_sym=3.0):
    """Downlink (training sequences, one echo) at +off and uplink bursts at -off
    on the same slot grid: bursts start (-3 dB point about 0.4 symbol later)
    at symbol 20 of every 4th slot."""
    import numpy as np
    import wave
    sps = int(fs / 18000)
    # the file source loops: whole multiframes of 4 slots (1020 symbols) and a
    # whole number of carrier periods (100 kHz at 576 kS/s: 4.5 symbols), so
    # slot grid and phase continue seamlessly at the loop point
    nsym = max(1, int(seconds * 18000) // 3060) * 3060
    rng = np.random.default_rng(5)
    dl = pi4_waveform(nsym, sps, rng)
    d = int(round(echo_us * 1e-6 * fs))
    dl_e = dl.copy()
    dl_e[d:] += echo_gain * dl[:-d]
    ul = pi4_waveform(nsym, sps, rng, training=False)
    tsym = np.arange(nsym * sps) / sps
    ph = np.mod(tsym, 4 * 255.0) - 20.0
    env = np.zeros(len(tsym))
    up_ = (ph >= -ramp_sym / 2) & (ph < ramp_sym / 2)
    env[up_] = 0.5 - 0.5 * np.cos(np.pi * (ph[up_] + ramp_sym / 2) / ramp_sym)
    flat = (ph >= ramp_sym / 2) & (ph <= 231 - ramp_sym / 2)
    env[flat] = 1.0
    dn = (ph > 231 - ramp_sym / 2) & (ph < 231 + ramp_sym / 2)
    env[dn] = 0.5 + 0.5 * np.cos(np.pi * (ph[dn] - (231 - ramp_sym / 2)) / ramp_sym)
    n = np.arange(len(tsym))
    x = 0.2 * dl_e * np.exp(2j * np.pi * off * n / fs) + 0.2 * ul * env * np.exp(-2j * np.pi * off * n / fs)
    x += 10 ** (-70 / 20) / np.sqrt(2) * (rng.standard_normal(len(x)) + 1j * rng.standard_normal(len(x)))
    out = np.empty((len(x), 2), dtype="<i2")
    out[:, 0] = np.clip(np.round(x.real * 32767), -32768, 32767)
    out[:, 1] = np.clip(np.round(x.imag * 32767), -32768, 32767)
    with wave.open(path, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(fs)
        w.writeframes(out.tobytes())


def test_timing_multipath():
    stats.test_start("test_timing_multipath")
    try:
        import numpy  # noqa: F401
    except ImportError:
        stats.test_pass("test_timing_multipath", "skipped (numpy missing)")
        return True
    tmpdir = tempfile.mkdtemp(prefix="tetra_tm_")
    rec = os.path.join(tmpdir, "baseband_430000000Hz_12-00-00_01-01-2026.wav")
    make_two_freq_recording(rec)
    cfg = get_base_config()
    cfg["moduleInstances"]["TETRA Demodulator"] = {"module": "ch_tetra_demodulator", "enabled": True}
    cfg["moduleInstances"]["File Source"] = {"module": "file_source", "enabled": True}
    cfg["source"] = "File"
    cfg["File Source"] = {"path": rec, "filename": rec}
    cfg["streams"]["TETRA Demodulator"] = {"muted": False, "sink": "None", "volume": 1.0}
    with SDRPPTestContext() as ctx:
        ctx.write_configs(cfg)
        if not ctx.start():
            stats.test_fail("test_timing_multipath", "SDR++ did not start")
            return False
        ctx.sleep(1.0)
        ctx.module_cmd("File Source", "set_filename", rec)
        http_get(ctx.base_url, "/sdr/start")
        ctx.module_cmd("TETRA Demodulator", "set_afc", "0")
        ctx.module_cmd("TETRA Demodulator", "set_offset", "100000")
        ctx.module_cmd("TETRA Demodulator", "set_burst_freq", "429900000")
        ctx.module_cmd("TETRA Demodulator", "set_multipath", "1")
        ctx.module_cmd("TETRA Demodulator", "set_windows", "1")
        best = None
        t0 = time.time()
        while time.time() - t0 < 30.0:
            ctx.sleep(1.0)
            if not alive(ctx):
                stats.test_fail("test_timing_multipath", "SDR++ stopped")
                return False
            m = ctx.module_cmd("TETRA Demodulator", "get_measurements", timeout=8.0)
            if "error" in m:
                stats.test_fail("test_timing_multipath", f"module not responding: {m['error']}")
                return False
            best = m
            if m.get("burst_timing_n", 0) >= 20 and m.get("mp_slots", 0) >= 300 and m.get("burst_trans_n", 0) >= 10:
                break
        stats.info("timing/multipath: " + str({k: v for k, v in (best or {}).items() if k.startswith(("burst", "mp_"))}))
        crash = log_has_crash(ctx)
        if crash:
            stats.test_fail("test_timing_multipath", f"log shows '{crash}'")
            return False
        b = best or {}
        problems = []
        if b.get("burst_slot_quality", 0) < 3.5:
            problems.append(f"downlink slot grid not found (quality {b.get('burst_slot_quality')}, {b.get('burst_status')})")
        if not (19.6 < b.get("burst_timing_sym", -999) < 21.4):
            problems.append(f"burst timing {b.get('burst_timing_sym')} sym (expected about 20.4)")
        if b.get("burst_timing_std", 99) > 1.0:
            problems.append(f"burst timing jitter {b.get('burst_timing_std')} sym")
        if b.get("burst_trans_n", 0) < 5:
            problems.append("no ramp spectrum")
        if b.get("burst_gap_dbfs", -999) < -150 or b.get("burst_gap_dbfs", -999) > -30:
            problems.append(f"uplink noise implausible ({b.get('burst_gap_dbfs')})")
        if not b.get("mp_valid"):
            problems.append(f"multipath: no result ({b.get('mp_slots')} slots, quality {b.get('mp_quality')})")
        else:
            if not (90.0 < b.get("mp_echo_us", -999) < 135.0):
                problems.append(f"echo delay {b.get('mp_echo_us')} us (generated 111 us)")
            if not (-9.5 < b.get("mp_echo_db", -999) < -3.0):
                problems.append(f"echo level {b.get('mp_echo_db')} dB (generated -6 dB)")
        if problems:
            stats.test_fail("test_timing_multipath", "; ".join(problems))
            return False
    stats.test_pass("test_timing_multipath", "slot grid, burst timing, ramp spectrum, uplink noise and echo as generated")
    return True


def test_bursts():
    stats.test_start("test_bursts")
    try:
        import numpy  # noqa: F401
    except ImportError:
        stats.test_pass("test_bursts", "skipped (numpy missing)")
        return True
    tmpdir = tempfile.mkdtemp(prefix="tetra_burst_")
    rec = os.path.join(tmpdir, "baseband_430000000Hz_12-00-00_01-01-2026.wav")
    make_burst_recording(rec)
    cfg = get_base_config()
    cfg["moduleInstances"]["TETRA Demodulator"] = {"module": "ch_tetra_demodulator", "enabled": True}
    cfg["moduleInstances"]["File Source"] = {"module": "file_source", "enabled": True}
    cfg["source"] = "File"
    cfg["File Source"] = {"path": rec, "filename": rec}
    cfg["streams"]["TETRA Demodulator"] = {"muted": False, "sink": "None", "volume": 1.0}
    with SDRPPTestContext() as ctx:
        ctx.write_configs(cfg)
        if not ctx.start():
            stats.test_fail("test_bursts", "SDR++ did not start")
            return False
        ctx.sleep(1.0)
        ctx.module_cmd("File Source", "set_filename", rec)
        http_get(ctx.base_url, "/sdr/start")
        ctx.module_cmd("TETRA Demodulator", "set_offset", "0")
        ctx.module_cmd("TETRA Demodulator", "set_bursts", "1")
        ctx.module_cmd("TETRA Demodulator", "set_windows", "1")
        best = None
        t0 = time.time()
        while time.time() - t0 < 25.0:
            ctx.sleep(1.0)
            if not alive(ctx):
                stats.test_fail("test_bursts", "SDR++ stopped during the burst analysis")
                return False
            m = ctx.module_cmd("TETRA Demodulator", "get_measurements", timeout=8.0)
            if "error" in m:
                stats.test_fail("test_bursts", f"module not responding: {m['error']}")
                return False
            best = m
            if m.get("burst_n_full", 0) >= 25 and m.get("burst_len_sym", -999) > 0:
                break
        stats.info("burst results: " + str({k: v for k, v in (best or {}).items() if k.startswith("burst")}))
        crash = log_has_crash(ctx)
        if crash:
            stats.test_fail("test_bursts", f"log shows '{crash}'")
            return False
        problems = []
        if not best or best.get("burst_n_full", 0) < 10:
            problems.append(f"too few bursts detected ({best.get('burst_n_full') if best else None}, {best.get('burst_status') if best else ''})")
        else:
            if not (222.0 < best.get("burst_len_sym", 0) < 240.0):
                problems.append(f"burst length {best.get('burst_len_sym')} sym (nominal 231)")
            if not (0.0 < best.get("burst_rise_sym", -999) < 8.0):
                problems.append(f"ramp-up {best.get('burst_rise_sym')} sym (3 sym ramp)")
            if not (0.0 < best.get("burst_fall_sym", -999) < 8.0):
                problems.append(f"ramp-down {best.get('burst_fall_sym')} sym (3 sym ramp)")
            if not (best.get("burst_ripple_pos_db", 99) < 1.5 and best.get("burst_ripple_neg_db", -99) > -1.5):
                problems.append(f"ripple {best.get('burst_ripple_pos_db')}/{best.get('burst_ripple_neg_db')} dB")
        if problems:
            stats.test_fail("test_bursts", "; ".join(problems))
            return False
    stats.test_pass("test_bursts", "synthetic bursts: count, length, ramps and ripple as generated")
    return True


if __name__ == "__main__":
    if not STATS_MODE:
        stats.verbose = True
        stats.section("SDR++ TETRA-SCHWARTZ edition tests")
    r1 = test_fresh_start()
    r2 = test_tetra_chain()
    r3 = test_bursts()
    r4 = test_timing_multipath()
    res = [r1, r2, r3, r4]
    ok = all(res)
    stats.final_summary(len(res), sum(int(r) for r in res), sum(int(not r) for r in res))
    sys.exit(0 if ok else 1)
