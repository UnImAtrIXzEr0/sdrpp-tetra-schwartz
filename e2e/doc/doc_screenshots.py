#!/usr/bin/env python3
"""Screenshots for the manual (run headless under Xvfb, 1600x1000).
Session A: the bundled TETRA recording (decoder, measurements, network,
stations, TX analysis, trends, carriers, report). Session B: a synthetic
two-frequency recording (burst analysis with timing and ramp spectrum,
multipath). Output: PNG files in the given folder."""

import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from e2e_common import SDRPPTestContext, get_base_config, http_get  # noqa: E402
import test_tetra_schwartz as T  # noqa: E402

OUT = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "docshots")
os.makedirs(OUT, exist_ok=True)
W, H = 1600, 1000


def grab(name, rect=None):
    path = os.path.join(OUT, name + ".png")
    subprocess.run(["import", "-window", "root", path], check=False)
    if rect and os.path.exists(path):
        x, y, w, h = rect
        subprocess.run(["convert", path, "-crop", f"{w}x{h}+{x}+{y}", "+repage", path], check=False)
    print("shot", name, os.path.exists(path))


def base_cfg(rec):
    cfg = get_base_config()
    cfg["moduleInstances"].pop("Radio", None)   # hidden by default in this edition
    cfg["streams"].pop("Radio", None)
    cfg["vfoOffsets"] = {}
    cfg["moduleInstances"]["TETRA Demodulator"] = {"module": "ch_tetra_demodulator", "enabled": True}
    cfg["moduleInstances"]["File Source"] = {"module": "file_source", "enabled": True}
    cfg["File Source"] = {"path": rec, "filename": rec}
    cfg["windowSize"] = {"w": W, "h": H}
    cfg["menuWidth"] = 360
    cfg["source"] = "File"
    cfg["streams"]["TETRA Demodulator"] = {"muted": False, "sink": "None", "volume": 1.0}
    return cfg


def make_demo_log(path, hours=6.0):
    """Synthetic 6-hour measurement log (fictional values): slow level changes,
    a period with a weak signal (antenna test), frequency drift with temperature,
    traffic in the evening hours."""
    import math
    import random
    rnd = random.Random(3)
    t0 = time.mktime((2026, 10, 14, 14, 0, 0, 0, 0, -1))
    cols = ["time", "level_dbfs", "level_dbm", "mer_db", "evm_rms_pct", "carrier_offset_hz", "rx_state", "bursts", "sync_lost",
            "crc_ndb_ok", "crc_ndb_fail", "crc_schf_ok", "crc_schf_fail", "dl_freq_hz", "carrier_vs_sysinfo_hz",
            "ts1_traffic_pct", "ts2_traffic_pct", "ts3_traffic_pct", "ts4_traffic_pct"]
    n = int(hours * 3600)
    fade = 0.0
    with open(path, "w") as f:
        f.write(";".join(cols) + "\n")
        for i in range(n):
            h = i / 3600.0
            fade = 0.97 * fade + 0.03 * rnd.gauss(0, 6)
            lvl = -74 + 3 * math.sin(h / 1.7) + fade
            if 2.0 < h < 3.2:   # antenna test with attenuator steps
                lvl -= 8 + 32 * min(1.0, (h - 2.0) / 0.9)
            mer = max(3.0, min(27.0, (lvl + 118) * 1.2)) + rnd.gauss(0, 0.6)
            p_err = 1.0 / (1.0 + math.exp((lvl + 109.5) / 1.4))
            ok_n = 36
            fail = sum(1 for _ in range(ok_n) if rnd.random() < p_err)
            lost = 1 if (p_err > 0.3 and rnd.random() < 0.08) else 0
            rx = 2 if p_err < 0.6 else 0
            temp = 3 * math.sin((h - 1.0) / 6.0 * math.pi)
            ppb = 38 + 6 * temp + rnd.gauss(0, 1.5)
            busy = 1.0 if 3.5 < h < 5.5 else 0.25
            ts = [0.0] + [max(0.0, min(100.0, busy * (35 + 25 * math.sin(i / 200.0 + k)) + rnd.gauss(0, 5))) for k in range(3)]
            v = [time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(t0 + i)), f"{lvl + 55:.1f}", f"{lvl:.1f}",
                 f"{mer:.2f}" if rx == 2 else "", f"{100 * 10 ** (-mer / 20):.2f}" if rx == 2 else "", f"{rnd.gauss(4, 2):.1f}",
                 str(rx), str(int(72 * (1 - p_err))), str(lost), str(ok_n - fail), str(fail), "4", "0",
                 "439100000", f"{ppb * 0.4391:.2f}"] + [f"{x:.1f}" for x in ts]
            f.write(";".join(v) + "\n")


def solo(ctx, win, rect):
    x, y, w, h = rect
    ctx.module_cmd("TETRA Demodulator", "only_window", f"{win} {x} {y} {w} {h}")
    time.sleep(2.5)
    grab("win_" + win, rect)


def session_a():
    src = T.recording()
    tmp = tempfile.mkdtemp(prefix="doc_a_")
    rec = os.path.join(tmp, "baseband_468128000Hz_12-00-00_01-01-2026.wav")
    if not T.upsample_recording(src, rec):
        shutil.copyfile(src, rec)
    with SDRPPTestContext() as ctx:
        ctx.write_configs(base_cfg(rec))
        if not ctx.start():
            print("start failed")
            return
        time.sleep(2.0)
        ctx.module_cmd("File Source", "set_filename", rec)
        http_get(ctx.base_url, "/sdr/start")
        ctx.module_cmd("TETRA Demodulator", "set_offset", "-3000")
        ctx.module_cmd("TETRA Demodulator", "set_afc", "1")
        ctx.module_cmd("TETRA Demodulator", "set_txa", "1")
        m = ctx.module_cmd("TETRA Demodulator", "get_measurements", timeout=8.0)
        if m.get("center_hz"):
            ctx.module_cmd("TETRA Demodulator", "add_carrier", str(int(m["center_hz"]) + 60000))
        time.sleep(40.0)
        grab("overview")
        rect = (380, 30, 860, 950)
        for win in ["const", "network", "stations", "txa", "trends", "carriers"]:
            solo(ctx, win, rect)
        # trends, stability and quality from a synthetic 6-hour log
        demo = os.path.join(tmp, "tetra_demo_log.csv")
        make_demo_log(demo)
        ctx.module_cmd("TETRA Demodulator", "only_window", "trends 380 30 860 950")
        time.sleep(1.0)
        ctx.module_cmd("TETRA Demodulator", "load_trend_csv", demo)
        time.sleep(2.5)
        grab("win_trends_log", rect)
        ctx.module_cmd("TETRA Demodulator", "set_trend_tab", "1")
        time.sleep(2.5)
        grab("win_stability", rect)
        ctx.module_cmd("TETRA Demodulator", "set_trend_tab", "2")
        time.sleep(2.5)
        grab("win_quality", rect)
        ctx.module_cmd("TETRA Demodulator", "set_trend_tab", "0")
        ctx.module_cmd("TETRA Demodulator", "set_windows", "0")
        time.sleep(1.5)
        grab("menu", (0, 0, 380, H))
        ctx.module_cmd("TETRA Demodulator", "set_layout", "1")
        time.sleep(3.0)
        grab("overview_tiles")
        ctx.module_cmd("TETRA Demodulator", "set_layout", "0")
        time.sleep(1.0)
        ctx.module_cmd("TETRA Demodulator", "make_report", "1")
        time.sleep(2.0)
        rp = ctx.module_cmd("TETRA Demodulator", "get_report", timeout=8.0).get("path", "")
        if rp and os.path.exists(rp):
            shutil.copyfile(rp, os.path.join(OUT, "report.html"))


def session_b():
    tmp = tempfile.mkdtemp(prefix="doc_b_")
    rec = os.path.join(tmp, "baseband_439000000Hz_12-00-00_01-01-2026.wav")
    T.make_two_freq_recording(rec, seconds=8.0)
    with SDRPPTestContext() as ctx:
        ctx.write_configs(base_cfg(rec))
        if not ctx.start():
            print("start failed")
            return
        time.sleep(2.0)
        ctx.module_cmd("File Source", "set_filename", rec)
        http_get(ctx.base_url, "/sdr/start")
        ctx.module_cmd("TETRA Demodulator", "set_afc", "0")
        ctx.module_cmd("TETRA Demodulator", "set_offset", "100000")
        ctx.module_cmd("TETRA Demodulator", "set_burst_freq", "438900000")
        ctx.module_cmd("TETRA Demodulator", "set_multipath", "1")
        time.sleep(30.0)
        grab("overview_b")
        rect = (380, 30, 860, 950)
        for win in ["bursts", "multipath"]:
            solo(ctx, win, rect)


if __name__ == "__main__":
    session_a()
    session_b()
