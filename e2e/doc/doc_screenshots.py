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
        ctx.module_cmd("TETRA Demodulator", "set_windows", "0")
        time.sleep(1.5)
        grab("menu", (0, 0, 380, H))
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
