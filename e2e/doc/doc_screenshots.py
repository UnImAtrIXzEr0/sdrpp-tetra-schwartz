#!/usr/bin/env python3
"""Screenshots for the manual (run headless under Xvfb, 1600x1000).
Instrument view (the standard user interface), English and German:
Session A: the bundled TETRA recording (Monitor, Measure with its views,
Drive test, Alignment, Settings, RF / Gain, trends of a synthetic log,
report, one picture of the classic layout). Session B: a synthetic
two-frequency recording (burst analysis with timing and ramp spectrum,
multipath). Session C: ETSI mode (network and cell). Session D: drive test
map of a synthetic drive and the saved maps dialog. Output: PNG files in
the given folder; a name ending in _en / _de shows the program in that
language."""

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
LANGS = ("en", "de")
# main view of the instrument view (between the content column and the softkeys)
MAIN = (482, 96, 792, 870)
# strip below the main view (Monitor)
STRIP = (482, 700, 730, 262)


def grab(name, rect=None):
    # pointer into the corner: no tooltip of the waterfall or a control in the picture
    subprocess.run(["xdotool", "mousemove", str(W - 1), str(H - 1)], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.3)
    path = os.path.join(OUT, name + ".png")
    subprocess.run(["import", "-window", "root", path], check=False)
    if rect and os.path.exists(path):
        x, y, w, h = rect
        subprocess.run(["convert", path, "-crop", f"{w}x{h}+{x}+{y}", "+repage", path], check=False)
        if rect == MAIN:
            # a view shorter than the main view: cut off the empty background below it
            # (colour of the bottom right corner, so it works with every theme)
            bg = subprocess.run(["convert", path, "-format", f"%[pixel:p{{{w - 3},{h - 3}}}]", "info:"],
                                capture_output=True, text=True).stdout.strip()
            if bg:
                subprocess.run(["convert", path, "-bordercolor", bg, "-border", "1", "-fuzz", "2%", "-trim", "+repage", path],
                               check=False)
    print("shot", name, os.path.exists(path))


def base_cfg(rec, ham=True):
    cfg = get_base_config()
    cfg["moduleInstances"].pop("Radio", None)   # hidden by default in this edition
    cfg["streams"].pop("Radio", None)
    cfg["vfoOffsets"] = {}
    cfg["moduleInstances"]["TETRA Analyzer"] = {"module": "ch_tetra_demodulator", "enabled": True}
    cfg["moduleInstances"]["File Source"] = {"module": "file_source", "enabled": True}
    cfg["File Source"] = {"path": rec, "filename": rec}
    cfg["windowSize"] = {"w": W, "h": H}
    cfg["menuWidth"] = 360
    cfg["source"] = "File"
    cfg["streams"]["TETRA Analyzer"] = {"muted": False, "sink": "None", "volume": 1.0}
    # instrument view (standard), amateur radio mode
    # startViewV21: no "Recommended view" retune on the first start (a recording
    # centred on 439 MHz would be moved to 440 MHz - 0.45 x sample rate)
    cfg["TETRA Analyzer"] = {"instUi": True, "hamMode": ham, "uiLang": "en", "instMode": 0, "instView": 0,
                             "startViewV21": True}
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


# fictional cell site of the synthetic drive (as in doc_map.cpp)
SITE = (50.1109, 8.6821)


def make_demo_drive(path):
    """Synthetic 30-minute drive around the fictional cell site SITE (no real
    measurement): level falling with the distance, a shadowed stretch, seconds
    without service far out. Time stamps with UTC offset (logs since v0.25.0)."""
    import math
    import random
    rnd = random.Random(5)
    lat0, lon0 = SITE
    kx = 111.2 * math.cos(math.radians(lat0))
    lat, lon, hd = lat0 - 0.010, lon0 - 0.035, 0.6
    cols = ["time", "level_dbfs", "level_dbm", "mer_db", "ber", "rx_state", "cell_match", "mcc", "mnc", "cc", "dl_freq_hz",
            "crc_ndb_ok", "crc_ndb_fail", "crc_schf_ok", "crc_schf_fail", "gps_fix", "lat", "lon", "speed_kmh"]
    with open(path, "w") as f:
        f.write(";".join(cols) + "\n")
        for i in range(1800):
            hd += 0.05 * rnd.gauss(0, 1)
            # keep within about 4 km x 5 km around the site: turn back towards it
            if abs(lat - lat0) > 0.035 or abs(lon - lon0) > 0.07:
                hd = math.atan2((lon0 - lon) * kx, (lat0 - lat) * 111.2) + rnd.gauss(0, 0.3)
            lat += 0.00009 * math.cos(hd)
            lon += 0.00014 * math.sin(hd)
            dkm = math.hypot((lat - lat0) * 111.2, (lon - lon0) * kx)
            dbm = -52.0 - 30.0 * math.log10(max(dkm, 0.2) / 0.2) + 4.0 * rnd.gauss(0, 1)
            if math.sin(lon * 900) > 0.8:   # shadowed stretches
                dbm -= 12.0
            svc = dbm > -112.0
            fail = 0 if dbm > -100 else 1 if dbm > -108 else 4
            t = f"2026-11-03T{10 + i // 3600:02d}:{(i // 60) % 60:02d}:{i % 60:02d}+01:00"
            v = [t, f"{dbm + 40:.1f}", f"{dbm:.1f}",
                 f"{min(30.0, (dbm + 118) * 0.6 + rnd.gauss(0, 1.5)):.2f}" if svc else "",
                 f"{min(0.2, 10 ** (-(dbm + 130) / 12)):.2e}" if svc else "",
                 "2" if svc else "0", "1" if svc else "0", "901", "9999", "1", "439100000",
                 str(30 - fail), str(fail), "4", "0", "1", f"{lat:.6f}", f"{lon:.6f}", "35"]
            f.write(";".join(v) + "\n")


def inst(ctx, *cmds, wait=2.0):
    for c in cmds:
        ctx.module_cmd("TETRA Analyzer", "inst", c)
    time.sleep(wait)


def session_a():
    src = T.recording()
    tmp = tempfile.mkdtemp(prefix="doc_a_")
    rec = os.path.join(tmp, "baseband_438240500Hz_12-00-00_01-01-2026.wav")
    if not T.upsample_recording(src, rec):
        shutil.copyfile(src, rec)
    with SDRPPTestContext() as ctx:
        ctx.write_configs(base_cfg(rec))
        if not ctx.start():
            print("start failed")
            return
        time.sleep(2.0)
        ctx.module_cmd("File Source", "set_filename", rec)
        # the GUI opens the new path again on its next frame (file source menu);
        # starting from the HTTP thread at the same time races with it in the core
        ctx.sleep(1.5)
        http_get(ctx.base_url, "/sdr/start")
        ctx.module_cmd("TETRA Analyzer", "set_offset", "-3000")
        ctx.module_cmd("TETRA Analyzer", "set_afc", "1")
        m = ctx.module_cmd("TETRA Analyzer", "get_measurements", timeout=8.0)
        if m.get("center_hz"):
            ctx.module_cmd("TETRA Analyzer", "add_carrier", str(int(m["center_hz"]) + 60000))
        # TX spectrum first: its average and the ACP are then in "Last analysis results"
        inst(ctx, "mode 1", "view 2", wait=25.0)
        for lang in LANGS:
            # "Playing" shows a transmission (the recording has no speech)
            ctx.module_cmd("TETRA Analyzer", "sim_play", "live 1 91 10000003 238 2 12")
            inst(ctx, "lang " + lang, "mode 0", wait=3.0)
            grab("inst_monitor_" + lang)
            if lang == "en":
                grab("inst_stations", STRIP)
            ctx.module_cmd("TETRA Analyzer", "sim_play", "off")
            inst(ctx, "mode 1", "view 0", wait=3.0)
            grab("inst_measure_" + lang)
            inst(ctx, "view 1", wait=3.0)
            grab("inst_const_" + lang)
            if lang == "en":
                grab("inst_const_main", MAIN)
            inst(ctx, "view 2", wait=6.0)
            grab("inst_txa_" + lang)
            if lang == "en":
                grab("inst_txa_main", MAIN)
            inst(ctx, "view 4", wait=3.0)
            grab("inst_carriers_" + lang)
            if lang == "en":
                grab("inst_carriers_main", MAIN)
            inst(ctx, "view 5", wait=3.0)
            grab("inst_trends_" + lang)
            inst(ctx, "mode 2", wait=3.0)
            grab("inst_drive_" + lang)
            if lang == "en":
                grab("inst_drive_main", MAIN)
            inst(ctx, "mode 3", wait=8.0)
            grab("inst_align_" + lang)
            inst(ctx, "settings 0", wait=2.0)
            grab("inst_settings_" + lang, (300, 100, 1000, 800))
            inst(ctx, "settings -1", "rf 1", wait=2.0)
            grab("inst_rf_" + lang)
            inst(ctx, "rf 0", wait=0.5)
        # trends, stability and quality from a synthetic 6-hour log (main view only)
        inst(ctx, "lang en", "mode 1", "view 5", wait=2.0)
        demo = os.path.join(tmp, "tetra_demo_log.csv")
        make_demo_log(demo)
        ctx.module_cmd("TETRA Analyzer", "load_trend_csv", demo)
        time.sleep(3.0)
        grab("inst_trends_log", MAIN)
        ctx.module_cmd("TETRA Analyzer", "set_trend_tab", "1")
        time.sleep(2.5)
        grab("inst_stability", MAIN)
        ctx.module_cmd("TETRA Analyzer", "set_trend_tab", "2")
        time.sleep(2.5)
        grab("inst_quality", MAIN)
        ctx.module_cmd("TETRA Analyzer", "set_trend_tab", "0")
        inst(ctx, "view 0", wait=1.0)
        ctx.module_cmd("TETRA Analyzer", "make_report", "1")
        rp = ""
        for _ in range(20):   # the report is written in the GUI thread
            time.sleep(1.0)
            rp = ctx.module_cmd("TETRA Analyzer", "get_report", timeout=8.0).get("path", "")
            if rp:
                break
        print("report:", rp)
        if rp and os.path.exists(rp):
            shutil.copyfile(rp, os.path.join(OUT, "report.html"))
        # the classic layout (Settings -> Display & operation -> Classic layout)
        inst(ctx, "mode 0", "ui 0", wait=3.0)
        grab("classic")


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
        # the GUI opens the new path again on its next frame (file source menu);
        # starting from the HTTP thread at the same time races with it in the core
        ctx.sleep(1.5)
        http_get(ctx.base_url, "/sdr/start")
        ctx.module_cmd("TETRA Analyzer", "set_afc", "0")
        ctx.module_cmd("TETRA Analyzer", "set_offset", "100000")
        ctx.module_cmd("TETRA Analyzer", "set_burst_freq", "438900000")
        # the analyses run only while their view is shown
        inst(ctx, "mode 1", "view 3", wait=25.0)
        grab("inst_burst", MAIN)
        inst(ctx, "view 6", wait=20.0)
        grab("inst_multipath", MAIN)


def session_c():
    """ETSI mode: the strip below the main view shows network and cell."""
    src = T.recording()
    tmp = tempfile.mkdtemp(prefix="doc_c_")
    rec = os.path.join(tmp, "baseband_438240500Hz_12-00-00_01-01-2026.wav")
    if not T.upsample_recording(src, rec):
        shutil.copyfile(src, rec)
    cfg = base_cfg(rec, ham=False)
    cfg["TETRA Analyzer"]["instBottomH"] = 560
    with SDRPPTestContext() as ctx:
        ctx.write_configs(cfg)
        if not ctx.start():
            print("start failed")
            return
        time.sleep(2.0)
        ctx.module_cmd("File Source", "set_filename", rec)
        # the GUI opens the new path again on its next frame (file source menu);
        # starting from the HTTP thread at the same time races with it in the core
        ctx.sleep(1.5)
        http_get(ctx.base_url, "/sdr/start")
        ctx.module_cmd("TETRA Analyzer", "set_offset", "-3000")
        ctx.module_cmd("TETRA Analyzer", "set_afc", "1")
        time.sleep(20.0)
        grab("inst_network", (482, 404, 730, 410))


def session_d():
    """Drive test: the map of a synthetic drive (loaded from a CSV log) with
    the cell site, and the dialog of the saved maps. No map tiles: the
    screenshots never use a real map service."""
    src = T.recording()
    tmp = tempfile.mkdtemp(prefix="doc_d_")
    drive = os.path.join(tmp, "tetra_20261103_100000.csv")
    make_demo_drive(drive)
    cfg = base_cfg(src)
    cfg["TETRA Analyzer"]["instMode"] = 2
    with SDRPPTestContext() as ctx:
        ctx.write_configs(cfg)
        if not ctx.start():
            print("start failed")
            return
        time.sleep(2.0)
        # the SDR is not started: the map shows the loaded drive only
        inst(ctx, "mode 2", "drive " + drive, "site %.4f %.4f" % SITE, wait=3.0)
        for lang in LANGS:
            inst(ctx, "lang " + lang, wait=2.0)
            grab("inst_drivemap_" + lang)
        # the dialog is modal and stays open: all map pictures first
        for lang in LANGS:
            inst(ctx, "lang " + lang, "mapsdlg", wait=2.0)
            grab("inst_savedmaps_" + lang, (380, 100, 840, 800))


if __name__ == "__main__":
    session_a()
    session_b()
    session_c()
    session_d()
