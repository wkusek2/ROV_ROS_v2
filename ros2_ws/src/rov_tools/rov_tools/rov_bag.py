#!/usr/bin/env python3
"""Analiza bagow ROV (ros2 run rov_tools rov_bag <komenda>). Dziala na PC i na Jetsonie, mcap i sqlite3.

  rov_bag info   <bag...>                      topiki, typy, liczba wiadomosci, czestotliwosc, dziury
  rov_bag report <bag...> [-o raport.md]       pelny raport: sesje ARM, mikser/saturacja, ESC (failsafe, opoznienie gaz->RPM),
                                               IMU, odpowiedz pojazdu (komenda->zyro), regulator (bledy, nasycenie),
                                               BMS, audyt operatora, bledy z /rosout
  rov_bag export <bag...> [-o katalog] [--topics REGEX] [--start S] [--end S]
                                               CSV na kazdy topik: t (czas nagrania, unix s), t_rel (od poczatku), t_hdr
                                               (header.stamp jesli jest), pola splaszczone (tablice -> _0.._n, JSON -> a.b)
  rov_bag plot   <bag...> [-o plik.png] [--start S] [--end S]
                                               wykres: komendy osi, gaz silnikow, RPM, zyro, katy, glebokosc, prad
<bag...> = katalog(i) bagu (z metadata.yaml) - kilka katalogow sklejanych po czasie. --start/--end w s od poczatku.
Pliki .mcap czytane strumieniowo (pip install mcap) - dziala tez na nagraniu w toku i urwanym przy odcieciu zasilania
(traci sie najwyzej ostatni niepelny blok). Bez biblioteki mcap / dla sqlite3: rosbag2_py (wymaga metadata.yaml).
"""
import argparse
import json
import os
import re
import subprocess
import sys
from collections import defaultdict

import numpy as np

# ---------------------------------------------------------------- odczyt


def _iter_mcap(files):
    """(topic, typ, dane CDR, t_ns) z plikow .mcap - strumieniowo, odporne na urwany koniec pliku."""
    from mcap.stream_reader import StreamReader
    from mcap.records import Channel, Message, Schema
    for fp in files:
        schemas, chans, n = {}, {}, 0
        with open(fp, "rb") as f:
            try:
                for rec in StreamReader(f).records:
                    if isinstance(rec, Schema):
                        schemas[rec.id] = rec.name
                    elif isinstance(rec, Channel):
                        chans[rec.id] = (rec.topic, schemas.get(rec.schema_id, ""))
                    elif isinstance(rec, Message) and rec.channel_id in chans:
                        n += 1
                        topic, typ = chans[rec.channel_id]
                        yield topic, typ, rec.data, rec.log_time
            except Exception as e:      # urwany plik (nagranie w toku / odciecie zasilania)
                print(f"[rov_bag] {os.path.basename(fp)}: koniec po {n} wiadomosciach ({type(e).__name__}) - plik niedomkniety", file=sys.stderr)


def _iter_rosbag2(path):
    import rosbag2_py
    meta = os.path.join(path, "metadata.yaml")
    sid = "sqlite3"
    if os.path.exists(meta):
        m = re.search(r"storage_identifier:\s*(\S+)", open(meta).read())
        sid = m.group(1) if m else sid
    else:
        print(f"[rov_bag] {path}: brak metadata.yaml - ros2 bag reindex", file=sys.stderr)
        subprocess.run(["ros2", "bag", "reindex", path, sid], check=False, capture_output=True)
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=sid), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    while r.has_next():
        topic, data, t_ns = r.read_next()
        yield topic, types[topic], data, t_ns


def iter_raw(path):
    path = path.rstrip("/")
    files = [path] if path.endswith(".mcap") else sorted(
        os.path.join(path, f) for f in os.listdir(path) if f.endswith(".mcap"))
    if files:
        try:
            import mcap  # noqa: F401
            return _iter_mcap(files)
        except ImportError:
            print("[rov_bag] brak biblioteki mcap (pip install mcap) - uzywam rosbag2_py", file=sys.stderr)
    return _iter_rosbag2(path)


def read_bag(paths, topics_re=None):
    """-> dict topic -> {"type": str, "rows": [(t_s, msg_dict)]}  (msg_dict = splaszczony)"""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict

    rx = re.compile(topics_re) if topics_re else None
    out, cls = {}, {}
    for path in paths:
        for topic, typ, data, t_ns in iter_raw(path):
            if rx and not rx.search(topic):
                continue
            if typ not in cls:
                try:
                    cls[typ] = get_message(typ)
                except Exception as e:
                    print(f"[rov_bag] nieznany typ {typ} ({topic}): {e} - zrodluj install/setup.bash", file=sys.stderr)
                    cls[typ] = None
            c = cls[typ]
            if c is None:
                continue
            try:
                msg = deserialize_message(data, c)
            except Exception:
                continue
            d = message_to_ordereddict(msg)
            row = {}
            if typ == "std_msgs/msg/String":
                s = d.get("data", "")
                j = None
                if s[:1] in "{[":
                    try:
                        j = json.loads(s)
                    except ValueError:
                        pass
                if isinstance(j, dict):
                    flatten(j, "", row)
                else:
                    row["data"] = s
            else:
                flatten(d, "", row)
            st = row.pop("header.stamp.sec", None), row.pop("header.stamp.nanosec", None)
            if st[0] is not None:
                row["t_hdr"] = st[0] + st[1] * 1e-9
            out.setdefault(topic, {"type": typ, "rows": []})["rows"].append((t_ns * 1e-9, row))
    for v in out.values():
        v["rows"].sort(key=lambda x: x[0])
    return out


def flatten(o, prefix, out):
    if isinstance(o, dict):
        for k, v in o.items():
            flatten(v, f"{prefix}.{k}" if prefix else str(k), out)
    elif isinstance(o, (list, tuple, np.ndarray)):
        if len(o) <= 64 and all(not isinstance(v, (dict, list, tuple)) for v in o):
            for i, v in enumerate(o):
                out[f"{prefix}_{i}"] = _scalar(v)
        elif len(o) <= 16:
            for i, v in enumerate(o):
                flatten(v, f"{prefix}_{i}", out)
        else:
            out[prefix] = json.dumps(list(o), default=str)
    elif isinstance(o, (bytes, bytearray)):
        out[prefix] = f"<{len(o)} B>"
    else:
        out[prefix] = _scalar(o)


def _scalar(v):
    if isinstance(v, (np.generic,)):
        return v.item()
    if isinstance(v, (bytes, bytearray)):
        return f"<{len(v)} B>"
    return v


def frames(bag, start=None, end=None):
    """bag -> dict topic -> pandas.DataFrame (kolumny t, t_rel, ...)."""
    import pandas as pd
    t0 = min((v["rows"][0][0] for v in bag.values() if v["rows"]), default=0.0)
    dfs = {}
    for topic, v in bag.items():
        df = pd.DataFrame([r for _, r in v["rows"]])
        df = df.rename(columns={"t": "t_msg", "t_rel": "t_rel_msg"})   # np. pole "t" w JSON audytu
        df.insert(0, "t", [t for t, _ in v["rows"]])
        df.insert(1, "t_rel", df["t"] - t0)
        if start is not None:
            df = df[df["t_rel"] >= start]
        if end is not None:
            df = df[df["t_rel"] <= end]
        dfs[topic] = df.reset_index(drop=True)
    return dfs, t0


# ---------------------------------------------------------------- pomocnicze


def rate_stats(t):
    t = np.asarray(t)
    if len(t) < 2:
        return {"n": len(t), "hz": 0.0, "max_gap": 0.0, "gaps": 0}
    dt = np.diff(t)
    med = float(np.median(dt))
    thr = max(3 * med, 0.2)
    return {"n": len(t), "hz": (len(t) - 1) / (t[-1] - t[0]) if t[-1] > t[0] else 0.0,
            "max_gap": float(dt.max()), "gaps": int((dt > thr).sum()), "gap_thr": thr}


def intervals(t, flag):
    """Przedzialy czasu, gdy flag==True -> [(t_start, t_end)]."""
    out, s = [], None
    for ti, f in zip(t, flag):
        if f and s is None:
            s = ti
        elif not f and s is not None:
            out.append((s, ti)); s = None
    if s is not None:
        out.append((s, t[-1]))
    return out


def xcorr_lag(ta, a, tb, b, fs=50.0, max_lag=1.0):
    """Opoznienie b wzgledem a [s] przy maks. |korelacji| (b reaguje na a). -> (lag, corr) albo None."""
    ta, a, tb, b = map(lambda x: np.asarray(x, dtype=float), (ta, a, tb, b))
    if len(ta) < 10 or len(tb) < 10:
        return None
    t = np.arange(max(ta[0], tb[0]), min(ta[-1], tb[-1]), 1.0 / fs)
    if len(t) < 2 * fs:
        return None
    A = np.interp(t, ta, a); B = np.interp(t, tb, b)
    A -= A.mean(); B -= B.mean()
    if A.std() < 1e-6 or B.std() < 1e-6:
        return None
    best = None
    for k in range(0, int(max_lag * fs) + 1):
        c = float(np.corrcoef(A[:len(A) - k], B[k:])[0, 1])
        if best is None or abs(c) > abs(best[1]):
            best = (k / fs, c)
    return best


def cols(df, prefix, n):
    return [f"{prefix}_{i}" for i in range(n) if f"{prefix}_{i}" in df]


def fmt(v, n=2):
    return "-" if v is None or (isinstance(v, float) and not np.isfinite(v)) else f"{v:.{n}f}"


def table(rows, header):
    s = "| " + " | ".join(header) + " |\n|" + "---|" * len(header) + "\n"
    for r in rows:
        s += "| " + " | ".join(str(x) for x in r) + " |\n"
    return s


# ---------------------------------------------------------------- komendy


def cmd_info(a):
    bag = read_bag(a.bag)
    rows = []
    for topic in sorted(bag):
        t = [x[0] for x in bag[topic]["rows"]]
        s = rate_stats(t)
        rows.append([topic, bag[topic]["type"], s["n"], fmt(s["hz"], 1), fmt(s["max_gap"], 3), s["gaps"]])
    print(table(rows, ["topic", "typ", "n", "Hz", "max dziura [s]", "dziury"]))


def cmd_export(a):
    bag = read_bag(a.bag, a.topics)
    dfs, t0 = frames(bag, a.start, a.end)
    out = a.output or (a.bag[0].rstrip("/") + "_csv")
    os.makedirs(out, exist_ok=True)
    for topic, df in dfs.items():
        p = os.path.join(out, topic.strip("/").replace("/", "__") + ".csv")
        df.to_csv(p, index=False, float_format="%.6g")
        print(f"{p}  ({len(df)} wierszy, {len(df.columns)} kolumn)")


def cmd_report(a):
    bag = read_bag(a.bag)
    dfs, t0 = frames(bag, a.start, a.end)
    R = []
    w = R.append
    from datetime import datetime
    tend = max((df["t"].iloc[-1] for df in dfs.values() if len(df)), default=t0)
    w(f"# Raport ROV: {', '.join(a.bag)}\n")
    w(f"Start {datetime.fromtimestamp(t0):%Y-%m-%d %H:%M:%S}, czas {tend - t0:.1f} s, topikow {len(dfs)}\n")
    for p in a.bag:
        for f in ("info.txt", "git.txt"):
            fp = os.path.join(p, "meta", f)
            if os.path.exists(fp):
                w(f"\n`meta/{f}`:\n```\n{open(fp).read().strip()[:1500]}\n```\n")

    # topiki
    w("\n## Topiki\n")
    rows = []
    for topic in sorted(dfs):
        s = rate_stats(dfs[topic]["t"].values)
        rows.append([topic, s["n"], fmt(s["hz"], 1), fmt(s["max_gap"], 3), s["gaps"]])
    w(table(rows, ["topic", "n", "Hz", "max dziura [s]", "dziury"]))

    # sesje ARM
    mx = dfs.get("/rov/debug/mixer")
    w("\n## Sesje ARM\n")
    if mx is not None and len(mx) and "armed" in mx:
        sess = intervals(mx["t_rel"].values, mx["armed"].astype(bool).values)
        w(table([[i + 1, fmt(s, 1), fmt(e, 1), fmt(e - s, 1)] for i, (s, e) in enumerate(sess)], ["#", "od [s]", "do [s]", "czas [s]"]) if sess else "brak ARM\n")
        w("\n## Mikser (/rov/debug/mixer)\n")
        w("Zrodlo komendy (udzial tickow): " + ", ".join(f"{k} {v * 100:.1f}%" for k, v in mx["src"].value_counts(normalize=True).items()) + "\n\n")
        armed = mx[mx["armed"].astype(bool)]
        if len(armed):
            w(f"Saturacja alokacji (sat<1) w ARM: {(armed['sat'] < 0.999).mean() * 100:.1f}% tickow, min sat {armed['sat'].min():.3f}\n\n")
            if "cmd_age" in armed:
                w(f"Wiek komendy pilota w ARM: srednio {armed['cmd_age'].mean() * 1000:.0f} ms, max {armed['cmd_age'].max() * 1000:.0f} ms; "
                  f"przejscia w timeout: {int(((mx['src'] == 'timeout') & (mx['src'].shift() == 'pilot')).sum())}\n\n")
            tc = cols(armed, "thr", 8)
            w(table([[f"M{i + 1}", fmt(armed[c].abs().mean(), 3), fmt(armed[c].abs().max(), 3), fmt((armed[c].abs() > 0.9).mean() * 100, 1)]
                     for i, c in enumerate(tc)], ["silnik", "sr |gaz|", "max |gaz|", "% > 0.9"]))
            ax = ["surge", "sway", "heave", "roll", "pitch", "yaw"]
            cc = cols(armed, "cmd", 6)
            w("\n" + table([[ax[i], fmt(armed[c].mean(), 3), fmt(armed[c].abs().max(), 3)] for i, c in enumerate(cc)], ["os", "sr cmd", "max |cmd|"]))
    else:
        w("brak /rov/debug/mixer (stary firmware stosu?)\n")

    # ESC
    esc = dfs.get("/rov/esc/telemetry")
    thr = dfs.get("/rov/thrusters")
    w("\n## ESC (/rov/esc/telemetry)\n")
    if esc is not None and len(esc):
        for flag, label in (("failsafe", "failsafe"), ("agent_connected", "brak agenta")):
            if flag in esc:
                f = esc[flag].astype(bool).values if flag == "failsafe" else ~esc[flag].astype(bool).values
                iv = intervals(esc["t_rel"].values, f)
                w(f"{label}: {len(iv)} razy, lacznie {sum(e - s for s, e in iv):.1f} s" + (": " + ", ".join(f"{s:.1f}-{e:.1f}" for s, e in iv[:15]) if iv else "") + "\n\n")
        rows = []
        for i in range(8):
            r = [f"M{i + 1}"]
            for c, fn, n in ((f"rpm_{i}", "max", 0), (f"kiss_current_a_{i}", "max", 2), (f"kiss_temp_c_{i}", "max", 0)):
                r.append(fmt(float(getattr(esc[c], fn)()), n) if c in esc else "-")
            for c in (f"bidir_crc_errors_{i}", f"bidir_no_reply_{i}"):
                r.append(int(esc[c].iloc[-1] - esc[c].iloc[0]) if c in esc else "-")
            lag = None
            if thr is not None and f"throttle_{i}" in thr and f"rpm_{i}" in esc:
                lag = xcorr_lag(thr["t"], thr[f"throttle_{i}"].abs(), esc["t"], esc[f"rpm_{i}"])
            r += [fmt(lag[0] * 1000, 0) if lag else "-", fmt(lag[1], 2) if lag else "-"]
            rows.append(r)
        w(table(rows, ["silnik", "max RPM", "max I [A]", "max T [C]", "bledy CRC", "brak odp.", "opozn. gaz->RPM [ms]", "korelacja"]))
        if "esc_current_a_0" in esc:
            tot = esc.get("esc_current_a_0", 0) + esc.get("esc_current_a_1", 0)
            w(f"\nPrad ESC (CUR, suma): srednio {tot.mean():.1f} A, max {tot.max():.1f} A\n")
    else:
        w("brak telemetrii ESC\n")

    # IMU
    imu = dfs.get("/rov/imu")
    w("\n## IMU (/rov/imu)\n")
    if imu is not None and len(imu):
        s = rate_stats(imu["t"].values)
        w(f"{s['hz']:.1f} Hz, max dziura {s['max_gap'] * 1000:.0f} ms, dziur {s['gaps']}\n\n")
        g = [c for c in ("angular_velocity.x", "angular_velocity.y", "angular_velocity.z") if c in imu]
        acc = [c for c in ("linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z") if c in imu]
        if g:
            w("Zyro max |w| [deg/s] (osie czujnika): " + ", ".join(f"{c[-1]} {np.degrees(imu[c].abs().max()):.1f}" for c in g) + "\n\n")
        if len(acc) == 3:
            n = np.sqrt(sum(imu[c] ** 2 for c in acc))
            w(f"|a| [m/s2]: srednio {n.mean():.3f}, std {n.std():.3f}, min {n.min():.2f}, max {n.max():.2f}\n")
        if "t_hdr" in imu:
            d = (imu["t"] - imu["t_hdr"]) * 1000
            w(f"\nCzas odbioru - stempel Pico [ms]: mediana {d.median():.1f}, p95 {d.quantile(0.95):.1f}, min {d.min():.1f} "
              f"(duze/ujemne = brak synchronizacji zegara Pico)\n")

    # odpowiedz pojazdu i regulator
    ct = dfs.get("/rov/debug/ctrl")
    w("\n## Odpowiedz pojazdu: komenda osi -> zyro (/rov/debug/ctrl)\n")
    if ct is not None and len(ct) and "active" in ct:
        act = ct[ct["active"].astype(bool)]
        rows = []
        for k, name, gcol, sgn in ((3, "roll", "gyro_dps_0", -1), (4, "pitch", "gyro_dps_1", -1), (5, "yaw", "gyro_dps_2", 1)):
            c = f"out_{k}"
            if c in act and gcol in act and len(act) > 20:
                lag = xcorr_lag(act["t"], act[c], act["t"], sgn * act[gcol], fs=20.0, max_lag=1.5)
                rows.append([name, fmt(act[c].abs().max(), 2), fmt(act[gcol].abs().max(), 1),
                             fmt(lag[0] * 1000, 0) if lag else "-", fmt(lag[1], 2) if lag else "-"])
        w(table(rows, ["os", "max |cmd|", "max |w| [deg/s]", "opoznienie [ms]", "korelacja (znak + = zgodny kierunek)"]) if rows else "za malo danych z aktywnym pilotem\n")
        w("\n## Regulator (osie wlaczone)\n")
        rows = []
        for k in ("yaw", "roll", "pitch", "depth"):
            e = f"pid.{k}_0"
            if e in ct and ct[e].notna().any():
                sub = ct[ct[e].notna()]
                sat = sub[f"pid.{k}_5"].astype(bool).mean() * 100 if f"pid.{k}_5" in sub else float("nan")
                rows.append([k, len(sub), fmt(np.sqrt((sub[e] ** 2).mean()), 3), fmt(sub[e].abs().max(), 3),
                             fmt(sub[f"pid.{k}_2"].abs().max(), 3), fmt(sat, 1)])
        w(table(rows, ["os", "tickow", "RMS bledu", "max |blad|", "max |I|", "% nasycenia"]) if rows else "zadna os regulatora nie byla aktywna\n")
        if "imu_age" in ct:
            w(f"\nWiek IMU w regulatorze: max {ct['imu_age'].max() * 1000:.0f} ms\n")
    else:
        w("brak /rov/debug/ctrl\n")

    dep = dfs.get("/rov/depth")
    if dep is not None and len(dep) and "data" in dep:
        w(f"\n## Glebokosc\nmin {dep['data'].min():.2f} m, max {dep['data'].max():.2f} m\n")

    bms = dfs.get("/rov/bms/status")
    if bms is not None and len(bms) and "voltage_v" in bms:
        on = bms[bms["online"].astype(bool)] if "online" in bms else bms
        if len(on):
            w(f"\n## BMS\nU {on['voltage_v'].min():.2f}..{on['voltage_v'].max():.2f} V, I min {on['current_a'].min():.1f} A (rozladowanie), "
              f"SOC {on['soc_pct'].iloc[0]:.0f}% -> {on['soc_pct'].iloc[-1]:.0f}%, ogniwo min {on['min_cell_mv'].min()} mV, "
              f"max temp {on['max_temp_c'].max()} C, bledy (fault_bits != 0): {int((on[cols(on, 'fault_bits', 8)].sum(axis=1) > 0).sum())} probek\n")

    for topic, title in (("/rov/leak", "Zalanie"),):
        df = dfs.get(topic)
        if df is not None and len(df) and "data" in df:
            ch = df[df["data"] != df["data"].shift()]
            w(f"\n## {title}\n" + ", ".join(f"{r.t_rel:.1f}s={r.data}" for r in ch.itertuples()) + "\n")

    au = dfs.get("/rov/audit")
    w("\n## Audyt operatora (/rov/audit)\n")
    if au is not None and len(au):
        keep = [c for c in au.columns if c not in ("t", "t_hdr", "t_msg", "sid", "ua")]
        for r in au[keep].itertuples(index=False):
            d = {k: v for k, v in zip(keep, r) if v is not None and not (isinstance(v, float) and np.isnan(v))}
            w(f"- {d.pop('t_rel'):.1f}s " + " ".join(f"{k}={v}" for k, v in d.items()) + "\n")
    else:
        w("brak wpisow\n")

    for topic in ("/rov/arm", "/rov/ctrl/mode", "/rov/ctrl/setpoint", "/rov/ctrl/gains", "/rov/gain", "/rov/mixer/config", "/rov/thruster_test", "/rov/esc/command"):
        df = dfs.get(topic)
        if df is not None and len(df):
            keep = [c for c in df.columns if c not in ("t", "t_hdr")]
            chg = df[keep].astype(str).ne(df[keep].astype(str).shift()).drop(columns="t_rel").any(axis=1)
            sel = df[chg].head(60)
            w(f"\n### {topic} ({len(df)} wiad., zmian {int(chg.sum())})\n")
            for r in sel[keep].itertuples(index=False):
                d = dict(zip(keep, r))
                w(f"- {d.pop('t_rel'):.1f}s " + " ".join(f"{k}={v}" for k, v in d.items()) + "\n")

    ro = dfs.get("/rosout")
    w("\n## /rosout ostrzezenia i bledy\n")
    if ro is not None and len(ro) and "level" in ro:
        bad = ro[ro["level"] >= 30]
        w(f"WARN: {int((ro['level'] == 30).sum())}, ERROR: {int((ro['level'] == 40).sum())}, FATAL: {int((ro['level'] == 50).sum())}\n\n")
        for r in bad.tail(60).itertuples():
            w(f"- {r.t_rel:.1f}s [{ {30: 'W', 40: 'E', 50: 'F'}.get(r.level, r.level)}] {r.name}: {str(r.msg)[:200]}\n")

    txt = "".join(R)
    if a.output:
        with open(a.output, "w") as f:
            f.write(txt)
        print(a.output)
    else:
        print(txt)


def cmd_plot(a):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    bag = read_bag(a.bag, r"^/rov/(debug/mixer|debug/ctrl|esc/telemetry|depth|attitude|imu)$")
    dfs, t0 = frames(bag, a.start, a.end)
    mx, ct, esc, dep, att = (dfs.get(k) for k in ("/rov/debug/mixer", "/rov/debug/ctrl", "/rov/esc/telemetry", "/rov/depth", "/rov/attitude"))
    fig, axs = plt.subplots(6, 1, figsize=(16, 20), sharex=True)
    ax6 = ["surge", "sway", "heave", "roll", "pitch", "yaw"]
    if mx is not None and len(mx):
        for i, c in enumerate(cols(mx, "cmd", 6)):
            axs[0].plot(mx["t_rel"], mx[c], label=ax6[i])
        for s, e in intervals(mx["t_rel"].values, mx["armed"].astype(bool).values):
            for x in axs:
                x.axvspan(s, e, color="green", alpha=0.06)
        if "sat" in mx:
            axs[0].plot(mx["t_rel"], mx["sat"], "k--", lw=0.8, label="sat")
        for i, c in enumerate(cols(mx, "thr", 8)):
            axs[1].plot(mx["t_rel"], mx[c], label=f"M{i + 1}")
    axs[0].set_ylabel("cmd osi (mikser)"); axs[1].set_ylabel("gaz M1..M8")
    if esc is not None and len(esc):
        for i, c in enumerate(cols(esc, "rpm", 8)):
            axs[2].plot(esc["t_rel"], esc[c], label=f"M{i + 1}")
        if "failsafe" in esc:
            for s, e in intervals(esc["t_rel"].values, esc["failsafe"].astype(bool).values):
                axs[2].axvspan(s, e, color="red", alpha=0.15)
        tot = sum(esc[c] for c in cols(esc, "esc_current_a", 2)) if cols(esc, "esc_current_a", 2) else None
        if tot is not None:
            axs[5].plot(esc["t_rel"], tot, "r", label="I ESC [A]")
    axs[2].set_ylabel("RPM (czerwone = failsafe)")
    if ct is not None and len(ct):
        for i, c in enumerate(cols(ct, "gyro_dps", 3)):
            axs[3].plot(ct["t_rel"], ct[c], label="wx wy wz".split()[i])
    axs[3].set_ylabel("zyro cialo [deg/s]")
    if att is not None and len(att):
        for c, l in (("x", "roll"), ("y", "pitch"), ("z", "yaw")):
            if c in att:
                axs[4].plot(att["t_rel"], att[c], label=l)
    axs[4].set_ylabel("katy [deg]")
    if dep is not None and len(dep) and "data" in dep:
        x2 = axs[5].twinx()
        x2.plot(dep["t_rel"], dep["data"], "b", label="glebokosc [m]"); x2.invert_yaxis(); x2.set_ylabel("glebokosc [m]")
    axs[5].set_ylabel("prad [A]"); axs[5].set_xlabel("t [s] od poczatku bagu (zielone = ARM)")
    for x in axs:
        x.grid(alpha=0.3)
        if x.get_legend_handles_labels()[0]:
            x.legend(loc="upper right", fontsize=8, ncol=8)
    fig.tight_layout()
    out = a.output or (a.bag[0].rstrip("/") + ".png")
    fig.savefig(out, dpi=80)
    print(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    for name in ("info", "report", "export", "plot"):
        p = sp.add_parser(name)
        p.add_argument("bag", nargs="+")
        p.add_argument("-o", "--output")
        p.add_argument("--start", type=float)
        p.add_argument("--end", type=float)
        if name == "export":
            p.add_argument("--topics", help="regex nazw topikow")
    a = ap.parse_args()
    {"info": cmd_info, "report": cmd_report, "export": cmd_export, "plot": cmd_plot}[a.cmd](a)


if __name__ == "__main__":
    main()
