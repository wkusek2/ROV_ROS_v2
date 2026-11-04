#!/usr/bin/env python3
"""Ciagle nagrywanie kamery ROV na PC (laptop operatora), bez ROS i bez ponownej kompresji.

Laczy sie ze strumieniem MJPEG z Jetsona (camera_node, http://<jetson>:8089/cam.mjpg) i zapisuje klatki 1:1:
  <out>/rov_YYYYmmdd_HHMMSS.mjpeg   - surowe JPEG-i jeden za drugim (odtwarzanie: ffplay -f mjpeg plik)
  <out>/rov_YYYYmmdd_HHMMSS.csv     - n, frame (nr klatki Jetsona), t_jetson, t_pc, bajty
Nowy plik co --segment minut, po zerwaniu polaczenia sam sie laczy ponownie (nowy segment),
--max-gb kasuje najstarsze nagrania. Ctrl+C = zamkniecie plikow i wyjscie.

  python3 scripts/pc_record.py                       # domyslnie 192.168.1.108, ~/rov_recordings
  python3 scripts/pc_record.py --host 192.168.1.108 --out ~/rov_recordings --segment 10 --max-gb 50
Konwersja do mp4 (opcjonalnie): ffmpeg -f mjpeg -framerate 50 -i plik.mjpeg -c:v libx264 -crf 20 plik.mp4
"""
import argparse
import glob
import os
import sys
import time
import urllib.request
from datetime import datetime


def read_frames(url, timeout):
    """Generator (jpeg_bytes, frame_nr|None, t_jetson|None) z multipart/x-mixed-replace."""
    with urllib.request.urlopen(url, timeout=timeout) as r:
        while True:
            line = r.readline()
            if not line:
                raise ConnectionError("koniec strumienia")
            if not line.startswith(b"--"):
                continue                       # szukaj granicy czesci
            hdr = {}
            while True:
                h = r.readline()
                if not h:
                    raise ConnectionError("koniec strumienia")
                h = h.strip()
                if not h:
                    break
                k, _, v = h.partition(b":")
                hdr[k.strip().lower()] = v.strip()
            n = int(hdr.get(b"content-length", b"0"))
            if n <= 0:
                continue
            data = r.read(n)
            if len(data) != n:
                raise ConnectionError("urwana klatka")
            fr = hdr.get(b"x-frame")
            ts = hdr.get(b"x-timestamp")
            yield data, int(fr) if fr else None, float(ts) if ts else None


class Segment:
    def __init__(self, out):
        name = datetime.now().strftime("rov_%Y%m%d_%H%M%S")
        self.path = os.path.join(out, name)
        self.f = open(self.path + ".mjpeg", "wb")
        self.csv = open(self.path + ".csv", "w")
        self.csv.write("n,frame,t_jetson,t_pc,bytes\n")
        self.t0, self.n = time.time(), 0
        print(f"[rec] {self.path}.mjpeg", flush=True)

    def write(self, data, fr, ts):
        self.f.write(data)
        self.csv.write(f"{self.n},{'' if fr is None else fr},{'' if ts is None else f'{ts:.6f}'},{time.time():.6f},{len(data)}\n")
        self.n += 1

    def close(self):
        self.f.close(); self.csv.close()
        if self.n == 0:                        # pusty segment (np. od razu zerwane polaczenie)
            for ext in (".mjpeg", ".csv"):
                try:
                    os.remove(self.path + ext)
                except OSError:
                    pass


def prune(out, max_bytes):
    files = sorted(glob.glob(os.path.join(out, "rov_*.mjpeg")))
    total = sum(os.path.getsize(f) for f in files)
    while files and total > max_bytes:
        f = files.pop(0)
        total -= os.path.getsize(f)
        for p in (f, f[:-6] + ".csv"):
            try:
                os.remove(p)
            except OSError:
                pass
        print(f"[rec] limit miejsca - usunieto {os.path.basename(f)}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.1.108")
    ap.add_argument("--port", type=int, default=8089)
    ap.add_argument("--out", default="~/rov_recordings")
    ap.add_argument("--segment", type=float, default=10.0, help="dlugosc pliku [min]")
    ap.add_argument("--max-gb", type=float, default=50.0, help="limit miejsca na nagrania (0 = bez limitu)")
    a = ap.parse_args()
    out = os.path.expanduser(a.out)
    os.makedirs(out, exist_ok=True)
    url = f"http://{a.host}:{a.port}/cam.mjpg"
    print(f"[rec] {url} -> {out}  (segment {a.segment} min, limit {a.max_gb} GB)", flush=True)

    seg, wait = None, 1.0
    try:
        while True:
            try:
                seg = Segment(out)
                t_rep, n_rep, last_fr, lost = time.time(), 0, None, 0
                for data, fr, ts in read_frames(url, timeout=5.0):
                    wait = 1.0
                    if time.time() - seg.t0 > a.segment * 60:
                        seg.close(); seg = Segment(out)
                        if a.max_gb > 0:
                            prune(out, a.max_gb * 1e9)
                    seg.write(data, fr, ts)
                    if fr is not None and last_fr is not None and fr > last_fr + 1:
                        lost += fr - last_fr - 1       # klatki zgubione po drodze (siec/PC nie nadaza)
                    last_fr = fr
                    n_rep += 1
                    dt = time.time() - t_rep
                    if dt >= 5.0:
                        lat = f", opoznienie {1000 * (time.time() - ts):.0f} ms" if ts else ""
                        print(f"[rec] {n_rep / dt:.1f} fps, {len(data) / 1024:.0f} kB/kl, zgubione {lost}{lat}", flush=True)
                        t_rep, n_rep = time.time(), 0
            except (OSError, ConnectionError, ValueError) as e:
                print(f"[rec] brak strumienia ({e}) - ponowna proba za {wait:.0f} s", flush=True)
            if seg:
                seg.close(); seg = None
            time.sleep(wait)
            wait = min(wait * 2, 10.0)
    except KeyboardInterrupt:
        pass
    finally:
        if seg:
            seg.close()
        print("[rec] stop", flush=True)


if __name__ == "__main__":
    sys.exit(main())
