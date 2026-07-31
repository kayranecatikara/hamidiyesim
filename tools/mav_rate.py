#!/usr/bin/env python3
"""MAVLink stream-hızı ölçer — telemetri frekansını DOĞRUDAN ölçer.

Neden: GPS güdüm loglarından türetilen "tazeleme hızı" EMA + yuvarlama yüzünden
dolaylı. Bu araç ham MAVLink akışını dinleyip her (sysid, mesaj_tipi) için gerçek
Hz'i sayar. Konum güncelleme hızının gerçekten yükselip yükselmediğini kanıtlar.

Kullanım:
    python3 tools/mav_rate.py               # 14551'i 8 sn dinle (MP kapalıyken)
    python3 tools/mav_rate.py --port 14551 --secs 10

14551 hem copter (sysid 5) hem plane (sysid 2) --out portu. Mission Planner açıksa
14551'i o bağladığından çakışır; o zaman MP'yi kapat ya da başka --out portu ver.
"""
import argparse
import time
from collections import defaultdict

from pymavlink import mavutil

WATCH = ("LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "ATTITUDE", "HEARTBEAT")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=14551)
    ap.add_argument("--secs", type=float, default=8.0)
    args = ap.parse_args()

    conn = mavutil.mavlink_connection(f"udpin:0.0.0.0:{args.port}")
    print(f"[mav_rate] udpin:0.0.0.0:{args.port} dinleniyor, {args.secs:.0f} sn...")

    counts = defaultdict(int)          # (sysid, type) -> adet
    t0 = None
    end = time.monotonic() + args.secs
    while time.monotonic() < end:
        msg = conn.recv_match(type=list(WATCH), blocking=True, timeout=1.0)
        if msg is None:
            continue
        if t0 is None:
            t0 = time.monotonic()      # ilk paketten itibaren say
        counts[(msg.get_srcSystem(), msg.get_type())] += 1

    dur = (time.monotonic() - t0) if t0 else 0.0
    if dur <= 0:
        print("[mav_rate] Hiç paket gelmedi — port/SITL çalışıyor mu?")
        return

    print(f"[mav_rate] Ölçüm süresi {dur:.1f} sn\n")
    print(f"{'sysid':>5} {'mesaj':<22} {'adet':>6} {'Hz':>7}")
    print("-" * 44)
    for (sid, typ) in sorted(counts):
        n = counts[(sid, typ)]
        print(f"{sid:>5} {typ:<22} {n:>6} {n/dur:>7.1f}")
    print()
    # Öne çıkan: konum hızı (güdümün gördüğü)
    for sid in sorted({s for (s, _) in counts}):
        arac = "iris/copter" if sid == 5 else ("plane" if sid == 2 else f"sys{sid}")
        lp = counts.get((sid, "LOCAL_POSITION_NED"), 0) / dur
        print(f"  → {arac} (sysid {sid}) LOCAL_POSITION_NED = {lp:.1f} Hz")


if __name__ == "__main__":
    main()
