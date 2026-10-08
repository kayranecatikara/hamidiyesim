#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
GÖREV SAĞLAMASI — "VAKKAS modeli GERÇEK güdüm komutlarını uçabiliyor mu?"
================================================================================
NİYE VAR:
  Eğim zarfı deneyi (tools/egim_zarfi.py) SET_ATTITUDE_TARGET ile araca
  ANINDA 30-45° yatma emri veriyor. Bu, güdümün ASLA ürettiği bir komut
  DEĞİLDİR — güdüm hız vektörü komut eder ve ArduPilot onu kendi ivme/jerk
  tavanlarıyla yumuşatarak eğime çevirir (mode_guided.cpp).
  Zarf deneyi tavanı bulmak için doğru araçtır; ama "araç uçuyor mu"
  sorusunun cevabı DEĞİLDİR. Bu betik onu sınar.

NE YAPAR:
  control/guidance/gps_guidance.py'nin kullandığı MESAJIN AYNISINI gönderir:
      SET_POSITION_TARGET_LOCAL_NED, type_mask = yalnız HIZ
  Kare şeklinde bir hız profili uçurur (kuzey → doğu → güney → batı) ve
  şunları ölçer:
    • hız takibi        — komut edilen hıza ulaşabiliyor mu
    • irtifa tutuşu     — manevra sırasında sarkma
    • motor doygunluğu  — u ≥ 0.99 kare oranı (eklenti tavanı 1.00)  (duruş yetkisi kalıyor mu)
    • duruş salınımı    — yatış işaret değişimi/s (CLAUDE.md §4: salınım
                          ölçülmeden "iyi" denmez)

⛔ SİM ölçümüdür; güdüm kabul kararı vermez (CLAUDE.md §2).
================================================================================
"""
import argparse
import math
import statistics
import sys
import time

from pymavlink import mavutil

GUIDED = 4
_S = {"alt": None, "rol": 0.0, "pit": 0.0, "yaw": 0.0,
      "vx": 0.0, "vy": 0.0, "vz": 0.0, "u": 0.0}


def durum(c, sure=0.1):
    t0 = time.time()
    while time.time() - t0 < sure:
        m = c.recv_match(type=["GLOBAL_POSITION_INT", "ATTITUDE",
                               "SERVO_OUTPUT_RAW", "LOCAL_POSITION_NED"],
                         blocking=True, timeout=0.5)
        if m is None:
            break
        t = m.get_type()
        if t == "GLOBAL_POSITION_INT":
            _S["alt"] = m.relative_alt / 1000.0
        elif t == "ATTITUDE":
            _S["rol"], _S["pit"], _S["yaw"] = m.roll, m.pitch, m.yaw
        elif t == "SERVO_OUTPUT_RAW":
            _S["u"] = ((m.servo1_raw + m.servo2_raw + m.servo3_raw
                        + m.servo4_raw) / 4.0 - 1100.0) / 800.0
        else:
            _S["vx"], _S["vy"], _S["vz"] = m.vx, m.vy, m.vz
    return _S


def hiz(c, vn, ve, vd):
    """gps_guidance.send_velocity ile AYNI mesaj/type_mask."""
    c.mav.set_position_target_local_ned_send(
        0, c.target_system, c.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, 0, 0, 0, vn, ve, vd, 0, 0, 0, 0, 0)


def bacak(c, vn, ve, sure, ad):
    print("\n  ── %s: komut (%+.0f, %+.0f) m/s, %.0f s" % (ad, vn, ve, sure))
    hedef = math.hypot(vn, ve)
    hizlar, irtifalar, us, yatislar = [], [], [], []
    t0 = time.time()
    while time.time() - t0 < sure:
        hiz(c, vn, ve, 0.0)
        d = durum(c, 0.1)
        if d["alt"] is None:
            continue
        hizlar.append(math.hypot(d["vx"], d["vy"]))
        irtifalar.append(d["alt"])
        us.append(d["u"])
        yatislar.append(math.degrees(d["rol"]))
    if len(hizlar) < 10:
        return None
    # son yarıda oturmuş sayılır
    y = len(hizlar) // 2
    v_med = statistics.median(hizlar[y:])
    doyum = sum(1 for x in us if x > 0.99) / float(len(us))
    isaret = sum(1 for i in range(1, len(yatislar))
                 if yatislar[i - 1] * yatislar[i] < 0) / sure
    print("     ulaşılan hız  : %.1f / %.1f m/s  (%%%.0f)"
          % (v_med, hedef, 100 * v_med / max(hedef, 1e-6)))
    print("     irtifa        : %.1f → %.1f m  (sarkma %+.1f m)"
          % (irtifalar[0], irtifalar[-1], irtifalar[-1] - irtifalar[0]))
    print("     |yatış| p90   : %.1f°   tepe motor u : %.3f   doyum: %%%.0f"
          % (sorted(abs(x) for x in yatislar)[int(0.9 * len(yatislar))],
             max(us), doyum * 100))
    print("     yatış işaret değişimi: %.2f /s  (salınım göstergesi)" % isaret)
    return {"ad": ad, "v": v_med, "hedef": hedef,
            "sarkma": irtifalar[-1] - irtifalar[0],
            "u_tepe": max(us), "doyum": doyum, "salinim": isaret}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--adres", default="udpin:127.0.0.1:14541")
    ap.add_argument("--irtifa", type=float, default=60.0)
    ap.add_argument("--hiz", type=float, default=18.0,
                    help="bacak hızı (m/s) — hedef Talon ~15-16 m/s uçuyor")
    ap.add_argument("--bacak", type=float, default=8.0)
    a = ap.parse_args()

    c = mavutil.mavlink_connection(a.adres)
    c.wait_heartbeat()
    c.mav.request_data_stream_send(c.target_system, c.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 15, 1)
    time.sleep(1)
    print("bağlandı: sys=%d" % c.target_system)

    t0 = time.time()
    while time.time() - t0 < 120:
        m = c.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=2)
        if m and (m.flags & 0x08) and (m.flags & 0x10):
            break
    time.sleep(2)

    armli = False
    t0 = time.time()
    while time.time() - t0 < 30 and not armli:
        c.mav.set_mode_send(c.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            GUIDED)
        time.sleep(0.5)
        c.mav.command_long_send(c.target_system, c.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, 0, 0, 0, 0, 0, 0)
        for _ in range(4):
            hb = c.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb and (hb.base_mode
                       & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                armli = True
                break
    if not armli:
        print("⛔ ARM olmadı")
        return 1

    c.mav.command_long_send(c.target_system, c.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                            0, 0, 0, 0, 0, 0, 0, a.irtifa)
    t0 = time.time()
    while time.time() - t0 < 120:
        d = durum(c, 0.3)
        if d["alt"] and d["alt"] >= a.irtifa - 2:
            break
    print("kalkış tamam: %.1f m  (%.0f s)" % (durum(c)["alt"], time.time() - t0))
    time.sleep(3)

    V = a.hiz
    sonuc = []
    for vn, ve, ad in ((V, 0, "KUZEY"), (0, V, "DOĞU"),
                       (-V, 0, "GÜNEY"), (0, -V, "BATI"),
                       (0, 0, "FREN")):
        r = bacak(c, vn, ve, a.bacak, ad)
        if r:
            sonuc.append(r)

    print("\n" + "=" * 76)
    print("  SONUÇ — VAKKAS GÖREV SAĞLAMASI (%.0f m/s kare, %.0f m irtifa)"
          % (V, a.irtifa))
    print("=" * 76)
    print("  %-7s %11s %10s %9s %8s %10s"
          % ("bacak", "hız (m/s)", "sarkma", "tepe u", "doyum", "salınım/s"))
    for r in sonuc:
        print("  %-7s %5.1f/%4.1f %9.1f m %9.3f %7.0f%% %10.2f"
              % (r["ad"], r["v"], r["hedef"], r["sarkma"], r["u_tepe"],
                 r["doyum"] * 100, r["salinim"]))
    ucus = [r for r in sonuc if r["hedef"] > 0]
    print()
    print("  hız takibi ortalaması : %%%.0f"
          % (100 * statistics.mean(r["v"] / r["hedef"] for r in ucus)))
    print("  en büyük irtifa sarkması: %.1f m" % max(abs(r["sarkma"]) for r in sonuc))
    print("  en yüksek doyum oranı  : %%%.0f" % (100 * max(r["doyum"] for r in sonuc)))
    print()
    if (max(r["doyum"] for r in sonuc) < 0.02
            and max(abs(r["sarkma"]) for r in sonuc) < 5.0
            and statistics.mean(r["v"] / r["hedef"] for r in ucus) > 0.85):
        print("  ✔ ARAÇ GÖREV KOMUTLARINI UÇUYOR: doyum yok, irtifa sağlam,")
        print("    komut edilen hıza ulaşıyor.")
    else:
        print("  ⚠ ölçütlerden biri sınırda — üstteki tabloya bak.")
    print("  ⛔ SİM ölçümüdür; güdüm kabul kararı vermez (CLAUDE.md §2).")
    c.mav.set_mode_send(c.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6)
    return 0


if __name__ == "__main__":
    sys.exit(main())
