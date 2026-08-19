#!/usr/bin/env python3
# ============================================================================
# upload_mission.py — QGC WPL 110 (.waypoints) görev dosyasını ArduPilot
# aracına pymavlink ile yükler ve mission_count'u geri okuyup doğrular.
#
# BU DOSYA YENİDİR — mevcut IBVS/güdüm koduna dokunmaz. Yalnızca MAVLink
# görev protokolü (MISSION_COUNT / MISSION_REQUEST(_INT) / MISSION_ITEM_INT /
# MISSION_ACK) üzerinden waypoint yükler.
#
# Kullanım:
#   python3 scripts/upload_mission.py missions/kare_40m.waypoints \
#       --connect tcp:127.0.0.1:5770 --sysid 2
#
#   --connect  MAVLink bağlantısı (varsayılan: udpin:0.0.0.0:14555 = hedef
#              drone'un Mission Planner çıkışı). ⚠ Mission Planner AÇIKKEN bu
#              portu O tutar; CLI ile yüklerken MP'yi kapat (ya da MP'nin kendi
#              WP yükleme aracını kullan). Alternatif: udpin:0.0.0.0:14551
#              (start_harmonic'te var, iki araç paylaşımlı — --sysid şart).
#   --sysid    Hedef araç sistem kimliği (varsayılan: 2 = ArduPlane hedef drone).
# ============================================================================
import argparse
import sys
import time

from pymavlink import mavutil


def dosyayi_oku(yol):
    """QGC WPL 110 (tab ile ayrılmış) dosyayı satır listesine çevirir.

    Her satır 12 alan: seq, current, frame, command, p1, p2, p3, p4,
    x(lat), y(lon), z(alt), autocontinue.
    """
    with open(yol) as f:
        satirlar = [s.rstrip("\n") for s in f]
    if not satirlar or not satirlar[0].startswith("QGC WPL 110"):
        sys.exit("HATA: dosya 'QGC WPL 110' başlığıyla başlamıyor: " + yol)
    kayitlar = []
    for s in satirlar[1:]:
        if not s.strip():
            continue
        p = s.split("\t")
        if len(p) != 12:
            sys.exit(f"HATA: 12 alan bekleniyordu, {len(p)} bulundu: {s!r}")
        kayitlar.append({
            "seq": int(p[0]), "current": int(p[1]), "frame": int(p[2]),
            "command": int(p[3]),
            "p1": float(p[4]), "p2": float(p[5]), "p3": float(p[6]),
            "p4": float(p[7]), "x": float(p[8]), "y": float(p[9]),
            "z": float(p[10]), "autocontinue": int(p[11]),
        })
    return kayitlar


def main():
    ap = argparse.ArgumentParser(description="ArduPilot'a QGC WPL görev yükler")
    ap.add_argument("dosya", help=".waypoints (QGC WPL 110) dosyası")
    ap.add_argument("--connect", default="udpin:0.0.0.0:14555",
                    help="MAVLink bağlantısı (varsayılan udpin:0.0.0.0:14555)")
    ap.add_argument("--sysid", type=int, default=2,
                    help="Hedef araç sysid (varsayılan 2 = ArduPlane)")
    args = ap.parse_args()

    kayitlar = dosyayi_oku(args.dosya)
    print(f"[UPLOAD] {len(kayitlar)} kalem okundu: {args.dosya}")

    print(f"[UPLOAD] bağlanılıyor: {args.connect} (sysid {args.sysid})")
    m = mavutil.mavlink_connection(args.connect, source_system=255)
    m.wait_heartbeat(timeout=15)
    if not m.target_system:
        sys.exit("HATA: heartbeat alınamadı — bağlantı/port yanlış olabilir.")
    # Doğru aracı hedefle (14550 gibi paylaşımlı bağlantıda kritik).
    m.target_system = args.sysid
    m.target_component = 1
    print(f"[UPLOAD] araç bulundu: sysid={m.target_system}")

    n = len(kayitlar)
    # 1) Yükleme başlat: MISSION_COUNT
    m.mav.mission_count_send(m.target_system, m.target_component, n,
                             mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    # 2) Araç sırayla her kalemi ister → MISSION_ITEM_INT gönder
    gonderilen = set()
    t0 = time.time()
    while len(gonderilen) < n:
        if time.time() - t0 > 30:
            sys.exit("HATA: yükleme zaman aşımı (araç kalem istemedi).")
        msg = m.recv_match(
            type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
            blocking=True, timeout=5)
        if msg is None:
            continue
        if msg.get_type() == "MISSION_ACK":
            # Erken ACK = hata (yükleme bitmeden)
            sys.exit(f"HATA: beklenmeyen MISSION_ACK: {msg.type}")
        i = msg.seq
        k = kayitlar[i]
        m.mav.mission_item_int_send(
            m.target_system, m.target_component, k["seq"], k["frame"],
            k["command"], k["current"], k["autocontinue"],
            k["p1"], k["p2"], k["p3"], k["p4"],
            int(round(k["x"] * 1e7)), int(round(k["y"] * 1e7)), k["z"],
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        gonderilen.add(i)
        print(f"  → kalem {i}/{n-1} gönderildi (cmd {k['command']})")

    # 3) MISSION_ACK bekle
    ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
    if ack is None or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        sys.exit(f"HATA: yükleme ACK başarısız: {ack.type if ack else 'yok'}")
    print("[UPLOAD] araç yüklemeyi KABUL etti (MISSION_ACCEPTED)")

    # 4) DOĞRULAMA — mission_count'u geri oku
    m.mav.mission_request_list_send(m.target_system, m.target_component,
                                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    say = m.recv_match(type="MISSION_COUNT", blocking=True, timeout=10)
    if say is None:
        sys.exit("HATA: doğrulama için MISSION_COUNT okunamadı.")
    print(f"[UPLOAD] DOĞRULAMA: araçtaki kalem sayısı = {say.count} "
          f"(yüklenen {n})")
    if say.count != n:
        sys.exit(f"HATA: sayı uyuşmuyor! araç {say.count}, dosya {n}")
    print("[UPLOAD] ✓ BAŞARILI — görev yüklendi ve doğrulandı.")


if __name__ == "__main__":
    main()
