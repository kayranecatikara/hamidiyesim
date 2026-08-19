# -*- coding: utf-8 -*-
"""
Talon: kalkış → AUTO'da rota → başlangıca dön → alçalarak İN.
============================================================
talon_kalkis_auto'nun uzantısı: rota (yüklü mission) uçulup bitince, projenin
KANITLANMIŞ iniş rutini (run_plane_scenario.inis, motoru açık tutan kademeli
gövde inişi) ile uçak BAŞLANGIÇ noktasına dönüp iner. (Bu model ArduPlane'in
kendi NAV_LAND'ini yapamıyor — süzülemeyip çakılıyor; inis() bu yüzden var.)

Mission'ın SONU RTL (cmd 20) olmalı: rota bitince mod RTL'e döner, biz bunu
yakalayıp inişi başlatırız.

Kullanım (repo kökünden):  python3 -m control.talon_rota_inis
Güdüm/IBVS/gps koduna dokunmaz — yalnız hedefin kalkış+rota+iniş devri.
"""
import time
from pymavlink import mavutil

from control.plane_functions import (
    connect_plane, get_conn, arm_plane,
    start_gcs_keepalive, stop_gcs_keepalive,
)
from control.mav_common import set_mode, PLANE_MODE_AUTO
from control.run_plane_scenario import takeoff, inis, _ev_noktasi

GUVENLI_ALT = 30.0
ROTA_MAX_SN = 400.0   # rota + dönüş için üst süre


def _rc_birak(conn):
    conn.mav.rc_channels_override_send(
        conn.target_system, conn.target_component, 0, 0, 0, 0, 0, 0, 0, 0)


def main():
    print("[TALON] Bağlanılıyor...")
    connect_plane()
    conn = get_conn()
    start_gcs_keepalive()

    print("[TALON] ARM...")
    r = arm_plane(warmup_duration=3.0)
    if r is None or (isinstance(r, (tuple, list)) and len(r) > 1 and r[1] != 0):
        print("[TALON] ARM başarısız."); stop_gcs_keepalive(); return

    print("[TALON] Kalkış (TAKEOFF modu → güvenli ~%.0f m)..." % GUVENLI_ALT)
    takeoff(conn, hedef_alt=GUVENLI_ALT)

    print("[TALON] AUTO — rotayı uçuyor (bitince RTL'e dönecek)...")
    stop_gcs_keepalive()
    _rc_birak(conn)
    time.sleep(0.5)
    set_mode(conn, PLANE_MODE_AUTO)

    # Rota bitişini bekle: mission sonu RTL kaleminde mod AUTO→RTL olur.
    print("[TALON] Rota uçuluyor; bitişte (mod=RTL) inişe geçilecek...")
    t0 = time.time(); rota_bitti = False
    while time.time() - t0 < ROTA_MAX_SN:
        hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if not hb or hb.get_srcSystem() != 2:
            continue
        mod = mavutil.mode_string_v10(hb)
        if mod == 'RTL':
            rota_bitti = True
            print("[TALON] ✓ Rota bitti (RTL) — başlangıca dönüp İNİŞE geçiliyor")
            break
    if not rota_bitti:
        print("[TALON] Rota bitişi yakalanamadı (süre doldu) — yine de inişe geçiliyor")

    # İniş: başlangıç noktasına yönel + motoru açık kademeli gövde inişi
    start_gcs_keepalive()          # inis() RC override kullanır
    ev = _ev_noktasi(conn)
    if ev is None:
        print("[TALON] ⛔ Başlangıç noktası belirlenemedi — iniş güvenli değil, iptal.")
        stop_gcs_keepalive(); return
    # İNİŞ İÇİN THR_MIN=0: rota irtifasını tutan THR_MIN=60, inişte gazı
    # %60'ta kilitleyip uçağı alçaltmıyordu (climb). İniş için taban 0 olmalı.
    conn.mav.param_set_send(conn.target_system, conn.target_component,
                            b"THR_MIN", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.5)
    print("[TALON] İNİŞ — başlangıç noktasına alçalarak iniyor (THR_MIN=0)...")
    inis(conn, hedef=ev)
    print("[TALON] ✓ İniş tamamlandı — Talon başlangıçta yerde.")
    stop_gcs_keepalive()


if __name__ == "__main__":
    main()
