# -*- coding: utf-8 -*-
"""
Talon: kanıtlanmış kalkış → AUTO (MP'de ÇİZDİĞİN rotayı uçar).
=============================================================
Kanatlı Talon MP'den elle kalkamıyor: yerden yuvarlanarak hız kazanamıyor ve
düşük irtifada (~4.5 m) banka girince takla atıyor (kod bunu belgeliyor:
run_plane_scenario.takeoff). Bu script projenin KANITLANMIŞ kalkışını
(TAKEOFF modu + RC override ile GÜVENLİ ~30 m'ye) yapar, sonra RC override'ı
bırakıp AUTO'ya geçer — böylece MP'de yükleyip **Write** ettiğin rota
(waypoint + DO_JUMP döngüsü) uçulur. Bitince rota kendini tekrarlar.

SIRA:
  1) MP'de rotanı: Plan → Load File → Write  (araca YAZ — zaten yaptın)
  2) Repo kökünden:  python3 -m control.talon_kalkis_auto
  3) Gazebo'da: mini_talon → sağ tık → Follow  (izle)

Güdüm/IBVS/gps koduna DOKUNMAZ — yalnız hedefin kalkış + AUTO devri (yeni dosya).
"""
import time

from control.plane_functions import (
    connect_plane, get_conn, arm_plane,
    start_gcs_keepalive, stop_gcs_keepalive,
)
from control.mav_common import set_mode, PLANE_MODE_AUTO
# Kanıtlanmış kalkış (güvenli irtifa kapısı + RC override ile gaz) — yeniden kullan
from control.run_plane_scenario import takeoff

GUVENLI_ALT = 30.0   # AUTO'ya geçmeden önce çıkılacak güvenli irtifa (m)


def _rc_birak(conn):
    """Tüm RC override kanallarını 0 = SERBEST bırak → AUTO gaz/yönü devralsın."""
    conn.mav.rc_channels_override_send(
        conn.target_system, conn.target_component, 0, 0, 0, 0, 0, 0, 0, 0)


def main():
    print("[TALON] Bağlanılıyor...")
    connect_plane()
    conn = get_conn()
    start_gcs_keepalive()                    # kalkışta RC override kabul edilsin

    print("[TALON] ARM ediliyor...")
    r = arm_plane(warmup_duration=3.0)
    if r is None or (isinstance(r, (tuple, list)) and len(r) > 1 and r[1] != 0):
        print("[TALON] ARM başarısız — durduruldu.")
        stop_gcs_keepalive()
        return

    print("[TALON] Kanıtlanmış kalkış (TAKEOFF modu → güvenli ~%.0f m)..." % GUVENLI_ALT)
    takeoff(conn, hedef_alt=GUVENLI_ALT)     # güvenli irtifaya çıkana kadar TAKEOFF'ta kalır

    print("[TALON] RC override bırakılıyor + AUTO'ya geçiliyor (rotanı uçacak)...")
    stop_gcs_keepalive()                     # RC override kaynağını durdur
    _rc_birak(conn)                          # kalan override'ları serbest bırak
    time.sleep(0.5)
    set_mode(conn, PLANE_MODE_AUTO)          # araca yüklü görevi (döngü) uç

    print("[TALON] ✓ AUTO aktif — Talon rotayı döngüyle uçuyor. Gazebo'da izle.")
    print("[TALON] (MP bağlı kaldığı sürece GCS failsafe tetiklenmez.)")
    # 60 sn irtifa izleme (sonra script çıkabilir; AUTO uçmaya devam eder)
    t0 = time.time()
    while time.time() - t0 < 60:
        m = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if m:
            print("  irtifa = %.0f m" % (m.relative_alt / 1000.0))
        time.sleep(2)
    print("[TALON] İzleme bitti — Talon AUTO'da uçmaya devam ediyor.")


if __name__ == "__main__":
    main()
