# -*- coding: utf-8 -*-
"""
Talon: ELLE FIRLATMA → rota → başa dön → in.
============================================
Gerçek hayattaki hand-launch'ı simüle eder: TKOFF_THR_MINACC=12 (uçak fırlatma
ivmesi bekler). Kalkışta Gazebo'nun apply-link-wrench eklentisiyle uçağın
gövdesine kısa bir ileri+yukarı İTİŞ (=fırlatma) uygulanır; bu ivme kalkış
gazını tetikler ve uçak elden atılmış gibi havalanır. Sonra rotayı uçar,
başlangıca döner ve motoru açık kademeli inişle iner.

Kullanım (repo kökünden):  python3 -m control.talon_firlat_rota_inis
Dünya adı 'avci' varsayılır (wrench topic /world/avci/wrench/persistent).
"""
import time
import subprocess
from pymavlink import mavutil

from control.plane_functions import (
    connect_plane, get_conn, arm_plane,
    start_gcs_keepalive, stop_gcs_keepalive,
)
from control.mav_common import set_mode, PLANE_MODE_TAKEOFF, PLANE_MODE_AUTO
from control.run_plane_scenario import inis, _ev_noktasi, _pump, _pos

WORLD = "avci"
LINK = "mini_talon::base_link"
GUVENLI_ALT = 30.0
# Fırlatma itişi (kalibre: 100N×1.2s→24 m/s; ~15 m/s için daha kısa)
FIRLAT_Y = 100.0    # ileri (kuzey) kuvvet, N
FIRLAT_Z = 40.0     # yukarı kuvvet, N (uçağı hafif kaldırır)
FIRLAT_SN = 0.7     # itiş süresi


def _wrench(force_y, force_z):
    subprocess.run(["gz", "topic", "-t", "/world/%s/wrench/persistent" % WORLD,
                    "-m", "gz.msgs.EntityWrench",
                    "-p", 'entity {name:"%s" type:LINK} wrench {force {x:0 y:%g z:%g}}'
                    % (LINK, force_y, force_z)],
                   timeout=5, capture_output=True)


def _wrench_temizle():
    subprocess.run(["gz", "topic", "-t", "/world/%s/wrench/clear" % WORLD,
                    "-m", "gz.msgs.Entity", "-p", 'name:"%s" type:LINK' % LINK],
                   timeout=5, capture_output=True)


def firlat(conn):
    """Elle fırlatma: kısa ileri+yukarı itiş uygula, sonra bırak."""
    print("[FIRLAT] Uçak fırlatılıyor (itiş %gN ileri, %gN yukarı, %.1fs)..."
          % (FIRLAT_Y, FIRLAT_Z, FIRLAT_SN))
    _wrench(FIRLAT_Y, FIRLAT_Z)
    t0 = time.time()
    while time.time() - t0 < FIRLAT_SN:
        _pump(conn)               # GCS canlı + RC akışı sürsün
        time.sleep(0.05)
    _wrench_temizle()
    print("[FIRLAT] İtiş bırakıldı — uçak serbest uçuşta")


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

    print("[TALON] TAKEOFF modu (uçak fırlatma bekliyor, TKOFF_THR_MINACC=12)...")
    set_mode(conn, PLANE_MODE_TAKEOFF)
    time.sleep(0.5)

    # ELLE FIRLATMA
    firlat(conn)

    # Güvenli irtifaya tırman (fırlatma sonrası motor devraldı)
    print("[TALON] Güvenli ~%.0f m'ye tırmanış bekleniyor..." % GUVENLI_ALT)
    t0 = time.time(); alt = 0.0
    while time.time() - t0 < 30:
        _pump(conn)                 # _pos'u günceller
        alt = -_pos["z"]            # NED z → irtifa (gerçek okuma)
        if alt >= GUVENLI_ALT:
            print("[TALON] ✓ %.0f m — güvenli irtifa" % alt); break
        time.sleep(0.1)
    print("[TALON] Tırmanış bitti (irtifa ~%.0f m)" % alt)

    # AUTO — rotayı uç
    print("[TALON] AUTO — rotayı uçuyor (bitince RTL)...")
    stop_gcs_keepalive()
    _rc_birak(conn)
    time.sleep(0.5)
    set_mode(conn, PLANE_MODE_AUTO)

    t0 = time.time(); rota_bitti = False
    while time.time() - t0 < 400:
        hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if not hb or hb.get_srcSystem() != 2:
            continue
        if mavutil.mode_string_v10(hb) == 'RTL':
            rota_bitti = True
            print("[TALON] ✓ Rota bitti (RTL) — İNİŞE geçiliyor"); break
    if not rota_bitti:
        print("[TALON] Rota bitişi yakalanamadı — yine de inişe geçiliyor")

    # İniş (THR_MIN=0 ile alçalabilsin)
    start_gcs_keepalive()
    conn.mav.param_set_send(conn.target_system, conn.target_component,
                            b"THR_MIN", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.5)
    ev = _ev_noktasi(conn)
    if ev is None:
        print("[TALON] ⛔ Başlangıç noktası yok — iniş iptal."); stop_gcs_keepalive(); return

    # KAYMA TELAFİSİ: sabit kanat temas edip ~KAYMA_M ileri kayıyor (ölçüldü:
    # nişan=başlangıç iken 108 m ötede durdu). Nişanı uçağın GİDİŞ YÖNÜNDE
    # KAYMA_M kadar GERİYE alırsak, kayma bitince tam başlangıcın üstünde olur.
    # (inis'in kendi kayma_m'i paylaşımlı kodda 'nis = hedef' ile devre dışı;
    #  senaryoları etkilememek için telafiyi burada yapıyoruz.)
    import math
    # KAYMA TELAFİSİ (gözlemle kalibre): nişan=başlangıç iken uçak bu rotada
    # sürekli ~100 m GÜNEY + ~39 m BATI'ya kayıyor (rota deterministik). Nişanı
    # bunun TAM TERSİNE (kuzey-doğu) alınca temas başlangıca oturur. NED ofset.
    KUZEY_OFS, DOGU_OFS = 100.0, 39.0
    time.sleep(3.0)   # uçak eve dönsün
    aim = (ev[0] + KUZEY_OFS, ev[1] + DOGU_OFS)
    print("[TALON] İNİŞ — nişan başlangıcın ~%.0f m KD'si (kayma telafisi), THR_MIN=0..."
          % math.hypot(KUZEY_OFS, DOGU_OFS))
    inis(conn, hedef=aim)
    print("[TALON] ✓ İniş tamam.")
    stop_gcs_keepalive()


if __name__ == "__main__":
    main()
