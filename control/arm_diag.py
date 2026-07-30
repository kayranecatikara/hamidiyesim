#!/usr/bin/env python3
"""
arm_diag.py — ArduPilot ARM reddini teşhis eder (ArduCopter + ArduPlane).

Bir araç ARM olmuyorsa sebebi bulur:
  1. Araca bağlanır (heartbeat ile sysid doğrulaması)
  2. Arming ile ilgili parametreleri OKUR (ARMING_CHECK, GPS, EKF, RC...)
  3. SYS_STATUS'tan unhealthy sensör bitlerini çözer
  4. ARM dener ve STATUSTEXT'ten reddin TAM sebebini yazdırır
     (ArduPilot red sebebini "PreArm: ..." metni olarak yollar)

Varsayılan davranış SALT-OKUNUR teşhistir; hiçbir parametre değiştirilmez.
--gevset ile arming kontrolleri gevşetilir (SADECE SITL/test için).

Kullanım:
    python3 -m control.arm_diag                     # Talon (sysid 2, port 14542)
    python3 -m control.arm_diag --iris              # iris  (sysid 5, port 14541)
    python3 -m control.arm_diag --port 14550 --sysid 2
    python3 -m control.arm_diag --gevset            # arming kontrollerini gevşet

NOT: ArduPilot'ta force ARM magic 2989'dur (21196 force DISARM'dır — PX4
sürümünde yanlışlıkla 21196 kullanılıyordu).
"""
import argparse
import time

from pymavlink import mavutil

MY_SYS = 251                    # GCS kaynak sysid (SYSID_MYGCS ile çakışmasın)
FORCE_ARM_MAGIC = 2989          # ArduPilot force ARM (force DISARM = 21196)
PLANE_MODE_MANUAL = 0           # ArduPlane MANUAL (PX4'te 1'di)
COPTER_MODE_STABILIZE = 0       # ArduCopter STABILIZE

# Varsayılanlar: Talon (ArduPlane) — bkz. scripts/start_ardupilot_sitl.sh
DEFAULT_PLANE = {"port": 14542, "sysid": 2, "ad": "Talon (ArduPlane)"}
DEFAULT_IRIS = {"port": 14541, "sysid": 5, "ad": "iris (ArduCopter)"}

# ARM'ı etkileyen ArduPilot parametreleri (salt okunur teşhis)
OKUNACAK_PARAMLAR = [
    "ARMING_CHECK",      # bit maskesi — hangi ön kontroller açık
    "ARMING_REQUIRE",    # throttle arming zorunluluğu (plane)
    "ARMING_RUDDER",     # rudder ile arm izni
    "BRD_SAFETY_DEFLT",  # safety switch varsayılanı
    "AHRS_EKF_TYPE",     # aktif EKF (3 = EKF3)
    "EK3_ENABLE",
    "GPS_TYPE",
    "SIM_GPS_DISABLE",   # SITL'de GPS kapalı mı
    "RC_PROTOCOLS",
    "SYSID_MYGCS",       # RC override / komut kabul edilen GCS sysid
    "FS_GCS_ENABL",      # GCS failsafe
    "BATT_MONITOR",
]

# ARMING_CHECK bit maskesi (ArduPilot ortak)
ARMING_CHECK_BITLERI = {
    1 << 0: "Tümü (ALL)",
    1 << 1: "Barometre",
    1 << 2: "Pusula (Compass)",
    1 << 3: "GPS kilidi",
    1 << 4: "INS (gyro/accel)",
    1 << 5: "Parametreler",
    1 << 6: "RC kanalları",
    1 << 7: "Kart voltajı",
    1 << 8: "Batarya seviyesi",
    1 << 10: "Log kaydı",
    1 << 11: "Safety switch",
    1 << 12: "GPS yapılandırması",
    1 << 13: "Sistem (System)",
    1 << 14: "Görev (Mission)",
    1 << 15: "Rangefinder",
    1 << 16: "Kamera",
    1 << 17: "Yardımcı yetki (AuxAuth)",
    1 << 18: "Görüş konumu (VisOdom)",
    1 << 19: "FFT",
}

# SYS_STATUS sensör bit maskeleri (MAV_SYS_STATUS_SENSOR)
SENSOR_BITLERI = {
    0x00000001: "3D Gyro",
    0x00000002: "3D Accel",
    0x00000004: "3D Mag",
    0x00000008: "Mutlak basınç (baro)",
    0x00000010: "Fark basıncı (pitot)",
    0x00000020: "GPS",
    0x00000040: "Optik akış",
    0x00000080: "Görüş konumu",
    0x00000100: "Lazer konum",
    0x00000200: "Harici yer gerçeği",
    0x00000400: "Açısal hız kontrolü",
    0x00000800: "Attitude stabilizasyonu",
    0x00001000: "Yaw konumu",
    0x00002000: "Z/İrtifa kontrolü",
    0x00004000: "X/Y konum kontrolü",
    0x00008000: "Motor çıkışları / ESC",
    0x00010000: "RC alıcısı",
    0x00020000: "3D Gyro 2",
    0x00040000: "3D Accel 2",
    0x00080000: "3D Mag 2",
    0x00100000: "Yer istasyonu (GCS)",
    0x00200000: "Batarya",
    0x02000000: "Ön-arm kontrolü (PreArm)",
}

# SITL/test için arming kontrollerini gevşeten parametreler (--gevset)
GEVSETME_PARAMLARI = {
    "ARMING_CHECK": 0,        # tüm ön kontroller kapalı
    "ARMING_REQUIRE": 0,      # throttle arming zorunlu değil
    "FS_GCS_ENABL": 0,        # GCS failsafe kapalı
    "BRD_SAFETY_DEFLT": 0,    # safety switch devre dışı
}


# ---------------------------------------------------------------------------
def baglan(port, sysid, ad):
    print(f"[DIAG] Bağlanılıyor: {ad} — udpin:127.0.0.1:{port} (sysid={sysid})")
    conn = mavutil.mavlink_connection(
        f"udpin:127.0.0.1:{port}",
        source_system=MY_SYS,
    )
    deadline = time.time() + 15
    while time.time() < deadline:
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.get_srcSystem() == sysid:
            conn.target_system = sysid
            conn.target_component = msg.get_srcComponent()
            print(f"[DIAG] ✓ Heartbeat: sys={sysid} comp={conn.target_component}")
            return conn
    raise TimeoutError(
        f"{ad} bulunamadı (sysid={sysid}, port={port}). "
        f"SITL çalışıyor mu? Port haritası: scripts/start_ardupilot_sitl.sh"
    )


def gcs_heartbeat(conn):
    """GCS heartbeat — GCS failsafe'in tetiklenmesini önler."""
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )


def param_oku(conn, ad, timeout=3.0):
    bname = ad.encode()[:16].ljust(16, b'\x00')[:16]
    conn.mav.param_request_read_send(
        conn.target_system, conn.target_component, bname, -1,
    )
    t = time.time()
    while time.time() - t < timeout:
        m = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if m and m.param_id.strip('\x00') == ad:
            return m.param_value
    return None


def param_yaz(conn, ad, deger):
    """Parametreyi yazar ve GERİ OKUYARAK uygulandığını doğrular."""
    bname = ad.encode()[:16].ljust(16, b'\x00')[:16]
    conn.mav.param_set_send(
        conn.target_system, conn.target_component,
        bname, float(deger),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )
    t = time.time()
    while time.time() - t < 2.0:
        m = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if m and m.param_id.strip('\x00') == ad:
            uygulandi = int(m.param_value)
            isaret = "✓" if uygulandi == int(deger) else "✗"
            print(f"  {isaret} {ad:20s} = {uygulandi}  (hedef={deger})")
            return uygulandi
    print(f"  ✗ {ad:20s} → ACK yok (parametre yok olabilir)")
    return None


# ---------------------------------------------------------------------------
def parametreleri_raporla(conn):
    print("\n[DIAG] --- ARM'ı etkileyen parametreler ---")
    arming_check = None
    for ad in OKUNACAK_PARAMLAR:
        v = param_oku(conn, ad)
        gcs_heartbeat(conn)
        if v is None:
            print(f"  {ad:20s} = (bu araçta yok)")
        else:
            print(f"  {ad:20s} = {v:g}")
            if ad == "ARMING_CHECK":
                arming_check = int(v)

    if arming_check is not None:
        print("\n[DIAG] --- Açık olan ön kontroller (ARMING_CHECK) ---")
        if arming_check == 0:
            print("  (hiçbiri — tüm ön kontroller KAPALI)")
        elif arming_check & 1:
            print("  TÜMÜ açık (bit 0 = ALL)")
        else:
            for mask, etiket in ARMING_CHECK_BITLERI.items():
                if arming_check & mask:
                    print(f"  • {etiket}")


def sys_status_raporla(conn):
    print("\n[DIAG] --- SYS_STATUS (sensör sağlığı) ---")
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 2, 1,
    )
    deadline = time.time() + 3
    durum = None
    while time.time() < deadline:
        m = conn.recv_match(type='SYS_STATUS', blocking=True, timeout=0.5)
        if m:
            durum = m
            break

    if not durum:
        print("  SYS_STATUS alınamadı (telemetri akışı gelmiyor)")
        return

    mevcut = durum.onboard_control_sensors_present
    etkin = durum.onboard_control_sensors_enabled
    saglik = durum.onboard_control_sensors_health
    sorunlu = mevcut & etkin & ~saglik

    print(f"  Mevcut  : 0x{mevcut:08X}")
    print(f"  Etkin   : 0x{etkin:08X}")
    print(f"  Sağlıklı: 0x{saglik:08X}")
    print(f"  SORUNLU : 0x{sorunlu:08X}")
    if sorunlu:
        for mask, etiket in SENSOR_BITLERI.items():
            if sorunlu & mask:
                print(f"  ❌ SAĞLIKSIZ: {etiket}")
    else:
        print("  ✓ Sağlıksız sensör yok")


def arm_dene(conn, plane):
    """Güvenli bir moda geçip ARM dener; red sebebini STATUSTEXT'ten okur."""
    mod = PLANE_MODE_MANUAL if plane else COPTER_MODE_STABILIZE
    mod_ad = "MANUAL" if plane else "STABILIZE"
    print(f"\n[DIAG] --- {mod_ad} moduna geç + ARM dene ---")
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mod,
    )
    time.sleep(1.0)

    print(f"[DIAG] ARM gönderiliyor (force magic={FORCE_ARM_MAGIC})...")
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1.0,                      # 1 = arm
        float(FORCE_ARM_MAGIC),   # force
        0, 0, 0, 0, 0,
    )

    print("[DIAG] STATUSTEXT / COMMAND_ACK (5 saniye):")
    deadline = time.time() + 5
    red_sebepleri = []
    while time.time() < deadline:
        gcs_heartbeat(conn)
        msg = conn.recv_match(type=['STATUSTEXT', 'COMMAND_ACK'],
                              blocking=True, timeout=0.3)
        if not msg:
            continue
        if msg.get_type() == 'COMMAND_ACK':
            if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                sonuc = "✓ ARM KABUL" if msg.result == 0 else f"✗ ARM RED (result={msg.result})"
                print(f"  ACK: {sonuc}")
        else:
            metin = msg.text.strip()
            print(f"  STATUSTEXT [{msg.severity}]: {metin}")
            if "PreArm" in metin or "Arm" in metin:
                red_sebepleri.append(metin)

    print("\n[DIAG] --- ÖZET ---")
    if red_sebepleri:
        print("  ARM reddinin sebepleri:")
        for s in red_sebepleri:
            print(f"    → {s}")
    else:
        print("  Red sebebi bildiren mesaj gelmedi "
              "(ARM başarılı olmuş ya da araç zaten armlı olabilir).")


def kontrolleri_gevset(conn):
    print("\n[DIAG] --- Arming kontrolleri GEVŞETİLİYOR (sadece SITL/test) ---")
    for ad, deger in GEVSETME_PARAMLARI.items():
        param_yaz(conn, ad, deger)
        gcs_heartbeat(conn)
        time.sleep(0.1)


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="ArduPilot ARM teşhis aracı")
    ap.add_argument("--iris", action="store_true",
                    help="Talon yerine iris'i (ArduCopter) teşhis et")
    ap.add_argument("--port", type=int, default=None, help="UDP portu")
    ap.add_argument("--sysid", type=int, default=None, help="Araç sysid'si")
    ap.add_argument("--gevset", action="store_true",
                    help="Arming kontrollerini gevşet (SADECE SITL/test)")
    args = ap.parse_args()

    varsayilan = DEFAULT_IRIS if args.iris else DEFAULT_PLANE
    port = args.port if args.port is not None else varsayilan["port"]
    sysid = args.sysid if args.sysid is not None else varsayilan["sysid"]
    plane = not args.iris

    conn = baglan(port, sysid, varsayilan["ad"])

    print("\n[DIAG] GCS heartbeat gönderiliyor (3 sn)...")
    for _ in range(15):
        gcs_heartbeat(conn)
        time.sleep(0.2)

    parametreleri_raporla(conn)
    sys_status_raporla(conn)

    if args.gevset:
        kontrolleri_gevset(conn)

    arm_dene(conn, plane)
    print("\n[DIAG] Teşhis tamamlandı.")


if __name__ == "__main__":
    main()
