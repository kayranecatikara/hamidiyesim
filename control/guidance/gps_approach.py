"""
=============================================================
  GPS YAKLAŞMA HATTI — eski sistemin (ana_kontrol.py) portu
=============================================================
Önceki sistemdeki GPS güdüm mekanizmasının (AvciKontrol.adim() [GPS-YAKLASMA]
yolu) bu simülasyona uyarlanmış hali. GÜDÜM YASASI AYNI; yalnız veri kaynakları
ve hareket komutları bu sisteme dönüştürüldü:

  ESKİ (oyun SDK)                          YENİ (avci_sim / ArduPilot)
  ─────────────────────────────            ─────────────────────────────
  birim: cm, z-YUKARI                      metre, NED (z-AŞAĞI)
  get_target_location() bozuk GNSS         get_plane() → _noisy_plane_telem
    + GNSSFiltre (2sn lead)                  + EMA pozisyon + sonlu-fark hız
  get_drone_location() (cm)                get_iris() → LOCAL_POSITION_NED (m)
  set_control_surfaces(thr,pitch,          SET_POSITION_TARGET_LOCAL_NED
    roll,yaw) @50Hz + rate-limit             hız+yaw @20Hz + ivme sınırı
  dikey iç döngü (KV_Z+integral→thr)       ArduPilot hız kontrolcüsü (vz komutu)
  world_to_body + PITCH/ROLL komutu        dünya-NED hız vektörü komutu

KORUNAN MEKANİKLER (eski koddaki şekliyle):
  - KUYRUK İSTASYONU (standoff): komut istasyonu hedefin HIZ YÖNÜNÜN
    APPROACH_STANDOFF gerisinde → drone yandan yetişse bile arkaya süzülür,
    hedef hep önde/kadrajda (GPS fazının amacı GÖRSEL TEMAS: YOLO tespiti).
  - LEAD → FEEDFORWARD: eski ayrık 0.5sn nişan lead'inin işini hız komutundaki
    hedef-hızı feedforward'u görür (sürekli lead; standoff gerçek hedefe göre).
  - FRENLEME (göreli): tavan = min(V_CAP_FAR, hedef_hızı + kapanma_payı(d));
    kapanma payı BRAKE_DIST altında V_CLOSE_FAR→V_CLOSE_NEAR iner. Hedef hızı
    ayrıca hız komutuna FEEDFORWARD eklenir (tutum→hız komut dönüşümünün gereği).
  - LOOK-UP GEOMETRİSİ: avcı hedefin ALTINDA uçar (menzil-ölçekli ofset,
    LOS yükseliş açısı ≥ LOOKUP_ELEV_DEG) → hedef GÖKYÜZÜ önünde siluet,
    YOLO tespiti kopmaz. Kamera 25° yukarı tilt'li → kadraj da doğru oturur.
    (NOT: gps_chase.py v2 hedefin ÜSTÜNDE uçuyordu; eski sistemin kanıtlanmış
    geometrisi ALTTAN bakıştır — dataset de bu geometriyle toplandı.)
  - ALÇALMA ÖNCELİĞİ: ref irtifanın üstündeysek yatay kovalama kısılır
    (ileri-uçuş taşıması alçalmayı engellemesin).
  - DİKEY KADEMELİ HIZ: ez → hedef vz (VZ_MAX tavanlı, trapez) — eski cascade'in
    dış döngüsü; iç döngü (throttle) ArduPilot'a devredildi.
  - YAW: burun daima HEDEFE (standoff noktasına değil) döner, deadband'li.
  - None YÖNETİMİ: telemetri donuk/değişmemişse HOLD_S boyunca son kestirimle
    devam (lead'li), sonrası DROPOUT → loiter (hover). frozen bayrağı okunur.
  - HANDOFF HİSTEREZİSİ: d_h < HANDOFF_RANGE → KILIT (görsel faz devralmaya
    hazır); d_h > HANDOFF_EXIT → geri ARAMA. Faz 4 supervisor bu durumu
    görsel kilit sayacıyla birleştirecek (bkz. supervisor.py).

Arayüz: run_gps_approach(conn, get_plane, get_iris, stop_event)
  get_plane() -> {x,y,z,yaw,frozen}  (m, NED; GPS-gürültülü hedef telemetrisi)
  get_iris()  -> {x,y,z}             (m, NED; kendi konum)
=============================================================
"""

import math
import time

from control.guidance.common import (
    clamp, normalize_angle, limit_acceleration, send_velocity,
)


# ══════════════════════════════════════════════════════════
#  CFG — eski Cfg ile aynı isimler, metreye çevrilmiş değerler
# ══════════════════════════════════════════════════════════
class Cfg:
    LOOP_HZ = 20.0                # sistem geleneği (eski 50 Hz; setpoint için 20 yeter)

    # --- STANDOFF / İSTASYON (anti-overshoot + kadraj) ---
    # Eski koddaki ayrık APPROACH_LEAD_S (0.5s nişan lead'i) hız alanında YOK:
    # hedef hızı feedforward'u sürekli lead sağlar; standoff GERÇEK hedefe göre.
    # 10m = eski sistemin KANITLANMIŞ EFEKTİF takip mesafesi (komut 5m + pursuit
    # lag ≈ 10m notu). Kadraj geometrisi: 10m geriden + 5m alttan → hedefe bakış
    # açısı atan(5/10)=26.6° ≈ kamera tilt 25° → hedef kadraj MERKEZİNDE; kutu
    # ~20px (200px·m/d) → YOLO tespiti rahat (model min 10px ile eğitildi).
    APPROACH_STANDOFF = 10.0      # m; istasyon hedefin bu kadar gerisinde
    TRACK_MIN_SPD     = 3.0       # m/s; hedef bundan hızlıysa istasyon HIZ YÖNÜNÜN
                                  # gerisinde (kuyruk takibi); yavaşsa LOS gerisi

    # --- LOOK-UP GEOMETRİSİ (alttan bakış; gökyüzü silueti) ---
    LOOKUP_ELEV_DEG     = 6.0     # LOS yükseliş açısı setpoint'i (0 = kapalı)
    APPROACH_ALT_OFFSET = 5.0     # m; hedefin altında kalınacak ASGARİ dikey ofset
    LOOKUP_MIN_ALT      = 8.0     # m; alçalma taban irtifası (yere çakılma koruması)

    # --- YAKLAŞMA HIZI PROFİLİ / FRENLEME ---
    # Tavan HEDEF HIZINA GÖRELİDİR: vcap = min(V_CAP_FAR, |v_hedef| + kapanma_payı(d)).
    # Eski mutlak tavan (5 m/s yakında) hedef 15 m/s uçarken ~50m'de dengeye
    # kilitleniyordu (kapanma hızı 0). Fren NİYETİ korunur: yakında kapanma payı
    # küçülür → overshoot yok; ama tavan asla hedefin kendi hızının altına inmez.
    V_CAP_FAR    = 19.0           # m/s; MUTLAK tavan (ANGLE_MAX=55° ile ~19.5 ölçüldü)
    V_CLOSE_FAR  = 14.0           # m/s; uzakta izin verilen KAPANMA hızı (hedef hızı üstü)
    V_CLOSE_NEAR = 2.5            # m/s; standoff yakınında kapanma hızı
    BRAKE_DIST   = 70.0           # m; bu mesafe altında kapanma payı kademeli düşer

    # --- PD KAZANÇLARI (hata m → hız m/s) ---
    # Eski KP_H=0.00025/cm tutum-komut alanındaydı (30 m hatada tam yetki);
    # hız alanı karşılığı: 0.8 1/s → ~24 m hatada tavana doyar. KD oranı (2.4s)
    # tutum salınımını söndürmek içindi; hız döngüsünü ArduPilot söndürür → küçük.
    KP_H = 0.8                    # yatay konum hatası → hız
    KD_H = 0.25                   # yatay türev (EMA'lı) → sönümleme
    DERIV_EMA = 0.2               # türev EMA katsayısı (eskiyle aynı)

    # --- DİKEY (eski cascade'in DIŞ döngüsü; iç döngü ArduPilot'ta) ---
    KP_Z_POS = 1.5                # 1/s; ez → hedef dikey hız (birimden bağımsız, aynı)
    VZ_MAX   = 3.5                # m/s; tırmanma/alçalma hız tavanı (eski 350 cm/s)

    # --- ALÇALMA ÖNCELİĞİ (dikey-yatay ayrıştırma) ---
    ALC_ONCELIK_M   = 8.0         # m; ref'in bu kadar üstünde yatay kovalama %15'e iner
    ALC_ONCELIK_MIN = 0.15

    # --- YAW ---
    YAW_DEADBAND = math.radians(3.0)
    YAW_RATE_MAX = math.radians(180.0)   # komut yaw dönüş hızı (10m istasyonda
                                         # yanlamasına geçişte kadraj kaçmasın)

    # --- DEADBAND / SINIRLAR ---
    POS_DEADBAND = 1.5            # m; çok yakında yatay jitter önle (eski 150 cm)
    MAX_ACCEL    = 10.0           # m/s²; komut hızı değişim sınırı (eski MAX_DELTA analogu)

    # --- None YÖNETİMİ ---
    HOLD_S = 6.0                  # s; donuk telemetride son kestirimle devam; ötesi loiter

    # --- HANDOFF (histerezisli) ---
    HANDOFF_RANGE = 40.0          # m; altında KILIT (görsel faz devralmaya hazır)
    HANDOFF_EXIT  = 50.0          # m; üstüne çıkınca handoff iptal

    # --- HEDEF TELEMETRİ FİLTRESİ (eski GNSSFiltre'nin yerine) ---
    POS_EMA = 0.4                 # gürültülü hedef pozisyonu EMA
    VEL_EMA = 0.3                 # sonlu-fark hedef hızı EMA


def closing_allow(d_horiz):
    """Mesafeye göre izin verilen KAPANMA hızı (m/s) — eski FRENLEME profili,
    hedef hızının ÜSTÜNE eklenen pay olarak. Yakında küçülür → yumuşak kapanış."""
    if d_horiz >= Cfg.BRAKE_DIST:
        return Cfg.V_CLOSE_FAR
    t = d_horiz / Cfg.BRAKE_DIST
    return Cfg.V_CLOSE_NEAR + (Cfg.V_CLOSE_FAR - Cfg.V_CLOSE_NEAR) * t


# Telemetri/arayüz için son durum (gcs_server okuyabilir; salt gözlem)
status = {
    "durum": "WARMUP", "d_h": None, "handoff": False,
    "fresh": False, "none_count": 0, "vcap": None,
}


def run_gps_approach(conn, get_plane, get_iris, stop_event):
    """Eski adim() döngüsünün portu: hedefe standoff'lu, frenli, alttan yaklaşma."""
    loop_period = 1.0 / Cfg.LOOP_HZ

    # hedef kestirimi (eski GNSSFiltre durumunun karşılığı)
    est_x = est_y = est_z = None          # EMA'lı hedef pozisyonu (m, NED)
    vel_x = vel_y = vel_z = 0.0           # hedef hızı (m/s; sonlu-fark + EMA)
    last_raw = None                       # tazelik tespiti (paket değişti mi?)
    t_last_fresh = None                   # son TAZE paketin zamanı (hız sonlu-farkı için)
    none_count = 0

    # PD durumu
    de = [0.0, 0.0, 0.0]                  # EMA'lı hata türevi
    e_prev = None
    t_prev_deriv = None

    # komut durumu
    vx_prev = vy_prev = vz_prev = 0.0
    cmd_yaw = None
    handoff = False
    handoff_announced = False
    prev_time = None
    loop_count = 0

    print("=" * 60)
    print("[GPS-YAKLASMA] Eski sistem güdüm yasası (ana_kontrol portu) aktif")
    print(f"[GPS-YAKLASMA] standoff={Cfg.APPROACH_STANDOFF}m (FF=sürekli lead) "
          f"kapanma_payı={Cfg.V_CLOSE_FAR}->{Cfg.V_CLOSE_NEAR}m/s@<{Cfg.BRAKE_DIST:.0f}m "
          f"(tavan=hedef_hızı+pay, mutlak {Cfg.V_CAP_FAR})")
    print(f"[GPS-YAKLASMA] look-up: hedefin >= {Cfg.APPROACH_ALT_OFFSET}m ALTINDA "
          f"(elev>={Cfg.LOOKUP_ELEV_DEG}°, taban {Cfg.LOOKUP_MIN_ALT}m)")
    print("=" * 60)

    while not stop_event.is_set():
        now = time.monotonic()
        dt = (now - prev_time) if prev_time is not None else loop_period
        dt = clamp(dt, 0.001, 0.2)
        prev_time = now

        iris = get_iris()
        ix, iy, iz = iris["x"], iris["y"], iris["z"]
        plane = get_plane()

        # ── 1) TAZELİK + FİLTRE (eski _hedef_temizle + _fresh karşılığı) ──
        raw = (plane["x"], plane["y"], plane["z"])
        frozen = bool(plane.get("frozen", False))
        fresh = (not frozen) and (raw != last_raw)
        if fresh:
            last_raw = raw
            none_count = 0
            if est_x is None:                       # ilk paket: filtre tohumla
                est_x, est_y, est_z = raw
            else:
                a = Cfg.POS_EMA
                nx = a * raw[0] + (1 - a) * est_x
                ny = a * raw[1] + (1 - a) * est_y
                nz = a * raw[2] + (1 - a) * est_z
                # Sonlu-fark hedef hızı: paketler döngüden SEYREK gelir (örn. 5Hz
                # paket / 20Hz döngü) → bölen, son taze paketten geçen GERÇEK süre.
                if t_last_fresh is not None:
                    fdt = now - t_last_fresh
                    if 1e-3 < fdt < 2.0:             # çok bayat aralıktan hız türetme
                        b = Cfg.VEL_EMA
                        vel_x = b * ((nx - est_x) / fdt) + (1 - b) * vel_x
                        vel_y = b * ((ny - est_y) / fdt) + (1 - b) * vel_y
                        vel_z = b * ((nz - est_z) / fdt) + (1 - b) * vel_z
                est_x, est_y, est_z = nx, ny, nz
            t_last_fresh = now
        else:
            none_count += 1

        status.update(fresh=fresh, none_count=none_count)

        # ── 2) WARMUP / DROPOUT (eski None yönetimi) ──
        if est_x is None:                            # henüz hiç kestirim yok
            _loiter(conn, cmd_yaw)
            status.update(durum="WARMUP", d_h=None)
            loop_count += 1
            _sleep(now, loop_period)
            continue
        if none_count * loop_period > Cfg.HOLD_S:    # uzun kesinti → loiter
            _loiter(conn, cmd_yaw)
            vx_prev = vy_prev = vz_prev = 0.0
            status.update(durum="DROPOUT", d_h=None)
            if loop_count % int(Cfg.LOOP_HZ * 3) == 0:
                print(f"[GPS-YAKLASMA] DROPOUT — hedef telemetri {none_count * loop_period:.1f}s "
                      f"donuk, loiter.")
            loop_count += 1
            _sleep(now, loop_period)
            continue
        # HOLD penceresi içinde: son kestirim + hedef hızıyla devam (lead taşır)

        # ── 3) HEDEFE HATA (yaw/handoff/standoff hepsi GERÇEK hedefe göre) ──
        # Eski koddaki ayrık lead noktası (hedef + 0.5s×hız) hız-komut alanında
        # KULLANILMAZ: intercept'i hız feedforward'u (adım 8) sürekli sağlar.
        # Lead noktasından standoff ölçmek, hızlı hedefte istasyonu hedefin
        # ÖNÜNE düşürüyordu (lead 7.5m - standoff 5m = +2.5m); eski sistemde
        # bunu pursuit lag maskeliyordu, hız komutunda maskeleyemez.
        ex = est_x - ix
        ey = est_y - iy
        d_h = math.hypot(ex, ey)

        # ── 4) DİKEY REFERANS: look-up geometrisi (NED; irtifa = -z) ──
        alt_tgt  = -est_z                             # hedef irtifası (m, yukarı+)
        alt_iris = -iz
        dh_off = (math.tan(math.radians(Cfg.LOOKUP_ELEV_DEG)) * d_h
                  if Cfg.LOOKUP_ELEV_DEG > 0.0 else 0.0)
        alt_off = max(Cfg.APPROACH_ALT_OFFSET, dh_off)   # açı >= eps garanti + kadraj tabanı
        alt_ref = max(alt_tgt - alt_off, Cfg.LOOKUP_MIN_ALT)
        ez = alt_ref - alt_iris                       # yukarı-pozitif dikey hata (eskiyle aynı)

        # ── 5) İSTASYON NOKTASI: hedefin İZİNİN GERİSİNDE (kuyruk takibi) ──
        # LOS gerisi değil, hedefin HIZ YÖNÜNÜN gerisi: drone hedefe hangi
        # yönden yetişirse yetişsin (yandan dahil) arkaya süzülür → kuyruktan
        # takip, LOS oranı ~0, kadraj kararlı — tespit için en iyi geometri.
        # (Yan takip sorunu: LOS-standoff mesafeyi korur ama YÖNÜ korumaz;
        # drone yandan yetişince yanda kalıyordu.) Hedef yavaşsa LOS gerisi.
        tgt_spd = math.hypot(vel_x, vel_y)
        if tgt_spd >= Cfg.TRACK_MIN_SPD:
            st_x = est_x - (vel_x / tgt_spd) * Cfg.APPROACH_STANDOFF
            st_y = est_y - (vel_y / tgt_spd) * Cfg.APPROACH_STANDOFF
        elif d_h > 1e-6:
            st_x = est_x - (ex / d_h) * Cfg.APPROACH_STANDOFF
            st_y = est_y - (ey / d_h) * Cfg.APPROACH_STANDOFF
        else:
            st_x, st_y = ix, iy
        ex_cmd = st_x - ix                            # istasyona hata vektörü
        ey_cmd = st_y - iy

        # ── 6) HANDOFF HİSTEREZİSİ → durum ──
        if not handoff and d_h < Cfg.HANDOFF_RANGE:
            handoff = True
        elif handoff and d_h > Cfg.HANDOFF_EXIT:
            handoff = False
            handoff_announced = False
        durum = "KILIT" if handoff else "ARAMA"
        if handoff and not handoff_announced:
            print(f"[GPS-YAKLASMA] HANDOFF: tespit menzilinde (d_h={d_h:.1f}m < "
                  f"{Cfg.HANDOFF_RANGE:.0f}m) — görsel faz devralabilir.")
            handoff_announced = True

        # ── 7) EMA TÜREV (eski _derivative) ──
        e_now = (ex_cmd, ey_cmd, ez)
        if e_prev is not None and t_prev_deriv is not None:
            ddt = now - t_prev_deriv
            if ddt > 1e-3:
                a = Cfg.DERIV_EMA
                for i in range(3):
                    raw_d = (e_now[i] - e_prev[i]) / ddt
                    de[i] = (1 - a) * de[i] + a * raw_d
        e_prev, t_prev_deriv = e_now, now

        # ── 8) YATAY HIZ KOMUTU: hedef hızı FEEDFORWARD + PD → göreli tavan →
        #       alçalma önceliği → deadband.
        #       FF şart: eski tutum komutları hızı ENTEGRE ediyordu; hız komutunda
        #       hedef hızı açıkça eklenmezse drone hedefle ancak hata birikimiyle
        #       eş hızlanır ve mutlak tavan onu ~50m'de kilitliyordu.
        vx = vel_x + Cfg.KP_H * ex_cmd + Cfg.KD_H * de[0]
        vy = vel_y + Cfg.KP_H * ey_cmd + Cfg.KD_H * de[1]

        vcap = min(Cfg.V_CAP_FAR, tgt_spd + closing_allow(d_h))   # GÖRELİ fren
        vmag = math.hypot(vx, vy)
        if vmag > vcap and vmag > 1e-6:
            s = vcap / vmag
            vx *= s
            vy *= s

        if ez < 0.0:                                  # ref irtifanın ÜSTÜNDEYİZ → alçalma önceliği
            alc = clamp(1.0 + ez / Cfg.ALC_ONCELIK_M, Cfg.ALC_ONCELIK_MIN, 1.0)
            vx *= alc
            vy *= alc

        if math.hypot(ex_cmd, ey_cmd) < Cfg.POS_DEADBAND:   # istasyondayız: FF ile süz
            vx, vy = vel_x, vel_y

        # ── 9) DİKEY HIZ: dış döngü aynen; iç döngü ArduPilot'un ──
        vz_up  = clamp(Cfg.KP_Z_POS * ez, -Cfg.VZ_MAX, Cfg.VZ_MAX)
        vz_ned = -vz_up                               # NED: aşağı+

        # ── 10) YAW: burun daima HEDEFE (deadband + dönüş hızı sınırı) ──
        bearing = math.atan2(ey, ex)
        if cmd_yaw is None:
            cmd_yaw = bearing
        yaw_err = normalize_angle(bearing - cmd_yaw)
        if abs(yaw_err) > Cfg.YAW_DEADBAND:
            step = clamp(yaw_err, -Cfg.YAW_RATE_MAX * dt, Cfg.YAW_RATE_MAX * dt)
            cmd_yaw = normalize_angle(cmd_yaw + step)

        # ── 11) İVME SINIRI (eski rate_limit karşılığı) + GÖNDER ──
        vx, vy, vz_ned = limit_acceleration(
            vx, vy, vz_ned, vx_prev, vy_prev, vz_prev, Cfg.MAX_ACCEL, dt)
        vx_prev, vy_prev, vz_prev = vx, vy, vz_ned
        send_velocity(conn, vx, vy, vz_ned, cmd_yaw)

        status.update(durum=durum, d_h=round(d_h, 1), handoff=handoff,
                      vcap=round(vcap, 1))

        loop_count += 1
        if loop_count % int(Cfg.LOOP_HZ * 3) == 0:
            print(f"[GPS-YAKLASMA] {durum} d_h={d_h:.1f}m ez={ez:+.1f}m "
                  f"v=({vx:+.1f},{vy:+.1f},{vz_ned:+.1f}) |v|={math.hypot(vx, vy):.1f}"
                  f"/{vcap:.1f}m/s hedef_v={tgt_spd:.1f} alt={alt_iris:.1f} "
                  f"ref={alt_ref:.1f} tgt={alt_tgt:.1f} yaw={math.degrees(cmd_yaw):.0f}° "
                  f"fresh={int(fresh)} hold={none_count * loop_period:.1f}s")

        _sleep(now, loop_period)

    send_velocity(conn, 0.0, 0.0, 0.0, cmd_yaw or 0.0)
    status.update(durum="DURDU")
    print("[GPS-YAKLASMA] Stop sinyali — döngü sonlandı.")


def _loiter(conn, cmd_yaw):
    """Eski _loiter: agresifliği kes, hover (hız 0 → GUIDED irtifayı korur)."""
    send_velocity(conn, 0.0, 0.0, 0.0, cmd_yaw or 0.0)


def _sleep(t_start, period):
    elapsed = time.monotonic() - t_start
    if elapsed < period:
        time.sleep(period - elapsed)
