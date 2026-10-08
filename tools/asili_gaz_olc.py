#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
ASILI GAZ ÖLÇER — aracın itki/ağırlık oranını SAYIYLA saptar
================================================================================
NİYE VAR (2026-09-06, VAKKAS geçişi):
  `sim/ardupilot_params/avci_copter.parm` yorumlarında itki/ağırlık için iki
  ayrı sayı yazıyor: bir yerde 7.08, başka yerde 6.4. İkisi de TAHMİN olabilir.
  VAKKAS'a (4 kg / 8 kgf → T/W = 2.0) geçerken hem ESKİ hem YENİ değeri
  ölçmeden "değiştirdim" diyemem (CLAUDE.md §5.1 mekanizma kapısı).

YÖNTEM — nereden geliyor (⚠ İLK SÜRÜMDEKİ FORMÜL YANLIŞTI, düzeltildi):

  ⛔ YANLIŞ OLAN: "VFR_HUD.throttle'ı al, T/W = 1/gaz²". İki hatası vardı:
     (a) VFR_HUD.throttle = motors->get_throttle() (GCS_MAVLink_Copter.cpp:251)
         ve bu ArduCopter'ın İTKİ İSTEĞİdir, motor devri değil. Aradan
         MOT_THST_EXPO (0.65), MOT_SPIN_MIN (0.15), MOT_SPIN_MAX (0.95)
         geçer. Yani gaz yüzdesi devirle DOĞRU ORANTILI DEĞİLDİR.
     (b) Bu yüzden ilk koşu "gaz %15 → T/W 44" dedi; oysa parametre
         dosyasının kendi yorumu 6.4 diyordu. 7 kat sapma.

  ✔ DOĞRU OLAN — motora giden PWM'i doğrudan ölç, modeli atla:
     ArduPilotPlugin PWM'i şu şekilde normalize eder (model.sdf <control>):
         u = (PWM - servo_min) / (servo_max - servo_min)      [servo 1100..1900]
     ve rotor açısal hızını doğrudan bundan üretir:
         ω = multiplier · u                                   [multiplier = 838]
     Gazebo LiftDrag eklentisinin ürettiği itki, dinamik basınç q = ½ρV²
     üzerinden hıza (dolayısıyla ω'ya) KARESEL bağlıdır:
         T = k · ω² = k · multiplier² · u²

     Asılı uçuşta toplam itki ağırlığa eşittir (T_asılı = m·g). Bu tek
     ölçüm k'yı kalibre eder:
         k · multiplier² = m·g / u_asılı²
     Motorun u = 1 (tam devir) durumundaki itkisi:

         T_max = m · g / u_asılı²

     Yani TEK ölçülmesi gereken şey ASILI PWM'dir. MOT_THST_EXPO,
     MOT_SPIN_MIN/MAX, ArduPilot'un iç itki modeli — hiçbiri girmiyor.

  ⚠ VARSAYIM: itki ∝ ω². LiftDrag'in q = ½ρV² terimi için tam doğru;
    gerçek pervanede indirgenmiş akış (inflow) bunu bir miktar bozar.
  ⚠ ArduPilot MOT_SPIN_MAX = 0.95 ile tavanı kendi kısar; kullanılabilir
    itki T_max'in yaklaşık %90'ıdır (0.95² = 0.9025). Rapor ikisini de verir.

  Aracın GERÇEKTEN asılı olduğunu doğrulamak için |vz| < 0.3 m/s kareler
  alınır — tırmanırken okunan PWM asılı PWM DEĞİLDİR.

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


def par_oku(c, ad, zaman_asimi=3.0):
    c.mav.param_request_read_send(c.target_system, c.target_component,
                                  ad.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < zaman_asimi:
        m = c.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if m and m.param_id.strip("\x00") == ad:
            return m.param_value
    return None


def par_yaz(c, ad, deger):
    c.mav.param_set_send(c.target_system, c.target_component, ad.encode(),
                         float(deger), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        m = c.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if m and m.param_id.strip("\x00") == ad:
            return m.param_value
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--adres", default="udpin:127.0.0.1:14541")
    ap.add_argument("--irtifa", type=float, default=50.0)
    ap.add_argument("--sure", type=float, default=25.0,
                    help="asılı kalıp gaz örneklenecek saniye")
    ap.add_argument("--etiket", default="olcum")
    ap.add_argument("--kutle", type=float, required=True,
                    help="modelin TOPLAM kütlesi (kg) — SDF'ten toplanır")
    ap.add_argument("--servo-min", type=float, default=1100.0,
                    help="model.sdf <control><servo_min>")
    ap.add_argument("--servo-max", type=float, default=1900.0,
                    help="model.sdf <control><servo_max>")
    ap.add_argument("--spin-max", type=float, default=0.95,
                    help="MOT_SPIN_MAX — ArduPilot'un kendi tavan kısıtı")
    a = ap.parse_args()

    print("bağlanılıyor: %s" % a.adres)
    c = mavutil.mavlink_connection(a.adres)
    c.wait_heartbeat()
    print("  heartbeat: sys=%d comp=%d" % (c.target_system, c.target_component))
    c.mav.request_data_stream_send(c.target_system, c.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    time.sleep(1.0)

    # ── Kaç param OKUNUYOR: aracın kendi beyanı, benim varsayımım değil
    print("\naracın beyan ettiği ayarlar:")
    for ad in ("ANGLE_MAX", "MOT_THST_HOVER", "MOT_HOVER_LEARN",
               "WPNAV_ACCEL", "PSC_JERK_XY"):
        print("  %-16s = %s" % (ad, par_oku(c, ad)))

    # Öğrenme KAPALI olsun: yoksa ArduPilot uçuş sırasında MOT_THST_HOVER'ı
    # güncelleyip bir sonraki koşunun başlangıç şartını değiştirir (kirli deney).
    par_yaz(c, "MOT_HOVER_LEARN", 0)

    print("\nEKF konum kestirimi bekleniyor…")
    t0 = time.time()
    hazir = False
    while time.time() - t0 < 120:
        m = c.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=2)
        if m and (m.flags & 0x08) and (m.flags & 0x10):
            hazir = True
            print("  hazır (%.0f s)" % (time.time() - t0))
            break
    if not hazir:
        print("  ⛔ EKF hazır olmadı — koşu GEÇERSİZ")
        return 1
    time.sleep(2)

    # ── ARM (tekrarlı: araç yerde 10 s sonra kendini disarm eder)
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
    print("\nARM: %s" % ("evet" if armli else "HAYIR"))
    if not armli:
        for _ in range(8):
            m = c.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
            if m:
                print("   %s" % m.text.strip())
        return 1

    # ── KALKIŞ
    c.mav.command_long_send(c.target_system, c.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                            0, 0, 0, 0, 0, 0, 0, a.irtifa)
    print("kalkış: %.0f m hedefleniyor…" % a.irtifa)
    alt = 0.0
    t0 = time.time()
    while time.time() - t0 < 120:
        m = c.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if m:
            alt = m.relative_alt / 1000.0
            if alt >= a.irtifa - 2.0:
                break
    print("  irtifa: %.1f m  (%.0f s)" % (alt, time.time() - t0))

    print("\noturması için 8 s bekleniyor…")
    time.sleep(8)

    # ── ÖLÇÜM: yalnız GERÇEKTEN asılı olan kareler
    #
    # SERVO_OUTPUT_RAW = motorlara giden HAM PWM. ArduPilotPlugin bunu okur.
    # 4 motorun ORTALAMASI alınır: tek motor duruş düzeltmesi yüzünden
    # diğerlerinden sapar, toplam itkiyi ortalama temsil eder.
    print("\n%.0f s boyunca motor PWM'i örnekleniyor (yalnız |vz| < 0.3 m/s)…"
          % a.sure)
    pwmler, gazlar, atilan = [], [], 0
    vz_son, alt_son = 0.0, alt
    t0 = time.time()
    while time.time() - t0 < a.sure:
        m = c.recv_match(type=["SERVO_OUTPUT_RAW", "VFR_HUD",
                               "GLOBAL_POSITION_INT"], blocking=True, timeout=2)
        if m is None:
            continue
        t = m.get_type()
        if t == "GLOBAL_POSITION_INT":
            vz_son = m.vz / 100.0          # cm/s → m/s
            alt_son = m.relative_alt / 1000.0
        elif t == "VFR_HUD":
            if abs(vz_son) < 0.3:
                gazlar.append(m.throttle)
        else:
            if abs(vz_son) < 0.3:
                pwmler.append((m.servo1_raw + m.servo2_raw
                               + m.servo3_raw + m.servo4_raw) / 4.0)
            else:
                atilan += 1

    if len(pwmler) < 10:
        print("  ⛔ yeterli asılı kare yok (%d) — koşu GEÇERSİZ" % len(pwmler))
        return 1

    pwm = statistics.median(pwmler)
    u = (pwm - a.servo_min) / (a.servo_max - a.servo_min)
    if u <= 0.01:
        print("  ⛔ PWM %.0f servo_min'in altında — ölçüm anlamsız" % pwm)
        return 1

    W = a.kutle * 9.81                 # N
    t_max = W / (u * u)                # N   (u = 1, yani tam devir)
    t_max_kgf = t_max / 9.81
    tw = t_max / W                     # = 1/u²
    tw_kullanilabilir = tw * (a.spin_max ** 2)

    print("\n" + "=" * 72)
    print("  SONUÇ — %s" % a.etiket)
    print("=" * 72)
    print("  örnek                  : %d asılı kare (%d atıldı: tırmanış/iniş)"
          % (len(pwmler), atilan))
    print("  irtifa (son)           : %.1f m" % alt_son)
    print("  varsayılan kütle       : %.2f kg  (ağırlık %.1f N)" % (a.kutle, W))
    print()
    print("  MOTOR PWM (medyan, 4 motor ort.) : %.0f µs" % pwm)
    print("  → normalize devir  u             : %.4f" % u)
    print("  → MAKS İTKİ  T_max = m·g/u²      : %.1f N  =  %.2f kgf" % (t_max, t_max_kgf))
    print("  → İTKİ/AĞIRLIK  (u=1)            : %.2f" % tw)
    print("  → İTKİ/AĞIRLIK  (MOT_SPIN_MAX %.2f ile kullanılabilir) : %.2f"
          % (a.spin_max, tw_kullanilabilir))
    if gazlar:
        print("  (bilgi) VFR_HUD gaz medyan       : %%%.1f  ← itki İSTEĞİ, devir DEĞİL"
              % statistics.median(gazlar))
    print()
    print("  bu araç ne kadar yatabilir (irtifa koruyarak):")
    for rez, ad in ((1.00, "teorik tavan (duruş rezervi YOK)"),
                    (0.80, "%20 duruş rezervi ile pratik tavan")):
        oran = tw_kullanilabilir * rez
        if oran <= 1.0:
            print("    %-38s : UÇAMAZ (itki yetmiyor)" % ad)
        else:
            th = math.degrees(math.acos(1.0 / oran))
            print("    %-38s : %5.1f°   (yatay ivme %5.1f m/s²)"
                  % (ad, th, 9.81 * math.tan(math.radians(th))))
    print()
    print("  ⛔ SİM ölçümüdür; güdüm kabul kararı vermez (CLAUDE.md §2).")

    # ── indir
    c.mav.set_mode_send(c.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6)  # RTL
    return 0


if __name__ == "__main__":
    sys.exit(main())
