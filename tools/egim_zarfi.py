#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
EĞİM ZARFI DENEYİ — "bu araç kaç dereceye kadar yatarken irtifasını tutar?"
================================================================================
NİYE VAR (2026-09-06, VAKKAS geçişi):
  Eğim tavanı 70° -> 45° indirildi. Bu bir HESAP kararıydı:
      irtifayı korumak için gereken itki = W / cos θ
      70° -> 2.92x  ·  45° -> 1.41x  ·  VAKKAS'ta mevcut 1.80x
  Hesap doğru olabilir ama CLAUDE.md §2: kabul kararını TAZE UÇUŞ verir.
  Bu deney, kararı uçarak sınar.

  Aynı zamanda 2026-09-02'de GERÇEK dronun düştüğü mekanizmanın simde
  yeniden üretilip üretilemediğini gösterir: o uçuşta bileşke eğim 75.5°'ye
  çıkmış, dikey itki oranı 0.25'e inmiş, gereken gaz %124 olmuştu.

YÖNTEM:
  ArduCopter'da SET_ATTITUDE_TARGET mesajının `thrust` alanı HAM İTKİ DEĞİL,
  TIRMANMA HIZI olarak yorumlanır (GCS_Mavlink.cpp): 0.5 = "irtifanı koru".
  Yani araca "şu kadar yat AMA irtifanı koru" diyoruz ve irtifa denetleyicisi
  elinden geleni yapıyor. Ölçtüğümüz şey: yapabiliyor mu?

  Her kol: 120 m'ye çık -> dur -> θ kadar yat, 3 s -> irtifa kaybını ölç.

KOLLAR:
  30°  KONTROL   gereken 1.15x -> rahat tutmalı
  45°  YENİ TAVAN gereken 1.41x -> tutmalı  (ANGLE_MAX 4500'ün gerekçesi)
  60°  SINIR     gereken 2.00x -> pay yok, sarkmalı
  70°  ESKİ TAVAN gereken 2.92x -> DÜŞMELİ (değişikliğin gerekçesi)

⛔ SİM ölçümüdür; güdüm kabul kararı vermez (CLAUDE.md §2).
================================================================================
"""
import argparse
import math
import sys
import time

from pymavlink import mavutil

GUIDED = 4
# type_mask: gövde açısal hız oranlarını YOKSAY, açı + thrust kullan
TM_ORAN_YOKSAY = 0b00000111


def q_kur(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


# ⛔ TEK OKUMA NOKTASI: ilk sürümde SERVO_OUTPUT_RAW ayrı bir non-blocking
#   recv_match ile alınmaya çalışılıyordu; durum() mesaj kuyruğunu tükettiği
#   için hemen hiç yakalanmadı ve "motor u = 0.000" basıldı (ölçüm kayboldu).
#   Bütün mesaj tipleri BURADA toplanır.
_SON = {"alt": None, "rol": 0.0, "pit": 0.0, "yaw": 0.0, "vz": 0.0,
        "u": 0.0, "x": None, "y": None, "vxy": 0.0}


def durum(c, sure=0.25):
    t0 = time.time()
    while time.time() - t0 < sure:
        m = c.recv_match(type=["GLOBAL_POSITION_INT", "ATTITUDE",
                               "SERVO_OUTPUT_RAW", "LOCAL_POSITION_NED"],
                         blocking=True, timeout=0.5)
        if m is None:
            break
        t = m.get_type()
        if t == "GLOBAL_POSITION_INT":
            _SON["alt"] = m.relative_alt / 1000.0
            _SON["vz"] = -m.vz / 100.0        # +yukarı
        elif t == "ATTITUDE":
            _SON["rol"], _SON["pit"], _SON["yaw"] = m.roll, m.pitch, m.yaw
        elif t == "SERVO_OUTPUT_RAW":
            _SON["u"] = ((m.servo1_raw + m.servo2_raw + m.servo3_raw
                          + m.servo4_raw) / 4.0 - 1100.0) / 800.0
        else:
            _SON["x"], _SON["y"] = m.x, m.y
            _SON["vxy"] = math.hypot(m.vx, m.vy)
    return _SON


def aci_komut(c, roll_deg, pitch_deg, yaw_rad, thrust=0.5):
    q = q_kur(math.radians(roll_deg), math.radians(pitch_deg), yaw_rad)
    c.mav.set_attitude_target_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        c.target_system, c.target_component, TM_ORAN_YOKSAY, q, 0, 0, 0, thrust)


def hiz_komut(c, vx, vy, vz):
    """GUIDED hız komutu — kolları arası frenleme için."""
    c.mav.set_position_target_local_ned_send(
        0, c.target_system, c.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)


def sabitlen(c, irtifa, zaman_asimi=90, tani=True):
    """Aracı hedef irtifada ve YATAY OLARAK DURGUN hâle getir.

    ⛔ İLK SÜRÜM GEÇERSİZ ÖLÇÜM ÜRETTİ (2026-09-06): yalnız dikey hıza
      bakıyordu ve beklerken hiç komut göndermiyordu. 45° kolu araç ZATEN
      5 m/s alçalırken başladı ve "45° irtifa tutmuyor" gibi göründü. Aracın
      değil, TEZGÂHIN sonucuydu (CLAUDE.md §5.1).

    Doğrusu: GUIDED KONUM hedefi ver ve ÜÇ şart birden KESİNTİSİZ 2 s sağlansın:
      |irtifa hatası| < 1.5 m ,  |dikey hız| < 0.4 m/s ,  |yatay hız| < 1.0 m/s
    Sağlanmazsa hangi şartın tutmadığı YAZILIR — sessizce "GEÇERSİZ" demek
    teşhis bırakmaz.
    """
    d = durum(c, 1.0)
    if d["x"] is None:
        print("  ⛔ LOCAL_POSITION_NED gelmiyor")
        return False
    hx, hy = d["x"], d["y"]

    dingin_bas = None
    t0 = time.time()
    son_rapor = 0.0
    while time.time() - t0 < zaman_asimi:
        c.mav.set_position_target_local_ned_send(
            0, c.target_system, c.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000, hx, hy, -irtifa,
            0, 0, 0, 0, 0, 0, 0, 0)
        d = durum(c, 0.15)
        if d["alt"] is None:
            continue
        h_irt = abs(irtifa - d["alt"])
        tamam = (h_irt < 1.5 and abs(d["vz"]) < 0.4 and d["vxy"] < 1.5)
        if tamam:
            if dingin_bas is None:
                dingin_bas = time.time()
            elif time.time() - dingin_bas >= 2.0:
                return True
        else:
            dingin_bas = None
        gecen = time.time() - t0
        if tani and gecen - son_rapor >= 5.0:
            son_rapor = gecen
            print("     [sabitleniyor %3.0f s] irtifa %6.1f (hata %5.1f m) "
                  "vz %+5.2f  vxy %5.2f  motor_u %.3f   eksik: %s"
                  % (gecen, d["alt"], h_irt, d["vz"], d["vxy"], d["u"],
                     ", ".join(x for x, k in
                               (("irtifa", h_irt < 1.5), ("vz", abs(d["vz"]) < 0.4),
                                ("vxy", d["vxy"] < 1.5)) if not k) or "—"))
    print("     ⛔ %.0f s'de sabitlenemedi" % zaman_asimi)
    return False


def kol(c, aci_deg, sure, irtifa):
    print("\n" + "=" * 74)
    gereken = 1.0 / math.cos(math.radians(aci_deg))
    print("  KOL %d°   irtifayı korumak için gereken itki: %.2fx asılı"
          % (aci_deg, gereken))
    print("=" * 74)
    if not sabitlen(c, irtifa):
        print("  ⛔ irtifaya dönülemedi — kol GEÇERSİZ")
        return None
    d = durum(c, 0.5)
    a0, yaw0 = d["alt"], d["yaw"]
    print("  başlangıç: irtifa %.1f m" % a0)
    # ⛔ AYIRICI ÖLÇÜM: motorlar DOYDU mu?
    #   Araç alçalıyor VE motorlar tavanda (u≈MOT_SPIN_MAX) ise sebep İTKİ
    #   YETERSİZLİĞİdir — zarf gerçekten dar demektir.
    #   Araç alçalıyor AMA motorlarda pay varsa sebep DENETLEYİCİdir
    #   (irtifa denetleyicisi yeterince gaz istemiyor) — zarf değil ayar sorunu.
    print("\n   t(s)  irtifa   Δirtifa   yatış    eğim  bileşke  dik.itki  motor_u")
    izler = []
    t0 = time.time()
    while time.time() - t0 < sure:
        aci_komut(c, aci_deg, 0.0, yaw0, 0.5)
        d = durum(c, 0.12)
        t = time.time() - t0
        oran = math.cos(d["rol"]) * math.cos(d["pit"])
        bil = math.degrees(math.acos(max(-1.0, min(1.0, oran))))
        izler.append((t, d["alt"], bil, oran, d["u"]))
        print("  %5.2f  %6.1f  %+8.1f  %+6.1f  %+6.1f  %5.1f°    %.3f    %.3f%s"
              % (t, d["alt"], d["alt"] - a0, math.degrees(d["rol"]),
                 math.degrees(d["pit"]), bil, oran, d["u"],
                 "  DOYUM" if d["u"] > 0.99 else ""))
    # ── TOPARLANMA: önce DÜZLE ve hızı bırak, sonra konuma kilitle.
    # ⛔ ÖLÇÜLDÜ (T07): manevra bitiminde doğrudan konum hedefine geçmek
    #   aracı düşürüyordu. Sebep: araç 25 m/s yatay hızla çıkıyor, frenlemek
    #   için yatması gerekiyor, yatınca dikey itki düşüyor ve payı olmayan
    #   araç toparlanamıyor. Önce 1.5 s düz uçup (yalnız irtifa tutarak) hız
    #   sürüklenmeyle azalsın, sonra konum kilidi devreye girsin.
    t_duz = time.time()
    while time.time() - t_duz < 1.5:
        aci_komut(c, 0, 0, yaw0, 0.5)
        durum(c, 0.1)
    # GUIDED hız komutuyla nazik fren (ArduPilot kendi ivme/jerk tavanlarını
    # uygular; konum hedefi ise anında büyük hata görüp sert komut ediyordu)
    t_fren = time.time()
    while time.time() - t_fren < 4.0:
        hiz_komut(c, 0, 0, 0)
        durum(c, 0.15)
    son = izler[-1]
    tepe = max(z[2] for z in izler)
    u_tepe = max(z[4] for z in izler)
    doyum = sum(1 for z in izler if z[4] > 0.99) / float(len(izler))
    kayip = a0 - son[1]
    print("\n  ▸ %d°: %.1f s'de irtifa değişimi %+.1f m  (tepe bileşke eğim %.1f°,"
          " tepe motor u %.3f, karelerin %%%.0f'ünde DOYUM)"
          % (aci_deg, son[0], -kayip, tepe, u_tepe, doyum * 100))
    return {"aci": aci_deg, "kayip": kayip, "tepe": tepe, "gereken": gereken,
            "u_tepe": u_tepe, "doyum": doyum}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--adres", default="udpin:127.0.0.1:14541")
    ap.add_argument("--irtifa", type=float, default=120.0)
    ap.add_argument("--sure", type=float, default=1.5)
    ap.add_argument("--acilar", default="30,45,60,70")
    a = ap.parse_args()

    c = mavutil.mavlink_connection(a.adres)
    c.wait_heartbeat()
    c.mav.request_data_stream_send(c.target_system, c.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 15, 1)
    time.sleep(1)
    print("bağlandı: sys=%d" % c.target_system)

    print("\nEKF bekleniyor…")
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
        if d["alt"] and d["alt"] >= a.irtifa - 3:
            break
    print("kalkış tamam: %.1f m" % (durum(c)["alt"] or -1))

    sonuc = []
    for aci in [int(x) for x in a.acilar.split(",")]:
        r = kol(c, aci, a.sure, a.irtifa)
        if r:
            sonuc.append(r)

    print("\n" + "=" * 74)
    print("  SONUÇ — VAKKAS EĞİM ZARFI  (itki/ağırlık ölçülen 1.99, kullanılabilir 1.80)")
    print("=" * 74)
    print("  %5s %13s %15s %9s %8s   %s"
          % ("eğim", "gereken itki", "irtifa değişimi", "tepe u", "doyum",
             "hüküm"))
    for r in sonuc:
        yeter = r["gereken"] <= 1.80
        uyum = "✔ uyuyor" if (yeter == (r["kayip"] < 5.0)) else "⛔ ÇELİŞİYOR"
        print("  %4d° %12.2fx %+14.1f m %8.3f %7.0f%%   %-7s %s"
              % (r["aci"], r["gereken"], -r["kayip"], r["u_tepe"],
                 r["doyum"] * 100, "TUTMALI" if yeter else "TUTAMAZ", uyum))
    print()
    print("  motor u = (PWM−1100)/800 · eklenti u değerini 1.00e KIRPAR; doyum = u ≥ 0.99.")
    print("  DOYUM varsa sebep İTKİ yetersizliği; yoksa sebep DENETLEYİCİdir.")
    print()
    print("  ⛔ SİM ölçümüdür; güdüm kabul kararı vermez (CLAUDE.md §2).")
    c.mav.set_mode_send(c.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6)
    return 0


if __name__ == "__main__":
    sys.exit(main())
