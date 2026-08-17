#!/usr/bin/env python3
"""DURUŞ DÖNGÜSÜ KALİTESİ — ArduPilot dataflash (BIN) logundan.

Kullanım:
    python3 tools/durus_kalitesi.py                 # en son avcı logu
    python3 tools/durus_kalitesi.py <log.BIN> ...   # belirli loglar

NİYE BIN: güdüm logu 20 Hz; iç döngü salınımı 2-5 Hz mertebesinde ve
§5.3 gereği 20 Hz ile ölçülemez. BIN'de RATE mesajı komut edilen ve
gerçekleşen açısal hızı yüksek hızda yan yana veriyor — ayarlamanın
tek doğru aleti bu.

BİRİNCİL ÖLÇÜT: hız döngüsü takip hatası p90 (°/s). Düşük = iyi.
GEÇERLİLİK EŞİ: PID çıkışı doyum oranı. Hata düşerken doyum %20'yi
aşıyorsa kazanç FAZLA demektir — salınıma gider.
"""
import glob, os, sys, statistics as st
from pymavlink import mavutil

TABAN = {"hiz_p90": 88.9, "aci_p90": 7.56, "cikis_p90": 0.49, "doyum": 1.2}


def p90(v):
    v = sorted(v)
    return v[int(.9 * len(v))] if v else float("nan")


def avci_loglari(n=1):
    """PSCN içeren = avcı (kopter). Plane'de o mesaj yok."""
    out = []
    for p in sorted(glob.glob(os.path.expanduser("~/ardupilot/logs/*.BIN")),
                    key=os.path.getmtime, reverse=True):
        try:
            m = mavutil.mavlink_connection(p)
            for _ in range(6000):
                msg = m.recv_match()
                if msg is None:
                    break
                if msg.get_type() == "PSCN":
                    out.append(p)
                    break
        except Exception:
            pass
        if len(out) >= n:
            break
    return out


def coz(p):
    m = mavutil.mavlink_connection(p)
    R, P, AT = [], [], []
    while True:
        msg = m.recv_match(type=["RATE", "ATT"])
        if msg is None:
            break
        if msg.get_type() == "RATE":
            R.append((msg.RDes, msg.R, msg.ROut))
            P.append((msg.PDes, msg.P, msg.POut))
        else:
            AT.append((msg.DesRoll, msg.Roll, msg.DesPitch, msg.Pitch))
    return R, P, AT


def rapor(p):
    R, P, AT = coz(p)
    if len(R) < 100:
        print(f"{os.path.basename(p)}: yetersiz veri (RATE {len(R)})")
        return None
    print("=" * 78)
    print(f"{os.path.basename(p)}   RATE n={len(R)}  ATT n={len(AT)}")
    print("=" * 78)
    sonuc = {}
    for ad, S in (("YATIS", R), ("EGIM", P)):
        err = [abs(d - a) for d, a, _ in S]
        out = [abs(o) for _, _, o in S]
        doyum = 100.0 * sum(1 for o in out if o > 0.95) / len(out)
        print(f"  {ad:<6} HIZ HATASI  ort {st.mean(err):6.2f}  p90 {p90(err):6.2f} °/s"
              f"   | PID cikisi p90 {p90(out):5.3f}  DOYUM %{doyum:4.1f}")
        if ad == "YATIS":
            sonuc = {"hiz_p90": p90(err), "cikis_p90": p90(out), "doyum": doyum}
    if AT:
        er = [abs(a[0] - a[1]) for a in AT]
        ep = [abs(a[2] - a[3]) for a in AT]
        print(f"  YATIS  ACI HATASI  ort {st.mean(er):5.2f}  p90 {p90(er):6.2f}°"
              f"   | komut |aci| p90 {p90([abs(a[0]) for a in AT]):5.1f}°")
        print(f"  EGIM   ACI HATASI  ort {st.mean(ep):5.2f}  p90 {p90(ep):6.2f}°"
              f"   | komut |aci| p90 {p90([abs(a[2]) for a in AT]):5.1f}°")
        sonuc["aci_p90"] = p90(er)
    return sonuc


def main():
    yollar = sys.argv[1:] or avci_loglari(1)
    if not yollar:
        print("⛔ avcı BIN logu bulunamadı"); return
    son = None
    for p in yollar:
        r = rapor(p)
        if r:
            son = r
    if son:
        print()
        print("KIYAS — FAZ A TABANI (kullanıcı uçuşu 20260817_142746, ayarsız):")
        print(f"  hız hatası p90 {TABAN['hiz_p90']:5.1f} °/s | açı hatası p90 "
              f"{TABAN['aci_p90']:4.2f}° | çıkış p90 {TABAN['cikis_p90']:4.2f} "
              f"| doyum %{TABAN['doyum']:.1f}")
        print(f"  ŞİMDİ          {son.get('hiz_p90',float('nan')):5.1f} °/s |"
              f"              {son.get('aci_p90',float('nan')):4.2f}° |"
              f"            {son.get('cikis_p90',float('nan')):4.2f} "
              f"| doyum %{son.get('doyum',float('nan')):.1f}")
        print()
        print("İLAN EDİLEN KAPI: hız hatası p90 < 25 °/s olmalı.")
        print("GEÇERLİLİK EŞİ  : doyum %20'yi AŞMAMALI (aşarsa kazanç fazla).")
        g = son.get("hiz_p90", 1e9) < 25.0
        d = son.get("doyum", 1e9) <= 20.0
        print(f"SONUÇ: kapı {'GEÇTİ ✓' if g else 'GEÇMEDİ ✗'} | "
              f"geçerlilik eşi {'TEMİZ ✓' if d else 'İHLAL ✗'}")


main()
