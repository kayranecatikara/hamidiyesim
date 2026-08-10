#!/usr/bin/env python3
"""kampanya.py — GÖZETİMSİZ çok-uçuşlu A/B kampanyası (saatlerce koşar).

NEDEN VAR (2026-08-10, kullanıcı isteği): tek tek uçuş koşmak insan başında
beklemeyi gerektiriyordu. Bu araç bir kol matrisini DÖNÜŞÜMLÜ olarak, her
koşuda simi baştan kurarak, süre dolana kadar tekrar tekrar uçurur ve her
koşunun sonucunu ANINDA diske yazar (çökme olursa veri kaybolmasın).

CLAUDE.md uyumu:
  §2  her koşu taze uçuş + kare kaydı (kacamak_testi kareleri yazar)
  §3.3 varsayılan senaryo: düz uçuş + tetiklenmiş kaçamak; `yok` kolu TABAN
  §4  TEK DEĞİŞKEN · DÖNÜŞÜMLÜ A/B · ölçütler önceden ilan · her koşu arşivli
      · her kol için TAM RESTART (koşu boyunca anahtar değişmediği garanti)

Kullanım:
    python3 tools/kampanya.py <cikti_dizini> <saat> [kayit_s]

Çıktı:
    <dir>/sonuclar.csv     her koşu bir satır (anında yazılır)
    <dir>/kosu_NNN_<kol>/  o koşunun kareleri + kacamak.csv + olay.json
    <dir>/kampanya.log     tam günlük
"""
import csv
import json
import os
import shutil
import subprocess
import sys
import time
import urllib.request

KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE = "http://127.0.0.1:8000"

# ── KOL MATRİSİ — TEK DEĞİŞKEN ÇİFTİ: Ö5 (hız tavanı) × ATC_ANGLE_MAX ──
# Ö6 (yatış 45→55°) bir ArduPilot parametresi olduğu için TAM RESTART ister;
# Ö5 bir kod anahtarı. İkisi de env'den veriliyor ve koşu başında panelden
# DOĞRULANIYOR (damga yalanına karşı — bkz. 08-09 GPS_RANGE dersi).
KOLLAR = [
    ("A", {},                                                    45),   # taban
    ("C", {"AVCI_IBVS_MANEVRA": "1"},                            45),   # Ö5
    ("B", {},                                                    55),   # Ö6
    ("D", {"AVCI_IBVS_MANEVRA": "1", "AVCI_IBVS_MANEVRA_ACI": "55"}, 55),  # ikisi
]
# Kaçamaklar — `yok` TABAN koşusudur, CLAUDE.md §3.3 gereği her turda koşulur.
KACAMAKLAR = ["yok", "yatay", "capraz"]


def gunluk(dizin, msg):
    satir = f"[{time.strftime('%H:%M:%S')}] {msg}"
    print(satir, flush=True)
    with open(os.path.join(dizin, "kampanya.log"), "a") as f:
        f.write(satir + "\n")


def kabuk(cmd, sn, log_yolu=None, env_ek=None):
    env = dict(os.environ)
    if env_ek:
        env.update(env_ek)
    with open(log_yolu or os.devnull, "a") as f:
        try:
            return subprocess.run(cmd, shell=True, cwd=KOK, timeout=sn,
                                  stdout=f, stderr=subprocess.STDOUT,
                                  env=env).returncode
        except subprocess.TimeoutExpired:
            return -1


def sim_kur(dizin, aci, log_yolu):
    """Gazebo + iki SITL'i baştan kurar. aci: ATC_ANGLE_MAX (derece)."""
    env = {"GZ_HEADLESS": "1"}
    if aci != 45:
        env["AVCI_PARM_EK"] = f"ATC_ANGLE_MAX {aci}"
    rc = kabuk("bash scripts/start_harmonic.sh yeniden", 360, log_yolu, env)
    return rc == 0


def gcs_kur(env_ek, log_yolu):
    """gcs_server'ı verilen env ile başlatır ve panel açılana kadar bekler."""
    kabuk("fuser -k 8000/tcp", 20)
    time.sleep(1.5)
    env = dict(os.environ)
    env.update({"AVCI_GZ_CAMERA": "1", "AVCI_NO_BROWSER": "1", "AVCI_BEKCI": "0"})
    # Önceki kolun anahtarları SIZMASIN — matristeki tüm anahtarlar temizlenir.
    for k in ("AVCI_IBVS_MANEVRA", "AVCI_IBVS_MANEVRA_ACI"):
        env.pop(k, None)
    env.update(env_ek)
    with open(log_yolu, "a") as f:
        subprocess.Popen([sys.executable, "-m", "control.gcs_server"], cwd=KOK,
                         stdout=f, stderr=subprocess.STDOUT, env=env,
                         start_new_session=True)
    for _ in range(40):
        time.sleep(2)
        try:
            urllib.request.urlopen(BASE + "/api/scenario_status", timeout=2).read()
            return True
        except Exception:
            pass
    return False


def kol_dogrula(beklenen_acik):
    """Panelden GERÇEK durumu okur — env'e değil, sunucunun söylediğine güven."""
    try:
        d = json.loads(urllib.request.urlopen(
            BASE + "/api/gudum_ozellikleri", timeout=5).read())
        o = [x for x in d["ozellikler"] if x["ad"] == "o5_manevra"][0]
        return bool(o["acik"]) == beklenen_acik, bool(o["acik"])
    except Exception as e:
        return False, f"okunamadı: {e}"


def sim_kapat():
    kabuk("bash scripts/start_harmonic.sh stop", 120)
    kabuk("fuser -k 8000/tcp", 20)
    time.sleep(2)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        raise SystemExit(1)
    dizin = os.path.abspath(sys.argv[1])
    saat = float(sys.argv[2])
    kayit_s = float(sys.argv[3]) if len(sys.argv) > 3 else 200.0
    os.makedirs(dizin, exist_ok=True)
    bitis = time.time() + saat * 3600.0
    sonuc_csv = os.path.join(dizin, "sonuclar.csv")
    sim_log = os.path.join(dizin, "sim.log")
    gcs_log = os.path.join(dizin, "gcs.log")

    alanlar = ["kosu", "kol", "manevra", "aci", "kacamak", "gecerli", "sebep",
               "tetik_m", "tetiklendi", "en_yakin_m", "imha", "manevra_kare", "toplam_kare",
               "v_yanal_p90", "hedef_hiz", "hedef_irtifa", "wall"]
    if not os.path.exists(sonuc_csv):
        with open(sonuc_csv, "w", newline="") as f:
            csv.DictWriter(f, alanlar).writeheader()

    gunluk(dizin, f"KAMPANYA BAŞLIYOR — {saat:.1f} saat, kayıt {kayit_s:.0f} s/koşu")
    gunluk(dizin, f"  matris: {len(KOLLAR)} kol × {len(KACAMAKLAR)} kaçamak, DÖNÜŞÜMLÜ")

    kosu = 0
    tur = 0
    while time.time() < bitis:
        tur += 1
        for kacamak in KACAMAKLAR:
            for kol_ad, env_ek, aci in KOLLAR:
                if time.time() >= bitis:
                    break
                kosu += 1
                etiket = f"{kosu:03d}_{kol_ad}_{kacamak}_{aci}"
                kalan = (bitis - time.time()) / 3600.0
                gunluk(dizin, f"── KOŞU {etiket}  (tur {tur}, kalan {kalan:.1f} sa)")

                sat = {a: "" for a in alanlar}
                sat.update(kosu=kosu, kol=kol_ad, aci=aci, kacamak=kacamak,
                           manevra=int("AVCI_IBVS_MANEVRA" in env_ek),
                           wall=time.strftime("%Y-%m-%d %H:%M:%S"))

                if not sim_kur(dizin, aci, sim_log):
                    sat.update(gecerli=0, sebep="sim kurulamadı")
                    yaz(sonuc_csv, alanlar, sat); gunluk(dizin, "  ✗ sim kurulamadı"); continue
                if not gcs_kur(env_ek, gcs_log):
                    sat.update(gecerli=0, sebep="panel açılmadı")
                    yaz(sonuc_csv, alanlar, sat); gunluk(dizin, "  ✗ panel açılmadı")
                    sim_kapat(); continue

                ok, gercek = kol_dogrula("AVCI_IBVS_MANEVRA" in env_ek)
                if not ok:
                    sat.update(gecerli=0, sebep=f"kol doğrulanamadı ({gercek})")
                    yaz(sonuc_csv, alanlar, sat)
                    gunluk(dizin, f"  ✗ KOL DOĞRULANAMADI: {gercek}")
                    sim_kapat(); continue

                kdizin = os.path.join(dizin, f"kosu_{etiket}")
                t_uc0 = time.time()
                rc = kabuk(f"{sys.executable} tools/kacamak_testi.py "
                           f"{kdizin} {kacamak} 25 {kayit_s:.0f}",
                           kayit_s + 420, os.path.join(dizin, "ucus.log"))
                sure = time.time() - t_uc0

                olc = olcumle(kdizin, t_uc0)
                sat.update(olc)
                sat["gecerli"] = olc.get("gecerli", 0)
                sat["sebep"] = olc.get("sebep", "" if rc == 0 else f"rc={rc}")
                yaz(sonuc_csv, alanlar, sat)
                gunluk(dizin, f"  → geçerli={sat['gecerli']} imha={sat['imha']} "
                              f"en_yakın={sat['en_yakin_m']} m "
                              f"manevra_kare={sat['manevra_kare']} ({sure:.0f} s)")
                sim_kapat()

    gunluk(dizin, f"KAMPANYA BİTTİ — {kosu} koşu, {tur} tur")
    sim_kapat()


def yaz(yol, alanlar, sat):
    with open(yol, "a", newline="") as f:
        csv.DictWriter(f, alanlar).writerow(sat)


def olcumle(kdizin, t0):
    """Koşunun sonucunu topla: olay.json + meta.csv + o aralıktaki bbox logu."""
    out = {"gecerli": 0, "sebep": "", "tetik_m": "", "tetiklendi": 0,
           "en_yakin_m": "", "imha": "",
           "manevra_kare": 0, "toplam_kare": 0, "v_yanal_p90": "",
           "hedef_hiz": "", "hedef_irtifa": ""}
    oj = os.path.join(kdizin, "olay.json")
    if os.path.exists(oj):
        try:
            d = json.load(open(oj))
            out["tetik_m"] = d.get("tetik_m", "")
            out["tetiklendi"] = int(bool(d.get("tetiklendi")))
            # Kaçamaklı kollarda ölçüt TETİKTEN SONRAKİ en yakın menzildir:
            # tetikten önceki yakınlaşma kaçamağa verilen tepkiyi ölçmez.
            ey = d.get("en_yakin_tetikten_sonra")
            out["en_yakin_m"] = ey if ey not in (None, "") else d.get("en_yakin", "")
            out["imha"] = int(bool(d.get("imha")))
        except Exception:
            pass
    # GEÇERLİLİK: hedef 20-250 m irtifa / 6-25 m/s bandında kaldı mı (CLAUDE.md §4)
    mc = os.path.join(kdizin, "meta.csv")
    if os.path.exists(mc):
        try:
            r = list(csv.DictReader(open(mc)))
            hz = [float(x["plane_spd"]) for x in r if x.get("plane_spd")]
            it = [-float(x["plane_z"]) for x in r if x.get("plane_z")]
            ucus = [(h, i) for h, i in zip(hz, it) if h > 3.0]
            if ucus:
                hz2 = sorted(h for h, _ in ucus); it2 = sorted(i for _, i in ucus)
                out["hedef_hiz"] = round(hz2[len(hz2)//2], 1)
                out["hedef_irtifa"] = round(it2[len(it2)//2], 1)
                bant = (6.0 <= hz2[len(hz2)//2] <= 25.0
                        and 20.0 <= it2[len(it2)//2] <= 250.0)
                out["gecerli"] = int(bant and out["imha"] != "")
                if not bant:
                    out["sebep"] = "hedef bant dışı"
        except Exception as e:
            out["sebep"] = f"meta okunamadı: {e}"
    # Özellik gerçekten tetiklendi mi (bbox logundan)
    try:
        import glob
        man = top = 0; vy = []
        for f in glob.glob(os.path.join(KOK, "logs", "bbox_ibvs_*.csv")):
            if os.path.getmtime(f) < t0:
                continue
            for x in csv.DictReader(open(f)):
                if not x.get("boyut"):
                    continue
                top += 1
                if x.get("manevra") == "1":
                    man += 1
                try:
                    vy.append(float(x["v_yanal"]))
                except (ValueError, TypeError, KeyError):
                    pass
        out["manevra_kare"] = man
        out["toplam_kare"] = top
        if vy:
            vy.sort()
            out["v_yanal_p90"] = round(vy[int(len(vy)*0.9)], 1)
    except Exception:
        pass
    return out


if __name__ == "__main__":
    main()
