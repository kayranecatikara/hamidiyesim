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

# ── KOL MATRİSİ — TEK DEĞİŞKEN: SEYİR FAZINDA LEAD (M3b, 2026-08-11) ──
# Ö5 (hız tavanı) ve Ö6 (yatış 45→55°) 158 koşuda ETKİSİZ çıktı, matristen
# ÇIKARILDI. Yeni değişken: erken lead + seyir tavanı. Ölçülen kusur: lead
# karelerin %71-77'sinde TAM SIFIR olduğu için drone dönen hedefe SAF TAKİP
# yapıyor; ıska ilk geçişte medyan 4.2-4.6 m (kaçamaksız 0.6 m).
# Tavan iki değerde deneniyor: 08-09'da 25° gölge etmeye yol açmıştı, kodun
# notu 8-10° öneriyordu — 8 ve 15 ile alt/üst sınırı tarıyoruz.
#
# G kolu AYRI BİR SORU: D0 devir ölçütü (2026-08-11'de Kayra'dan alındı).
# "Ardışık N kare" kural ölçütüne sadık ama gürültülü tespitte devri
# GECİKTİREBİLİR — kayan pencere tam bu yüzden konmuştu (07-31). G, E ile
# YALNIZ bu anahtarda ayrılır, yani E↔G tek değişkenli kıyastır.
KOLLAR = [
    ("A", {},                                                          45),
    ("E", {"AVCI_IBVS_LEAD_ERKEN": "1", "AVCI_IBVS_LEAD_MAX_SEYIR": "8"},  45),
    ("F", {"AVCI_IBVS_LEAD_ERKEN": "1", "AVCI_IBVS_LEAD_MAX_SEYIR": "15"}, 45),
    ("G", {"AVCI_IBVS_LEAD_ERKEN": "1", "AVCI_IBVS_LEAD_MAX_SEYIR": "8",
           "AVCI_HYBRID_ARDISIK": "0"},                                45),
]
# Kaçamaklar — `yok` TABAN koşusudur, CLAUDE.md §3.3 gereği her turda koşulur.
# 08-11: altı kaçamağın TAMAMI (§3.3 "hepsi denenmeli"). Dikey üçlü ilk kez
# uçuyor; kamera sabit +25° yukarı baktığı için asıl kör nokta orada olabilir.
KACAMAKLAR = ["yok", "yatay", "capraz", "dikey_yukari", "dikey_asagi", "hizlan"]

# ── HEDEFİN SEYİR İRTİFASI ──
# 08-11 pilotu: 60 m DENENDİ ve GERİ ALINDI. Drone 60 m'ye tırmanırken ~90 s
# harcıyor, hedef o sırada 300 m uzaklaşıyor ve buluşma 150 s'lik kayda
# sığmıyor (en yakın 136 m, tetik hiç olmadı). 30 m'de ise dalış kaçamağı
# ölçüldü: hedef 38 m'de seyredip en düşük 31.2 m'ye iniyor — 20 m tabanının
# çok üstünde, yani irtifayı yükseltmeye GEREK YOK.
SCN_ALT = os.environ.get("KAMPANYA_ALT", "30")

# Matristeki TÜM anahtarlar — kol değişince öncekiler sızmasın diye temizlenir.
_MATRIS_ANAHTARLARI = ("AVCI_IBVS_LEAD_ERKEN", "AVCI_IBVS_LEAD_MAX_SEYIR",
                       "AVCI_HYBRID_ARDISIK", "AVCI_HYBRID_CONF",
                       "AVCI_IBVS_MANEVRA", "AVCI_IBVS_MANEVRA_ACI")


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
    env.update({"AVCI_GZ_CAMERA": "1", "AVCI_NO_BROWSER": "1", "AVCI_BEKCI": "0",
                "AVCI_SCN_ALT": SCN_ALT})
    # Önceki kolun anahtarları SIZMASIN — matristeki tüm anahtarlar temizlenir.
    for k in _MATRIS_ANAHTARLARI:
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


def kol_dogrula(env_ek):
    """Panelden GERÇEK durumu okur — env'e değil, sunucunun söylediğine güven.

    Hem erken lead anahtarını hem SEYİR TAVANININ SAYISINI doğrular; kol E ile
    F yalnız o sayıda ayrıldığı için tavan yanlışsa iki kol aynı şeyi uçurur ve
    kampanya sessizce anlamsızlaşır (bkz. 08-09 GPS_RANGE damga yalanı dersi).
    """
    bek_lead = env_ek.get("AVCI_IBVS_LEAD_ERKEN") == "1"
    bek_tavan = env_ek.get("AVCI_IBVS_LEAD_MAX_SEYIR")
    bek_ardisik = env_ek.get("AVCI_HYBRID_ARDISIK", "1") == "1"
    try:
        d = json.loads(urllib.request.urlopen(
            BASE + "/api/gudum_ozellikleri", timeout=5).read())
        oz = {x["ad"]: x for x in d["ozellikler"]}
        lead = bool(oz["m3_erken_lead"]["acik"])
        tavan = float(oz["m3b_lead_seyir_tavan"]["deger"])
        ardisik = bool(oz["d0_ardisik"]["acik"])
        gercek = f"lead={int(lead)} tavan={tavan:.0f} ardisik={int(ardisik)}"
        if lead != bek_lead:
            return False, gercek
        if bek_tavan is not None and abs(tavan - float(bek_tavan)) > 1e-6:
            return False, gercek
        if ardisik != bek_ardisik:
            return False, gercek
        return True, gercek
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

    # ilk15_m = BİRİNCİL ÖLÇÜT (bkz. olcumle); en_yakin_m ikincil.
    alanlar = ["kosu", "kol", "lead", "tavan", "ardisik", "aci", "kacamak", "gecerli", "sebep",
               "tetik_m", "tetiklendi", "ilk15_m", "gecikme_s", "en_yakin_m", "imha",
               "lead_kare", "lead_med_deg", "toplam_kare", "devir_m",
               "hedef_hiz", "hedef_irtifa", "hedef_irt_min", "wall"]
    if not os.path.exists(sonuc_csv):
        with open(sonuc_csv, "w", newline="") as f:
            csv.DictWriter(f, alanlar).writeheader()

    # PİLOT MODU: matrisi daraltıp birkaç koşuluk doğrulama uçuşu yapmak için
    # (gece kampanyasını kurmadan önce "kollar gerçekten ayrışıyor mu",
    # "yeni kaçamak bandı kırıyor mu" sorularını ucuza cevaplar).
    _kf = os.environ.get("KAMPANYA_KOLLAR")
    _af = os.environ.get("KAMPANYA_KACAMAKLAR")
    kollar = [k for k in KOLLAR if k[0] in _kf.split(",")] if _kf else KOLLAR
    kacamaklar = _af.split(",") if _af else KACAMAKLAR
    max_kosu = int(os.environ.get("KAMPANYA_MAX_KOSU", "0")) or None

    gunluk(dizin, f"KAMPANYA BAŞLIYOR — {saat:.1f} saat, kayıt {kayit_s:.0f} s/koşu")
    gunluk(dizin, f"  matris: {len(kollar)} kol × {len(kacamaklar)} kaçamak, DÖNÜŞÜMLÜ")
    gunluk(dizin, f"  kollar: {[k[0] for k in kollar]}  kaçamaklar: {kacamaklar}")
    gunluk(dizin, f"  hedef seyir irtifası: {SCN_ALT} m (AVCI_SCN_ALT)")
    if max_kosu:
        gunluk(dizin, f"  ⚠ PİLOT MODU — en fazla {max_kosu} koşu")

    kosu = 0
    tur = 0
    while time.time() < bitis and not (max_kosu and kosu >= max_kosu):
        tur += 1
        for kacamak in kacamaklar:
            for kol_ad, env_ek, aci in kollar:
                if time.time() >= bitis or (max_kosu and kosu >= max_kosu):
                    break
                kosu += 1
                etiket = f"{kosu:03d}_{kol_ad}_{kacamak}_{aci}"
                kalan = (bitis - time.time()) / 3600.0
                gunluk(dizin, f"── KOŞU {etiket}  (tur {tur}, kalan {kalan:.1f} sa)")

                sat = {a: "" for a in alanlar}
                sat.update(kosu=kosu, kol=kol_ad, aci=aci, kacamak=kacamak,
                           lead=int(env_ek.get("AVCI_IBVS_LEAD_ERKEN") == "1"),
                           tavan=env_ek.get("AVCI_IBVS_LEAD_MAX_SEYIR", ""),
                           ardisik=int(env_ek.get("AVCI_HYBRID_ARDISIK", "1") == "1"),
                           wall=time.strftime("%Y-%m-%d %H:%M:%S"))

                if not sim_kur(dizin, aci, sim_log):
                    sat.update(gecerli=0, sebep="sim kurulamadı")
                    yaz(sonuc_csv, alanlar, sat); gunluk(dizin, "  ✗ sim kurulamadı"); continue
                if not gcs_kur(env_ek, gcs_log):
                    sat.update(gecerli=0, sebep="panel açılmadı")
                    yaz(sonuc_csv, alanlar, sat); gunluk(dizin, "  ✗ panel açılmadı")
                    sim_kapat(); continue

                ok, gercek = kol_dogrula(env_ek)
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
                gunluk(dizin, f"  → geçerli={sat['gecerli']} ilk15={sat['ilk15_m']} m "
                              f"imha={sat['imha']} en_yakın={sat['en_yakin_m']} m "
                              f"lead={sat['lead_kare']}/{sat['toplam_kare']} kare "
                              f"med {sat['lead_med_deg']}° ({sure:.0f} s)"
                              + (f"  ⚠ {sat['sebep']}" if sat['sebep'] else ""))
                sim_kapat()

    gunluk(dizin, f"KAMPANYA BİTTİ — {kosu} koşu, {tur} tur")
    sim_kapat()


def yaz(yol, alanlar, sat):
    with open(yol, "a", newline="") as f:
        csv.DictWriter(f, alanlar).writerow(sat)


def olcumle(kdizin, t0):
    """Koşunun sonucunu topla: olay.json + meta.csv + o aralıktaki bbox logu."""
    out = {"gecerli": 0, "sebep": "", "tetik_m": "", "tetiklendi": 0,
           "ilk15_m": "", "gecikme_s": "", "en_yakin_m": "", "imha": "",
           "lead_kare": 0, "lead_med_deg": "", "toplam_kare": 0, "devir_m": "",
           "hedef_hiz": "", "hedef_irtifa": "", "hedef_irt_min": ""}
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
            if d.get("tetiklendi") and d.get("en_yakin_t") is not None:
                out["gecikme_s"] = round(d["en_yakin_t"] - d["tetik_t"], 1)
            # ── BİRİNCİL ÖLÇÜT: İLK GEÇİŞ ISKASI ──
            # Tetikten sonraki 15 s içindeki en yakın menzil, 10 Hz logdan.
            # NEDEN toplam en_yakin DEĞİL: 08-10 kampanyasında imhaların yalnız
            # %3-16'sı ilk geçişteydi, gecikme medyanı 52 s — yani "vurdu"
            # sayısı uzun yeniden dalışlardan geliyor ve asıl kusuru gizliyor.
            kc = os.path.join(kdizin, "kacamak.csv")
            if d.get("tetiklendi") and os.path.exists(kc):
                # ⚠ `t0` DEĞİL: o, fonksiyonun duvar-saati parametresi. Burada
                # uçuşa göreli tetik anı kullanılır; aynı adı vermek parametreyi
                # ezip aşağıdaki bbox mtime filtresini sessizce devre dışı
                # bırakıyordu (08-11'de yakalandı: toplam_kare 128 yerine 39405).
                t_tet = d["tetik_t"]
                v = [float(x["mesafe"]) for x in csv.DictReader(open(kc))
                     if x.get("mesafe") and t_tet <= float(x["t"]) <= t_tet + 15.0]
                if v:
                    out["ilk15_m"] = round(min(v), 2)
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
                out["hedef_irt_min"] = round(it2[0], 1)
            # ── GEÇERLİLİK: UÇ DEĞERLER, AMA YALNIZ ÖLÇÜM PENCERESİNDE ──
            # MEDYAN YETMEZ: dikey kaçamak bandı GEÇİCİ kırar, medyan görmez.
            # TÜM KAYIT DA OLMAZ (08-11 pilotunda yakalandı): yakın geçişten
            # SONRA hedef bozuluyor (hız 7.7, irtifa 14 m) ve koşu ölçüm bitmiş
            # olmasına rağmen geçersiz sayılıyordu — CLAUDE.md §8'in "uçuş
            # sonrası artefakt" sınıfı. Pencere: kaydın başından tetik+20 s'ye.
            kc2 = os.path.join(kdizin, "kacamak.csv")
            if os.path.exists(kc2):
                try:
                    kr = list(csv.DictReader(open(kc2)))
                    tt = None
                    try:
                        tt = json.load(open(oj)).get("tetik_t")
                    except Exception:
                        pass
                    ust = (tt + 20.0) if tt else 1e9
                    w = [(float(x["plane_spd"]), float(x["plane_alt"])) for x in kr
                         if x.get("plane_spd") and float(x["t"]) <= ust
                         and float(x["plane_spd"]) > 3.0]
                    if w:
                        hw = sorted(h for h, _ in w); iw = sorted(i for _, i in w)
                        bant = (6.0 <= hw[0] and hw[-1] <= 25.0
                                and 20.0 <= iw[0] and iw[-1] <= 250.0)
                        out["gecerli"] = int(bant and out["imha"] != "")
                        if not bant:
                            out["sebep"] = (f"bant dışı (hız {hw[0]:.1f}-{hw[-1]:.1f}, "
                                            f"irt {iw[0]:.0f}-{iw[-1]:.0f})")
                except Exception as e:
                    out["sebep"] = f"kacamak.csv okunamadı: {e}"
        except Exception as e:
            out["sebep"] = f"meta okunamadı: {e}"
    # ÖZELLİK GERÇEKTEN İŞ GÖRDÜ MÜ (bbox logundan) — kol doğrulaması anahtarın
    # AÇIK olduğunu söyler; bu, açık anahtarın kadraja YANSIDIĞINI söyler.
    # 08-10'da lead karelerin %71-77'sinde sıfırdı; aynı körlüğe düşmemek için
    # sıfırdan farklı lead kare sayısı her koşuda kaydedilir.
    try:
        import glob
        top = 0; ld = []; ilk_boyut = None
        # Dosyalar ZAMAN SIRASINDA gezilir — ilk kutulu kare devir anıdır.
        for f in sorted((f for f in glob.glob(os.path.join(KOK, "logs",
                                                           "bbox_ibvs_*.csv"))
                         if os.path.getmtime(f) >= t0), key=os.path.getmtime):
            for x in csv.DictReader(open(f)):
                if not x.get("boyut"):
                    continue
                top += 1
                try:
                    b = float(x["boyut"])
                    if ilk_boyut is None and b > 0:
                        ilk_boyut = b
                except (ValueError, TypeError):
                    pass
                try:
                    ld.append(abs(float(x["lead_az_deg"])))
                except (ValueError, TypeError, KeyError):
                    pass
        out["toplam_kare"] = top
        out["lead_kare"] = sum(1 for x in ld if x > 0.01)
        nz = sorted(x for x in ld if x > 0.01)
        if nz:
            out["lead_med_deg"] = round(nz[len(nz)//2], 2)
        # ── DEVİR MENZİLİ: E↔G kıyasının ASIL ölçütü ──
        # Görsel faz devir anında başlar, yani bbox logunun İLK kutulu karesi
        # devir anıdır. Menzil kutu boyutundan: R = MENZIL_PX_M / boyut.
        # "Ardışık N" ölçütünün bilinen riski devri GECİKTİRMEK; gecikirse
        # devir DAHA YAKINDA olur ve bu sayı düşer.
        if ilk_boyut:
            out["devir_m"] = round(160.0 / ilk_boyut, 1)
    except Exception:
        pass
    return out


if __name__ == "__main__":
    main()
