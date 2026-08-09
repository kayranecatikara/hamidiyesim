"""control/yorunge_kamera.py — Yörünge (orbit) dış görüş kameraları.

⚠ ŞU AN BAĞLANTISIZ: bu modülü hiçbir yer import etmiyor ve dünya SDF'inde
karşılığı olan kızak modelleri YOK (o değişiklik geri alındı). Dosya, işin
bittiği yerden devam edilebilsin diye duruyor; tek başına hiçbir şey yapmaz.
Devreye almak için gereken üç şey: (1) avci_harmonic.sdf'e yorunge_iris /
yorunge_talon modellerini eklemek, (2) chase sensörlerini uçak modellerinden
sökmek, (3) gcs_server'dan baslat() çağırmak.

Operatör GCS'teki dış görüş panellerinde (AVD / TLD) sürükleyerek azimut ve
yükselişi, tekerlekle mesafeyi değiştirir; kamera aracı kadraj merkezinde
tutarak çevresinde dolaşır. Görüntü TARAYICIDA DÖNDÜRÜLMEZ — fare hareketi
yalnız bir DURUM üretir, kamera Gazebo içinde hareket eder, yeni kareler zaten
açık olan MJPEG akışından geri gelir.

── NEDEN AYRI MODEL (ölçülerek seçildi) ─────────────────────────────────
Dış görüş kameraları model.sdf'te aracın base_link'ine ÇİVİLİ sensörler.
Gazebo çalışan bir simülasyonda bir SENSÖRÜN pozunu değiştirecek servis
sunmuyor; çivili sensöre set_pose uygulamak ARACIN KENDİSİNİ oynatır —
üstelik avcının base_link'i tespit hattının FPV kamerasını da taşıyor. Bu
yüzden sensörlerin dünyaya bağımsız birer model olarak taşınması gerekir;
yayın adları (iris_chase/image, talon_chase/image) korunursa gcs_server'ın
kamera hattında ve arayüzün akış adreslerinde tek satır değişmez.

Yeniden başlatma gerektirmeyen yol denendi ve KAPALI çıktı: /world/avci/create
ile çalışma anında eklenen kamera modeli oluşuyor (servis true dönüyor) ama
sensör RENDER EDİLMİYOR; statik ve tam gövdeli iki varyantta da tek kare
gelmedi. Kamera dünya SDF'inde, Gazebo AÇILIRKEN var olmak zorunda.

Buna karşılık /world/avci/set_pose ÖLÇÜLEREK doğrulandı.

── GÜDÜME ETKİSİ: YOK ───────────────────────────────────────────────────
Bu modül yalnız kamera kızaklarının pozunu YAZAR. Araçlara, güdüme, tespite,
kilit hattına, görev FSM'ine, kayda hiçbir şey yazmaz; yalnız sim_truth'tan
poz OKUR. Kızakların çarpışma geometrisi ve kütlesi yoktur (static) — fiziğe
karışamaz.

⚠ AVCI_TRUTH_TOPIC=pose/info YAPMAYIN. Park hâlindeki araç sorununu çözmek
için cazip görünür ama sim_truth'u BÜTÜN sunucu için yeniden yönlendirir —
menzil() dahil, yani vuruş-menzil hattını. Bu modülün kendi yedeği var
(bkz. _arac_pozu); sim_truth'a dokunulmaz.
"""

import math
import os
import threading
import time

# ── Dünya / model adları ────────────────────────────────────────────────
# DUNYA worlds/avci_harmonic.sdf'teki <world name> ile aynı olmalı.
DUNYA = os.environ.get("AVCI_GZ_WORLD", "avci")

PANELLER = ("iris_chase", "talon_chase")

# panel -> (sahnedeki model adı, sim_truth.pozlar() anahtarı)
# ⚠ Model adları set_pose'un anahtarıdır; SDF'teki <model name> ile birebir.
MODELLER = {
    "iris_chase":  (os.environ.get("AVCI_YORUNGE_MODEL_IRIS",  "yorunge_iris"),  "iris"),
    "talon_chase": (os.environ.get("AVCI_YORUNGE_MODEL_TALON", "yorunge_talon"), "plane"),
}

# ── YÖRÜNGE YÖN İŞARETLERİ — TEK DÜZELTME NOKTASI ───────────────────────
# KABUL ÖLÇÜTÜ (tek geçerli test): fareyi SAĞA sürüklerken araç kadraj
# merkezinde kalır ama UZAKTAKİ ZEMİN/UFUK SAĞA kayar — "sahneyi tutup
# sürüklemek" hissi, yani Gazebo GUI'sinin kendi orbit davranışı.
#
# İŞARET BURADAN BAŞKA HİÇBİR YERDE DÜZELTİLMEZ:
#   • JS'te yok — istemci ham dx/dy biriktirir, hiçbir yerde eksilemez
#     (istemcinin gönderdiği azimut/yukselis "sürükleme uzayı"ndadır).
#   • _poz_hesapla/_quat içinde yok — o matematik 450/450 doğrulandı.
# Yön ters geliyorsa YALNIZ aşağıdaki iki sabitten biri çevrilir ve yalnız
# gcs_server yeniden başlatılır (Gazebo'ya dokunmadan).
#
# İLK İLKELERDEN BEKLENTİ: yaw_kamera = a + 180 olduğundan azimut ARTIŞI
# kamerayı kendi sağına kaydırıp +da yaw'lar → uzak arka plan SAĞA kayar
# (→ +1). Yükseliş ARTIŞI kamerayı yükseltip aşağı baktırır → arka plan
# YUKARI kayar; "aşağı sürükle → kamera aracın altına alçalır" istendiğine
# göre dy>0 yükselişi AZALTMALI (→ -1).
#
# ⚠ HENÜZ ÖLÇÜLMEDİ — aşağıdaki değerler yalnız TAHMİN. Ölçüm için
# tools/yorunge_isaret_testi.py, Gazebo yörünge modelleriyle açıldıktan sonra.
AZIMUT_ISARETI = +1
YUKSELIS_ISARETI = -1

# ── Sınırlar (GEOMETRİ uzayında; sürükleme uzayına işaretle eşlenir) ────
YUKSELIS_MIN, YUKSELIS_MAKS = -20.0, 85.0
MESAFE_MIN, MESAFE_MAKS = 3.0, 200.0
# Kızak yer altına inmesin: e=-20°, r=3 → z = 0.195 + 3·sin(-20°) = -0.83.
# Bu bir işaret düzeltmesi DEĞİL, adlandırılmış geometrik korumadır.
KAMERA_MIN_Z = 0.30

# Çift tık sıfırlaması: "kamerayı ŞU AN aracın arkasına al".
SIFIRLAMA_YUKSELIS = 20.0
SIFIRLAMA_YARICAP = 15.0
SIFIRLAMA_AZIMUT_OFSET = 180.0

KIPLER = ("dunya", "govde")

# ── Çalışma parametreleri ───────────────────────────────────────────────
_HZ = float(os.environ.get("AVCI_YORUNGE_HZ", "20"))
_HZ_MIN = float(os.environ.get("AVCI_YORUNGE_HZ_MIN", "5"))
# 20 Hz'de 100 ms'den uzun süren bir set_pose zaten değersiz; ayrıca gz
# bağlaması bloklama boyunca GIL'i tutuyorsa maruziyeti sınırlar.
_ISTEK_MS = int(os.environ.get("AVCI_YORUNGE_ISTEK_MS", "100"))
_ESIK_MS = float(os.environ.get("AVCI_YORUNGE_ESIK_MS", "15"))
_LOG_S = float(os.environ.get("AVCI_YORUNGE_LOG_S", "10"))
_YENIDEN_DENE_S = 3.0           # model sahnede yoksa bu aralıkla tekrar dene

# Araç hiç görülmediyse kullanılacak doğuş pozu: "x,y,z,yaw_deg"
_VARSAYILAN = {
    "iris":  os.environ.get("AVCI_YORUNGE_VARSAYILAN_IRIS",  "0,0,0.195,90"),
    "plane": os.environ.get("AVCI_YORUNGE_VARSAYILAN_PLANE", "12,0,0.195,90"),
}

_kilit = threading.Lock()
_calisiyor = False
_isparcaciklari = []


def _dogus_yaw(anahtar):
    """AVCI_YORUNGE_VARSAYILAN_* içindeki 'x,y,z,yaw_deg' dizisinden yaw."""
    try:
        return float(_VARSAYILAN[anahtar].split(",")[3])
    except Exception:
        return 0.0


def _yeni_durum(kip, dogus_yaw):
    # Başlangıç açısı = SDF'teki kızak pozunun ta kendisi (aracın arkası):
    # a_bagil = doğuş yaw + 180. Böylece sürücü ilk tikte kamerayı zıplatmaz.
    # ⚠ _sarmala burada KULLANILAMAZ: modül daha yüklenirken çalışıyoruz,
    # o fonksiyon aşağıdaki geometri bölümünde tanımlı.
    return {
        "azimut": (AZIMUT_ISARETI * (dogus_yaw + SIFIRLAMA_AZIMUT_OFSET)) % 360.0,
        "yukselis": YUKSELIS_ISARETI * SIFIRLAMA_YUKSELIS,
        "yaricap": SIFIRLAMA_YARICAP,
        "kip": kip,
        "surum": 0,
        "hazir": False,
        "sebep": "başlatılmadı",
        "poz_kaynagi": "yok",
        "hz": _HZ,
        "gecikme_ms": 0.0,
        "zorla": False,
    }


_baslangic_kip = os.environ.get("AVCI_YORUNGE_KIP", "dunya")
if _baslangic_kip not in KIPLER:
    _baslangic_kip = "dunya"

_durum = {p: _yeni_durum(_baslangic_kip, _dogus_yaw(MODELLER[p][1]))
          for p in PANELLER}
# Araç pozu hiç gelmemişse bile sıfırlama makul bir açı üretsin diye son
# görülen yaw panel başına saklanır.
_son_yaw = {p: None for p in PANELLER}
_olcum = {p: {"n": 0, "ewma_ms": 0.0, "maks_ms": 0.0, "hata": 0,
              "halka": [], "dongu_ms": 0.0} for p in PANELLER}


# ══ Geometri ════════════════════════════════════════════════════════════

def _quat(pitch, yaw):
    """roll=0 için (w,x,y,z). Gazebo'da POZİTİF pitch AŞAĞI baktırır — mevcut
    chase kamerası bunu doğruluyor (pose ... 0 0.35 0, yorumu '~20° aşağı')."""
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cp * cy, -sp * sy, sp * cy, cp * sy)


def _poz_hesapla(hedef_pos, azimut_deg, yukselis_deg, mesafe):
    """Kamera dünya pozu + hedefe bakan yönelim.

    Kamera hedefin çevresinde küresel koordinatta durur; Gazebo kamerasının
    bakış yönü olan +X ekseni hedefe çevrilir: yaw = azimut + 180°,
    pitch = yükseliş (yukarıdayken aşağı bakar).

    ⚠ 450/450 doğrulandı — bu iki fonksiyona işaret düzeltmesi GİRMEZ."""
    az, el = math.radians(azimut_deg), math.radians(yukselis_deg)
    tx, ty, tz = hedef_pos
    x = tx + mesafe * math.cos(el) * math.cos(az)
    y = ty + mesafe * math.cos(el) * math.sin(az)
    z = tz + mesafe * math.sin(el)
    return (x, y, z), _quat(el, az + math.pi)


def _yaw_derece(quat):
    """(w,x,y,z) → ENU yaw (derece). ZYX/Tait-Bryan, Gazebo pose ile aynı."""
    w, x, y, z = quat
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


def _kelepce(v, alt, ust):
    return max(alt, min(ust, v))


def _sarmala(a):
    return a % 360.0


# ── Sürükleme uzayı ↔ geometri uzayı ────────────────────────────────────
# İstemci MUTLAK durum gönderir ama İŞARETSİZ "sürükleme uzayında" biriktirir.
# Sunucu tek çeviri noktasıdır. ±1 bir involution olduğu için eşleme kendi
# tersidir — kırpmayı geometri uzayında yapıp geri eşlemek KAYIPSIZDIR.
#
#   a_bagil = AZIMUT_ISARETI   * azimut_surukleme      (araca göre bağıl açı)
#   a_geo   = a_bagil + (arac_yaw if kip == "govde" else 0)
#   e_geo   = YUKSELIS_ISARETI * yukselis_surukleme

def _bagil_azimut(d):
    return AZIMUT_ISARETI * d["azimut"]


def _yukselis_geo(d):
    return YUKSELIS_ISARETI * d["yukselis"]


# ══ Durum yazma (kilit altında; gz'ye DOKUNMAZ, BLOKLAMAZ) ══════════════

def durum(panel=None):
    """panel=None → {panel: durum} tümü."""
    with _kilit:
        if panel is None:
            return {p: dict(_durum[p]) for p in PANELLER}
        return dict(_durum[panel]) if panel in _durum else {}


def olcum(panel=None):
    """set_pose zamanlama özeti."""
    def _ozet(p):
        o = _olcum[p]
        halka = sorted(o["halka"])
        p95 = halka[int(len(halka) * 0.95)] if halka else 0.0
        return {"panel": p, "n": o["n"], "ort_ms": round(o["ewma_ms"], 2),
                "p95_ms": round(p95, 2), "maks_ms": round(o["maks_ms"], 2),
                "hata": o["hata"], "hz": _durum[p]["hz"],
                "dongu_ms": round(o["dongu_ms"], 2)}
    with _kilit:
        if panel is None:
            return {p: _ozet(p) for p in PANELLER}
        return _ozet(panel) if panel in _olcum else {}


def ayarla(panel, azimut=None, yukselis=None, yaricap=None, kip=None,
           sifirla=False):
    """MUTLAK durum yazar (delta DEĞİL) ve yankı sözlüğü döndürür.

    ⚠ Yalnız bir kilit alır ve aritmetik yapar — gz'ye dokunmaz, ASLA
    bloklamaz. asyncio olay döngüsünden doğrudan çağrılabilmesinin tek sebebi
    bu: set_pose'un zaman aşımı olay döngüsüne girseydi tek bir takılma MJPEG
    akışını ve /ws telemetrisini de durdururdu.

    ⚠ İstemciye güvenilmez: her alan burada doğrulanır ve kırpılır.
    """
    if panel not in _durum:
        return {"hata": f"bilinmeyen panel: {panel}"}

    with _kilit:
        d = _durum[panel]
        zorla = False
        yaw = _son_yaw[panel]

        # 1) Kip değişimi — GÖRÜNTÜYÜ ZIPLATMAZ: açı, yeni çerçevede AYNI
        #    görüşü verecek şekilde yeniden ifade edilir. Bu bir işaret
        #    düzeltmesi değil, çerçeve dönüşümüdür.
        kip_degisti = False
        if kip is not None and kip in KIPLER and kip != d["kip"]:
            if yaw is None:
                d["sebep"] = "araç yönü bilinmiyor — kip değişmedi"
                zorla = True
            else:
                a_bagil = _bagil_azimut(d)
                a_bagil += -yaw if kip == "govde" else yaw
                d["azimut"] = _sarmala(AZIMUT_ISARETI * a_bagil)
                d["kip"] = kip
                kip_degisti = True
                zorla = True

        # 2) Sıfırlama — tek kural: "kamerayı ŞU AN aracın arkasına al".
        #    dunya : a_bagil = yaw + 180  (anlık dondurma; araç dönünce kamera
        #            dünyada kalır)
        #    govde : a_bagil = 180        (buruna göre; canlı yaw'ı döngü ekler)
        #    ⚠ İkisinde de +180 BİR KEZ sayılır (yaw çift sayılmaz).
        if sifirla:
            if d["kip"] == "govde":
                a_bagil = SIFIRLAMA_AZIMUT_OFSET
            elif yaw is not None:
                a_bagil = yaw + SIFIRLAMA_AZIMUT_OFSET
            else:
                a_bagil = _bagil_azimut(d)      # açıyı koru
                d["sebep"] = "araç yönü bilinmiyor — azimut korundu"
            d["azimut"] = _sarmala(AZIMUT_ISARETI * a_bagil)
            d["yukselis"] = YUKSELIS_ISARETI * SIFIRLAMA_YUKSELIS
            d["yaricap"] = SIFIRLAMA_YARICAP
            zorla = True
        else:
            # 3) Mutlak yazma + kırpma. Kırpma GEOMETRİ uzayında yapılır,
            #    sonra sürükleme uzayına geri eşlenir (involution → kayıpsız).
            #
            # ⚠ Kip DEĞİŞTİYSE gelen azimut YOK SAYILIR. İstemci her tikte
            # bütün durumu gönderiyor; kip değişimiyle aynı pakette gelen
            # azimut, yukarıdaki çerçeve dönüşümünü ezip görüntüyü zıplatırdı.
            # Doğru sahip sunucudur: zorla=True ile yankılanır, istemci
            # sunucunun değerini benimser.
            if azimut is not None and not kip_degisti:
                try:
                    d["azimut"] = _sarmala(float(azimut))
                except (TypeError, ValueError):
                    pass
            if yukselis is not None:
                try:
                    e_geo = YUKSELIS_ISARETI * float(yukselis)
                    e_kirpik = _kelepce(e_geo, YUKSELIS_MIN, YUKSELIS_MAKS)
                    if abs(e_kirpik - e_geo) > 1e-9:
                        zorla = True        # kırptık → istemci bizi benimsesin
                    d["yukselis"] = YUKSELIS_ISARETI * e_kirpik
                except (TypeError, ValueError):
                    pass
            if yaricap is not None:
                try:
                    r = float(yaricap)
                    r_kirpik = _kelepce(r, MESAFE_MIN, MESAFE_MAKS)
                    if abs(r_kirpik - r) > 1e-9:
                        zorla = True
                    d["yaricap"] = r_kirpik
                except (TypeError, ValueError):
                    pass

        d["surum"] += 1
        d["zorla"] = zorla
        yanki = dict(d)
        yanki["panel"] = panel
        return yanki


# ══ Araç pozu — üç kademeli yedek ═══════════════════════════════════════

def _varsayilan_poz(anahtar):
    try:
        x, y, z, yaw = (float(s) for s in _VARSAYILAN[anahtar].split(","))
        return {"pos": (x, y, z)}, yaw
    except Exception:
        return {"pos": (0.0, 0.0, 0.195)}, 0.0


def _arac_pozu(sim_truth, anahtar):
    """(arac, yaw_deg, kaynak) döndürür.

    1) taze      : pozlar(0.5)
    2) bayat     : pozlar(çok büyük max_yas) — dynamic_pose/info yalnız
                   HAREKETLİ modelleri taşır, yani park hâlindeki araç veri
                   yayınlamaz. Duran aracın SON pozu ZATEN onun pozudur;
                   pozlar() max_yas parametresini kabul ettiği için bu yedek
                   sim_truth'ta SIFIR değişiklik gerektirir.
    3) varsayilan: hiç görülmediyse doğuş pozu (Gazebo yeni açıldı).
    """
    for max_yas, kaynak in ((0.5, "taze"), (1e9, "bayat")):
        try:
            pozlar = sim_truth.pozlar(max_yas)
        except Exception:
            pozlar = None
        if pozlar:
            arac = pozlar.get(anahtar)
            if arac and arac.get("pos"):
                yaw = _yaw_derece(arac["quat"]) if arac.get("quat") else None
                return arac, yaw, kaynak
    arac, yaw = _varsayilan_poz(anahtar)
    return arac, yaw, "varsayilan"


# ══ Sürücü döngüsü — kamera başına bir iş parçacığı ═════════════════════

def _dongu(panel):
    """⚠ Kamera başına AYRI iş parçacığı. node.request BLOKLAYAN bir çağrı;
    tek iş parçacığı ikisini de sürseydi bir zaman aşımı diğer kamerayı da
    geciktirirdi (head-of-line blocking). Ayrıca bir kızak sahnede yoksa
    diğeri donmasın diye hata yalıtımı gerekiyor."""
    try:
        from gz.transport13 import Node
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.boolean_pb2 import Boolean
    except Exception as e:
        with _kilit:
            _durum[panel]["sebep"] = f"gz-transport Python yok: {e}"
        print(f"[YÖRÜNGE] gz-transport yok, {panel} pasif: {e}")
        return

    from control import sim_truth

    model, anahtar = MODELLER[panel]
    node = Node()
    servis = f"/world/{DUNYA}/set_pose"
    o = _olcum[panel]

    hz = _HZ
    son_hata_t = 0.0
    son_log_t = time.time()
    ardisik_hata = 0
    yavas_baslangic = None
    temiz_baslangic = time.time()
    pencere_t = time.time()
    pencere_toplam_ms = 0.0

    while _calisiyor:
        dongu_t0 = time.perf_counter()
        arac, yaw, kaynak = _arac_pozu(sim_truth, anahtar)

        with _kilit:
            d = _durum[panel]
            if yaw is not None:
                _son_yaw[panel] = yaw
            a_bagil = _bagil_azimut(d)
            a_geo = (a_bagil + (yaw or 0.0)) if d["kip"] == "govde" else a_bagil
            e_geo = _kelepce(_yukselis_geo(d), YUKSELIS_MIN, YUKSELIS_MAKS)
            r = d["yaricap"]
            d["poz_kaynagi"] = kaynak

        (x, y, z), (qw, qx, qy, qz) = _poz_hesapla(arac["pos"], a_geo, e_geo, r)
        z = max(z, KAMERA_MIN_Z)

        msg = Pose()
        msg.name = model
        msg.position.x, msg.position.y, msg.position.z = x, y, z
        msg.orientation.w, msg.orientation.x = qw, qx
        msg.orientation.y, msg.orientation.z = qy, qz

        basarili = False
        t0 = time.perf_counter()
        try:
            # ⚠ İMZA: request() (sonuc_bool, cevap_mesaji) döndürür — bu sırayla.
            # vision/capture_dataset.py:126 ve capture_runway_negatives.py:99
            # bunu `_, ok = ...` diye TERS bağlamış; protobuf mesajı her zaman
            # truthy olduğundan o iki betik reddedilen set_pose'u hiç fark
            # edemiyor. O kalıp buraya kopyalanmadı.
            ok, rep = node.request(servis, msg, Pose, Boolean, _ISTEK_MS)
            # rep.data kontrolü "Gazebo cevap verdi ama sahnede bu model yok"
            # (eski dünya hâlâ koşuyor) durumunu ayırt eden şeydir.
            basarili = bool(ok) and bool(rep.data)
        except Exception as e:
            if time.time() - son_hata_t > 10:
                son_hata_t = time.time()
                print(f"[YÖRÜNGE] {panel} set_pose hatası: {e}")
        gecikme_ms = (time.perf_counter() - t0) * 1000.0

        with _kilit:
            d = _durum[panel]
            if basarili and not d["hazir"]:
                print(f"[YÖRÜNGE] ✓ {panel} kızağı sürülüyor "
                      f"({model} @ /world/{DUNYA})")
            d["hazir"] = basarili
            d["gecikme_ms"] = round(gecikme_ms, 2)
            d["hz"] = hz
            if not basarili:
                d["sebep"] = ("kızak modeli sahnede yok — Gazebo, yörünge "
                              "kameraları eklenmiş dünyayla yeniden başlatılmalı")
            elif d["sebep"]:
                d["sebep"] = ""
            o["n"] += 1
            o["ewma_ms"] = (0.9 * o["ewma_ms"] + 0.1 * gecikme_ms
                            if o["n"] > 1 else gecikme_ms)
            o["maks_ms"] = max(o["maks_ms"], gecikme_ms)
            o["halka"].append(gecikme_ms)
            if len(o["halka"]) > 200:
                del o["halka"][0]
            if not basarili:
                o["hata"] += 1

        simdi = time.time()

        # ── Hız düşürme merdiveni: 20 → 10 → 5 Hz ──────────────────────
        pencere_toplam_ms += gecikme_ms
        if simdi - pencere_t >= 1.0:
            if pencere_toplam_ms > 250.0 and hz > _HZ_MIN:
                print(f"[YÖRÜNGE] {panel} hız düşürüldü {hz:.0f}→{_HZ_MIN:.0f} Hz "
                      f"(1 s'de toplam {pencere_toplam_ms:.0f} ms set_pose, "
                      f"%25 görev süresi sınırı aşıldı)")
                hz = _HZ_MIN
            pencere_t, pencere_toplam_ms = simdi, 0.0

        if basarili:
            ardisik_hata = 0
        else:
            ardisik_hata += 1

        yavas = o["ewma_ms"] > _ESIK_MS
        if yavas:
            yavas_baslangic = yavas_baslangic or simdi
            temiz_baslangic = simdi
        else:
            yavas_baslangic = None

        dusur = ((yavas and yavas_baslangic and simdi - yavas_baslangic >= 2.0)
                 or ardisik_hata >= 3)
        if dusur and hz > _HZ_MIN:
            yeni = max(_HZ_MIN, hz / 2.0)
            sebep = (f"ort {o['ewma_ms']:.1f} ms > {_ESIK_MS:.0f} ms eşiği, "
                     f"{simdi - yavas_baslangic:.1f} s boyunca" if yavas
                     else f"{ardisik_hata} ardışık hata")
            print(f"[YÖRÜNGE] {panel} hız düşürüldü {hz:.0f}→{yeni:.0f} Hz ({sebep})")
            hz, yavas_baslangic, ardisik_hata = yeni, None, 0
        elif (not yavas and ardisik_hata == 0 and hz < _HZ
              and simdi - temiz_baslangic >= 20.0):
            yeni = min(_HZ, hz * 2.0)
            print(f"[YÖRÜNGE] {panel} hız yükseltildi {hz:.0f}→{yeni:.0f} Hz "
                  f"(ort {o['ewma_ms']:.1f} ms, 20 s temiz)")
            hz, temiz_baslangic = yeni, simdi

        # ── Dönemsel zamanlama logu ────────────────────────────────────
        if simdi - son_log_t >= _LOG_S and o["n"] > 0:
            son_log_t = simdi
            ol = olcum(panel)
            print(f"[YÖRÜNGE] {panel:12s} {hz:.0f} Hz | set_pose n={ol['n']} "
                  f"ort {ol['ort_ms']:.1f} ms p95 {ol['p95_ms']:.1f} ms "
                  f"maks {ol['maks_ms']:.1f} ms | hata {ol['hata']} | "
                  f"döngü {ol['dongu_ms']:.1f} ms | poz: {kaynak}")

        dongu_ms = (time.perf_counter() - dongu_t0) * 1000.0
        with _kilit:
            o["dongu_ms"] = dongu_ms

        # Model sahnede yoksa her karede servis çağırıp Gazebo'yu yormayalım.
        if basarili:
            time.sleep(max(0.0, (1.0 / hz) - dongu_ms / 1000.0))
        else:
            time.sleep(_YENIDEN_DENE_S)


# ══ Yaşam döngüsü ═══════════════════════════════════════════════════════

def baslat():
    """Panel başına BİR sürücü iş parçacığı başlatır. AVCI_YORUNGE=0 kapatır.

    gz-transport yoksa sessizce False döner — gcs_server'ı ASLA düşürmez."""
    global _calisiyor
    if os.environ.get("AVCI_YORUNGE", "1") != "1":
        with _kilit:
            for p in PANELLER:
                _durum[p]["sebep"] = "AVCI_YORUNGE=0 ile kapatıldı"
        print("[YÖRÜNGE] AVCI_YORUNGE=0 — kameralar SDF başlangıç pozunda kalır")
        return False
    if _calisiyor:
        return True
    _calisiyor = True
    for p in PANELLER:
        t = threading.Thread(target=_dongu, args=(p,), daemon=True,
                             name=f"yorunge-{p}")
        t.start()
        _isparcaciklari.append(t)
    print(f"[YÖRÜNGE] {len(PANELLER)} kamera sürücüsü başladı "
          f"({_HZ:.0f} Hz, istek {_ISTEK_MS} ms, dünya '{DUNYA}')")
    return True


def durdur():
    """⚠ Bağlantı kopması BURAYI ÇAĞIRMAZ. Kamera son açısında KALIR;
    /ws/kamera kapanışında sıfırlama YOKTUR — bu bilinçli bir karardır."""
    global _calisiyor
    _calisiyor = False
