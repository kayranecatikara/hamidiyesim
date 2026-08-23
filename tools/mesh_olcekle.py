#!/usr/bin/env python3
"""
tools/mesh_olcekle.py — TALON MESH ÖLÇEKLEME (Mini Talon → büyük X-UAV Talon)

    python3 tools/mesh_olcekle.py                        # KONTROL (yazmaz)
    python3 tools/mesh_olcekle.py --olcek 1.342          # KONTROL, farkı gösterir
    python3 tools/mesh_olcekle.py --olcek 1.342 --uygula # yedekle + yaz
    python3 tools/mesh_olcekle.py --geri                 # yedekten geri al

════════════════════════════════════════════════════════════════════════
NEDEN VAR
════════════════════════════════════════════════════════════════════════
Gazebo'daki hedef, gerçek X-UAV **Mini** Talon (1300 mm) ölçülerinde.
Hedeflenen araç gerçek X-UAV **Talon** (1718 mm). Ölçek çarpanı:

    s = 1718 / 1280 = 1.342          (ölçülen mesh açıklığı 1.280 m)

En/boy oranı üç uçakta da aynı (1.572 / 1.566 / 1.562, %0.7 içinde), bu
yüzden DÜZGÜN (uniform) ölçekleme yeterli — parça yeniden modelleme yok.

════════════════════════════════════════════════════════════════════════
NİYE `<scale>` ETİKETİ KULLANMIYORUZ  (⚠ sessiz bozulma tuzağı)
════════════════════════════════════════════════════════════════════════
`model.sdf`'e `<scale>1.342 1.342 1.342</scale>` yazmak kolay olurdu.
AMA `vision/geometry.py:_stl_vertices()` collision STL'lerini DOĞRUDAN
okuyor; SDF'i hiç görmüyor. O yolu seçersek:

    Gazebo   → 1.718 m'lik uçak çizer ve çarpıştırır
    geometry → 1.280 m'lik uçağın 3B kutusunu üretir

ve HİÇBİR HATA MESAJI ÇIKMAZ. Otomatik etiketleme ile poz doğrulaması
sessizce yanlış olur. Aynı gerekçe DAE'deki `<unit meter="...">` için de
geçerli: STL'de böyle bir kavram yok, ikisi ayrışır.

⇒ Bu script köşe koordinatlarının KENDİSİNİ ölçekler. Yükleyicinin ne
   yaptığından bağımsızdır ve görsel ile çarpışma gövdesi hep eş kalır.

════════════════════════════════════════════════════════════════════════
NE ÖLÇEKLENİR, NE ÖLÇEKLENMEZ
════════════════════════════════════════════════════════════════════════
STL (ikili/binary, çarpışma gövdesi):
  ✓ üçgen köşeleri (üçgen başına 9 float)
  ✗ üçgen normalleri — DÜZGÜN ölçekte normal yönü DEĞİŞMEZ; ölçeklersek
    birim uzunluk bozulur ve bazı fizik/çizim yolları bunu yanlış okur

DAE (COLLADA XML, görsel):
  ✓ `<input semantic="POSITION">` ile işaret edilen `<float_array>`'ler
  ✓ `<matrix sid="transform">` içindeki ÖTELEME sütunu (indeks 3, 7, 11)
    — 4×4 matriste ilk 3 sütun dönme/ölçek, 4. sütun konumdur. Bu
    ölçeklenmezse parçalar büyür ama eski yerlerinde kalır → uçak dağılır
  ✓ `<translate>` düğümleri (bu dosyalarda yok, savunma amaçlı)
  ✗ NORMAL ve TEXCOORD kaynakları
  ✗ `<unit meter>` — yukarıdaki gerekçe

DOKUNULMAYANLAR (Talon gövdesi değil, ayrı karar konusu):
  propdrive_3536_*.dae, iris_prop_cw.dae  — motor ve pervane. Gerçek büyük
  Talon 3542 motor kullanıyor (mini'de 2814); bu ayrı bir iş. Sadece
  görsel, aerodinamiği etkilemiyor.

════════════════════════════════════════════════════════════════════════
İKİ KEZ ÇALIŞTIRMA GÜVENLİĞİ
════════════════════════════════════════════════════════════════════════
Script HER ZAMAN yedekteki ORİJİNAL (s=1.0) dosyadan okur, ölçekleyip
hedefe yazar. Bu yüzden:
  - iki kez çalıştırmak 1.342² = 1.80 vermez, yine 1.342 verir
  - ölçeği değiştirmek için önce `--geri` yapmak gerekmez
İlk `--uygula` çağrısında orijinaller `_mesh_yedek/` altına kopyalanır ve
BİR DAHA ASLA üzerine yazılmaz.

════════════════════════════════════════════════════════════════════════
⚠ BU SCRIPT TEK BAŞINA YETMEZ
════════════════════════════════════════════════════════════════════════
Mesh'ler büyür ama `model.sdf` içindeki link `<pose>` değerleri (aileron
ve ruddervator menteşeleri, tekerlek, motor, kamera) ESKİ yerinde kalır →
kontrol yüzeyleri gövdenin içine gömülür. Ayrıca `<mass>`, `<inertia>` ve
LiftDrag `<area>`/`<cp>` de elle güncellenmeli. Sıradaki adım o.
"""

import argparse
import glob
import math
import os
import re
import shutil
import struct
import sys
import xml.etree.ElementTree as ET

KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_KOK = os.path.join(KOK, "sim", "gazebo_harmonic", "models")
YEDEK_KOK = os.path.join(MODEL_KOK, "_mesh_yedek")

MODELLER = ["mini_talon_vtail", "mini_talon_target"]
DESENLER = ["mini_talon_*.stl", "mini_talon_*.dae"]

COLLADA_NS = "{http://www.collada.org/2005/11/COLLADASchema}"


# ══════════════════════════════════════════════════════════════════════
# sayı biçimlendirme
# ══════════════════════════════════════════════════════════════════════
def _fmt(v):
    """Float → COLLADA metni. 7 anlamlı basamak; konumlar metre cinsinden
    ~0.9 m'yi geçmediği için bu 0.1 mikrometre çözünürlük demektir —
    Blender'ın kendi ihracat hassasiyetiyle aynı."""
    if v == 0.0:
        return "0"
    return f"{v:.7g}"


# ══════════════════════════════════════════════════════════════════════
# ikili STL
# ══════════════════════════════════════════════════════════════════════
def stl_oku(yol):
    """İkili STL → (başlık80, üçgen_sayısı, ham_bayt). ASCII ise hata."""
    with open(yol, "rb") as f:
        ham = f.read()
    if ham[:5].lower().lstrip() == b"solid" and b"facet normal" in ham[:2048]:
        raise ValueError(f"{yol}: ASCII STL — bu script yalnız ikili STL yazar")
    if len(ham) < 84:
        raise ValueError(f"{yol}: 84 bayttan kısa, STL değil")
    n = struct.unpack("<I", ham[80:84])[0]
    beklenen = 84 + n * 50
    if len(ham) != beklenen:
        raise ValueError(
            f"{yol}: boy tutmuyor — {n} üçgen için {beklenen} bayt beklenir, "
            f"{len(ham)} var")
    return ham, n


def stl_kose_kutusu(yol):
    """STL'in eksen-hizalı sınırlayıcı kutusu → (minv, maxv), her biri 3'lü."""
    ham, n = stl_oku(yol)
    mn = [math.inf] * 3
    mx = [-math.inf] * 3
    for i in range(n):
        o = 84 + i * 50 + 12                      # +12: normali atla
        for v in range(3):
            xyz = struct.unpack("<3f", ham[o + v * 12: o + v * 12 + 12])
            for k in range(3):
                if xyz[k] < mn[k]:
                    mn[k] = xyz[k]
                if xyz[k] > mx[k]:
                    mx[k] = xyz[k]
    return mn, mx


def stl_olcekle(kaynak, hedef, s):
    """İkili STL'i ölçekleyip yaz. Normaller DEĞİŞMEZ (düzgün ölçek)."""
    ham, n = stl_oku(kaynak)
    buf = bytearray(ham)
    for i in range(n):
        o = 84 + i * 50 + 12
        for v in range(3):
            p = o + v * 12
            x, y, z = struct.unpack("<3f", buf[p:p + 12])
            struct.pack_into("<3f", buf, p, x * s, y * s, z * s)
    with open(hedef, "wb") as f:
        f.write(buf)
    return n


# ══════════════════════════════════════════════════════════════════════
# COLLADA (.dae)
# ══════════════════════════════════════════════════════════════════════
def dae_konum_dizileri(metin):
    """POSITION anlamıyla işaret edilen <float_array> id'lerini bul.

    Zincir:  <input semantic="POSITION" source="#SRC"/>
             <source id="SRC"> <float_array id="ARR"> ... </float_array>
    NORMAL / TEXCOORD kaynakları bilerek DIŞARIDA bırakılır."""
    kok = ET.fromstring(metin)

    konum_kaynaklari = set()
    for inp in kok.iter(f"{COLLADA_NS}input"):
        if inp.get("semantic") == "POSITION":
            src = inp.get("source", "")
            if src.startswith("#"):
                konum_kaynaklari.add(src[1:])

    dizi_idleri = set()
    for src in kok.iter(f"{COLLADA_NS}source"):
        if src.get("id") in konum_kaynaklari:
            for fa in src.iter(f"{COLLADA_NS}float_array"):
                if fa.get("id"):
                    dizi_idleri.add(fa.get("id"))
    return dizi_idleri, len(konum_kaynaklari)


def dae_olcekle(kaynak, hedef, s):
    """DAE'yi ölçekle. XML'i yeniden ÜRETMEZ — yalnız ilgili sayı
    bloklarını metin üzerinde değiştirir. Böylece dosyanın geri kalanı
    (yorumlar, biçim, sıralama) bayt bayt korunur."""
    with open(kaynak, "r", encoding="utf-8") as f:
        metin = f.read()

    dizi_idleri, n_kaynak = dae_konum_dizileri(metin)
    sayac = {"dizi": 0, "float": 0, "matris": 0, "translate": 0}

    # ── 1) POSITION float_array'leri ──
    def _dizi_degistir(m):
        icerik = m.group(2)
        parcalar = icerik.split()
        yeni = " ".join(_fmt(float(p) * s) for p in parcalar)
        sayac["dizi"] += 1
        sayac["float"] += len(parcalar)
        return m.group(1) + yeni + m.group(3)

    for aid in sorted(dizi_idleri):
        kalip = re.compile(
            r'(<float_array\b[^>]*\bid="' + re.escape(aid) + r'"[^>]*>)'
            r'(.*?)'
            r'(</float_array>)', re.DOTALL)
        metin, k = kalip.subn(_dizi_degistir, metin)
        if k != 1:
            raise ValueError(
                f"{kaynak}: float_array id={aid} için {k} eşleşme (1 bekleniyordu)")

    # ── 2) <matrix> öteleme sütunu (indeks 3, 7, 11) ──
    def _matris_degistir(m):
        parcalar = m.group(2).split()
        if len(parcalar) != 16:
            return m.group(0)                     # 4×4 değilse dokunma
        d = [float(p) for p in parcalar]
        for i in (3, 7, 11):
            d[i] *= s
        sayac["matris"] += 1
        return m.group(1) + " ".join(_fmt(v) for v in d) + m.group(3)

    metin = re.sub(r'(<matrix\b[^>]*>)(.*?)(</matrix>)', _matris_degistir,
                   metin, flags=re.DOTALL)

    # ── 3) <translate> düğümleri (bu dosyalarda yok, savunma amaçlı) ──
    def _translate_degistir(m):
        parcalar = m.group(2).split()
        if len(parcalar) != 3:
            return m.group(0)
        sayac["translate"] += 1
        return m.group(1) + " ".join(
            _fmt(float(p) * s) for p in parcalar) + m.group(3)

    metin = re.sub(r'(<translate\b[^>]*>)(.*?)(</translate>)',
                   _translate_degistir, metin, flags=re.DOTALL)

    # ── yazmadan önce hâlâ geçerli XML mi ──
    ET.fromstring(metin)

    with open(hedef, "w", encoding="utf-8") as f:
        f.write(metin)
    sayac["kaynak"] = n_kaynak
    return sayac


# ══════════════════════════════════════════════════════════════════════
# dosya listesi ve yedek
# ══════════════════════════════════════════════════════════════════════
def dosyalari_bul():
    """[(model, dosya_adi, calisma_yolu, yedek_yolu)] — sıralı."""
    out = []
    for model in MODELLER:
        mdir = os.path.join(MODEL_KOK, model, "meshes")
        if not os.path.isdir(mdir):
            raise SystemExit(f"⛔ dizin yok: {mdir}")
        bulunan = set()
        for desen in DESENLER:
            bulunan.update(glob.glob(os.path.join(mdir, desen)))
        for yol in sorted(bulunan):
            ad = os.path.basename(yol)
            out.append((model, ad, yol,
                        os.path.join(YEDEK_KOK, model, ad)))
    return out


def haric_tutulanlar():
    """Ölçeklenmeyen dosyalar — görünür olsun diye ayrıca listelenir."""
    out = []
    for model in MODELLER:
        mdir = os.path.join(MODEL_KOK, model, "meshes")
        if not os.path.isdir(mdir):
            continue
        alinan = set()
        for desen in DESENLER:
            alinan.update(os.path.basename(p)
                          for p in glob.glob(os.path.join(mdir, desen)))
        for ad in sorted(os.listdir(mdir)):
            if ad.lower().endswith((".dae", ".stl")) and ad not in alinan:
                out.append((model, ad))
    return out


def yedek_al(dosyalar):
    """Orijinalleri bir kez kopyala. Var olan yedeğin ÜZERİNE YAZMAZ."""
    yeni, mevcut = 0, 0
    for model, ad, calisma, yedek in dosyalar:
        os.makedirs(os.path.dirname(yedek), exist_ok=True)
        if os.path.exists(yedek):
            mevcut += 1
        else:
            shutil.copy2(calisma, yedek)
            yeni += 1
    return yeni, mevcut


def kaynak_yolu(calisma, yedek):
    """Ölçekleme HER ZAMAN orijinalden yapılır → iki kez çalıştırmak
    çarpanı kareye çıkarmaz."""
    return yedek if os.path.exists(yedek) else calisma


# ══════════════════════════════════════════════════════════════════════
# ölçüm raporu
# ══════════════════════════════════════════════════════════════════════
# (dosya, eksen, işaret) — collision STL'lerden gövde ölçüleri
def gövde_olculeri(model_dir):
    """Talon'un açıklık / boy / yükseklik ölçüleri, collision STL'lerden."""
    m = os.path.join(model_dir, "meshes")

    def kutu(ad):
        return stl_kose_kutusu(os.path.join(m, f"mini_talon_{ad}_collision.stl"))

    fus_mn, fus_mx = kutu("fuselage")
    lw_mn, lw_mx = kutu("left_wing")
    rw_mn, rw_mx = kutu("right_wing")
    lt_mn, lt_mx = kutu("left_tail")
    rt_mn, rt_mx = kutu("right_tail")

    aciklik = lw_mx[1] - rw_mn[1]                       # Y: sol uç − sağ uç
    boy = fus_mx[0] - min(fus_mn[0], lt_mn[0], rt_mn[0])   # X: burun − kuyruk
    yukseklik = max(fus_mx[2], lt_mx[2], rt_mx[2]) - min(fus_mn[2], lt_mn[2])
    return aciklik, boy, yukseklik


def olcu_yaz(baslik, model_dir):
    a, b, y = gövde_olculeri(model_dir)
    print(f"    {baslik:22s} açıklık {a:6.3f} m   boy {b:6.3f} m   "
          f"yükseklik {y:6.3f} m   (en/boy {a / b:.3f})")
    return a, b, y


# ══════════════════════════════════════════════════════════════════════
# ana akış
# ══════════════════════════════════════════════════════════════════════
def komut_geri():
    dosyalar = dosyalari_bul()
    if not os.path.isdir(YEDEK_KOK):
        raise SystemExit(f"⛔ yedek dizini yok: {YEDEK_KOK} — geri alınacak bir şey yok")
    n = 0
    for model, ad, calisma, yedek in dosyalar:
        if os.path.exists(yedek):
            shutil.copy2(yedek, calisma)
            n += 1
    print(f"✓ {n} dosya yedekten geri alındı (orijinal ölçek s = 1.0)")
    print()
    olcu_yaz("geri alınmış hâl", os.path.join(MODEL_KOK, "mini_talon_vtail"))
    print()
    print("⚠ model.sdf'i de elle geri almayı unutma (kütle/atalet/LiftDrag/pose).")


def komut_olcekle(s, uygula):
    dosyalar = dosyalari_bul()
    haric = haric_tutulanlar()
    vtail_dir = os.path.join(MODEL_KOK, "mini_talon_vtail")

    print("═" * 74)
    print(f"TALON MESH ÖLÇEKLEME   —   s = {s:.4f}"
          f"        {'UYGULA' if uygula else 'KONTROL (yazılmayacak)'}")
    print("═" * 74)

    # ── ölçüler: çalışma dosyası ve (varsa) yedekteki orijinal ──
    # Ölçekleme HER ZAMAN orijinalden yapılır; bu yüzden hedef ölçüler de
    # orijinalden hesaplanır. Yedek yoksa çalışma dosyası zaten orijinaldir.
    yedek_vtail = os.path.join(YEDEK_KOK, "mini_talon_vtail")
    yedek_var = os.path.exists(
        os.path.join(yedek_vtail, "meshes", "mini_talon_fuselage_collision.stl"))

    print("\n── ÖLÇÜLER ──")
    olcu_yaz("çalışma dizini", vtail_dir)
    if yedek_var:
        oa, ob, oy = olcu_yaz("yedek (orijinal)", yedek_vtail)
        print("    (ölçekleme yedekten yapılır → iki kez çalıştırmak çarpanı"
              " kareye ÇIKARMAZ)")
    else:
        oa, ob, oy = gövde_olculeri(vtail_dir)
        print("    (yedek yok — ilk çalıştırma; çalışma dosyaları orijinal"
              " kabul ediliyor)")

    print(f"\n── HEDEF (s = {s:.4f}, orijinale uygulanınca) ──")
    print(f"    {'':22s} açıklık {oa * s:6.3f} m   boy {ob * s:6.3f} m   "
          f"yükseklik {oy * s:6.3f} m   (en/boy {oa / ob:.3f} — DEĞİŞMEZ)")
    print(f"    {'gerçek X-UAV Talon':22s} açıklık  1.718 m   boy  1.100 m")
    print(f"    {'sapma':22s} açıklık {(oa * s - 1.718) / 1.718 * 100.0:+6.2f} %"
          f"   boy {(ob * s - 1.100) / 1.100 * 100.0:+6.2f} %")

    print(f"\n── ÖLÇEKLENECEK DOSYALAR ({len(dosyalar)}) ──")
    for model, ad, calisma, yedek in dosyalar:
        kb = os.path.getsize(calisma) / 1024.0
        print(f"    {model:18s} {ad:38s} {kb:8.0f} KB")

    if haric:
        print(f"\n── DOKUNULMAYACAK ({len(haric)}) — Talon gövdesi değil ──")
        for model, ad in haric:
            print(f"    {model:18s} {ad}")

    if not uygula:
        print("\n" + "─" * 74)
        print("KONTROL modu — hiçbir dosya değişmedi.")
        print(f"Uygulamak için:  python3 tools/mesh_olcekle.py --olcek {s} --uygula")
        return

    # ⛔ ÇİFT ÖLÇEKLEME KORUMASI
    # Yedek yoksa çalışma dosyaları "orijinal" kabul edilir. Ama depoda
    # ölçeklenmiş mesh'ler duruyorsa (taze klon, ya da yedek silinmişse)
    # bu varsayım YANLIŞTIR ve ikinci kez ölçekleme s² verir.
    if not yedek_var and oa > 1.5:
        raise SystemExit(
            f"\n⛔ DUR — yedek yok ama mevcut açıklık {oa:.3f} m.\n"
            "   Mini Talon 1.280 m'dir; bu dosyalar ZATEN ölçeklenmiş görünüyor.\n"
            "   Tekrar ölçeklemek çarpanı kareye çıkarır.\n"
            "   Orijinali git'ten al:\n"
            "     git log --oneline -- sim/gazebo_harmonic/models/mini_talon_vtail/meshes\n"
            "     git checkout <olcekleme_oncesi_commit> -- \\\n"
            "       sim/gazebo_harmonic/models/mini_talon_vtail/meshes \\\n"
            "       sim/gazebo_harmonic/models/mini_talon_target/meshes\n")

    print("\n── YEDEK ──")
    yeni, mevcut = yedek_al(dosyalar)
    print(f"    {YEDEK_KOK}")
    print(f"    yeni kopyalanan {yeni}, zaten duran {mevcut} "
          f"(mevcut yedeğin üzerine YAZILMAZ — hep s=1.0 kalır)")

    print("\n── YAZILIYOR ──")
    for model, ad, calisma, yedek in dosyalar:
        kaynak = kaynak_yolu(calisma, yedek)
        if ad.endswith(".stl"):
            n = stl_olcekle(kaynak, calisma, s)
            print(f"    {model:18s} {ad:38s} STL  {n:6d} üçgen")
        else:
            c = dae_olcekle(kaynak, calisma, s)
            print(f"    {model:18s} {ad:38s} DAE  "
                  f"{c['kaynak']:2d} POSITION kaynağı, {c['float']:7d} koordinat, "
                  f"{c['matris']:3d} matris, {c['translate']} translate")

    print("\n── DOĞRULAMA (yeniden ölçüldü) ──")
    a2, b2, y2 = olcu_yaz("yazılan hâl", vtail_dir)
    bekle = oa * s
    if abs(a2 - bekle) / bekle > 1e-4:
        print(f"    ⛔ beklenen açıklık {bekle:.4f} m, çıkan {a2:.4f} m —"
              " ölçekleme eksik uygulandı, --geri yap")
    hata = abs(a2 - 1.718) / 1.718 * 100.0
    if hata < 2.0:
        print(f"    ✓ açıklık gerçek Talon'a %{hata:.2f} yakın")
    else:
        print(f"    ⛔ açıklık %{hata:.2f} sapıyor — beklenmeyen sonuç, --geri yap")

    print("\n" + "─" * 74)
    print("⚠ SIRADAKİ ADIM — bu script model.sdf'e DOKUNMADI:")
    print("    • link <pose> değerleri (aileron/ruddervator menteşe, tekerlek,")
    print("      motor, kamera) hâlâ eski yerinde → kontrol yüzeyleri gövdenin")
    print("      içinde kalır")
    print("    • <mass> 1.8 → 2.75 kg,  <inertia> ×2.753")
    print("    • LiftDrag <area> 0.16 → 0.30 (kanat), 0.02 → 0.0375 (V-kuyruk)")
    print("    • LiftDrag <cp> ×1.342")
    print("    • bbox_ibvs MENZIL_PX_M_CARPIM 185.7 → ~249.2 (sonra ÖLÇÜLECEK)")
    print("\nGeri almak için:  python3 tools/mesh_olcekle.py --geri")


def main():
    ap = argparse.ArgumentParser(
        description="Talon mesh'lerini düzgün (uniform) ölçekler.")
    ap.add_argument("--olcek", type=float, default=1718.0 / 1280.0,
                    help="ölçek çarpanı (varsayılan 1718/1280 = 1.3422)")
    ap.add_argument("--uygula", action="store_true",
                    help="gerçekten yaz (verilmezse yalnız rapor)")
    ap.add_argument("--geri", action="store_true",
                    help="yedekten orijinalleri geri al")
    a = ap.parse_args()

    if a.geri:
        komut_geri()
        return
    if a.olcek <= 0:
        raise SystemExit("⛔ ölçek pozitif olmalı")
    komut_olcekle(a.olcek, a.uygula)


if __name__ == "__main__":
    main()
