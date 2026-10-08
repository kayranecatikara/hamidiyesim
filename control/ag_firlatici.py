#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
AĞ FIRLATICI — VAKKAS'ın taret namlusundan ağ atma mekanizması  (SAF PYTHON)
================================================================================
Kullanıcı kuralı (2026-09-06): *"ağ fırlatıcı taşınsın, C++ falan istemem
python olsun."*  Bu yüzden `vakkas-entegre` dalındaki C++ gz-sim eklentileri
(NetLauncherPlugin / NetCapturePlugin) TAŞINMADI. Aynı iş burada yapılıyor:

  • balistik  → bu dosyada Python ile entegre edilir (yerçekimi + hava
                direnci + ağın kademeli açılması)
  • görselleştirme → Gazebo'daki `ag_konisi` modeli her adımda
                `/world/<dünya>/set_pose` servisiyle taşınır
  • yakalama  → hedefin ağ ağzının içine girip girmediği GEOMETRİK olarak
                burada sınanır

⛔ BUNUN BEDELİ AÇIKÇA SÖYLENİR:
  Ağ, Gazebo'nun fizik motorunda RİJİT BİR CİSİM DEĞİLDİR. Bir şeye çarpınca
  sekmez, dolanmaz, hedefe kuvvet uygulamaz. Sorduğumuz soru "ağ hedefi
  sardı mı" olduğu için bu yeterlidir ve ölçmesi daha temizdir; ama "ağ
  pervaneye dolandı mı" sorusu bu modelle CEVAPLANAMAZ.

────────────────────────── BALİSTİK MODEL ──────────────────────────────────
Ağ, namlu ucundan çıkar. İlk hızı iki bileşenin toplamıdır:

    v0 = v_arac + NAMLU_HIZI · n̂        (n̂ = namlu birim doğrultusu)

⚠ `v_arac` TOPLANMAK ZORUNDADIR: 18 m/s giden bir araçtan 20 m/s'lik atış
  yere göre 38 m/s eder. Unutulursa menzil yarıya iner.

Her adımda ivme:

    a = g⃗ − (½·ρ·C_d·A(t)·|v⃗|·v⃗) / m

  ρ   : hava yoğunluğu, 1.225 kg/m³ (deniz seviyesi)
  C_d : ETKİN sürükleme katsayısı — aşağıya bak, gözeneklilik kritiktir
  A(t): ağın ANLIK kesit alanı — ağ katlı çıkar, havada açılır
  m   : ağın kütlesi

⛔ AĞ GÖZENEKLİDİR — İLK MODELİM BUNU ATLADI VE MENZİLİ 4 KAT KISA VERDİ
  Ağı dolu bir disk gibi (C_d = 1.2) modelleyince 20 m/s'de menzil 2.8 m
  çıkıyordu; gerçek ağ tabancaları 10-20 m yapar. Sebep: ağ çoğunlukla
  DELİKTİR, hava içinden geçer. Gözenekli perde (screen) için etkin katsayı:

      C_d,etkin = C_d,dolu · (1 − β²)          β = açık alan oranı

  β = 0   (dolu levha)  → C_d,etkin = C_d,dolu   ✔ sınır doğru
  β = 1   (ağ yok)      → C_d,etkin = 0          ✔ sınır doğru
  β = 0.9 (tipik ağ)    → C_d,etkin = 1.2·0.19 = 0.23   (5 kat az sürükleme)

  Bu bir YAKLAŞIMDIR (gerçek perde akışı Reynolds'a da bağlıdır) ama
  gözenekliliğin baskın etkisini doğru yönde ve doğru mertebede verir.
  AVCI_AG_GOZENEK ile değiştirilebilir; ağın gerçek örgüsü belli olunca
  ölçülmelidir.

AĞIN AÇILMASI ve NİYE ÖNEMLİ:
  Ağ tam açık çıksaydı, 20 m/s'de sürükleme kuvveti
      ½·1.225·1.2·π·0.6²·20² = 166 N
  olurdu; 0.30 kg'lık ağ için 554 m/s² yavaşlama — yani 20 m/s'yi 0.04 s'de
  kaybeder ve 0.4 m gider. GERÇEK ağ tabancaları bu yüzden ağı KATLI atar;
  ağ havada açılır. Modelde yarıçap ilk `ACILMA_S` saniyede
  KATLI_YARICAP → AGIZ_YARICAP arasında doğrusal büyür.

YAKALAMA ÖLÇÜTÜ:
  Hedef merkezinin ağ ağzı düzlemine olan uzaklığı, o anki ağız yarıçapı +
  hedefin yarı açıklığından küçükse YAKALANDI sayılır. Yani "ağ hedefi
  içine aldı" demek.

────────────────────────── KULLANIM ────────────────────────────────────────
  # Panelden:  ATEŞ düğmesi  (POST /api/ag/ates)
  # Terminalden:
  python3 -m control.ag_firlatici --ates
  python3 -m control.ag_firlatici --ates --hiz 25 --dunya avci
================================================================================
"""
import argparse
import math
import os
import sys
import threading
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import numpy as np  # noqa: E402

from vision import geometry as geo  # noqa: E402

# ══════════════════════════ AYARLANABİLİR BÜYÜKLÜKLER ══════════════════════
AG_KUTLE_KG = float(os.environ.get("AVCI_AG_KUTLE", 0.30))
NAMLU_HIZI = float(os.environ.get("AVCI_AG_HIZ", 20.0))     # m/s (namluya göre)
AGIZ_YARICAP_M = float(os.environ.get("AVCI_AG_YARICAP", 0.60))
KATLI_YARICAP_M = 0.08          # ağ katlıyken görünen yarıçap
ACILMA_S = float(os.environ.get("AVCI_AG_ACILMA", 0.20))
SURUKLEME_KAT = 1.2             # C_d — DOLU levha/koni değeri
# β: ağın AÇIK ALAN ORANI (gözeneklilik). Tipik yakalama ağı ~0.85-0.93.
# Etkin sürükleme C_d·(1−β²) olur; bkz. dosya başındaki açıklama.
AG_GOZENEKLILIK = float(os.environ.get("AVCI_AG_GOZENEK", 0.90))
SURUKLEME_ETKIN = SURUKLEME_KAT * (1.0 - AG_GOZENEKLILIK ** 2)
HAVA_YOGUNLUGU = 1.225          # kg/m³
YERCEKIMI = 9.81
MAX_UCUS_S = float(os.environ.get("AVCI_AG_SURE", 4.0))
ADIM_HZ = 100.0
HEDEF_YARI_ACIKLIK_M = 0.65     # mini Talon yarı kanat açıklığı (yakalama payı)

DUNYA = os.environ.get("AVCI_TARET_DUNYA", "avci")
ARAC = os.environ.get("AVCI_TARET_ARAC", "iris_with_ardupilot")
HEDEF = os.environ.get("AVCI_AG_HEDEF", "mini_talon")
AG_MODEL = "ag_konisi"
PARK_POZ = (0.0, 0.0, -50.0)    # ateşlenmediğinde yerin altında bekler


def _q(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def _quat_to_rpy(x, y, z, w):
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    return roll, pitch, math.atan2(siny, cosy)


class AgFirlatici:
    """Gazebo'ya bağlanır, namlu pozunu hesaplar, ağı Python'da uçurur."""

    def __init__(self, dunya=DUNYA, arac=ARAC, hedef=HEDEF):
        self.dunya, self.arac, self.hedef = dunya, arac, hedef
        self.pozlar = {}          # model adı -> (pos ndarray, rpy tuple, t)
        self.hizlar = {}          # model adı -> ndarray (sonlu farkla)
        self.taret = {"pan": 0.0, "tilt": geo.TARET_TILT_VARSAYILAN}
        self.durum = {"asama": "HAZIR", "mesaj": "", "son": None}
        self._kilit = threading.Lock()
        self._ucuyor = False
        self._node = None
        self._msgs = {}
        self._baglan()

    # ───────────────────────────────────── gz-transport
    def _baglan(self):
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.model_pb2 import Model
        from gz.msgs10.boolean_pb2 import Boolean
        self._msgs = {"Pose": Pose, "Boolean": Boolean}
        self._node = Node()

        def poz_cb(msg):
            simdi = time.time()
            with self._kilit:
                for p in msg.pose:
                    if p.name not in (self.arac, self.hedef, AG_MODEL):
                        continue
                    yeni = np.array([p.position.x, p.position.y, p.position.z])
                    eski = self.pozlar.get(p.name)
                    if eski is not None and simdi - eski[2] > 1e-3:
                        self.hizlar[p.name] = (yeni - eski[0]) / (simdi - eski[2])
                    self.pozlar[p.name] = (
                        yeni,
                        _quat_to_rpy(p.orientation.x, p.orientation.y,
                                     p.orientation.z, p.orientation.w),
                        simdi)

        def eklem_cb(msg):
            for j in msg.joint:
                if j.name == "turret_yaw_joint":
                    self.taret["pan"] = j.axis1.position
                elif j.name == "turret_pitch_joint":
                    self.taret["tilt"] = j.axis1.position

        ok1 = self._node.subscribe(
            Pose_V, "/world/%s/dynamic_pose/info" % self.dunya, poz_cb)
        # ⚠ ag_konisi STATİK olduğu için dynamic_pose/info'da GÖRÜNMEZ;
        #   ama biz onu okumuyoruz, yalnız yazıyoruz. Araç ve hedef dinamik.
        ok2 = self._node.subscribe(
            Model, "/world/%s/model/%s/joint_state" % (self.dunya, self.arac),
            eklem_cb)
        if not (ok1 and ok2):
            raise RuntimeError("gz-transport aboneliği kurulamadı "
                               "(dünya=%s, araç=%s)" % (self.dunya, self.arac))

    def _poz_yaz(self, ad, pos, rpy):
        req = self._msgs["Pose"]()
        req.name = ad
        req.position.x, req.position.y, req.position.z = (float(v) for v in pos)
        (req.orientation.x, req.orientation.y,
         req.orientation.z, req.orientation.w) = _q(*rpy)
        try:
            _, ok = self._node.request("/world/%s/set_pose" % self.dunya, req,
                                       self._msgs["Pose"],
                                       self._msgs["Boolean"], 300)
            return ok
        except Exception:
            return False

    def park(self):
        """Ağı görünmez yere (yerin altına) çek."""
        self._poz_yaz(AG_MODEL, PARK_POZ, (0.0, 0.0, 0.0))

    # ───────────────────────────────────── namlu dünyada nerede, nereye bakıyor
    def namlu_dunya(self):
        with self._kilit:
            kayit = self.pozlar.get(self.arac)
            v_arac = self.hizlar.get(self.arac, np.zeros(3)).copy()
            pan, tilt = self.taret["pan"], self.taret["tilt"]
        if kayit is None:
            return None
        pos, rpy, _ = kayit
        uc_g, yon_g = geo.namlu_ekstrinsik(pan, tilt)     # gövde çerçevesi
        R = geo.rot_rpy(*rpy)                             # gövde → dünya
        return pos + R @ uc_g, R @ yon_g, v_arac

    # ───────────────────────────────────── ATEŞ
    def ates(self, hiz=None, engelle_yeniden=True):
        if self._ucuyor and engelle_yeniden:
            return {"ok": False, "mesaj": "ağ zaten uçuyor"}
        n = self.namlu_dunya()
        if n is None:
            return {"ok": False,
                    "mesaj": "araç pozu gelmedi (%s dinleniyor mu?)" % self.arac}
        threading.Thread(target=self._ucus, args=(hiz or NAMLU_HIZI,),
                         daemon=True, name="ag-ucus").start()
        return {"ok": True, "mesaj": "ateşlendi"}

    def _ucus(self, namlu_hizi):
        self._ucuyor = True
        uc, yon, v_arac = self.namlu_dunya()
        p = uc.copy()
        v = v_arac + namlu_hizi * yon
        t0 = time.time()
        self.durum.update(
            asama="UÇUYOR", mesaj="",
            son={"cikis_hizi": float(np.linalg.norm(v)),
                 "arac_hizi": float(np.linalg.norm(v_arac)),
                 "namlu_hizi": namlu_hizi})
        iz = []
        sonuc, en_yakin = "ISKA", 1e9
        # ══════════════════════════════════════════════════════════════
        #  ZAMAN TABANI — iki kez yanlış yapıldı, burada duruyor ki üçüncü
        #  kez yapılmasın (2026-09-06):
        #
        #  1. HATA: `t` duvar saatinden, entegrasyon SABİT dt = 1/100 s ile.
        #     Döngü 100 Hz'ten yavaş koştuğu için sim zamanı duvar zamanının
        #     gerisinde kaldı; 3.84 s "uçan" ağ yalnız 1.74 m gitti.
        #  2. HATA: dt'yi gerçek geçen süre yapıp 0.05 s'e KIRPMAK. İplik
        #     0.3 s aç kalınca 0.25 s'lik fizik çöpe gitti; menzil yine kısa.
        #     (Aç kalmanın sebebi: bu iplik gcs_server içinde YOLO ile aynı
        #      GIL'i paylaşıyor ve YOLO 2028×1520 kare işliyor.)
        #
        #  ✔ DOĞRUSU — ALT ADIMLI YETİŞME:
        #     Fizik SABİT ve KÜÇÜK bir adımla (DT_FIZIK) ilerler; her uyanışta
        #     duvar saatiyle arasındaki farkı kapatana kadar ARDIŞIK adım atar.
        #     Böylece iplik ne kadar aç kalırsa kalsın sim zamanı duvar
        #     zamanına EŞİT kalır ve sonuç ipliğin yüküne bağlı olmaz.
        #     Görselleştirme (set_pose) fizikten AYRI, daha seyrek yapılır —
        #     çizim hızı fiziği bağlamamalı.
        DT_FIZIK = 0.004          # s; 250 Hz — balistik için fazlasıyla ince
        CIZIM_ARALIK = 0.02       # s; 50 Hz set_pose
        MAX_ALT_ADIM = 400        # tek uyanışta en fazla bu kadar yetiş
        t_sim = 0.0
        son_cizim = 0.0
        adim = 0
        while True:
            hedef_t = time.time() - t0
            if hedef_t > MAX_UCUS_S:
                sonuc = "SÜRE DOLDU"
                break
            n_adim = 0
            while t_sim < hedef_t and n_adim < MAX_ALT_ADIM:
                # ── ağın açılması: katlıdan açığa doğrusal
                oran = min(1.0, t_sim / ACILMA_S) if ACILMA_S > 1e-6 else 1.0
                r = KATLI_YARICAP_M + (AGIZ_YARICAP_M - KATLI_YARICAP_M) * oran
                A = math.pi * r * r
                # ── ivme = yerçekimi + hava direnci
                hiz_b = float(np.linalg.norm(v))
                a = np.array([0.0, 0.0, -YERCEKIMI])
                if hiz_b > 1e-6:
                    a = a - (0.5 * HAVA_YOGUNLUGU * SURUKLEME_ETKIN * A * hiz_b
                             / AG_KUTLE_KG) * v
                v = v + a * DT_FIZIK
                p = p + v * DT_FIZIK
                t_sim += DT_FIZIK
                n_adim += 1
                adim += 1
                # ── yakalama sınaması HER FİZİK ADIMINDA (hızlı geçişi kaçırma)
                with self._kilit:
                    hk = self.pozlar.get(self.hedef)
                if hk is not None:
                    d = float(np.linalg.norm(hk[0] - p))
                    if d < en_yakin:
                        en_yakin = d
                    if d <= r + HEDEF_YARI_ACIKLIK_M:
                        sonuc = "YAKALANDI"
                        break
                if p[2] <= 0.0:
                    sonuc = "YERE DÜŞTÜ"
                    break
            if sonuc in ("YAKALANDI", "YERE DÜŞTÜ"):
                iz.append((t_sim, p.copy(), r, float(np.linalg.norm(v))))
                self._poz_yaz(AG_MODEL, p, (0.0, 0.0, 0.0))
                break
            iz.append((t_sim, p.copy(), r, float(np.linalg.norm(v))))
            # ── çizim: fizikten ayrı, seyrek
            if t_sim - son_cizim >= CIZIM_ARALIK:
                son_cizim = t_sim
                hy = float(np.linalg.norm(v))
                if hy > 1e-6:
                    yaw = math.atan2(v[1], v[0])
                    pit = -math.asin(max(-1.0, min(1.0, v[2] / hy)))
                else:
                    yaw = pit = 0.0
                self._poz_yaz(AG_MODEL, p, (0.0, pit, yaw))
            time.sleep(0.005)

        menzil = float(np.linalg.norm(p - uc))
        self.durum.update(asama="BİTTİ", mesaj=sonuc)
        duvar = time.time() - t0
        self.durum["son"].update(
            sonuc=sonuc, menzil_m=round(menzil, 2),
            ucus_s=round(t_sim, 2),
            en_yakin_m=round(en_yakin, 2) if en_yakin < 1e8 else None,
            son_hiz=round(float(np.linalg.norm(v)), 2),
            # ⚠ MEKANİZMA KAPISI: sim zamanı duvar zamanına EŞİT olmalı.
            #   Sapma varsa fizik geri kalmıştır ve menzil YANLIŞTIR.
            duvar_s=round(duvar, 2),
            zaman_sapmasi=round(duvar - t_sim, 3),
            fizik_adim=adim)
        time.sleep(1.5)
        self.park()
        self.durum["asama"] = "HAZIR"
        self._ucuyor = False


# ══════════════════════════ KOMUT SATIRI ══════════════════════════
def main():
    ap = argparse.ArgumentParser(description="VAKKAS ağ fırlatıcı (Python)")
    ap.add_argument("--ates", action="store_true", help="bir ağ at")
    ap.add_argument("--hiz", type=float, default=NAMLU_HIZI, help="namlu hızı m/s")
    ap.add_argument("--dunya", default=DUNYA)
    ap.add_argument("--arac", default=ARAC)
    ap.add_argument("--hedef", default=HEDEF)
    ap.add_argument("--menzil-tablosu", action="store_true",
                    help="uçmadan, balistik modelin menzil tablosunu yaz")
    a = ap.parse_args()

    if a.menzil_tablosu:
        print("AĞ BALİSTİĞİ — model tablosu (araç duruyor varsayımı)")
        print("  kütle %.2f kg · ağız yarıçapı %.2f m · açılma %.2f s"
              % (AG_KUTLE_KG, AGIZ_YARICAP_M, ACILMA_S))
        print("  gözeneklilik β=%.2f → C_d etkin = %.1f·(1−β²) = %.3f"
              % (AG_GOZENEKLILIK, SURUKLEME_KAT, SURUKLEME_ETKIN))
        print("  %8s %10s %10s %10s" % ("namlu", "yatay menzil", "düşüş", "uçuş"))
        for v0 in (10, 15, 20, 25, 30):
            p = np.zeros(3)
            v = np.array([float(v0), 0.0, 0.0])
            t, dt = 0.0, 1.0 / ADIM_HZ
            while t < MAX_UCUS_S and p[2] > -50:
                oran = min(1.0, t / ACILMA_S) if ACILMA_S > 1e-6 else 1.0
                r = KATLI_YARICAP_M + (AGIZ_YARICAP_M - KATLI_YARICAP_M) * oran
                hb = float(np.linalg.norm(v))
                acc = np.array([0.0, 0.0, -YERCEKIMI])
                if hb > 1e-6:
                    acc = acc - (0.5 * HAVA_YOGUNLUGU * SURUKLEME_ETKIN
                                 * math.pi * r * r * hb / AG_KUTLE_KG) * v
                v = v + acc * dt
                p = p + v * dt
                t += dt
                if p[2] < -3.0:      # 3 m düşünce "bitti" say
                    break
            print("  %6.0f m/s %9.1f m %9.1f m %9.2f s"
                  % (v0, p[0], -p[2], t))
        print("\n  ⚠ Ağ tabancalarının menzili KISADIR — sürükleme hıza göre")
        print("    KARESEL artar. Uzun menzil isteniyorsa ağı daha GEÇ açmak")
        print("    (ACILMA_S büyütmek) hızdan çok daha etkilidir.")
        return 0

    f = AgFirlatici(a.dunya, a.arac, a.hedef)
    print("bağlandı — araç pozu bekleniyor…")
    t0 = time.time()
    while f.namlu_dunya() is None and time.time() - t0 < 15:
        time.sleep(0.3)
    n = f.namlu_dunya()
    if n is None:
        print("⛔ araç pozu gelmedi. Gazebo ayakta mı? dünya adı doğru mu?")
        return 1
    uc, yon, va = n
    print("  namlu ucu (dünya): %s" % np.round(uc, 2))
    print("  atış doğrultusu  : %s   (yükseliş %+.1f°, azimut %+.1f°)"
          % (np.round(yon, 3), math.degrees(math.asin(yon[2])),
             math.degrees(math.atan2(yon[1], yon[0]))))
    print("  aracın hızı      : %.1f m/s" % float(np.linalg.norm(va)))
    if not a.ates:
        print("  (ateşlemek için --ates ekle)")
        return 0
    print(f.ates(a.hiz))
    while f.durum["asama"] != "HAZIR":
        time.sleep(0.2)
    s = f.durum["son"]
    print("\n  SONUÇ: %s" % s.get("sonuc"))
    for k in ("cikis_hizi", "arac_hizi", "menzil_m", "ucus_s", "en_yakin_m",
              "son_hiz"):
        if k in s:
            print("    %-12s : %s" % (k, s[k]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
