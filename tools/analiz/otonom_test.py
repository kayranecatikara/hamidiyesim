#!/usr/bin/env python3
"""
tools/analiz/otonom_test.py — Gözetimsiz uçuş testi koşucusu.

Kullanıcı başında olmadan bir uçuşu baştan sona sürer ve ÖLÇER:
  gaz ayarla → senaryo başlat → hedefin irtifası/hızı OTURANA KADAR bekle
  → chase başlat → menzil/kilit/faz izle → durdur → özet yaz.

Her adımda arıza tanır ve sebebini yazar (hedef stall etti, avcı çakıldı,
menzil kapanmıyor, görsel temas kurulamadı). Uçuş çıktısı logs/ altındaki
gps_guidance_*.csv ve visual_lead_*.csv dosyalarıdır; bu script onları
üretmek ve koşulların geçerli olduğunu doğrulamak içindir.

NOT: Bu script simülasyona KOMUT GÖNDERİR (CLAUDE.md §1'in istisnası —
yalnız kullanıcı açıkça gözetimsiz test istediğinde çalıştırılır).

Kullanım:
    python3 -m tools.analiz.otonom_test <senaryo> [gaz] [chase_sure_s]
    python3 -m tools.analiz.otonom_test square 500 180
"""

import json
import math
import sys
import time
import urllib.request

TABAN = "http://localhost:8000"


def _istek(yol, veri=None, zaman_asimi=5.0):
    url = TABAN + yol
    if veri is None:
        req = urllib.request.Request(url)
    else:
        req = urllib.request.Request(
            url, data=json.dumps(veri).encode(),
            headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=zaman_asimi) as r:
        gövde = r.read().decode()
    try:
        return json.loads(gövde)
    except Exception:
        return {"raw": gövde}


def _truth():
    """Gazebo gerçek pozları — ölçümün zemini (MAVLink değil)."""
    from control import gz_truth
    gz_truth.baslat()
    return gz_truth


def _durum():
    try:
        return _istek("/api/chase_status")
    except Exception:
        return {}


def _yaz(s):
    print(s, flush=True)


class Ucus:
    def __init__(self, gz, senaryo, gaz):
        self.gz, self.senaryo, self.gaz = gz, senaryo, gaz
        self.olaylar = []
        self.avci_kalkti = False   # çakılma kontrolü ancak kalkıştan SONRA anlamlı
        self._durdur_ucu = "stop_chase"

    def olay(self, tip, mesaj):
        self.olaylar.append((tip, mesaj))
        _yaz(f"  [{tip}] {mesaj}")

    # ---- arıza tanıma ----
    def _araclar(self):
        i, h = self.gz.get_ikisi()
        return i, h

    def hedef_dustu(self):
        _i, h = self._araclar()
        return h is not None and (h["z"] < 3.0 or abs(math.degrees(h["roll"])) > 100)

    def avci_dustu(self):
        """Chase başlarken avcı YERDE olur; o hali çakılma sayarsak test daha
        ilk karede biter. Bir kez 5 m'nin üstüne çıktıktan SONRA kontrol edilir."""
        i, _h = self._araclar()
        if i is None:
            return False
        if i["z"] > 5.0:
            self.avci_kalkti = True
        if not self.avci_kalkti:
            return False
        return i["z"] < 1.0 or abs(math.degrees(i["roll"])) > 100

    # ---- adımlar ----
    def gaz_ayarla(self):
        _istek("/api/plane_throttle", {"throttle": self.gaz})
        okunan = _istek("/api/plane_throttle").get("throttle")
        if okunan != self.gaz:
            self.olay("HATA", f"gaz {self.gaz} istendi ama sunucu {okunan} diyor")
            return False
        self.olay("OK", f"gaz = {okunan}")
        return True

    def senaryo_baslat(self):
        _istek(f"/api/command/plane/scenario/{self.senaryo}", {})
        self.olay("OK", f"senaryo başlatıldı: {self.senaryo}")
        return True

    def hedef_otursun(self, azami=150.0, pencere=20.0, tolerans=4.0):
        """Hedefin irtifası `pencere` saniye boyunca `tolerans` metre içinde
        kalana kadar bekle. Düşerse False döner.

        AGRESİF SENARYO İSTİSNASI (2026-08-04): `aggressive` TANIMI GEREĞİ
        irtifasını oturtmaz — rastgele tırmanış/dalış/spiral yapar. Sabit
        bant kapısı orada hiç açılmaz ve uçuş, sistemde hiçbir arıza yokken
        "hedef irtifası oturmadı" diye düşer (ilk denemede bu oldu). O
        senaryoda kapı ölçüt DEĞİŞTİRİR: bant aranmaz, yalnız hedefin
        HAVADA ve manevra irtifasında olması beklenir."""
        if self.senaryo == "aggressive":
            return self._hedef_havalansin(azami)
        self.olay("BEKLE", f"hedefin irtifası oturuyor (en fazla {azami:.0f} s)")
        t0 = time.time()
        gecmis = []
        while time.time() - t0 < azami:
            _i, h = self._araclar()
            if h is None:
                time.sleep(1.0)
                continue
            simdi = time.time()
            gecmis.append((simdi, h["z"]))
            # Pencereden BİRAZ FAZLASINI tut: aşağıdaki kontrol "en eski örnek
            # >= pencere kadar eski mi" diye soruyor. Filtre tam `pencere` ile
            # kırpılırsa iki koşul birbirini dışlar ve kapı hiç açılmaz.
            gecmis = [(t, z) for t, z in gecmis if simdi - t <= pencere * 1.5]
            if h["z"] < 3.0 and simdi - t0 > 30:
                self.olay("ARIZA", f"hedef yere indi (irtifa {h['z']:.1f} m) — "
                                   "stall ya da senaryo irtifayı tutamıyor")
                return False
            if len(gecmis) >= 8 and simdi - gecmis[0][0] >= pencere:
                z = [v for _t, v in gecmis]
                if max(z) - min(z) <= tolerans and min(z) > 20.0:
                    self.olay("OK", f"irtifa oturdu: {h['z']:.1f} m "
                                    f"(son {pencere:.0f}+ s bandı {max(z)-min(z):.1f} m)")
                    return True
            time.sleep(1.0)
        self.olay("ARIZA", "hedef irtifası verilen sürede oturmadı")
        return False

    def _hedef_havalansin(self, azami=150.0, min_irtifa=35.0, kararlilik_s=10.0):
        """Agresif senaryo kapısı: bant değil, YÜKSEKLİK + SÜREKLİLİK.

        Hedef `kararlilik_s` boyunca kesintisiz `min_irtifa` üstünde kalırsa
        manevraya başlamış ve düşmemiş demektir; chase o noktada anlamlıdır.
        35 m: agresif senaryo 60 m'den dalışlarla ~40 m'ye iniyor (ölçüm
        2026-08-01), taban bunun altında olmalı ki dalış kapıyı düşürmesin."""
        self.olay("BEKLE", f"agresif: hedef {min_irtifa:.0f} m üstünde "
                           f"{kararlilik_s:.0f} s kalsın (en fazla {azami:.0f} s)")
        t0 = time.time()
        yukselis_baslangic = None
        while time.time() - t0 < azami:
            _i, h = self._araclar()
            if h is None:
                time.sleep(1.0)
                continue
            if h["z"] >= min_irtifa:
                if yukselis_baslangic is None:
                    yukselis_baslangic = time.time()
                elif time.time() - yukselis_baslangic >= kararlilik_s:
                    self.olay("OK", f"agresif: hedef manevrada, irtifa {h['z']:.1f} m")
                    return True
            else:
                yukselis_baslangic = None      # dalışta sayaç sıfırlanır
            time.sleep(1.0)
        self.olay("ARIZA", "agresif: hedef manevra irtifasına çıkamadı")
        return False

    def hedef_hizi(self, sure=12.0, ornek_araligi=1.0):
        """Hedefin gerçek yer hızı (m/s), Gazebo SİM saatiyle.

        Sim saati duvar saatinden yavaş akıyor (ölçülen RTF 0.5-0.7), o yüzden
        örnekler arası sim farkı duvar uykusundan KISA olur. Eskiden eşik
        (0.5 s sim) uyku süresine (0.5 s duvar) eşitti ve RTF<1 iken hiç
        sağlanmıyordu — ölçüm her seferinde boş dönüyordu.
        """
        onc = None
        hizlar = []
        t0 = time.time()
        while time.time() - t0 < sure:
            _i, h = self._araclar()
            if h is not None:
                if onc is not None and h["stamp"] - onc[0] > 0.3:
                    hizlar.append(math.hypot(h["x"] - onc[1], h["y"] - onc[2])
                                  / (h["stamp"] - onc[0]))
                    onc = (h["stamp"], h["x"], h["y"])
                elif onc is None:
                    onc = (h["stamp"], h["x"], h["y"])
            time.sleep(ornek_araligi)
        return sorted(hizlar)[len(hizlar) // 2] if hizlar else None

    def chase(self, sure, gorsel=False):
        """gorsel=True → GPS fazını ATLA, doğrudan IBVS görsel güdümü çalıştır
        (/api/command/iris/start_visual). Lead yasasını izole ölçmenin tek yolu;
        GPS fazının hız marjı sorunu bu ölçümü bloklamasın diye ayrı tutuldu."""
        uc = "start_visual" if gorsel else "start_chase"
        self._durdur_ucu = "stop_visual" if gorsel else "stop_chase"
        _istek(f"/api/command/iris/{uc}", {})
        self.olay("OK", f"{'GÖRSEL (izole)' if gorsel else 'chase'} başladı "
                        f"({sure:.0f} s izlenecek)")
        t0 = time.time()
        en_yakin = 1e9
        kilit_en_yuksek = 0
        gorsel_gordu = False
        son_rapor = 0.0
        while time.time() - t0 < sure:
            d = self.gz.menzil()
            s = _durum()
            sup = s.get("supervisor", {})
            gud = s.get("guidance", {})
            if d is not None:
                en_yakin = min(en_yakin, d)
            kilit_en_yuksek = max(kilit_en_yuksek, sup.get("kilit_sayac") or 0)
            if sup.get("faz") == "VISUAL":
                gorsel_gordu = True
            if self.avci_dustu():
                self.olay("ARIZA", f"avcı düştü (t+{time.time()-t0:.0f} s, "
                                   f"en yakın menzil {en_yakin:.1f} m)")
                break
            if self.hedef_dustu():
                self.olay("ARIZA", f"hedef düştü (t+{time.time()-t0:.0f} s, "
                                   f"en yakın menzil {en_yakin:.1f} m)")
                break
            if sup.get("faz") == "VURULDU":
                self.olay("OK", "HEDEF VURULDU")
                break
            if time.time() - son_rapor >= 15:
                son_rapor = time.time()
                _yaz("    t+%3.0f s  menzil=%6.1f  faz=%-7s durum=%-8s kilit=%s"
                     % (time.time() - t0, d if d else -1,
                        sup.get("faz"), gud.get("durum"), sup.get("kilit_sayac")))
            time.sleep(1.0)
        try:
            _istek(f"/api/command/iris/{self._durdur_ucu}", {})
        except Exception:
            pass
        return {"en_yakin": en_yakin, "kilit_en_yuksek": kilit_en_yuksek,
                "gorsel": gorsel_gordu}

    def temizle(self):
        for yol in ("/api/command/iris/stop_chase", "/api/command/iris/stop_visual",
                    "/api/command/plane/stop_scenario"):
            try:
                _istek(yol, {})
            except Exception:
                pass


def kos(senaryo, gaz, chase_sure, gorsel=False):
    gz = _truth()
    time.sleep(3.0)
    u = Ucus(gz, senaryo, gaz)
    _yaz(f"\n{'='*66}\nUÇUŞ: senaryo={senaryo} gaz={gaz} süre={chase_sure:.0f}s "
         f"mod={'GÖRSEL-İZOLE' if gorsel else 'hibrit'}\n{'='*66}")
    sonuc = {"senaryo": senaryo, "gaz": gaz, "mod": "gorsel" if gorsel else "hibrit",
             "basarili": False}
    try:
        if not u.gaz_ayarla():
            return sonuc
        if not u.senaryo_baslat():
            return sonuc
        if not u.hedef_otursun():
            return sonuc
        hh = u.hedef_hizi()
        u.olay("OLCUM", f"hedef hızı ≈ {hh:.1f} m/s" if hh else "hedef hızı ölçülemedi")
        sonuc["hedef_hiz"] = hh
        r = u.chase(chase_sure, gorsel=gorsel)
        sonuc.update(r)
        sonuc["basarili"] = True
        u.olay("SONUÇ", f"en yakın menzil {r['en_yakin']:.1f} m, "
                        f"en yüksek pose kilidi {r['kilit_en_yuksek']}, "
                        f"görsel faz {'GİRDİ' if r['gorsel'] else 'girmedi'}")
    finally:
        u.temizle()
        sonuc["olaylar"] = u.olaylar
    return sonuc


if __name__ == "__main__":
    sen = sys.argv[1] if len(sys.argv) > 1 else "square"
    gaz = int(sys.argv[2]) if len(sys.argv) > 2 else 500
    sur = float(sys.argv[3]) if len(sys.argv) > 3 else 180.0
    gor = len(sys.argv) > 4 and sys.argv[4].lower().startswith("gor")
    s = kos(sen, gaz, sur, gorsel=gor)
    _yaz("\nÖZET: " + json.dumps(
        {k: v for k, v in s.items() if k != "olaylar"}, ensure_ascii=False))
