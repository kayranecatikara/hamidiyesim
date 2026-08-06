"""Kilit süresi muhasebesi — SAF sınıf (şartname 6.1.4).

kilitlenme.py'deki _pencere/_kumulatif mantığının 6.1.4 uyumlu genişletmesidir,
ancak ESKİ KODA DOKUNMADAN bağımsız yazılmıştır. I/O yok, thread yok.

Model
-----
- Zaman damgalı segmentler; SÜRE tabanlı hesap, KARE SAYMA YOK.
- t sim saatidir (header.stamp) ve MONOTON varsayılır.
- Kayan PENCERE_SN penceresinde kümülatif kilit süresi hesaplanır.
- Boşluk köprüleme: bir segment içindeki kısa kilit kayıpları, o ana dek
  bildirilen (kesintisiz) sürenin KARE_TOLERANS_ORAN katı bütçesiyle köprülenir.
  Segmentin BAŞI ve SONU her zaman gerçek kilitli örnekle tanımlanır; bu yüzden
  köprü segment başında (bütçe=0) ve sonunda (kapatan kilit yok) geçersizdir.
- Otomatik temizlik YOK: pencere dışına taşan süreler kümülatif hesapta doğal
  olarak düşer. Tam sıfırlama SADECE açık reset() ile yapılır
  (RESET_POLICY = "kumulatif_korunur").

Köprülenen boşluk süresi, segment sürekli sayıldığından hem kesintisiz hem de
kümülatif süreye dahil edilir (beyan edilen sürenin TAMAMI köprülenmiş kabul
edilir — 6.1.4).
"""

from dataclasses import dataclass

from config.kilit_sabitler import (
    KARE_TOLERANS_ORAN,
    KESINTISIZ_SN,
    KUMULATIF_SN,
    ORNEK_TAVAN_SN,
    PENCERE_SN,
)


@dataclass
class SureDurum:
    kumulatif_sn: float
    kesintisiz_sn: float
    pencere_ok: bool      # kumulatif_sn >= KUMULATIF_SN
    kesintisiz_ok: bool   # kesintisiz_sn >= KESINTISIZ_SN


class KilitSure:
    """Kilit örneklerini (t, kilit) tüketip süre durumunu üretir. Saf/durumlu."""

    def __init__(self):
        self.reset()

    def reset(self):
        """Tam sıfırlama — açık çağrı (RESET_POLICY = kumulatif_korunur)."""
        self._t_prev = None
        self._kilit_prev = False
        # Bitişik kilitli aralıklar (köprülenmiş boşluklar dahil): [start, end].
        # Uç noktalar daima gerçek kilitli örneklerdir.
        self._spans = []
        self._seg_alive = False   # son span hâlâ aktif bir segment mi
        self._seg_locked = 0.0    # aktif segmentte GERÇEK kilitli süre (bütçe tabanı)
        self._seg_bridged = 0.0   # aktif segmentte köprülenen toplam boşluk
        self._pending_gap = 0.0   # son gerçek kilitten bu yana biriken boşluk

    def guncelle(self, t, kilit):
        """Bir örnek işle, güncel SureDurum döndür."""
        if self._t_prev is None:
            if kilit:
                self._segment_baslat(t)
            self._t_prev = t
            self._kilit_prev = bool(kilit)
            return self._durum(t)

        dt = t - self._t_prev
        if dt < 0.0:
            dt = 0.0  # monoton olmayan/yinelenen damga: aralık yok say

        # İki uçta da kilit VE örnek aralığı tavanın altında ise aralık kilitlidir.
        # dt tavanı aşarsa (donma/atlama), aralık KİLİTLİ SAYILMAZ → boşluk gibi
        # işlenir ve köprü bütçesi karar verir (6.1.4-uyumlu MAKS_DT karşılığı).
        aralik_kilitli = self._kilit_prev and kilit and dt <= ORNEK_TAVAN_SN

        if aralik_kilitli:
            # Aralık tamamen kilitli → aktif span'i uzat, gerçek kilitli süreyi ekle.
            if self._seg_alive and self._spans:
                self._spans[-1][1] = t
                self._seg_locked += dt
            else:
                # Beklenmedik durum: aktif segment yok ama iki uçta da kilit.
                self._segment_baslat(t)
        elif kilit:  # şimdi kilit ama aralık boşluk (önce açık YA DA dt tavanı aştı)
            self._pending_gap += dt
            if self._seg_alive and self._spans:
                seg = self._spans[-1]
                # Bütçe TABANI yalnız gerçek kilitli süredir; köprülenen boşluklar
                # bütçeyi ŞİŞİRMEZ (pozitif geri besleme yok — 6.1.4).
                butce = KARE_TOLERANS_ORAN * self._seg_locked
                if self._seg_bridged + self._pending_gap <= butce:
                    # Köprüle: boşluk sürekli segmentin parçası olur.
                    self._seg_bridged += self._pending_gap
                    seg[1] = t
                else:
                    self._segment_baslat(t)
            else:
                self._segment_baslat(t)
            self._pending_gap = 0.0
        else:
            # Şimdi kilit yok (önce kilitli ya da açık) → boşluk birikir.
            self._pending_gap += dt
            self._segment_yasam_kontrol()

        self._t_prev = t
        self._kilit_prev = bool(kilit)
        return self._durum(t)

    # --- iç yardımcılar ---

    def _segment_baslat(self, t):
        self._spans.append([t, t])
        self._seg_alive = True
        self._seg_locked = 0.0
        self._seg_bridged = 0.0
        self._pending_gap = 0.0

    def _segment_yasam_kontrol(self):
        """Açık boşluk bütçeyi aşarsa aktif segmenti öldürür (kesintisiz sıfırlanır)."""
        if not (self._seg_alive and self._spans):
            return
        # Bütçe tabanı yalnız gerçek kilitli süre (köprü şişirmez).
        butce = KARE_TOLERANS_ORAN * self._seg_locked
        if self._seg_bridged + self._pending_gap > butce:
            self._seg_alive = False

    def _kumulatif(self, t):
        w_start = t - PENCERE_SN
        # Tamamen pencere dışında kalan span'ler 0 katkı verir → bellek için at.
        while self._spans and self._spans[0][1] < w_start and not (
            len(self._spans) == 1 and self._seg_alive
        ):
            self._spans.pop(0)
        total = 0.0
        for s, e in self._spans:
            a = s if s > w_start else w_start
            b = e if e < t else t
            if b > a:
                total += b - a
        return total

    def _kesintisiz(self):
        if self._seg_alive and self._spans:
            s, e = self._spans[-1]
            return e - s
        return 0.0

    def beyan_araligi(self, t, pencere_sn=PENCERE_SN):
        """SALT-OKUR: bildirim için beyan aralığı. Durumu DEĞİŞTİRMEZ.

        [t - pencere_sn, t] penceresindeki kilitli parçalardan:
          baslangic = pencere içi ilk gerçek kilitli an,
          bitis     = pencere içi SON parçanın sonu (= son gerçek kilitli örnek;
                      t'nin kendisi DEĞİL — 6.1.4 "segment gerçek kilitli kareyle
                      biter": t tespitsiz bir kareyse bitis onu kapsamaz),
          kumulatif = Σ parça süreleri (köprülenen boşluklar dahil).
        Dönüş: (baslangic, bitis, kumulatif) | None (pencerede kilitli parça yoksa).
        Pencere referansı parametriktir (organizasyon tanımına hazır).
        """
        w = t - pencere_sn
        parcalar = []
        for s, e in self._spans:
            a = s if s > w else w
            b = e if e < t else t           # e <= t: bitis daima gerçek kilitli an
            if b > a:
                parcalar.append((a, b))
        if not parcalar:
            return None
        baslangic = parcalar[0][0]
        bitis = parcalar[-1][1]
        kumulatif = sum(b - a for a, b in parcalar)
        return (baslangic, bitis, kumulatif)

    def _durum(self, t):
        kumulatif = self._kumulatif(t)
        kesintisiz = self._kesintisiz()
        return SureDurum(
            kumulatif_sn=kumulatif,
            kesintisiz_sn=kesintisiz,
            pencere_ok=kumulatif >= KUMULATIF_SN,
            kesintisiz_ok=kesintisiz >= KESINTISIZ_SN,
        )
