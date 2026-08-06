"""comms/kilit_bildirim.py — Yarışma sunucusu bildirim iskeleti (Adım 8B).

Kilitlenme, yarışma sunucusuna paketle bildirilir (şartname 6.1.4). Paketin
GERÇEK formatı haberleşme dokümanıyla gelecek; bu yüzden bildirim SOYUT bir
arayüz olarak yazılır. Şimdilik tek implementasyon KURU-KOŞU modudur
(logs/bildirim_*.csv'ye yazar). Gerçek gönderici (soket/HTTP/MAVLink) ileride
AYRI bir sınıf olarak eklenecek; arayüz DEĞİŞMEYECEK.

Beyan aralığı KilitSure.beyan_araligi'ndan gelir: (baslangic_t, bitis_t,
kumulatif_sn). Beyan edilen sürenin (kumulatif_sn) TAMAMI şartı sağlamalı
(6.1.4) — sart_saglandi bunu doğrular.
"""

import abc
import os
import threading
import time

from config.kilit_sabitler import KUMULATIF_SN

_LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "logs")


class KilitBildirici(abc.ABC):
    """Soyut bildirim arayüzü. Gerçek paket göndericisi bu arayüzü uygular."""

    @abc.abstractmethod
    def bildir(self, baslangic_t, bitis_t, kumulatif_sn):
        """Bir kilitlenme beyanını yarışma sunucusuna bildir.

        baslangic_t : beyan aralığının başı (gerçek kilitli an)
        bitis_t     : beyan aralığının sonu (gerçek kilitli an)
        kumulatif_sn: beyan edilen kümülatif kilit süresi (TAMAMI şartı sağlar)
        """
        raise NotImplementedError


class KuruKosuBildirici(KilitBildirici):
    """KURU-KOŞU: paket göndermez, logs/bildirim_*.csv'ye satır yazar.

    Bildirim seyrektir (kilit başına ~1) → doğrudan append+flush; kare hattını
    bloklamaz (ayrı thread gereksiz). Thread-safe (küçük kilit).
    """

    _ALANLAR = ["gonderim_t", "baslangic_t", "bitis_t", "kumulatif_sn",
                "beyan_suresi", "sart_saglandi"]

    def __init__(self, csv_yol=None):
        self._lock = threading.Lock()
        os.makedirs(_LOG_DIR, exist_ok=True)
        self.csv_yol = csv_yol or os.path.join(
            _LOG_DIR, time.strftime("bildirim_%Y%m%d_%H%M%S.csv"))
        self._f = open(self.csv_yol, "w", newline="")
        import csv
        self._w = csv.DictWriter(self._f, fieldnames=self._ALANLAR)
        self._w.writeheader()
        self._f.flush()
        print(f"[BILDIRIM] kuru-koşu bildirim logu: {self.csv_yol}")

    def bildir(self, baslangic_t, bitis_t, kumulatif_sn):
        beyan_suresi = bitis_t - baslangic_t
        sart = 1 if kumulatif_sn >= KUMULATIF_SN else 0
        with self._lock:
            self._w.writerow({
                "gonderim_t": round(bitis_t, 4),
                "baslangic_t": round(baslangic_t, 4),
                "bitis_t": round(bitis_t, 4),
                "kumulatif_sn": round(kumulatif_sn, 4),
                "beyan_suresi": round(beyan_suresi, 4),
                "sart_saglandi": sart,
            })
            self._f.flush()
        print(f"[BILDIRIM] kilitlenme beyanı: [{baslangic_t:.2f}, {bitis_t:.2f}] "
              f"kümülatif {kumulatif_sn:.2f} s, şart {'SAĞLANDI' if sart else 'SAĞLANMADI'}")

    def kapat(self):
        with self._lock:
            if not self._f.closed:
                self._f.flush()
                self._f.close()
