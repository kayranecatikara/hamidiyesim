"""GERÇEK ÇARPIŞMA DURUMU — süreç içi tek kaynak (A5).

Neden ayrı modül: temas olayını Gazebo'dan `gcs_server` dinliyor, ama vuruş
kararını `guidance/visual_lead` veriyor. visual_lead'in gcs_server'ı import
etmesi döngüsel bağımlılık olurdu (gcs_server zaten guidance'ı import ediyor)
ve testlerde tüm sunucuyu ayağa kaldırmak gerekirdi. Bu modülün hiçbir bağımlılığı
yok; iki taraf da buraya bakar.

İKİ AYRI BİLGİ tutulur ve ikisi de gerekli:

  temas_var()  — çarpışma OLDU mu?
  kaynak_var() — çarpışmayı duyabilecek bir dinleyici ÇALIŞIYOR mu?

İkincisi olmadan güvenli bir yedek yol yazılamaz. Temas kaynağı yokken
(gz-transport kurulu değil, dünyada contact-system eksik, AVCI_HASAR=0)
"temas gelmedi" ile "temas olmadı" ayırt edilemez; o durumda güdüm eski
yakınlık ölçütüne (`VURUS_MENZIL`) düşer. Kaynak çalışıyorsa yakınlık TEK
BAŞINA vuruş sayılmaz — A5'in bütün amacı bu.
"""

import threading
import time

_kilit = threading.Lock()

_durum = {
    "temas": False,     # gerçek fiziksel temas oldu mu
    "t": None,          # ilk temasın duvar saati
    "detay": None,      # "temas: <collision1> ↔ <collision2>"
    "sayac": 0,         # toplam temas bildirimi (sıfırlamalar arası)
}
_kaynak_hazir = False   # temas dinleyicisi abone oldu mu


def kaynak_bildir(hazir):
    """Temas dinleyicisi aboneliği kurunca True, vazgeçince False geçer."""
    global _kaynak_hazir
    with _kilit:
        _kaynak_hazir = bool(hazir)


def kaynak_var():
    """Gerçek temas duyulabiliyor mu? False ise yakınlık yedeği devreye girer."""
    with _kilit:
        return _kaynak_hazir


def temas_bildir(detay=None):
    """Gazebo'dan gerçek temas geldi. İlk temasın anı korunur."""
    with _kilit:
        _durum["sayac"] += 1
        if not _durum["temas"]:
            _durum["temas"] = True
            _durum["t"] = time.time()
            _durum["detay"] = detay


def temas_var():
    with _kilit:
        return _durum["temas"]


def durum():
    with _kilit:
        return dict(_durum, kaynak_hazir=_kaynak_hazir)


def sifirla():
    """Yeni denemeden önce. Kaynak bayrağına DOKUNMAZ — dinleyici hâlâ ayakta."""
    with _kilit:
        _durum.update(temas=False, t=None, detay=None, sayac=0)
