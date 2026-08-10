"""
ozellik_bayraklari.py — ÇALIŞMA-ANI (runtime) özellik/kill-switch bayrak deposu.

AMAÇ: her algoritma özelliği (kill-switch) UI'da SOL panelde bir aç/kapa
düğmesi olarak görünür ve gcs_server YENİDEN BAŞLATILMADAN çalışma anında
açılıp kapatılabilir. Kullanıcı özellikleri birlikte ya da ayrı ayrı sınar.

TEK DOĞRULUK KAYNAĞI: aşağıdaki REGISTRY. Yeni bir özellik eklemek =
REGISTRY'ye TEK bir sözlük girdisi eklemek. O girdi hem API'yi (GET/POST
/api/ozellikler) hem UI'yı (düğme veri-güdümlü üretilir) hem bbox_ibvs'in
okuduğu depoyu besler — başka hiçbir yeri elle güncellemek gerekmez.

SÖZLEŞME (byte-aynı garanti): depo, her özelliğin KENDİ env varsayılanıyla
tohumlanır (aşağıda _ilk_deger). Böylece hiçbir düğmeye dokunulmazsa depo
env varsayılanını (hepsi OFF) verir → davranış native/byte-aynı kalır. Env
hâlâ çalışır (CI/uçuş A-B env ile de açılabilir); UI yalnızca aynı bayrağı
çalışma anında değiştirir.

Bu modül GÜDÜM FORMÜLLERİNE dokunmaz; yalnız aç/kapa boolean'ının NEREDEN
geldiğini (import-anı env → çalışma-anı depo, aynı env ile tohumlu) taşır.
"""

import os
import threading


def _env_bool(name, default=False):
    """Aç/kapa bayrağı — on/off/1/0/true/false kabul eder (bbox_ibvs._env_bool
    ile AYNI sözleşme; depo tohumu bununla okunur ki env davranışı korunsun)."""
    v = os.environ.get(name)
    if v is None:
        return default
    return v.strip().lower() in ("on", "1", "true", "yes", "evet")


# ══════════════════════════════════════════════════════════════════════════
# REGISTRY — TEK DOĞRULUK KAYNAĞI
# ══════════════════════════════════════════════════════════════════════════
# Her girdi bir özelliği tanımlar:
#   anahtar  : kısa iç kimlik (API gövdesi + bbox_ibvs._ozellik("...") bunu kullanır)
#   env      : bu özelliğin env kill-switch adı (depo tohumu + geriye uyumluluk)
#   ad       : UI'da düğme başlığı
#   aciklama : UI'da düğme altı kısa açıklama (ne yapar)
# YENİ ÖZELLİK: buraya TEK satır ekle → düğme UI'da otomatik belirir, API'de
# otomatik listelenir, bbox_ibvs _ozellik(anahtar) ile çalışma-anı okur.
# NOT (2026-08-10): intercept/vlead/gainsched/commit uçuş testinde KÖTÜLEŞTİRDİ
# (vlead churn, gainsched terminal ateşlemedi, commit/intercept en-yakın geriledi)
# → CLAUDE.md "kötüleştiren ÇIKAR" gereği PANELDEN kaldırıldı. Kod hâlâ duruyor
# (bbox_ibvs kill-switch'leri, env AVCI_IBVS_* ile açılabilir) ama panelde YOK.
# Panelde yalnız KAZANAN config: pn+pred+tboost+duzterm + yapiskanlik.
REGISTRY = [
    {
        "anahtar": "pn",
        "env": "AVCI_IBVS_PN",
        "ad": "PN yatay lead",
        "aciklama": "Yatay/yaw kanalında PN-tipi lead (N·λ̇·t_go); "
                    "LOS dönüş hızını sıfıra sürer. Dikey vz'ye dokunmaz.",
    },
    {
        "anahtar": "pred",
        "env": "AVCI_IBVS_PRED",
        "ad": "Kestirim + coast",
        "aciklama": "Görüntü-düzlemi hedef kestirimi (alpha-beta); kutu "
                    "yokken donmuş komut yerine ileri-tahminle izlemeyi sürdürür.",
    },
    {
        "anahtar": "tboost",
        "env": "AVCI_IBVS_TBOOST",
        "ad": "Terminal hız artışı",
        "aciklama": "Terminal hücum hızını 18→~23 m/s çıkarır; hedef kadrajdan "
                    "kaçmadan son metreyi deler (net kapanma 3→8 m/s, ram).",
    },
    {
        "anahtar": "duzterm",
        "env": "AVCI_IBVS_DUZTERM",
        "ad": "Düz kuyruk (pure pursuit)",
        "aciklama": "Terminalde PN lead'i bastırır (±DUZTERM_LEAD_MAX ile kapar) → "
                    "yaw salınımını keser, hedefin ARKASINDAN düz gidip doğrudan "
                    "çarpar. 6 uçuş A/B: salınım yön-değişimi 279→7-19.",
    },
    {
        "anahtar": "yapiskanlik",
        "env": "AVCI_YAPISKANLIK",
        "ad": "Görsel yapışkanlık (GPS'e dönme)",
        "aciklama": "AÇIK: FSM kilit-kayıp toleransını (KILIT_KAYIP_SN) 2.0→3.5 s "
                    "yükseltir → görsel güdüme geçince kısa boşluklarda GPS'e "
                    "geri dönmez, tek dalışta vurur. Gerçek >3.5s kayıpta yine döner.",
    },
]


# ── ÇALIŞMA-ANI DEPO (iş parçacığı güvenli) ────────────────────────────────
# Her anahtar KENDİ env varsayılanıyla tohumlanır → dokunulmazsa byte-aynı.
_kilit = threading.RLock()
_durum = {g["anahtar"]: _env_bool(g["env"], False) for g in REGISTRY}


def get(anahtar):
    """anahtar özelliğinin çalışma-anı aç/kapa durumunu döner (bool).
    Bilinmeyen anahtar → False (native/güvenli varsayılan)."""
    with _kilit:
        return bool(_durum.get(anahtar, False))


def set(anahtar, deger):
    """anahtar özelliğini deger (bool) yapar; yeni durumu döner.
    REGISTRY'de olmayan anahtar sessizce yok sayılır (durum değişmez)."""
    with _kilit:
        if anahtar in _durum:
            _durum[anahtar] = bool(deger)
        return bool(_durum.get(anahtar, False))


def hepsi():
    """Tüm özelliklerin anlık durumunu {anahtar: bool} olarak döner (kopya)."""
    with _kilit:
        return dict(_durum)


def registry():
    """REGISTRY listesini döner (API + UI bunu okur; TEK doğruluk kaynağı)."""
    return REGISTRY
