"""
control/sim_truth.py — SİM GERÇEK POZ köprüsü (gz-transport, "vuruldu" doğrusu).

Neden: MAVLink'ten türetilen menzil_gercek İKİ AYRI telemetri akışının zaman
hizasız farkıydı (plane pozu efektif ~10 Hz, 250 ms'ye kadar bayat) → 20-25 m/s
kapanmada 5-6 m hata; 105022 logunda menzil AÇILIRKEN tek örnekte −2.88 m/33 ms
sıçrayıp sahte "vuruldu" üretti (2026-08-02 teşhisi). Burada iki aracın pozu
Gazebo'nun TEK pose mesajından okunur: zaman hizalı, fizik doğruluğunda.
Gerçek donanımda bu görevi yakınlık/menzil sensörü üstlenir.

Kullanım (gcs_server startup):
    from control import sim_truth
    sim_truth.start()          # AVCI_TRUTH=off kapatır; gz yoksa sessizce False
    m = sim_truth.menzil()     # araçlar arası gerçek mesafe (m) veya None
"""

import math
import os
import threading
import time

_WORLD = os.environ.get("AVCI_TRUTH_WORLD", "avci")
_IRIS = os.environ.get("AVCI_TRUTH_IRIS", "iris_with_ardupilot")
_PLANE = os.environ.get("AVCI_TRUTH_PLANE", "mini_talon")
# dynamic_pose/info yalnız hareketli modelleri taşır (canlı uçuş için doğru ve
# hafif); statik-modelli test dünyalarında pose/info gerekir → env ile seçilir.
_TOPIC = os.environ.get("AVCI_TRUTH_TOPIC", "dynamic_pose/info")
_MAX_YAS_S = 0.5                # bundan eski ölçüm "yok" sayılır
# Temas sensörü (mini_talon_vtail base_link'teki "temas_sensoru"): fizik motoru
# GERÇEK temas olayı yayınlar — vuruş tespitinin altın standardı.
_TEMAS_TOPIC = os.environ.get(
    "AVCI_TEMAS_TOPIC",
    f"/world/{_WORLD}/model/{_PLANE}/link/base_link/sensor/temas_sensoru/contact")

_lock = threading.Lock()
_son = {"menzil": None, "iris": None, "plane": None, "wall": 0.0, "msg_sayisi": 0}
_temas = {"mevcut": False,      # topic dünya üzerinde VAR (sensör+eklenti kurulu)
          "iris_temas": False,  # iris'le temas görüldü (LATCH — sıfırlanana dek)
          "son_detay": None,    # son temasın collision adı (log için)
          "msg_sayisi": 0}
_node = None                    # gz Node referansı (GC olmasın)


def _cb(msg):
    poz = {}
    for p in msg.pose:
        if p.name in (_IRIS, _PLANE):
            o = p.orientation
            poz[p.name] = {"pos": (p.position.x, p.position.y, p.position.z),
                           "quat": (o.w, o.x, o.y, o.z)}
    if len(poz) == 2:           # iki araç da AYNI mesajda → zaman hizalı
        a, b = poz[_IRIS]["pos"], poz[_PLANE]["pos"]
        d = math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)
        with _lock:
            _son["menzil"] = d
            _son["iris"] = poz[_IRIS]
            _son["plane"] = poz[_PLANE]
            _son["wall"] = time.time()
            _son["msg_sayisi"] += 1


def _temas_cb(msg):
    """Contacts mesajı: talon'un HERHANGİ bir temasında gelir. Yalnız karşı
    tarafı iris olan temas vuruş sayılır (yer/pist temasları elenir)."""
    with _lock:
        _temas["msg_sayisi"] += 1
    for c in msg.contact:
        adlar = (getattr(c.collision1, "name", ""), getattr(c.collision2, "name", ""))
        for ad in adlar:
            if _IRIS in ad:
                with _lock:
                    if not _temas["iris_temas"]:
                        print(f"[TRUTH] ✓ FİZİKSEL TEMAS: {adlar[0]} ↔ {adlar[1]}")
                    _temas["iris_temas"] = True
                    _temas["son_detay"] = f"{adlar[0]} ↔ {adlar[1]}"
                return


def _temas_topic_var():
    """Temas topic'i dünyada yayında mı? (sensör+eklenti kurulu değilse vuruş
    kararı menzil fallback'inde kalmalı — sessizce asla-vurulmaz olmamalı)."""
    import subprocess
    try:
        out = subprocess.run(["gz", "topic", "-l"], capture_output=True,
                             text=True, timeout=10).stdout
        return _TEMAS_TOPIC in out
    except Exception:
        return False


def start():
    """gz pose aboneliğini başlat. Dönüş: True (dinliyor) / False (kapalı/yok).
    dynamic_pose/info yalnız HAREKETLİ modelleri içerir — iki araç da uçuşta
    hareketli; yerde park hâlindeyken mesaj gelmeyebilir, menzil() None döner
    (visual_lead bu durumda telemetri fallback'ine düşer)."""
    global _node
    if _node is not None:
        return True
    if os.environ.get("AVCI_TRUTH", "on").lower() in ("off", "0"):
        print("[TRUTH] Kapalı (AVCI_TRUTH=off) — menzil telemetriden hesaplanacak")
        return False
    try:
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
    except ImportError as e:
        print(f"[TRUTH] gz-transport yok ({e}) — menzil telemetriden hesaplanacak")
        return False
    node = Node()
    topic = f"/world/{_WORLD}/{_TOPIC}"
    if not node.subscribe(Pose_V, topic, _cb):
        print(f"[TRUTH] Abonelik başarısız: {topic}")
        return False
    _node = node
    print(f"[TRUTH] Gerçek menzil kaynağı hazır: {topic} "
          f"({_IRIS} ↔ {_PLANE}; AVCI_TRUTH=off kapatır)")
    # Temas sensörü: topic gerçekten yayındaysa abone ol ve vuruş kararını
    # temasa devret; yoksa menzil yolu geçerli kalır.
    if _temas_topic_var():
        from gz.msgs10.contacts_pb2 import Contacts
        if node.subscribe(Contacts, _TEMAS_TOPIC, _temas_cb):
            with _lock:
                _temas["mevcut"] = True
            print(f"[TRUTH] Temas sensörü hazır: {_TEMAS_TOPIC} — "
                  f"vuruş kararı FİZİKSEL TEMASTAN")
        else:
            print(f"[TRUTH] Temas aboneliği başarısız — vuruş menzilden")
    else:
        print("[TRUTH] Temas topic'i yayında değil (sensör/eklenti eski dünya?) "
              "— vuruş menzilden")
    return True


def menzil(max_yas=_MAX_YAS_S):
    """Araçlar arası GERÇEK mesafe (m). Taze ölçüm yoksa None (fallback'e düş)."""
    with _lock:
        if _son["menzil"] is None or time.time() - _son["wall"] > max_yas:
            return None
        return _son["menzil"]


def pozlar(max_yas=_MAX_YAS_S):
    """İki aracın GERÇEK pozu + yönelimi (aynı mesajdan, zaman hizalı):
    {"iris": {"pos": (x,y,z), "quat": (w,x,y,z)}, "plane": {...}} veya None.
    GT rotasyon modu (görsel güdüm lead yönü) bunu kullanır."""
    with _lock:
        if _son["iris"] is None or time.time() - _son["wall"] > max_yas:
            return None
        return {"iris": dict(_son["iris"]), "plane": dict(_son["plane"])}


def temas():
    """Vuruş kararı için üç durumlu temas sorgusu:
    True  = iris'le fiziksel temas OLDU (latch — temas_sifirla'ya dek kalır)
    False = temas akışı KURULU, temas yok (menzil vuruşu devre dışı bırakılır)
    None  = temas sensörü yok/kapalı → menzil fallback geçerli"""
    with _lock:
        if not _temas["mevcut"]:
            return None
        return _temas["iris_temas"]


def temas_sifirla():
    """Yeni görev başlangıcında latch temizlenir (önceki uçuşun teması yeni
    görevde anında 'vuruldu' üretmesin)."""
    with _lock:
        _temas["iris_temas"] = False
        _temas["son_detay"] = None


def durum():
    """Gözlem/status için özet."""
    with _lock:
        yas = (time.time() - _son["wall"]) if _son["menzil"] is not None else None
        return {"aktif": _node is not None, "menzil": _son["menzil"],
                "yas_s": yas, "msg_sayisi": _son["msg_sayisi"],
                "temas_mevcut": _temas["mevcut"],
                "iris_temas": _temas["iris_temas"],
                "temas_detay": _temas["son_detay"],
                "temas_msg": _temas["msg_sayisi"]}
