"""
supervisor.py — Faz 4: GPS ↔ görsel güdüm geçişi (hibrit müdahale).

run_hybrid tek görev döngüsüdür (start_chase bunu çalıştırır):

  GPS fazı (gps_guidance) hedefe yaklaşır. Görsel temas oturunca
  (KILIT_N ardışık pose karesi, conf ≥ POSE_CONF_MIN, VE handoff menzili
  içindeyiz YA DA GPS düşmüş/DROPOUT) → GÖRSEL faza (visual_lead) geçilir.
  Görsel temas kesilirse (KAYIP_M ardışık pose'suz kare veya kare akışının
  durması) → GPS fazına dönülür. stop_chase gelene (veya araç vurulana)
  kadar bu döngü sürer.

Menzil kapısının (GATE_KILIT) nedeni: görsel fazın kapanma hızı sabit
(V_KAPANMA); uzaktan erken geçilirse hızlı hedefe yetişilemez. GPS handoff
histerezisi (≤40 m) zaten "yetişilmiş" durumu işaretler. GPS jam/DROPOUT'ta
menzil bilinemez → görsel temas tek başına yeter (jamming fallback).
"""

import collections
import os
import threading

from control.guidance import gps_guidance as _ga
from control.guidance.gps_guidance import run_gps_guidance
from control.guidance.guidance_core import Cfg as LeadCfg
from control.guidance.visual_lead import run_visual_lead


class SupCfg:
    # ── DEVİR KAPISI: ARDIŞIK → KAYAN PENCERE (2026-07-31) ──
    # Eskiden KILIT_N ARDIŞIK güvenli kare aranıyordu. Tespit gürültülü olduğu
    # için (gerçek uçuşlarda karelerin yalnız %12'si temiz `ok`) bu şart çok geç
    # sağlanıyordu: devir kapısı 20 m'ye ayarlı olmasına rağmen görsel faz
    # 6-10 m'de başlıyordu ve elinde 0.6-1.9 s kalıyordu — hedefin 4.65 m altında
    # devralınan dikey farkı kapatmaya yetmiyor.
    # Kayan pencere aynı güveni verir ama tek bir kötü kare sayacı sıfırlamaz.
    # ⚠ 10 → 7 DENENDİ VE GERİ ALINDI (2026-08-02). DÜŞÜRMEYİN.
    #
    # Gerekçe iyiydi: A5 sonrası 17 geçişte vuranlar görsel faza medyan
    # 11.11 m'de, ıskalayanlar 9.05 m'de girmişti. Kapıyı gevşetmek devri
    # uzaklaştırıp terminale daha çok tırmanma süresi bırakacaktı.
    #
    # Ölçüm bunu ÇÜRÜTTÜ — her ölçütte kötüleşti:
    #
    #                        KILIT_N=10 (5 uçuş)   KILIT_N=7 (1 uçuş)
    #   faz / uçuş                  3.4                  8.0
    #   giriş menzili medyan      10.00 m               9.62 m   ← DÜŞTÜ
    #   en yakın menzil medyan     1.73 m               2.08 m
    #   kor_dalis medyan            %19                  %27
    #   <1.5 s'de kopan faz        2/17                  4/8
    #   vuruş                      3/17                 1/8
    #
    # MEKANİZMA: kapı cılız tespitte de açılıyor. Erken devirler gerçekten
    # oluyor (14.73 m, 10.47 m'de girdi) ama hemen ölüyor — o iki faz 0.9 ve
    # 1.3 s sürdü, birinde kareler %69 kör_dalış, diğerinde %100 tespit_yok.
    # Faz KAYIP_M yiyip GPS'e dönüyor, drone bu arada yaklaşmış oluyor, bir
    # sonraki devir DAHA YAKINDA gerçekleşiyor. Net etki ters.
    #
    # Yani devir menzili ile vuruş arasındaki bağıntı nedensel DEĞİL: ikisi de
    # "tespit o an gerçekten sağlam mı"ya bağlı. Kapıyı gevşetmek sağlamlığı
    # üretmiyor, sadece sağlam sanılan anları çoğaltıyor.
    # Asıl kaldıraç terminal algı sürekliliği (vuran 4 geçişin dördünde de
    # kor_dalis ≤ %3) — bkz. UYGULANACAK.md B6.
    KILIT_N = int(os.environ.get("AVCI_HYBRID_KILIT_N", 10))
    KILIT_PENCERE = 15    # kayan pencere boyu (~0.5 s @30 Hz)
    KAYIP_M = 20          # ardışık pose'suz kare → GPS'e dön (~0.66 s)
    POSE_CONF_MIN = 0.5
    GATE_KILIT = True     # geçiş için menzil kapısı (VEYA GPS DROPOUT — jamming)
    # Devir menzili: GPS handoff bayrağı 40 m'de açılıyor ama orada kutu ~7 px,
    # pose güvenilmez (uzakta devralınca hedef hemen kaçtı — 2026-07-24 log).
    # 20 m'de kutu ~7 px hâlâ küçük; pose asıl 10-12 m'de sağlam. GPS istasyonu
    # 10 m; kapı 20 → GPS yaklaşırken pose kilidini bu banda çeker.
    GATE_MENZIL = float(os.environ.get("AVCI_HYBRID_GATE_MENZIL", 20.0))




# Telemetri/arayüz için son durum (gcs_server okur; salt gözlem)
status = {"faz": "GPS", "gecis_sayisi": 0, "kilit_sayac": 0, "son_sebep": None}


def _kopru(parent_event, child_event):
    """parent set olunca child'ı da set eder (faz thread'i ana stop'u duysun)."""
    def izle():
        while not parent_event.is_set() and not child_event.is_set():
            parent_event.wait(0.5)
        if parent_event.is_set():
            child_event.set()
    threading.Thread(target=izle, daemon=True).start()


def run_hybrid(conn, get_plane, get_iris, wait_pose, get_plane_truth,
               stop_event, sup_cfg=SupCfg, lead_cfg=LeadCfg, get_temas=None,
               get_menzil=None, get_gercek=None):
    status.update(faz="GPS", gecis_sayisi=0, kilit_sayac=0, son_sebep=None)

    while not stop_event.is_set():
        # ══ GPS FAZI ══ (gps_guidance kendi 20 Hz döngüsünde; izci pose akışını sayar)
        status["faz"] = "GPS"
        faz_stop = threading.Event()
        _kopru(stop_event, faz_stop)
        tetik = {"gorsel": False}

        def izci():
            pencere = collections.deque(maxlen=sup_cfg.KILIT_PENCERE)
            son_seq = 0
            while not faz_stop.is_set():
                kayit = wait_pose(son_seq, timeout=0.5)
                if kayit is None:
                    continue
                son_seq = kayit["seq"]
                pose = kayit["pose"]
                pencere.append(pose is not None
                               and pose.get("conf", 0.0) >= sup_cfg.POSE_CONF_MIN)
                sayac = sum(pencere)          # kayan pencerede güvenli kare sayısı
                status["kilit_sayac"] = sayac
                if sayac >= sup_cfg.KILIT_N:
                    d_h = _ga.status.get("d_h")
                    yakin = (d_h is not None and d_h < sup_cfg.GATE_MENZIL)
                    dropout = _ga.status.get("durum") == "DROPOUT"  # jamming fallback
                    kapi = (not sup_cfg.GATE_KILIT) or yakin or dropout
                    if kapi:
                        tetik["gorsel"] = True
                        faz_stop.set()          # gps_guidance döngüsünü kır
                        return

        threading.Thread(target=izci, daemon=True).start()
        print(f"[SUPERVISOR] GPS fazı (görsel kilit: {sup_cfg.KILIT_N} ardışık kare"
              f"{' + handoff/DROPOUT kapısı' if sup_cfg.GATE_KILIT else ''})")
        run_gps_guidance(conn, get_plane, get_iris, faz_stop)

        if stop_event.is_set() or not tetik["gorsel"]:
            break

        # ══ GÖRSEL FAZ ══ (temas kesilene ya da stop'a kadar)
        status["faz"] = "VISUAL"
        status["gecis_sayisi"] += 1
        print(f"[SUPERVISOR] ✓ GÖRSEL TEMAS — görsel güdüme geçildi "
              f"(geçiş #{status['gecis_sayisi']})")
        sebep = run_visual_lead(conn, wait_pose, get_plane_truth, stop_event,
                                cfg=lead_cfg, kayip_kare_esik=sup_cfg.KAYIP_M,
                                get_temas=get_temas, get_menzil=get_menzil,
                                get_gercek=get_gercek)
        status["son_sebep"] = sebep
        if sebep == "vuruldu":
            status["faz"] = "VURULDU"
            print("[SUPERVISOR] ✓✓ HEDEF VURULDU — görev tamamlandı.")
            return
        if sebep == "kayip":
            print("[SUPERVISOR] Görsel temas kesildi → GPS fazına dönülüyor")
            continue
        break                                    # durduruldu

    status["faz"] = "DURDU"
    print("[SUPERVISOR] Hibrit güdüm sonlandı.")
