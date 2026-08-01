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

import os
import threading
import time

from control.guidance import gps_guidance as _ga
from control.guidance.gps_guidance import run_gps_guidance
from control.guidance.guidance_core import Cfg as LeadCfg
from control.guidance.visual_lead import run_visual_lead


class SupCfg:
    KILIT_N = 10          # ardışık güvenli pose karesi → görsel faza geç (~0.33 s)
    KAYIP_M = 20          # ardışık pose'suz kare → GPS'e dön (~0.66 s)
    POSE_CONF_MIN = 0.5
    GATE_KILIT = True     # geçiş için menzil kapısı (VEYA GPS DROPOUT — jamming)
    # Devir menzili: GPS handoff bayrağı 40 m'de açılıyor ama orada kutu ~7 px,
    # pose güvenilmez (uzakta devralınca hedef hemen kaçtı — 2026-07-24 log).
    # 20 m'de kutu ~7 px hâlâ küçük; pose asıl 10-12 m'de sağlam. GPS istasyonu
    # 10 m; kapı 20 → GPS yaklaşırken pose kilidini bu banda çeker.
    GATE_MENZIL = float(os.environ.get("AVCI_HYBRID_GATE_MENZIL", 20.0))


# Telemetri/arayüz için son durum (gcs_server okur; salt gözlem)
status = {"faz": "GPS", "gecis_sayisi": 0, "kilit_sayac": 0, "son_sebep": None,
          # Devir kapısı teşhisi (izci doldurur; /api/chase_status ile okunur)
          "pose_toplam": 0, "pose_var": 0, "pose_guvenli": 0, "kilit_en_uzun": 0}


def _kopru(parent_event, child_event):
    """parent set olunca child'ı da set eder (faz thread'i ana stop'u duysun)."""
    def izle():
        while not parent_event.is_set() and not child_event.is_set():
            parent_event.wait(0.5)
        if parent_event.is_set():
            child_event.set()
    threading.Thread(target=izle, daemon=True).start()


def run_hybrid(conn, get_plane, get_iris, wait_pose, get_plane_truth,
               stop_event, sup_cfg=SupCfg, lead_cfg=LeadCfg, gercek=None):
    """gercek: Gazebo ground truth okuyucusu (gz_truth.get_ikisi) — görsel faza
    aktarılır, yalnız CSV doğruluk kolonları için. Güdüme girmez."""
    status.update(faz="GPS", gecis_sayisi=0, kilit_sayac=0, son_sebep=None,
                  pose_toplam=0, pose_var=0, pose_guvenli=0, kilit_en_uzun=0)

    while not stop_event.is_set():
        # ══ GPS FAZI ══ (gps_guidance kendi 20 Hz döngüsünde; izci pose akışını sayar)
        status["faz"] = "GPS"
        faz_stop = threading.Event()
        _kopru(stop_event, faz_stop)
        tetik = {"gorsel": False}

        def izci():
            sayac, son_seq = 0, 0
            # Devir kapısı açılmadığında SEBEBİNİ söyleyebilmek için sayaçlar.
            # 2026-08-01 ölçümünde menzil 5.5 m'ye indi ama kilit 4'te kaldı ve
            # NEDEN kaldığı hiçbir logdan çıkarılamadı: GPS fazı sırasındaki pose
            # tespitleri hiçbir yere yazılmıyordu. Bu sayaçlar "hedef hiç
            # görünmedi mi, göründü de güven mi düşüktü, yoksa görünüp kayboldu
            # mu" sorusunu ayırır.
            top = var = guvenli = 0
            en_uzun = 0
            son_rapor = time.time()
            while not faz_stop.is_set():
                kayit = wait_pose(son_seq, timeout=0.5)
                if kayit is None:
                    continue
                son_seq = kayit["seq"]
                pose = kayit["pose"]
                top += 1
                if pose is not None:
                    var += 1
                if pose is not None and pose.get("conf", 0.0) >= sup_cfg.POSE_CONF_MIN:
                    guvenli += 1
                    sayac += 1
                    en_uzun = max(en_uzun, sayac)
                else:
                    sayac = 0
                status.update(kilit_sayac=sayac, pose_toplam=top, pose_var=var,
                              pose_guvenli=guvenli, kilit_en_uzun=en_uzun)
                if time.time() - son_rapor >= 10.0 and top:
                    son_rapor = time.time()
                    d_h = _ga.status.get("d_h")
                    print(f"[SUPERVISOR] pose: {var}/{top} kare tespit "
                          f"(%{100.0*var/top:.0f}), {guvenli} güvenli, "
                          f"en uzun ardışık {en_uzun}/{sup_cfg.KILIT_N}"
                          + (f", d_h={d_h:.0f}m" if d_h is not None else ""))
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
                                gercek=gercek)
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
