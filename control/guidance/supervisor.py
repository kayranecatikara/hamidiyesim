"""
supervisor.py — Faz 4: GPS ↔ görsel güdüm geçişi (hibrit müdahale).

run_hybrid tek görev döngüsüdür (start_chase bunu çalıştırır):

  GPS fazı (gps_approach) hedefe yaklaşır. Görsel temas oturunca
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

import threading

from control.guidance import gps_approach as _ga
from control.guidance.gps_approach import run_gps_approach
from control.guidance.guidance_core import Cfg as LeadCfg
from control.guidance.visual_lead import run_visual_lead


class SupCfg:
    KILIT_N = 10          # ardışık güvenli pose karesi → görsel faza geç (~0.33 s)
    KAYIP_M = 20          # ardışık pose'suz kare → GPS'e dön (~0.66 s)
    POSE_CONF_MIN = 0.5
    GATE_KILIT = True     # geçiş için handoff (≤40 m) VEYA GPS DROPOUT şartı


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
               stop_event, sup_cfg=SupCfg, lead_cfg=LeadCfg):
    status.update(faz="GPS", gecis_sayisi=0, kilit_sayac=0, son_sebep=None)

    while not stop_event.is_set():
        # ══ GPS FAZI ══ (gps_approach kendi 20 Hz döngüsünde; izci pose akışını sayar)
        status["faz"] = "GPS"
        faz_stop = threading.Event()
        _kopru(stop_event, faz_stop)
        tetik = {"gorsel": False}

        def izci():
            sayac, son_seq = 0, 0
            while not faz_stop.is_set():
                kayit = wait_pose(son_seq, timeout=0.5)
                if kayit is None:
                    continue
                son_seq = kayit["seq"]
                pose = kayit["pose"]
                if pose is not None and pose.get("conf", 0.0) >= sup_cfg.POSE_CONF_MIN:
                    sayac += 1
                else:
                    sayac = 0
                status["kilit_sayac"] = sayac
                if sayac >= sup_cfg.KILIT_N:
                    kapi = ((not sup_cfg.GATE_KILIT)
                            or _ga.status.get("handoff")
                            or _ga.status.get("durum") == "DROPOUT")
                    if kapi:
                        tetik["gorsel"] = True
                        faz_stop.set()          # gps_approach döngüsünü kır
                        return

        threading.Thread(target=izci, daemon=True).start()
        print(f"[SUPERVISOR] GPS fazı (görsel kilit: {sup_cfg.KILIT_N} ardışık kare"
              f"{' + handoff/DROPOUT kapısı' if sup_cfg.GATE_KILIT else ''})")
        run_gps_approach(conn, get_plane, get_iris, faz_stop)

        if stop_event.is_set() or not tetik["gorsel"]:
            break

        # ══ GÖRSEL FAZ ══ (temas kesilene ya da stop'a kadar)
        status["faz"] = "VISUAL"
        status["gecis_sayisi"] += 1
        print(f"[SUPERVISOR] ✓ GÖRSEL TEMAS — görsel güdüme geçildi "
              f"(geçiş #{status['gecis_sayisi']})")
        sebep = run_visual_lead(conn, wait_pose, get_plane_truth, stop_event,
                                cfg=lead_cfg, kayip_kare_esik=sup_cfg.KAYIP_M)
        status["son_sebep"] = sebep
        if sebep == "kayip":
            print("[SUPERVISOR] Görsel temas kesildi → GPS fazına dönülüyor")
            continue
        break                                    # durduruldu

    status["faz"] = "DURDU"
    print("[SUPERVISOR] Hibrit güdüm sonlandı.")
