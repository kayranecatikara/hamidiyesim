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

    # ── GEOMETRİ KAPISI (2026-08-04): menzil TEK BAŞINA yetmiyor ──
    # Ölçüm (visual_lead_20260801_173610, tek gerçek devir): devir 8-10 m'de
    # OLDU — menzil kapısı sağlandı — ama geometri felaketti: yandanlik_f 0.89-1.03
    # (tam YANDAN geçiş) ve nişan hatası daha ilk karede −51°. 0.30 s sonra bbox
    # x=232'den 18'e yürüdü, sonraki 20 kare tespit_yok. Yani kapı "yakınız"
    # diyordu ama devraldığımız an hedef zaten kadrajdan çıkmak üzereydi.
    #
    # İki ek koşul, ikisi de GPS fazının zaten ölçtüğü büyüklükler:
    #  (a) KADRAJ: hedef kamera merkezine yakın olsun. GPS fazı bunu YAPABİLİYOR —
    #      aynı gün 22784 karede |kadraj_yaw| medyanı 2.8°, p90 6.7°. Yani 25°
    #      eşiği fazın normal çalışmasını engellemez, yalnız savrulma anında
    #      devri geciktirir.
    #  (b) KUYRUK AÇISI: hedefin arkasında olalım. Pose modeli kuyruktan bakışta
    #      en güvenilir, LOS açısal hızı orada en düşük ve lead yasası orada
    #      küçük düzeltmeyle çalışır. 60° eşiği yandanlık ≤ 0.87 demektir —
    #      ölçülen 0.89-1.03'ü keser, ama kapıyı imkânsız kılacak kadar dar değil.
    #
    # İKİSİ DE GEVŞETİLEBİLİR: kapı hiç açılmıyorsa asıl sorun geometridir,
    # eşik değil — [SUPERVISOR] teşhis satırı hangi koşulun bloklandığını yazar.
    GATE_KADRAJ_YAW = float(os.environ.get("AVCI_HYBRID_GATE_KADRAJ", 25.0))   # °
    GATE_KUYRUK_ACI = float(os.environ.get("AVCI_HYBRID_GATE_KUYRUK", 60.0))   # °


# Telemetri/arayüz için son durum (gcs_server okur; salt gözlem)
status = {"faz": "GPS", "gecis_sayisi": 0, "kilit_sayac": 0, "son_sebep": None,
          # Devir kapısı teşhisi (izci doldurur; /api/chase_status ile okunur)
          "pose_toplam": 0, "pose_var": 0, "pose_guvenli": 0, "kilit_en_uzun": 0,
          # Devir kapısını o an HANGİ koşulun bloklandığı (arayüz + uçuş sonrası
          # teşhis). _kapi_degerlendir doldurur.
          "kapi_engel": None}


def _kapi_degerlendir(sup_cfg):
    """Devir kapısı: açık mı, değilse HANGİ koşul blokluyor.

    Dönüş: (acik: bool, sebep: str). sebep her iki durumda da doldurulur —
    kapı açılmadığında "neden açılmadı" sorusu uçuş sonrası logdan
    cevaplanabilsin diye (2026-08-01 uçuşunda kilit 4'te kaldı ve sebebi
    hiçbir yerden çıkarılamadı).

    GPS DROPOUT (jamming) tüm geometri koşullarını ATLAR: menzil de kadraj da
    hedef telemetrisine dayanır, telemetri yoksa ölçülemezler. O durumda görsel
    temasın kendisi tek kapıdır — zaten jamming fallback'inin amacı budur.
    """
    if not sup_cfg.GATE_KILIT:
        return True, "kapı kapalı (GATE_KILIT=False)"
    if _ga.status.get("durum") == "DROPOUT":
        return True, "GPS DROPOUT — jamming fallback"

    d_h = _ga.status.get("d_h")
    if d_h is None or d_h >= sup_cfg.GATE_MENZIL:
        return False, f"menzil {d_h if d_h is not None else '?'} ≥ {sup_cfg.GATE_MENZIL:.0f} m"

    kad = _ga.status.get("kadraj_yaw_deg")
    if kad is not None and abs(kad) > sup_cfg.GATE_KADRAJ_YAW:
        return False, f"kadraj yaw {kad:+.0f}° > {sup_cfg.GATE_KADRAJ_YAW:.0f}°"

    kuy = _ga.status.get("kuyruk_aci_deg")
    if kuy is not None and kuy > sup_cfg.GATE_KUYRUK_ACI:
        return False, f"kuyruk açısı {kuy:.0f}° > {sup_cfg.GATE_KUYRUK_ACI:.0f}° (yandan)"

    return True, (f"menzil {d_h:.1f} m, kadraj {kad if kad is not None else '?'}°, "
                  f"kuyruk {kuy if kuy is not None else '?'}°")


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
                  pose_toplam=0, pose_var=0, pose_guvenli=0, kilit_en_uzun=0,
                  kapi_engel=None)

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
                    _, engel = _kapi_degerlendir(sup_cfg)
                    status["kapi_engel"] = engel
                    print(f"[SUPERVISOR] pose: {var}/{top} kare tespit "
                          f"(%{100.0*var/top:.0f}), {guvenli} güvenli, "
                          f"en uzun ardışık {en_uzun}/{sup_cfg.KILIT_N} | kapı: {engel}")
                if sayac >= sup_cfg.KILIT_N:
                    kapi, sebep = _kapi_degerlendir(sup_cfg)
                    status["kapi_engel"] = sebep
                    if kapi:
                        print(f"[SUPERVISOR] devir kapısı açıldı ({sebep})")
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
