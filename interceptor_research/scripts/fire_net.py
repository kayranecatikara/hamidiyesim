#!/usr/bin/env python3
"""Agi atar (Asama 4).

Tek mesaj: NetLauncherPlugin ayirma + cikis hizi vermeyi simulatorun icinde
ayni fizik adiminda yapar.

TARIHCE (neden boyle):
  Ilk surum hazir sistemleri kullaniyordu -
    gz-sim-detachable-joint-system  (ayirma)  +
    gz-sim-apply-link-wrench-system (impuls)
  Iki AYRI topic'e disaridan iki AYRI mesaj gerekiyordu. Aradaki gecikme
  kontrol edilemedigi icin impuls ya ag hala askidayken gelip yutuluyor
  ya da ag once dusuyordu; ayni parametrelerle olculen menzil kosumlar
  arasi 2 m ile 108 m arasinda oynadi. Bkz. plugins/NetLauncherPlugin.hh

Kullanim:
    ./fire_net.py                  # varsayilan 20 m/s
    ./fire_net.py --hiz 25
"""
import argparse
import sys
import time

from gz.msgs10.double_pb2 import Double
from gz.transport13 import Node

VARSAYILAN_MODEL = "avci_net_interceptor"


def main() -> int:
    ap = argparse.ArgumentParser(description="Ag firlatma")
    ap.add_argument("--hiz", type=float, default=20.0,
                    help="namlu cikis hizi (m/s). 20 m/s -> ~22.7 m menzil "
                         "(olculdu: docs/bench_raw/menzil_taramasi.csv); "
                         "DroneCatcher speci 20 m.")
    ap.add_argument("--model", default=VARSAYILAN_MODEL,
                    help=f"ates topic'i bundan turetilir (varsayilan: {VARSAYILAN_MODEL})")
    ap.add_argument("--topic", default=None,
                    help="ates topic'ini dogrudan ver (--model'i gecersiz kilar)")
    # Geriye donuk uyumluluk: 40_verify.sh bunlari geciyor ama artik
    # atis yonunu taretin KENDI yonelimi belirliyor (eklenti namludan okuyor).
    ap.add_argument("--pan", type=float, default=None, help=argparse.SUPPRESS)
    ap.add_argument("--tilt", type=float, default=None, help=argparse.SUPPRESS)
    ap.add_argument("--dunya", default=None, help=argparse.SUPPRESS)
    args = ap.parse_args()

    if args.pan is not None or args.tilt is not None:
        print("  not: --pan/--tilt artik yok sayiliyor; atis yonu namlunun "
              "gercek yoneliminden aliniyor (taret nereye bakiyorsa oraya).")

    topic = args.topic if args.topic else f"/{args.model}/net/fire"

    node = Node()
    pub = node.advertise(topic, Double)
    time.sleep(0.4)  # discovery

    msg = Double()
    msg.data = args.hiz

    print(f"ATES -> {topic}  (cikis hizi {args.hiz} m/s)")
    if not pub.publish(msg):
        print("HATA: atesleme mesaji yayinlanamadi", file=sys.stderr)
        return 1

    time.sleep(0.2)   # eklentinin ayirma + hiz adimlarini tamamlamasi icin
    print("Gonderildi. Yorunge icin: scripts/net_track.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
