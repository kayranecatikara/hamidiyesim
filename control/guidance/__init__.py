"""
control.guidance — Avcı drone güdüm hatları.

İki fazlı hibrit müdahale (GPS kadraj merkezleme → görsel IBVS), supervisor
geçişli:
  - gps_guidance.py  : GPS fazı — hedefi kamera kadrajının MERKEZİNE ve tespit
                       modelinin güvenilir çalıştığı menzil bandına oturtur
                       (geometrik kadraj noktası + PD hız + hedef-hızı feedforward)
  - guidance_core.py : IBVS lead pursuit çekirdeği (platformdan bağımsız:
                       bbox merkezi → saf takip; lead adaptörün azimut-oranı
                       kanalında (adapter_copter._yatay_pn) üretilir)
  - adapter_copter.py: copter komut adaptörü (u_govde → NED hız + yaw;
                       dikey yumuşatma/PN/co-altitude)
  - visual_lead.py   : IBVS döngüsü (olay güdümlü, kameraya kilitli, CSV log,
                       terminal kör-dalış)
  - supervisor.py    : GPS ↔ görsel faz geçiş denetleyicisi (run_hybrid)
  - common.py        : paylaşılan matematik + send_velocity (MAVLink GUIDED)

Mimari (mevcut gerçeklik): docs/GUIDANCE.md
Yol haritası (vizyon):     docs/GUIDANCE_ROADMAP.md
"""
