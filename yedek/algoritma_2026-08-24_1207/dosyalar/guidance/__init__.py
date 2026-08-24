"""
control.guidance — Avcı drone güdüm hatları.

İki fazlı hibrit müdahale (GPS kadraj merkezleme → görsel IBVS), supervisor
geçişli:
  - gps_guidance.py  : GPS fazı — hedefi kamera kadrajının MERKEZİNE ve pose
                       modelinin güvenilir çalıştığı menzil bandına oturtur
                       (geometrik kadraj noktası + PD hız + hedef-hızı feedforward)
  - guidance_core.py : IBVS lead pursuit çekirdeği (platformdan bağımsız:
                       pose keypoint → menzil bağımsız lead → u_govde/hata açıları)
  - adapter_copter.py: copter komut adaptörü (u_govde → NED hız + yaw;
                       dikey yumuşatma/PN/co-altitude)
  - visual_lead.py   : IBVS döngüsü (olay güdümlü, kameraya kilitli, CSV log,
                       terminal kör-dalış)
  - supervisor.py    : GPS ↔ görsel faz geçiş denetleyicisi (run_hybrid)
  - common.py        : paylaşılan matematik + send_velocity (MAVLink GUIDED)

Mimari (mevcut gerçeklik): docs/GUIDANCE.md
Yol haritası (vizyon):     docs/GUIDANCE_ROADMAP.md
"""
