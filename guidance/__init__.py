"""
guidance — Avcı drone güdüm hatları (algoritma türüne göre klasörlenmiş).

Alt klasörler:
  gps_gudum/       — SAF GPS/telemetri tabanlı güdüm
    - gps_approach.py: VARSAYILAN GPS yaklaşma — eski sistemin (ana_kontrol.py)
                       kanıtlanmış güdüm yasasının portu (standoff + lead +
                       speed_cap fren + look-up alttan bakış + handoff histerezisi)
    - gps_chase.py   : chase v2 (SPRINT→APPROACH→LOCK→STRIKE) — AVCI_GPS_LAW=v2 ile
    - gps_strike.py  : SAF GPS terminal vuruş (Proportional Navigation)

  gorsel_gudum/    — Kamera/pose tabanlı IBVS güdüm
    - guidance_core.py    : IBVS lead pursuit çekirdeği (platformdan bağımsız:
                            pose keypoint → menzil bağımsız lead → u_govde/hata açıları)
    - visual_lead.py      : IBVS döngüsü (olay güdümlü, kameraya kilitli, CSV log)
    - adapter_copter.py   : copter komut adaptörü (u_govde → NED hız + yaw)
    - adapter_fixedwing.py: sabit kanat adaptörü (STUB — NotImplementedError)

  hibrit_gudum/    — GPS ↔ görsel geçiş yönetimi
    - supervisor.py  : hibrit müdahale döngüsü (GPS fazı → görsel faz geçişi)

  ortak/           — Hatların paylaştığı yardımcılar
    - common.py      : EMA, PID, setpoint, matematik yardımcıları

Ayrıntılı yol haritası: docs/GUIDANCE_ROADMAP.md
"""
