"""
control.guidance — Avcı drone güdüm hatları.

İki ayrı güdüm hattı:
  - gps_approach.py: VARSAYILAN GPS yaklaşma — eski sistemin (ana_kontrol.py)
                     kanıtlanmış güdüm yasasının portu (standoff + lead +
                     speed_cap fren + look-up alttan bakış + handoff histerezisi)
  - gps_chase.py   : chase v2 (SPRINT→APPROACH→LOCK→STRIKE) — AVCI_GPS_LAW=v2 ile
  - gps_strike.py  : SAF GPS terminal vuruş (Proportional Navigation)
  - guidance_core.py    : IBVS lead pursuit çekirdeği (platformdan bağımsız:
                          pose keypoint → menzil bağımsız lead → u_govde/hata açıları)
  - adapter_copter.py   : copter komut adaptörü (u_govde → NED hız + yaw)
  - adapter_fixedwing.py: sabit kanat adaptörü (STUB — NotImplementedError)
  - visual_lead.py      : IBVS döngüsü (olay güdümlü, kameraya kilitli, CSV log)
  - common.py      : hatların paylaştığı yardımcılar (EMA, PID, setpoint, matematik)

Ayrıntılı yol haritası: docs/GUIDANCE_ROADMAP.md
"""
