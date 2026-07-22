"""
control.guidance — Avcı drone güdüm hatları.

İki ayrı güdüm hattı ve bunları yöneten supervisor:
  - gps_approach.py: VARSAYILAN GPS yaklaşma — eski sistemin (ana_kontrol.py)
                     kanıtlanmış güdüm yasasının portu (standoff + lead +
                     speed_cap fren + look-up alttan bakış + handoff histerezisi)
  - gps_chase.py   : chase v2 (SPRINT→APPROACH→LOCK→STRIKE) — AVCI_GPS_LAW=v2 ile
  - gps_strike.py  : SAF GPS terminal vuruş (Proportional Navigation)
  - visual_guidance.py : (Faz 3) IBVS — kamera bbox tabanlı görsel güdüm
  - supervisor.py  : (Faz 4) GPS ↔ görsel geçiş + jamming fallback
  - common.py      : hatların paylaştığı yardımcılar (EMA, PID, setpoint, matematik)

Ayrıntılı yol haritası: docs/GUIDANCE_ROADMAP.md
"""
