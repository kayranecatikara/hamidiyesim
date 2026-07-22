"""
control.guidance — Avcı drone güdüm hatları.

İki ayrı güdüm hattı ve bunları yöneten supervisor:
  - gps_chase.py   : SAF GPS/telemetri takip (SPRINT→APPROACH→LOCK→STRIKE)
  - gps_strike.py  : SAF GPS terminal vuruş (Proportional Navigation)
  - visual_guidance.py : (Faz 3) IBVS — kamera bbox tabanlı görsel güdüm
  - supervisor.py  : (Faz 4) GPS ↔ görsel geçiş + jamming fallback
  - common.py      : hatların paylaştığı yardımcılar (EMA, PID, setpoint, matematik)

Ayrıntılı yol haritası: docs/GUIDANCE_ROADMAP.md
"""
