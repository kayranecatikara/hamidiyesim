"""
control — ArduPilot SITL araç kontrolü, güdüm ve yer kontrol istasyonu.

Modüller:
    mav_common          — Ortak düşük seviye MAVLink altyapısı (ArduPilot)
    drone_functions     — iris (ArduCopter) kontrolü
    plane_functions     — Talon (ArduPlane) kontrolü
    plane_patterns      — Talon manevra desenleri
    run_plane_scenario  — Hedef İHA uçuş senaryoları (gcs_server bunu başlatır)
    gcs_server          — Web yer kontrol istasyonu (FastAPI + kamera + görev)
    arm_diag            — ARM reddi teşhis aracı

Alt paketler:
    guidance            — Hibrit güdüm (GPS kadraj merkezleme ↔ görsel IBVS)
    demos               — Elle çalıştırılan bağımsız uçuş demoları
"""
