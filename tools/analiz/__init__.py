"""
tools.analiz — visual_lead CSV loglarının SALT-OKUMA analizi.

Bu paketteki hiçbir script simülasyona bağlanmaz, uçuş başlatmaz, MAVLink
göndermez. Yalnız `logs/visual_lead_*.csv` okur (bkz. CLAUDE.md §1).

Kullanım:
    python3 -m tools.analiz.analiz_yonelim          # en yeni CSV
    python3 -m tools.analiz.analiz_menzil  logs/visual_lead_*.csv
    python3 -m tools.analiz.analiz_devir   <dosya>
"""
