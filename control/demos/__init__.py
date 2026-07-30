"""
control.demos — Elle çalıştırılan bağımsız uçuş demoları.

Görev akışının parçası DEĞİLDİR; öğrenme, tek tek fonksiyon denemesi ve veri
toplama uçuşu için kullanılır. Görev akışı `control.gcs_server` üzerinden
yürür (hedef senaryoları için `control.run_plane_scenario`).

Çalıştırma (depo kökünden):
    python3 -m control.demos.run_drone_takeoff      # kalkış + hover
    python3 -m control.demos.run_drone_hover        # kalkış + hareket + yaw
    python3 -m control.demos.run_drone_square       # kare yörünge (negatif veri uçuşu)
    python3 -m control.demos.run_plane_arm          # keepalive + ARM
    python3 -m control.demos.run_plane_aggressive   # agresif manevralar
    python3 -m control.demos.run_dual_demo          # iki araç eş zamanlı

Ön koşul: Gazebo + SITL çalışıyor olmalı (bkz. docs/SIMULASYON_CALISTIRMA.md).
Portlar: iris 14541, Talon 14542.
"""
