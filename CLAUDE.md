# Kilitlenme Görevi — Kalıcı Kurallar (şartname 6.1 ile doğrulandı)

## Geometri (Şekil 2 + 6.1.4)
- AK = kamera karesi (W×H). AV (hedef vuruş alanı, SARI) = yatay [0.25W, 0.75W],
  dikey [0.10H, 0.90H]. AH (kilitlenme dörtgeni, KIRMIZI) = hedef bbox. HH = hedef.
- KİLİT KOŞULU (ikisi birden):
  a) hedefin MERKEZİ AV içinde (bbox'ın tamamı değil),
  b) AH_en >= ESIK·W  VEYA  AH_boy >= ESIK·H (eksenlerden EN AZ BİRİ; VE değil).
- Resmi şart %5; İÇ KARAR/BİLDİRİM EŞİĞİ 0.06 (şartname tavsiyesi: tam %5'te
  paket göndermek hakem incelemesinde hatalı sayılabilir).
- marj = max(en/(ESIK·W), boy/(ESIK·H)).

## Süre kuralları (6.1.4)
- 10.0 sn değerlendirme penceresinde kümülatif kilit >= 5.0 sn; kesik kesik olabilir.
- KARE TOLERANSI: kilit segmenti içindeki kısa boşluklar, bildirilen sürenin
  %5'i bütçesiyle (5 sn için 200 ms) köprülenir; tolerans segmentin BAŞINDA ve
  SONUNDA geçersiz — segment gerçek kilitli kareyle başlar ve biter.
- Değerlendirme penceresi organizasyon tanımlı olabilir (ör. bildirim anı
  referanslı) — bildirim tarafında pencere referansı parametrik.

## Bildirim (6.1.4)
- Kilitlenme, yarışma sunucusuna paketle bildirilir (kilitlenme bitiş zamanı;
  format haberleşme dokümanıyla gelecek — modül soyut arayüz olarak yazılır).
- Pakette beyan edilen sürenin TAMAMI şartı sağlamalı.

## Tespit doğrulama (6.1.1)
- Tek karelik tespit yeterli değil: süreli/çok kareli doğrulama kapısı; tespit
  çıktısı telemetri/görev kaydından doğrulanabilir olmalı.

## Angajman (6.1.3)
- Çarpışma öncesi SON 3.0 sn: aktif takip + güdüm komutları hedefe yönelik
  (telemetriyle ispat) + çarpışma vektörü hedef doğrultusunda + mesafe
  SİSTEMATİK azalıyor.
- ANGAJMAN fazında merkez/%5 şartı ARANMAZ (yakın mesafede bbox ekranı kaplar);
  kriter aktif takiptir. Log şeması ispat üretir: t, faz, bbox, marj,
  komut vektörü, LOS, mesafe.
- Kilit/takip fazında mesafe korunur; angajmanda kapanır.

## Çözünürlük kuralı
- W,H hiçbir dosyada sabit yazılmaz; çalışma anında kaynak kareden okunur.
- Eşikler ve AV sınırları, bbox ile AYNI koordinat çerçevesinde hesaplanır
  (küçültülmüş YOLO karesi kullanılıyorsa geri ölçekleme doğrulanır).

## Dokunulmazlar
- gps_guidance.py'ye tek karakter dokunulmaz. send_velocity / DoW köprü hattı
  değiştirilmez; gerekirse aynı imzalı sarmalayıcı yazılır.

## Çalışma disiplini
- Her istekte SADECE istenen adım yapılır, bitince kısa özet verilir ve DURULUR.
