# K-V2 KAMPANYASI — ölçütler (KOŞMADAN ÖNCE yazıldı, 2026-08-13)

## Özellik
`AVCI_KURT_V2=1` — kurtarma bekçisinin kilitlenme düzeltmesi:
1. Yaw hedefi TETİK ANINDA kilitlenir (`kilit_yaw`), aracı takip etmez.
2. Bırakma şartında yalnız `|roll|` ve yaw hızı — pitch YOK.
   (pitch TETİKTE kalır: >60° hâlâ tetikler)
3. GPS fazı kurtarma sırasında da CSV satırı yazar.

## Kanıt (kullanıcının uçuş kaydı ucus_20260813_154505)
Bekçi 70 s'de 4 kez tetiklendi, her seferinde 8.6-14.9 s bırakmadı.
Araç havada asılı kaldı (hız 0.0 m/s), hedef 60 → 125 m açıldı.
gcs logu: "kontrol kaybı (yaw 318-429°/s)" → "toparlandı (8.6-14.9 s)".

## BİRİNCİL ÖLÇÜTLER (kullanıcının onayladığı hâliyle)
1. **KURTARMA SÜRESİ (s)** — `gcs_*.log` içindeki "toparlandı (X s)"
   satırlarının medyanı. KÜÇÜK olan kazanır.
2. **KURTARMA SIRASINDA AÇILAN MESAFE (m)** — olay başlangıcı ile bitişi
   arasında hedefe olan mesafenin artışı. `kacamak.csv` (10 Hz) üzerinden.
   KÜÇÜK olan kazanır.

- Geçerlilik eşi (§5.2): **olay sayısı**. Hiç tetiklenmeyen kolda süre
  ölçülemez ve "0" sahte iyi görünür. Olay sayısı ayrıca raporlanır;
  bir kolda olay yoksa o kol kıyasa GİRMEZ.

## İKİNCİL ÖLÇÜTLER
1. Olay sayısı / koşu (düzeltme tetiklenmeyi azaltmamalı — o ayrı iş).
2. İSABET, en yakın menzil — gerileme kontrolü.
3. Görsel temas oranı.
4. "8 s'dir toparlanamıyor" uyarısının sayısı (V2'de sıfıra inmeli).

## KARAR KURALI (önceden ilan)
- K-V2 **girer** eğer: kurtarma süresi medyanı belirgin düşer VE isabet /
  en yakın menzilde gerileme yok.
- K-V2 **girmez** eğer: süre düşmez, ya da isabet gerilemesi var, ya da
  araç bir koşuda kurtarılamayıp düşerse (EMNİYET — tek olay yeter).
- Sonuç bölünmüşse ölçüt DEĞİŞTİRİLMEZ, kullanıcıya götürülür (§5.6).

## n ve denge (§5.4, §5.9)
8 uçuş: 4v4, her kolda 2 yatay + 2 capraz. Dönüşümlü K, V, K, V...
Olay oranı yüksek (koşu başına 2-6) olduğu için n=4/kol'da bile
olay sayısı 10+ olacak — Ö12'deki "seyrek olay" sorunu burada YOK.

## REGRESYON (§5.10)
Bekçi HEM GPS HEM GÖRSEL fazda çalışıyor → ikisi de bu testte kapsanıyor
(kaçamak testi iki fazı da geçiyor).
Birim testleri: K9 (V2 kapalıyken eski davranış korunur), K11 (gerçek
takla hâlâ tutuyor), K12 (pitch tetikte kaldı).
⚠ EMNİYET RİSKİ: çıkış şartı gevşedi. Bir koşuda bile araç düşerse
özellik ANINDA elenir — bu kural diğerlerinden ÜSTÜNDÜR.
