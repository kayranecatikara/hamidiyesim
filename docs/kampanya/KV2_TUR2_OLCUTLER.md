# K-V2 TUR 2 — ölçütler (KOŞMADAN ÖNCE yazıldı, 2026-08-14)

## Neden ikinci tur
Tur 1'de K-V2 kolunda yalnız **3 tamamlanmış kurtarma olayı** vardı ve
kıyasın ağırlığı senaryo bakımından EŞLENMEMİŞ tarihsel havuzdan (35 olay)
geliyordu. Kullanıcı ayrıca iki şey sordu:
  (a) "çok da bir işe yarıyor mu bilmiyorum" → tek başına doğrulanmalı
  (b) "K-V2 dikeydeki kaçırmanın sebebi olabilir mi?"

Bu tur ikisini birden cevaplıyor. Panelde başka düğme YOK (Ö-D ve Ö-D2
komple silindi) → tek değişken garantili.

## Özellik
`AVCI_KURT_V2=1` — kurtarma bekçisi kilitlenme düzeltmesi:
1. Yaw hedefi TETİK ANINDA kilitlenir (`kilit_yaw`), aracı takip etmez.
2. Bırakma şartında yalnız `|roll|` ve yaw hızı — pitch YOK.
   (pitch TETİKTE kalır: >60° hâlâ tetikler)
3. GPS fazı kurtarma sırasında da CSV satırı yazar.

## BİRİNCİL ÖLÇÜTLER
1. **KURTARMA SÜRESİ (s)** — `gcs_*.log`'daki "toparlandı (X s)" medyanı.
   KÜÇÜK kazanır. Tur 1: 10.7 s (n=35) → 4.9 s (n=3).
2. **KURTARMA SIRASINDA AÇILAN MESAFE (m)** — `gps_guidance_*.csv`'de
   KURTARMA bloğunun öncesi/sonrası `menzil` farkı. KÜÇÜK kazanır.
   Tur 1: +54.3 m (n=1) → +27.5 m (n=3).

## ZORUNLU EŞ ÖLÇÜTLER (§5.2 geçerlilik eşi)
1. **Olay sayısı** — hiç tetiklenmeyen kolda süre ölçülemez, "0" sahte iyi
   görünür. Bir kolda olay yoksa o kol kıyasa GİRMEZ.
2. **Görsel temas oranı** — Ö-D2 tam buradan geriledi (%73 → %45).
3. **⚑ DİKEY ISKA (kullanıcının gözlemi)** — en yakın andaki
   `iris_alt − plane_alt`. (− = drone ALTTA.) `kacamak.csv`, 10 Hz.
   Tur 1 ara verisi (n=4v4): kontrol medyan **+0.10 m** (altta 1/4),
   K-V2 medyan **−0.10 m** (altta 2/4). Yön kullanıcının gözlemiyle AYNI
   ama n=4'te ayırt edilemez. Bu turda n=8'e çıkıyor.
   ⚠ Dikey isabet zarfı YATAYDAN 5 KAT DAR (+0.29 / −0.13 m) — 0.20 m'lik
   bir kayma bile anlamlı olabilir.
   MEKANİZMA (neden olabilir): K-V2 bırakma şartından pitch'i çıkardı →
   güdüm araç BÜYÜK PITCH'teyken devralabiliyor. Dikey hata ham pikselden
   okunuyor (`bbox_ibvs.py`, `eps_elev = atan((cy−CY_NISAN)/FY)`) ve
   PITCH TELAFİSİ YOK (T1b uçmadan elenmişti).
4. İSABET, en yakın menzil — gerileme kontrolü.

## KARAR KURALI (önceden ilan, DEĞİŞTİRİLEMEZ)
- K-V2 **GİRER** (varsayılan AÇIK) eğer: kurtarma süresi belirgin düşer
  **VE** görsel temas gerilemez **VE** dikey ıska medyanı negatife kaymaz
  **VE** isabet/en yakın menzilde gerileme yok.
- K-V2 **ÇIKAR** (komple silinir, §5.12) eğer: dikey ıska medyanı negatife
  kayarsa (kullanıcının gözlemini doğrularsa) ya da isabet gerilerse.
- **EMNİYET (üstün kural):** bir koşuda bile araç düşerse K-V2 ANINDA elenir.
- Sonuç bölünmüşse ölçüt DEĞİŞTİRİLMEZ, kullanıcıya götürülür (§5.6).

## n ve denge (§5.4, §5.9)
8 uçuş: 4v4, her kolda 2 `yatay` + 2 `capraz`, dönüşümlü K, V, K, V...
Tur 1'in 8 uçuşu (Y01-Y08) da aynı kurulumda olduğu için dikey ölçüt
n=8v8'e çıkacak (Y + bu tur birleşik, tür dağılımı eşit).

## ETKİ ALANI TABLOSU (§5.10)
| etkilenebilecek davranış | neden | nerede sınanır |
|---|---|---|
| Dikey nişan | bırakma şartından pitch çıktı → yüksek pitch'te devralma | bu turun 3. eş ölçütü |
| Görsel temas | kurtarma daha erken bırakırsa burun/kamera farklı yerde | 2. eş ölçüt |
| Takla koruması | çıkış şartı gevşedi | birim testleri K11 (roll 75° tutuyor), K12 (pitch 75° tetikliyor) |
| Görsel faz | bekçi orada da çalışıyor | aynı uçuşlarda kapsanıyor |
