# DENEY — A2 / A3

```bash
GZ_HEADLESS=1 bash scripts/start_harmonic.sh    # Terminal A, ~50 s
bash scripts/gcs.sh A2                          # Terminal B (veya A3)
python3 tools/gudum_karne.py
```

| adım | GT poz | pose modeli | pose kilidi kapısı | HybridSORT | kilitli-ID |
|---|---|---|---|---|---|
| **A2** | açık | açık | açık | kapalı | kapalı |
| **A3** | açık | açık | açık | **açık** | kapalı |
| **A4** | açık | açık | açık | açık | **açık** |
| **pose** | **kapalı** | açık | açık | kapalı | kapalı |

Log ilk satırındaki `yapilandirma` sütunu hangi adım olduğunu yazar;
`menzil_kaynak` sütunu `gz` olmalı.

---

## Sonuç: A2 daha iyi

Eşit örneklem (17'şer faz, damgalı loglar):

| adım | faz | vuruş | vuruş oranı | <1.5 m | min medyan | devir |
|---|---|---|---|---|---|---|
| **A2** | 17 | **3** | **%17** | 12/17 | 0.85 m | 6.0 m |
| A3 | 17 | 1 | %5 | 11/17 | 0.65 m | 5.6 m |

A3 medyanda biraz daha yakın geçiyor ama **vuruşa çeviremiyor** — A2'nin
vuruş oranı 3 katı. Takip son metrelerde kutuya gecikme/kayma ekliyor olabilir.

**Karar: A2 taban alınsın.** Takip kapalı kalsın (`AVCI_TRACKER=off` zaten
varsayılan).

---

## ASIL DARBOĞAZ: 10.6 m'de kilit beklemesi

**Belirti:** drone hedefin ~11 m gerisinde asılı kalıyor, yaklaşmıyor. Kilit
bir şekilde otururken yaklaşıp vurabiliyor — sorun güdümde değil, geçişin
tetiklenmemesinde.

**Mekanizma:** GPS fazı istasyonunu `RANGE_SET = 11.0` m slant menzile kuruyor;
15° istasyon yükselişinde bu **10.63 m yatay + 2.85 m alt**
(`gps_guidance.py:141`). Drone oraya oturuyor ve KALIYOR:

- **Menzil kapısı** (`GATE_MENZIL = 20 m`) çoktan sağlanmış.
- **Pose kilidi kapısı** (son 15 karenin 10'unda conf ≥ 0.5) 10.6 m'de
  marjinal — bazen oturuyor, bazen hiç oturmuyor.

GPS fazının "kilit gelmiyorsa daha yaklaş" davranışı yok. Yaklaşmak için
geçmek, geçmek için kilit gerekiyor.

### Denenecekler — ucuzdan pahalıya

**1. İstasyonu yaklaştır (kod değişikliği YOK, önce bunu dene):**

```bash
AVCI_GPS_RANGE=8 bash scripts/gcs.sh A2      # 6'yı da dene
```

Kutu büyür → pose güveni artar → kilit kendiliğinden oturur. Tarihteki en iyi
uçuş (08-03 22:13, %50 vuruş) görsel faza **medyan 4.5 m**'de giriyordu;
bugünkü A2 6.0 m'de giriyor. Bu yön doğru görünüyor.

**2. Kilit çıtasını düşür:**

```bash
AVCI_HYBRID_KILIT_N=6 bash scripts/gcs.sh A2
```

⚠ `KILIT_N` 10 → 7 daha önce denenip geri alınmıştı (`supervisor.py` SupCfg
yorumu) — ama o pose güdümlü rejimdeydi. GT modunda kilit yalnız bir
zamanlayıcı; gerekçe artık aynı değil.

**3. Kilit zaman aşımı (kod işi, en hedefli):** menzil kapısı içindeyken N
saniye kilit gelmezse devri zorla. `supervisor.py` `izci()` içine eklenir.

---

## Tarihteki en iyi uçuş: 08-03 22:13 (%50 vuruş)

| | 08-03 22:13 | bugünkü A2 |
|---|---|---|
| vuruş | **3/6 (%50)** | 3/17 (%17) |
| min medyan | 0.53 m | 0.85 m |
| görsel faza giriş | **4.5 m** | 6.0 m |
| faz uzunluğu | 37-86 kare | 100-400 kare |
| güdüm | **pose modeli (GT KAPALI)** | GT poz |
| takip | kapalı | kapalı |

**Dikkat: en iyi sonuç GT modundan değil, POSE modundan geldi.**

### SEBEBİ ÖLÇÜLDÜ: görsel faz GT modunda ÇOK UZAKTAN devralıyor

Güdümün fiilen çalıştığı menzil (yalnız `durum=ok` kareler):

| | medyan | p90 | >15 m karelerin payı |
|---|---|---|---|
| POSE modu (08-03 en iyi) | **1.9 m** | 4.5 m | **%0** |
| GT modu (08-04, damgalı) | 11.8 m | 67.0 m | **%42** |

Pose modunda görsel güdüm yalnız **son 1-5 metrede** çalışıyor; yaklaşmayı
GPS fazı yapıyor — ki tasarımı bu. GT modunda ise algı hiç kopmadığı için
görsel faz 67 m'ye kadar devrede kalıyor ve yaklaşmayı da o üstleniyor.
Görsel faz bunun için tasarlanmadı: sabit `V_KAPANMA` ile kapanıyor,
istasyon tutmuyor, hedefin hızına uyum sağlamıyor.

Yani GT modu algıyı iyileştirirken **faz paylaşımını bozuyor**. Pose modelinin
"uzakta göremiyor" olması bir kusur değil, farkında olmadan **doğru faz
sınırını çizen bir filtre**ymiş.

Yan kanıt: `kalite` ortalaması pose modunda 1.000, GT modunda 0.553 —
kalite ölçekten (dolayısıyla menzilden) türüyor, yani GT modu sürekli
"kalitesiz" bandda çalışıyor.

**Sonuç:** `bash scripts/gcs.sh pose` yeniden ölçülmeli. GT modu bir teşhis
aracı olarak kalsın; taban ölçüm pose modu olmalı.

---

## GEREKSİZ İRTİFA: sebebi ölçüldü (2026-08-04)

**Belirti:** GT modunda drone hedefin 15-25 m üstüne çıkıyor; pose modunda
neredeyse hiç çıkmıyor.

| | GT | POSE |
|---|---|---|
| istasyon aşımı medyanı | **8.5 m** | 3.2 m |
| >5 m aşan uçuş | 55/89 | 7/18 |
| GPS fazına >4 m/s tırmanarak giriş | **15/92** | **0/18** |
| en hızlı giriş tırmanması | **9.5 m/s** | 3.9 m/s |

**Zincir:**

1. Görsel faz hedefe **alttan** yaklaştığı için sürekli TIRMANMA komutu verir
   (karelerin %61-82'si negatif `vz_cmd`).
2. GT modunda görsel faz uzaktan devralıp uzun süre çalışıyor (medyan 11.8 m,
   p90 67 m) ve orada **20 m/s'ye kadar** tırmanma komutu veriyor
   (pose modunda tavan ~11-14 m/s).
3. Temas kopunca kontrol GPS fazına döner — drone **hâlâ hızla tırmanıyorken**.
4. ArduPilot dikey hızı yalnız **~1 m/s²** ile azaltıyor (`WP_ACC_Z`, üç uçuşun
   kara kutusuyla ölçülmüştü). Yani:

   | kalan tırmanma | durması | bu sırada yükselme |
   |---|---|---|
   | 2 m/s | 2 s | 2 m |
   | 4 m/s | 4 s | **8 m** |
   | 7 m/s | 7 s | **24 m** |

5. Sonuç: GPS fazı alçalma komutu verirken drone hâlâ yükseliyor. Gözlenen
   15-25 m aşımlar tam olarak bu.

**Pose modunda neden yok:** görsel faz yalnız son 1-5 m'de, kısa süre çalışıyor
→ büyük tırmanma hızı birikmiyor → >4 m/s ile GPS'e dönüş hiç olmuyor (0/18).

**Asıl kusur simetrisizlik:** 20 m/s tırmanma komutu verebiliyoruz ama onu
1 m/s² ile söndürebiliyoruz. Çözüm iki uçtan biri:
- `adapter_copter`'da dikey komut tavanını düşür, veya
- ArduCopter'da `WP_ACC_Z`/`PSC_ACCZ` yükselt (komut yetkisiyle frenleme
  yetkisini eşitle).

`AVCI_GPS_RANGE=8`'in işe yaramasının sebebi de bu: istasyon yakınlaşınca
görsel faz daha yakında devralıyor, dikey savrulma birikmiyor.
