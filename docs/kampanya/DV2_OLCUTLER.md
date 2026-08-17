# D-V2 · Terminal dikey tavanı 5 → 8 m/s — ölçüt ilanı

**Tarih:** 2026-08-17 (kampanya koşulmadan ÖNCE yazıldı, §4)
**Kol anahtarı:** `AVCI_IBVS_VZT` · panel: "D-V2 · Terminal dikey tavanı 5 → 8 m/s"
**Kontrol:** 5.0 · **Deney:** 8.0
**⚠ TABAN: D-V (`KAPANMA_MIN`=6.0) İKİ KOLDA DA AÇIK**

---

## Neden — D-V'nin bedelini kaldırmak

D-V kampanyası (22 koşu) şunu gösterdi:

**Kazanım:** dikey ıska 1.07 → 0.56 m (koşu-başı), aynı seviye geçiş
%51 → %80, temas 0 → 2.

**Bedel:** dikey komut `VZ_MAX_TERM`=5 tavanına dayandı (terminal karelerinin
**%20-68'i**). Tavan doyunca kodun dikey bütçe kısıtı devreye giriyor:

```python
if v_dikey * tan(nişan_elev) > VZ_MAX_TERM:
    v_los = max(V_TERM_MIN, VZ_MAX_TERM / tan(nişan_elev))
```

→ yatay hız kesiliyor (bir koşuda 16 → **10 m/s**, `V_TERM_MIN` tabanı)
→ hedef 15.15 m/s uçarken **kapanma negatife dönüyor**
→ araç 6-12 m bandında sıkışıyor.

Ölçüldü (D-V verisi, sonradan): takılma oranı kapalı %26 → açık %49
(dağılımlar örtüşüyor, **eğilim** düzeyinde).

## Ne yapar

Tavanı 8 m/s'ye çıkarır → dikey talep karşılanır → bütçe kısıtı tetiklenmez
→ yatay hız 16'da kalır → D-V'nin dikey kazanımı **bedelsiz** gelir.

⚠ **D-V2 TEK BAŞINA ANLAMSIZ.** Dikey talep büyümediyse tavanı gevşetmek
hiçbir şey yapmaz. Bu yüzden D-V her iki kolda da AÇIK; kollar arasında
değişen tek şey tavan. Kampanya scripti bunu her koşuda doğruluyor
(`taban D-V: true` satırı, aksi halde koşu iptal).

⚠ **Aracın fiziksel dikey hız tavanı `WPNAV_SPEED_UP` = 600 cm/s = 6 m/s.**
8 komut edilse de araç 6'da doyar. Asıl kazanç hız değil, **bütçe kısıtının
tetiklenmemesi** — yani yatay hızın kesilmemesi.

---

## ÖLÇÜTLER

### Birincil
**Koşu başına en iyi menzilin medyanı** (kara kutudan).
⚠ "Tüm geçişlerin medyanı" kullanılmaz (D-V dersi: geçiş sayısından etkileniyor).

### İkincil
1. **Takılma oranı** — 6-12 m bandında geçen terminal karesi (D-V2'nin
   doğrudan hedefi: bu oran düşmeli)
2. **`v_los` medyanı** — 16'da kalmalı, 10'a düşmemeli
3. Temas sayısı (≤0.5 m geçiş)
4. |dikey| ıska — D-V'nin kazanımı korunmalı

### Mekanizma kapısı (§5.1)
Deney kolunda terminal `|vz_cmd|` **5.0'ı aşan** kare bulunmalı (tavan 8
olduğu için mümkün). Kontrol kolunda 5.0'ı aşan kare **olmamalı**.

Bu, doğrudan özelliğin değiştirdiği büyüklük — sonuç ölçütü değil.
(D-V'de mekanizma kapısı sonuç ölçütüyle, D-N'de kendi kendine referanslı
ölçütle karıştırılmıştı; ikisi de tekrarlanmıyor.)

### Geçerlilik eşi (§5.2)
| ölçüt | kötü sebeple iyileşir mi | zorunlu eş |
|---|---|---|
| takılma azalması | **evet** — hedefi kaybedip uzaklaşmak da takılmayı bitirir | görsel temas + geçiş sayısı |
| en yakın menzil | evet — savrulup şans eseri | salınım |
| `v_los` yükselmesi | **evet** — dikey talep hiç oluşmadıysa da yüksek kalır | mekanizma kapısı (vz>5 kare sayısı) |

**Görsel temas %60 altına inen kol GÜVENİLMEZ.**

### Salınım
`cx` işaret değişimi/s · `|roll|` p90

### Geçerlilik (§4)
Hedef 20-250 m / 6-25 m/s. Dışına çıkan koşu SAYILMAZ.

---

## ETKİ ALANI TABLOSU (§5.10)

| etkilenebilecek davranış | neden | nerede sınanır |
|---|---|---|
| Yatay hız kesilmesi | doğrudan hedef | `duz` — `v_los` medyanı |
| Takılma | bütçe kısıtı kalkarsa kapanma sürer | `duz` — 6-12 m bandı oranı |
| **Dikey aşım** | tavan gevşeyince araç hedefin ÜSTÜNE çıkabilir | `duz` — dikey ıskanın İŞARETİ (üstten geçiş oranı artmamalı) |
| Aracın dikey doyumu | `WPNAV_SPEED_UP`=6 m/s zaten sınırlıyor | ölçülür (vz>6 kare oranı) |
| Seyir fazı | **etkilenmez** — `VZ_MAX_TERM` yalnız terminal dalında | birim testi |

**Cevaplanacak soru:** "yatay hız kaybını çözdü ama dikeyde aşım yarattı mı?"

---

## KOŞU PLANI

`duz`, hibrit, dönüşümlü, 4 tur × 2 koşu, 150 s ölçüm.
Ö-T, D-N, D-S **iki kolda da KAPALI**. D-V **iki kolda da AÇIK** (taban).
İrtifa tutucu **AÇIK**.

## KARAR KURALI (önceden ilan)

- Birincil + takılma oranı deney lehine → **GİRER** (D-V ile birlikte)
- Birincil kötüleşir **veya** üstten geçiş oranı artarsa → **GİRMEZ**
- Bölünürse kullanıcıya (§5.6)
- n<4/kol → ara veri (§5.4)

⚠ D-V2 girerse **D-V ile BİRLİKTE** girer — tek başına anlamı yok.
Bu, Ö-M + V_TERMINAL 16'nın birlikte girmesiyle aynı desen.
