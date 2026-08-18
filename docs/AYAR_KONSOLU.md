# 🎚 AYAR KONSOLU — sistemdeki tüm katsayılar tek panelde

> **Kullanıcı isteği (2026-08-18):** *"arayüze bir buton koy, bu butona basınca
> bir panel açılsın ve bu panelden sistemdeki tüm tune edilmesi gereken
> şeylerin parametrelerin katsayılarını slidebarlardan ayarlayabileyim.
> Bunları değiştire değiştire sistemi en iyi haline getirmeye çalışayım…
> her şeyin de ne işe yaradığı, neyi kontrol ettiği, neyi artırıp neyi
> azalttığı bilinsin."*

**Açılış:** panelin sağ üstünde **🎚 AYAR KONSOLU** düğmesi. `Esc` kapatır.

---

## 1 · ⛔ İKİ PANEL, İKİ AYRI İŞ — karıştırma

| | 🎛 GÜDÜM ÖZELLİKLERİ (sol panel) | 🎚 AYAR KONSOLU (bu) |
|---|---|---|
| ne için | **karar bekleyen** özellik | **keşif / tarama** |
| kaç şey | CLAUDE.md §0.2: aynı anda **en fazla bir** yeni özellik | sistemin **tamamı** (56 ayar) |
| anlamı | buradan geçen şey sisteme **girer ya da elenir** | buradan oynatmak **kayda geçmez** |

Konsoldan bir değeri değiştirmek onu "sisteme girdi" yapmaz. Kalıcı olması
için §1'in beş adımı gerekir: öner → ölç → raporla → göster → doğrula.

⚠ Bazı alanlar **iki yerde birden** görünür (ör. `V_TERMINAL` hem ③ düğmesi
hem G4'te). İkisi **aynı** `Cfg` alanına yazar — çelişki yok, birinden
değiştirince diğeri de o değeri gösterir.

## 2 · Neler var

**56 ayar, 8 grup:**

| grup | n | ne hakkında |
|---|---|---|
| ① YATAY KANAL (yaw) | 10 | nişanlama, yaw kazancı/tavanı, sönümleme, PN |
| ② DİKEY KANAL | 7 | nişan noktası, dikey kazanç/tavan/sönümleme, dikey kapı |
| ③ HIZ (kutu boyutundan PI) | 7 | BOYUT_REF, K_FWD, K_I, hız tavanı, ivme sınırı |
| ④ TERMİNAL (hücum) | 8 | geçiş eşiği, hücum hızı, kör hücum süresi, kapanma ölçeği |
| ⑤ TERMİNAL ADAYLARI | 5 | **D2, D1, D3, A1, T1c** — ölçüldü, kararı verilmedi |
| ⑥ LEAD | 5 | hedefi öne alma süresi/tavanı/sönümü |
| ⑦ ALGI ve FAZ GEÇİŞİ | 7 | güven eşikleri, **E1**, kilit/kayıp kare sayıları |
| ⑧ ARAÇ (ArduPilot) | 7 | ANGLE_MAX, WPNAV_*, PSC_JERK_XY — canlı `PARAM_SET` |

Her satırın **?** düğmesi şunları açar:
- **ne yapar** — hangi denklemde, neyi kontrol ediyor
- **▲ ARTARSA** — ne iyileşir, ne bozulur (ölçüm varsa sayısıyla)
- **▼ AZALIRSA** — aynısı ters yön
- **açılış değeri** ve **aralık**

## 3 · Kullanım

- **Kayan çubuk**: sürüklerken yalnız ekran güncellenir; **fareyi bırakınca**
  sunucuya yazılır. (Güdüm 20 Hz koşuyor — saniyede 60 istek ikisini de boğar.)
- **Sayı kutusu**: elle yaz, `Enter`/odak kaybında uygulanır.
- Sınır dışı değer **kırpılır** ve konsola not düşülür.
- **Değişen satır sarıya boyanır**; başlıktaki rozet kaç tanesini
  oynattığını gösterir.
- **sadece değişenler** kutusu ile ayıklanır.
- **📋 DEĞİŞENLERİ KOPYALA** → panoya: neyi neye çevirdiğinin okunur listesi
  **+ aynı ayarı geri getiren `curl` satırları**. Bana yapıştırırsan ne
  denediğini birebir görürüm.
- **↺ VARSAYILANA DÖN** → tüm güdüm ayarları açılış değerine. ⚠ **Araç
  parametreleri (⑧) etkilenmez** — onlar araca yazıldı, elle geri alınır.

**Değişiklikler CANLI.** `bbox_ibvs.Cfg` bir SINIF ve güdüm her karede
`cfg.<ALAN>` okuyor → bir sonraki kareden itibaren geçerli. Uçuş sırasında,
yeniden başlatmadan.

⚠ **Kalıcı değil.** Sunucu yeniden başlayınca env varsayılanlarına döner.
Kalıcı istiyorsan kopyaladığın `curl` satırlarını kullan ya da `AVCI_*`
env ile başlat.

## 4 · ⑤ TERMİNAL ADAYLARI — neden buradalar

Bunlar sistemin parçası **değil**. Hepsi varsayılan **KAPALI** ve kapalı
hâlde davranış `manevrada-iyi-terminalde-kotu` etiketiyle **BİT BİT AYNI** —
972 girdi kombinasyonunda (cx/cy/boyut/terminal/roll/pitch/kapanma) `komut()`
çıktısının vx, vy, vz, yaw farkı **0.000e+00**.

| ayar | ne | ölçüm |
|---|---|---|
| `TERM_HIZ_KORU` | **D2** — terminalde fren yok | 16 uçuş: kadraj dışı %73→%0 (p=0.039), en yakın 1.75→1.20 m (p=0.030), isabet 3/6→5/6. **Bedeli:** 3 m içinde \|dikey\| 0.21→1.06 m |
| `TERM_SAF3B` | **D1** — saf takip 3B | dikey tavan 38.7°'de bağlar |
| `TERM_YAVASLA` | **D3** — kaçıracaksan yavaşla (kullanıcının fikri) | 45°'de D1 tek başına takılıyor, D3 ile 45.0° (v=14.1) |
| `TERM_TAM_HIZ` | **A1** — dikeyde tam hız ölçeği | — |
| `TERM_ROLL` | **T1c** — terminalde roll telafisi | 4 uçuş, kollar AYIRT EDİLEMEDİ |
| `sup:POSE_CONF_MIN` | **E1** — faz geçişi güven eşiği (⑦'de) | faz zıplamasının kök nedeni: iki katman 0.0 vs 0.35 |

⚠ **D3 yalnız D1 açıkken anlamlıdır.**

Ölçümlerin tamamı: `docs/kampanya/H_TERMINAL_HIZ.md`,
`docs/kampanya/SORUN_ENVANTERI.md`.

## 5 · Yeni ayar eklemek

`control/ayar_konsolu.py` içindeki `AYARLAR` listesine bir satır:

```python
("ad", "ALAN", "Etiket", "G4", "sayi", "m/s", 0.0, 30.0, 0.5,
 "ne yapar", "artarsa ne olur", "azalırsa ne olur"),
```

Alan adı `"sup:ALAN"` biçiminde supervisor'a da yazabilir.
Tip: `"sayi"` | `"bool"` | `"param"` (araç, MAVLink `PARAM_SET`).
**Arayüze dokunmak gerekmez** — panel listeyi `/api/ayarlar`'dan çeker.

## 6 · Nerede ne var

| dosya | ne |
|---|---|
| `control/ayar_konsolu.py` | 56 ayarın kaydı (etiket, aralık, açıklama) |
| `control/gcs_server.py` | `/api/ayarlar` GET/POST + `/api/ayarlar/sifirla` |
| `control/gcs_ui/index.html` | düğme + katman iskeleti |
| `control/gcs_ui/script.js` | `ayarCiz` / `ayarYaz` / `ayarKopyala` |
| `control/gcs_ui/style.css` | `.ayar-*` sınıfları |
