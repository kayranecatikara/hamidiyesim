# 🎚 AYAR KONSOLU — sistemdeki tüm katsayılar tek panelde

> **Kullanıcı isteği (2026-08-18):** *"arayüze bir buton koy, bu butona basınca
> bir panel açılsın ve bu panelden sistemdeki tüm tune edilmesi gereken
> şeylerin parametrelerin katsayılarını slidebarlardan ayarlayabileyim.
> Bunları değiştire değiştire sistemi en iyi haline getirmeye çalışayım…
> her şeyin de ne işe yaradığı, neyi kontrol ettiği, neyi artırıp neyi
> azalttığı bilinsin."*

**Açılış:** panelin sağ üstünde **🎚 AYAR KONSOLU** düğmesi. `Esc` kapatır.

---

## 1 · TEK AYAR YÜZEYİ

⚠ **2026-08-19:** sol paneldeki **🎛 GÜDÜM ÖZELLİKLERİ** kutusu KALDIRILDI.
Kullanıcı: *"şu sol paneldeki ayar şeylerini de kaldır, bu şekilde default
yaparak; zaten bunlar da o diğer büyük ayar panelinde var."* Beş düğmenin
değerleri `Cfg` varsayılanı oldu; ayarlanabilir olanlar bu konsolda duruyor,
terminale ait olanlar zaten silindi.

Konsoldan bir değeri değiştirmek onu "sisteme girdi" YAPMAZ — burası
keşif/tarama içindir, karar defteri değil. Kalıcı olması için §1'in beş
adımı gerekir: öner → ölç → raporla → göster → doğrula.

## 2 · Neler var

**42 ayar, 7 grup:**

| grup | n | ne hakkında |
|---|---|---|
| ① **HÜCUM** | 6 | `V_HUCUM`, denge kutusu, `K_ELEV`, `K_VZ_D`, yavaşlama |
| ② YATAY KANAL (yaw) | 10 | nişanlama, yaw kazancı/tavanı, sönümleme, PN |
| ③ DİKEY KANAL | 2 | `VZ_MAX` tavanı ve bütçesi |
| ④ HIZ (PI kazançları) | 6 | `K_FWD`, `K_I`, ivme sınırı, kaçış telafisi |
| ⑤ LEAD | 4 | hedefi öne alma süresi/tavanı/sönümü |
| ⑥ ALGI ve FAZ GEÇİŞİ | 7 | güven eşikleri, kilit/kayıp kare sayıları |
| ⑦ ARAÇ (ArduPilot) | 7 | ANGLE_MAX, WPNAV_*, PSC_JERK_XY — canlı `PARAM_SET` |

⚠ **Terminal fazına ait iki grup (8+5 = 13 ayar) SİLİNDİ** — terminal fazı
2026-08-19'da koddan tamamen çıkarıldı.

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

## 4 · TEK GÖRSEL YASA

Görsel güdüm **tek parçadır** — ayrı bir "terminal" fazı yoktur:

```
YATAY : hız vektörünün YÖNÜ hedefe döner   (yaw + K_YAW·eps_yaw)
DİKEY : AYNI matematik, aynı eksen için    (K_ELEV·elev_los)
HIZ   : denge kutusu = TEMAS kutusu → hep kapat, V_HUCUM'da otur
```

Kaldırılmadan önce mandal atıldığı an dokuz şey birden değişiyordu ve
ölçülen bütün bitiriş sorunları oradan çıkıyordu. 8 uçuşta **isabet
2/4 → 4/4**, dikey ıska **1.77 → 0.66 m**, kör hücum **376 → 0 kare**.
Ölçümler: `docs/kampanya/TF_TEK_FAZ.md`.

## 5 · Yeni ayar eklemek

`control/ayar_konsolu.py` içindeki `AYARLAR` listesine bir satır:

```python
("ad", "ALAN", "Etiket", "G0", "sayi", "m/s", 0.0, 30.0, 0.5,
 "ne yapar", "artarsa ne olur", "azalırsa ne olur"),
```

Alan adı `"sup:ALAN"` biçiminde supervisor'a da yazabilir.
Tip: `"sayi"` | `"bool"` | `"param"` (araç, MAVLink `PARAM_SET`).
**Arayüze dokunmak gerekmez** — panel listeyi `/api/ayarlar`'dan çeker.

## 6 · Nerede ne var

| dosya | ne |
|---|---|
| `control/ayar_konsolu.py` | 42 ayarın kaydı (etiket, aralık, açıklama) |
| `control/gcs_server.py` | `/api/ayarlar` GET/POST + `/api/ayarlar/sifirla` |
| `control/gcs_ui/index.html` | düğme + katman iskeleti |
| `control/gcs_ui/script.js` | `ayarCiz` / `ayarYaz` / `ayarKopyala` |
| `control/gcs_ui/style.css` | `.ayar-*` sınıfları |
