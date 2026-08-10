# ⭐ KARARLI HAL — GPS GÜDÜMÜ (2026-08-06)

> Bu dosya, GPS güdümünün **uçuşta doğrulanmış en iyi hâlini** kayda geçirir.
> Bir şey bozulursa dönülecek nokta burasıdır: `git checkout kararli-gps-gudumu`

> ### ⚠ TAM MERGE SONRASI NELER FARKLI (2026-08-07)
>
> Bu belge `gps_kararli_hal` dalında yazıldı. O dal 2026-08-07'de bu dala
> **bütün olarak** merge edildi (TODO.md §0). GPS tarafı birebir geldi; görsel
> taraf bu dalın hâlinde bırakıldı. Aşağıyı okurken:
>
> | belgede yazan | bu dalda gerçek |
> |---|---|
> | `ISTASYON_ELEV_DEG 15°`, `KD_H 0.60`, `IC_KAYMA 14` | ✅ aynı |
> | araç parametreleri | ✅ değerler aynı (25 m/s, 8 m/s², 2.5 m/s², 45°), **adlar SI şemasında** — bu firmware `WPNAV_*`/`ANGLE_MAX` tanımıyor, bkz. `avci_copter.parm` başlığı |
> | "Görsel faz kapalı" | ❌ bu dalda görsel faz **açık** (hibrit güdüm) |
> | "Truth modu açık, keypoint zinciri devre dışı" | ❌ keypoint zinciri **kaldırıldı** (2026-08-07 kullanıcı kararı, kesin). Görsel faz bbox ile çalışıyor; o dönemin `gercek_geometri` / `_process_gercek` yolu merge'de sökülmüştür (bkz. POSEA_GERI_DONMEK_ISTERSENIZ/). GT modu (`AVCI_GT_ROT`) varsayılan KAPALI |
> | "Panel 8 m yalan söylüyordu" | ✅ bu dalın arayüzü baştan yazıldı, o hata yok |
> | menzil sonuçları (13-16 m) | ⚠ o zarfla ölçüldü; bu dalda **henüz uçulmadı** — TODO.md §0 taban ölçümü |

---

## Uçuşta ölçülen performans

Hedef uçak ~14-15 m/s, avcı drone ArduCopter (iris). Değerler arayüzdeki
**MESAFE** alanından, düzeltilmiş panelle okundu.

| Senaryo | Hedefe mesafe |
|---|---|
| Düz uçuş | **13-14 m** |
| Daire ⌀96 m | **15-16 m** |
| Daire ⌀71 m | **15-16 m** |
| Daire ⌀55 m | **15-16 m** |
| Daire ⌀41 m | **16-17 m** |
| Kare — kenarlarda | **14 m** |
| Kare — köşelerde (manevra anı) | 21 m |

Karşılaştırma için başlangıç noktası: aynı senaryolarda menzil **34 m** ve
dönen hedefte **hiç yakınsamıyordu** (63 dakikalık uçuşta menzil 42-147 m
arasında salındı).

---

## Kararlı yapılandırma

### Güdüm (`control/guidance/gps_guidance.py`)

```
RANGE_SET          = 11.0 m     istasyonun hedefe slant uzaklığı
ISTASYON_ELEV_DEG  = 15.0°      istasyonun LOS yükselişi (kamera tilt'i 25°)
V_MAX              = 18.0 m/s   yatay hız tavanı
KP_H               = 0.8        konum hatası → hız
KD_H               = 0.60       LEAD terimi  (bkz. aşağıda)
KP_Z               = 1.0        dikey
VZ_MAX             = 6.0 m/s
IC_KAYMA           = 14.0 m     iç daire nişanı (sabit metre)
IC_OMEGA_REF       = 0.15 rad/s kaymanın tam devreye girdiği dönüş hızı
IC_ORAN            = 0.0        yarıçap-oranlı sürüm KAPALI
```

### Araç (`sim/ardupilot_params/avci_copter.parm`)

```
WP_YAW_BEHAVIOR 0       ANGLE_MAX      4500   (45°)
WPNAV_SPEED  2500       WPNAV_ACCEL     800   (8 m/s²)
WPNAV_SPEED_UP 600      WPNAV_SPEED_DN  400
WPNAV_ACCEL_Z  250      WPNAV_JERK        4
PSC_VELXY_P   2.0       FS_GCS_ENABLE / FS_THR_ENABLE = 0
```

### Diğer

- Görsel faz **kapalı** (`AVCI_GORSEL=on` ile açılır) — yalnız GPS güdümü
- Truth modu **açık** — o dönemin keypoint zinciri devre dışı (⚠ bu satır
  2026-08-07 öncesine aittir; env anahtarı artık yok)
- FRPN modülü rafta (`AVCI_GPS_GUDUM=frpn` ile denenebilir, varsayılan değil)

---

## Buraya nasıl gelindi

Kronolojik değil, **etkisine göre** sıralı. Her madde uçuşta ölçüldü.

### 1. Araç parametreleri iki kez yanlış adla yazılmıştı

ArduCopter ve ArduPlane **farklı isim şemaları** kullanıyor. Plane'den
genelleme yapılıp Copter'a `WP_SPD` / `WP_ACC` / `ATC_ANGLE_MAX` yazılmıştı;
ArduPilot tanımadığı parametreyi **sessizce yok sayar**. Sonuç: araç aylarca
firmware varsayılanlarıyla uçtu (yatay ivme 2.5 m/s², eğim 30°).

Düzeltildikten sonra yanal ivme 2.5 → 9.31 m/s², dönüş hızı %95'te 37.8°/s.

> **Kural:** parametre eklerken adı `~/ardupilot/mav_5_1.parm` içinde
> `grep` ile DOĞRULA. Çıktı boşsa parametre uygulanmıyor demektir.

### 2. `KD_H` bir "sönümleme" değil, LEAD teriminin kendisiymiş

`de[]` istasyon hatasının türevi, yani ≈ göreli hız. Yasa açılınca:

```
v_cmd = v_hedef + KP_H·Δp + KD_H·Δv
```

Bu, FRPN'in hız formuyla **aynı üç terimli yapı**. Yani güdümde lead zaten
vardı, sadece ~3 kat zayıftı. 0.20 → 0.60: menzil 34.3 → 29.4 m.

Denge analizi 1.0'ın salındıracağını öngörüyordu (Δv⊥ ← −K·Δv⊥) ve bağımsız
bir katsayı taraması da 0.60'ı buldu.

### 3. İç daire nişanı — en büyük tek kazanç

Dairesel kovalamacada zorunlu bir bağ var:

```
yarıçap = hız / açısal_hız
```

İstasyon "hedefin hız yönünün gerisi"ne konuyordu; o nokta hedefin **kendi
çemberinin üzerinde**. Drone onu kovaladığı sürece aynı yarıçapta uçmak
zorunda, dolayısıyla aynı hıza muhtaç — ve hedeften hızlı değilse asla
yaklaşamaz.

Çözüm: istasyonu dönüşün **içine** kaydır. Drone daha küçük yarıçapta, daha
az hızla aynı açısal hızı tutturur.

| Kayma | Menzil | En yakın | drone R − hedef R |
|---|---|---|---|
| 0 m | 34.1 m | 31.3 m | +2 m (aynı çember) |
| 8 m | 22.8 m | 6.9 m | −7 m (içeride) |
| **14 m** | **9.8 m** | **3.2 m** | **−11 m (içeride)** |

Kayma hedefin açısal hızıyla ölçekleniyor, yani **düz uçuşta tam sıfır** —
düz kovalama davranışı hiç bozulmuyor.

### 3b. İç daire kayması neden SABİT, neden oranlı değil

Sezgi "sabit bir sayı yalnız belli bir çapta doğru olur, yarıçapla orantılı
olmalı" diyordu. **Ölçüm bunu çürüttü.** Üç yapılandırma, dört daire çapı:

| | ⌀96 | ⌀71 | ⌀55 | ⌀41 |
|---|---|---|---|---|
| **sabit 14 m** | **14-15** | **15-16** | **15-16** | **16-17** |
| oranlı 0.27·R | 17-18 | 15-16 | 16-17 | 22-23 |
| sabit 20 m | 18-19 | 18-19 | 17-18 | 17-18 + **düşüş** |

Sabit 14, 35-96 m yarıçap aralığında (2.7 kat) **her yerde** kazandı.

**Mekanizma** — mesafe iki bileşenden oluşuyor (drone hedefin çemberinin
içinde uçtuğu için):

| Kayma | Radyal (içeride olma bedeli) | Teğetsel (geri kalma) | Toplam |
|---|---|---|---|
| 0 m | 0 m | 34.1 m | 34.1 m |
| **14 m** | 7 m | **13.9 m** | **15.6 m** |
| 20 m | 12 m | 13.8 m | 18.3 m |

Kaymanın işi teğetsel gecikmeyi düşürmek (34 → 14 m). Ama **14'te doyuyor**:
20'ye çıkınca teğetsel 13.9 → 13.8 (kazanç yok), radyal 7 → 12 (saf bedel).

Doyma noktasını **dairenin büyüklüğü değil, drone'un kendi dinamiği**
belirliyor — açısal olarak yetişebilmek için ne kadar "yükten kurtulması"
gerekiyorsa o kadar. Bu miktar yarıçapla ölçeklenmediği için sabit bir sayı,
orantılı kuraldan iyi.

Oranlı sürüm her iki uçtan da kaybetti: ⌀96'da 25 m kaydırıyor (fazla),
⌀41'de 11 m (az). Kod duruyor ama kapalı (`AVCI_GPS_IC_ORAN=0.27`).

> **Açık:** 35 m'den dar daireler test edilmedi (⌀32 senaryosu kaldırıldı).
> Orada 14 m, yarıçapın %40'ı olur ve radyal bedel baskın hale gelebilir.

### 4. Gecikme birikmesi — üç ayrı hatta aynı hata

Kamera callback'i tüm işi senkron yapıyordu (YOLO + tracker + overlay + JPEG,
medyan 21.8 ms) ama kamera 30 Hz yayın yapıyor (bütçe 33.3 ms). Bütçe aşılınca
gz-transport kuyruğu **hiç boşalmıyordu** — arayüz Gazebo'nun giderek gerisine
düşüyordu.

Aynı hata telemetride de vardı: her turda tek mesaj okunup 5 ms uyunuyordu
(tavan 200 msg/s), oysa iki araç birden ~300-500 msg/s yayın yapıyor.

Çözüm: **en son veri kazanır**. İzole ölçüm (30 Hz üretici, 40 ms işleme):
eski desen 80 → 579 ms (sürekli büyüyor), yeni desen 19 → 19 ms (sabit).

### 5. Tespit modeli yenilendi

Hayalet tespit %68.9 → %54.7, doğru kilit %73.2 → %77.2 (güven eşiği 0.5).

### 6. ⚠ Panel 8 metre yalan söylüyordu

`script.js`'te `(data.distance + 8)` — deponun **ilk commit'inden** beri.
Arayüz gerçek mesafeye sabit 8 m ekliyordu.

Aynı ekranda dört sayı vardı: panel **21.3 m**, sim ground-truth **13.2 m**,
güdümün yatay ölçümü **11.4 m**, ekrandaki konumlardan elle hesap **13.3 m**.
21.3 − 8 = 13.3 → birebir.

Bu yüzden **bütün uçuş gözlemleri 8 m şişik okundu** ve güdüm olduğundan çok
daha kötü sanıldı. Panel sayısına dayanılarak alınan bazı kararlar
(örneğin "V_MAX artırmak işe yaramadı") yanlış temele oturmuştu.

> **Kural:** iki bağımsız ölçüm sürekli çelişiyorsa, aradaki farkın sabit
> olup olmadığına bak. Sabitse bir yerde ofset vardır.

---

## Denendi ve GERİ ALINDI (tekrar denemeyin)

| Deneme | Sonuç |
|---|---|
| `V_MAX` 18 → 24 | Drone hızlanınca çemberi büyüdü (38→43 m), menzil 29→41 m **açıldı** |
| `WPNAV_ACCEL` 8 → 12 + `ANGLE_MAX` 45 → 55 | ⌀55'te 46-48 m, ⌀41'de 40-41 m, ⌀32'de kontrol kaybı |
| `RANGE_SET` 11 → 5 | **Hiçbir etki yok** — menzil >16 m'de komut zaten V_MAX'a doygun, istasyonun yeri komutu etkilemiyor |
| İstasyon yönü normalizasyonu | Uçuşta fayda göstermedi |
| İstasyon hızı ileri beslemesi (ω×s) | Formül doğru ama komut doygun olduğu için etkisi yutuldu |
| FRPN yasası (varsayılan olarak) | Uçuşta eski yasadan iyi çıkmadı (31.1 vs 29.4 m) |
| `ATC_ANG_YAW_P` 4.5 → 3.0 | Yaw hatası 1.36° → 4.94°, toplam 11.96 tur dönme |

---

## Bilinen açıklar

- **Kare köşelerinde 21 m.** Keskin manevrada mesafe açılıyor; dönüş bitince
  toparlıyor. Dar dairelerde (⌀41) da aynı eğilim var.
- **İç daire kayması sabit metre.** Çok dar dairede nişanı fazla içeri iter.
  Yarıçap-oranlı sürüm kodda var ama kapalı (`AVCI_GPS_IC_ORAN=0.27`).
- **⌀32 senaryosu kaldırıldı.** O açısal hızda sürdürülebilir hız
  (`a_max/ω` = 14.2 m/s) hedefin hızına eşit — sıfır pay, kontrol kaybı.
- **Görsel faz kapalı.** Devir menzilinde hedef kutusu 10-12 piksel kalıyordu;
  dedektör o boyutta tutunamıyor.
