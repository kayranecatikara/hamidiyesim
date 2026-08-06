# Pose Veri Seti ve Model Eğitimi — Ne Yaptık, Neden Yaptık

Bu doküman, **hedef İHA'nın rotasyonunu (yaw/pitch/roll) görüntüden çıkaran**
pose zincirinin baştan sona nasıl düzeltildiğini anlatır. Kronolojik yazılmıştır:
başlangıçta ne vardı, hangi sorunu bulduk, ne değiştirdik, sonra hangi sorun
çıktı, o nasıl çözüldü.

**Yanlış çıkan denemeler de yazılmıştır.** Bunlar boşluk doldurmak için değil;
aynı yola tekrar girilmesin diye. Her iddianın yanında onu doğrulayan ölçüm
sayısı vardır — hiçbir sonuç tahmine dayanmıyor.

> **Okuyucu notu (insan veya yapay zekâ):** Tablolardaki sayılar gerçek uçuş ve
> doğrulama ölçümlerinden alınmıştır. Ölçümü üreten araç her bölümde belirtilir,
> böylece iddialar tekrar üretilebilir.

---

## 0. Sistemin özeti

Avcı bir multikopter (iris), sabit kanatlı bir hedefi (mini Talon) kovalar.
Hedefin **nereye baktığını** bilmek, önden kesme (lead) ve müdahale için gerekir.

Zincir şöyle işler:

```
Gazebo kamerası (640×480)
      → YOLO detection            → hedefin kutusu
      → kutunun etrafından KROP   → 192×192'ye büyütülür
      → YOLO-pose                 → 6 keypoint (burun, kuyruk, 2 kanat ucu, 2 V-tail)
      → rotasyon çözümü           → hedefin dünya yaw/pitch/roll'u
      → ArduPilot telemetrisiyle karşılaştırma (doğruluk ölçümü)
```

6 keypoint'in gövde çerçevesindeki 3B konumu, Talon'un collision mesh'lerinden
otomatik türetilir (`vision/geometry.py: talon_keypoints`). Veri seti tamamen
otomatik etiketlenir: hedef ve kamera bilinen pozlara taşınır, keypoint'ler
projeksiyonla hesaplanır, örtülü kalanlar ışın-mesh testiyle işaretlenir.
Manuel etiketleme yoktur.

---

## 1. Başlangıç durumu (bu çalışmadan önce repoda olan)

| bileşen | durum |
|---|---|
| pose veri seti | `talon_pose_krop`, 20 000 kare, geometry'nin **gerçek kutusundan** kroplanmış |
| pose modeli | `avci_pose_krop.pt`, yolo11n-pose, 100 epoch, **pose mAP50-95 = 0.534** |
| rotasyon çözümü | PnP (`solvePnP`), 6 keypoint veya kanatsız 4 keypoint |
| uçuşta yaw hatası | **67.2°** |
| çözüm üretilen kare | **%9** |

Yani sistem çalışıyordu ama rotasyon çıktısı kullanılamaz haldeydi: arayüzdeki
"gerçek" ve "tahmin" eğrileri birbirini hiç tutmuyordu.

---

## 2. Birinci sorun — model iki kanadı üst üste koyuyordu

### Bulgu

Val setinde ölçüldü (`tools/pose_kanat_olc.py`, 400 kare):

| ölçüm | değer |
|---|---|
| gerçek kanat açıklığı | 80.1 px |
| modelin verdiği açıklık | 3.6 px |
| **oran** | **0.05** |
| kanat noktası hatası | ~40 px |

40 px, kanat açıklığının **tam yarısı**. Bu rastgele bir hata değil: model iki
kanadı da gövdenin ortasına koyuyordu. Karelerin **%99.8'inde** iki kanat
çakışıktı.

### Bu neden oluyor

Model "sol kanat" ve "sağ kanat"ı ayırt edemiyor. 38 m'de hedef görüntüde
~6 piksel; o ölçekte hangi kanadın sol olduğunu gösteren görsel ipucu yok.
Belirsizlik altında, L2 kaybını en aza indiren tahmin **iki ihtimalin ortasıdır**.
Model matematiksel olarak doğru olanı yapıyordu; sorun ondan istediğimiz şeydeydi.

### Denenip işe yaramayan: `fliplr=0`

Augmentation'ın aynalama yaptığı, bunun da sol/sağ'ı bozduğu düşünülerek model
`fliplr=0` ile yeniden eğitildi (`avci_pose_krop_v2.pt`, ~42 dakika).

| | oran |
|---|---|
| önce | 0.05 |
| `fliplr=0` sonrası | **0.05** |

Hiç değişmedi. Sorun augmentation değildi.

### Sıra sorunu mu, konum sorunu mu?

Ayırt etmek için her karede iki eşleştirme denendi:

| ölçüm | değer |
|---|---|
| sıralı eşleştirme hatası | 39.9 px |
| **en iyi (çapraz dahil) eşleştirme** | **39.9 px** |
| modelin sırayı ters verdiği kare | %10.9 |
| iki kanadı çakışık verdiği kare | **%99.8** |

Çapraz eşleştirme hatayı hiç düşürmedi. Yani sıra karışıklığı değil — model
kanadı gerçekten "bulamıyor", ortalamaya kaçıyordu.

---

## 3. Birinci çözüm — keypoint'leri piksel sırasına çevirmek

### Fikir

Modelden **"sol kanat / sağ kanat"** istemeyi bırakmak. Bunlar 3B anlamdır ve
küçük hedefte ayırt edilemez. Yerine **"görüntüde soldaki / sağdaki kanat"**
istemek — bu her karede piksel x'ine bakarak kesin belirlenir, belirsizlik kalmaz,
ortalamaya kaçacak bir şey olmaz.

Aynı düzeltme V-tail çiftine de uygulandı.

### Uygulama

`vision/capture_pose_dataset.py` içinde, etiket yazılmadan hemen önce:

```python
for _a, _b in ((2, 3), (4, 5)):          # kanatlar, V-tail'ler
    if kp_krop[_a] > kp_krop[_b]:        # (x, y) sözlük sırası
        kp_krop[_a], kp_krop[_b] = kp_krop[_b], kp_krop[_a]
        kpts[[_a, _b]] = kpts[[_b, _a]]
```

Mevcut 20 000 etiketi yeniden çekmeden dönüştürmek için
`tools/etiket_piksel_sirala.py` yazıldı — **%51.8** satırda çift takaslandı
(beklenen: rastgele yarısı).

`dataset.yaml` içindeki `flip_idx: [0,1,3,2,5,4]` bu düzenle uyumludur:
aynalamadan sonra soldaki nokta sağa geçer, `flip_idx` takası onu doğru indekse
koyar. Bu yüzden `fliplr` augmentation'ı **açık bırakılabildi**.

### Sonuç

Önce 20 epoch'luk ön test yapıldı (boşuna 40 dakika harcamamak için):

| ölçüm | eski model | 20 epoch ön test |
|---|---|---|
| kanat açıklığı oranı | 0.05 | **1.03** |
| kanat noktası hatası | ~40 px | **2.0 px** |
| pose mAP50-95 | 0.534 | **0.817** |

Ön test tuttuğu için tam eğitime geçildi (100 epoch, `avci_pose_sirali.pt`):

| ölçüm | değer |
|---|---|
| kanat açıklığı oranı | **1.04** |
| V-tail açıklığı oranı | **1.00** |
| tüm keypoint hataları | 1.9 – 3.3 px |
| pose mAP50-95 | **0.875** |
| uçuşta yaw hatası | 67.2° → **5.7°** |

---

## 4. İkinci sorun — model uçuşta, val'de olduğundan çok daha kötüydü

### Bulgu

Aynı model, aynı hedef:

| ortam | keypoint hatası (krop pikselinde) |
|---|---|
| val seti | **2.2 px** |
| uçuş | **14 – 28 px** |

6–13 kat fark. Model kötü değildi; uçuşta **başka bir görüntü** görüyordu.

### Sebep: krop penceresinin kaynağı

- **Eğitimde** krop, geometry'nin hesapladığı **gerçek kutudan** kesiliyordu.
- **Uçuşta** krop, **detection modelinin kutusundan** kesiliyor.

Detection küçük hedefte kutuyu şişiriyor:

| mesafe | gerçek kutu | detection kutusu | şişme |
|---|---|---|---|
| 13 m | 16.6 px | 19 px | 1.14× |
| 38.9 m | 5.5 px | 13.5 px | **2.46×** |

Krop penceresi o kutudan türetildiği için hedef krop içinde küçülüyordu:

| | hedef krop içinde | merkezden sapma |
|---|---|---|
| eğitim (val) | 79 px | ~0 |
| uçuş | **31.5 px** | **17 px** |

Modelin hassasiyeti hedef boyutuna sert bağlı (val ölçümü,
`tools/krop_olcek_olc.py`):

| hedef boyutu | keypoint hatası |
|---|---|
| 79 px | 2.27 px |
| 55 px | 2.07 px |
| 44 px | 2.14 px |
| **32 px** | **5.00 px** ← uçuş tam burada |

Yani uçuş, modelin bozulma eşiğinde çalışıyordu.

### Denenip işe yaramayan: krop marjını daraltmak

`KROP_MARGIN` 2.5 → 1.5 yapıldı (hedef krop içinde 43.8 px'e çıkıyor):

| ölçüm | marj 2.5 | marj 1.5 |
|---|---|---|
| yaw (ham) | 7.4° | 9.6° |
| pitch | 5.3° | 9.4° |
| PnP mesafe hatası | 20.9 m | **14.5 m** |

Mesafe düzeldi ama rotasyon kötüleşti — çünkü model 2.5 marjla eğitilmişti,
marjı değiştirmek onu dağılımdan bir kez daha uzaklaştırdı. Ayrıca şişme oranı
mesafeye bağlı olduğu için **sabit bir marj bunu kapatamaz**. Geri alındı.

---

## 5. İkinci çözüm — veri setini uçuşun gördüğü gibi üretmek

### Fikir

Veri seti de kropu **detection modelinin kendi kutusundan** alsın. O zaman
eğitim ile çıkarım aynı pencereyi görür; şişme, kayma, bulanıklık hepsi
eğitim dağılımına dahil olur.

### Uygulama

`vision/capture_pose_dataset.py`:

```python
krop_bb = bb                              # geometry'nin gerçek kutusu
if _det is not None:
    _d = _det.detect_talon(frame)
    if _d is None:
        continue                          # uçuşta da pose çalışmazdı
    krop_bb = _d["bbox"]                  # DETECTION kutusu
krop, kx1, ky1, olcek = _krop.krop_al(frame, krop_bb)
```

Etiketler yine geometry'nin gerçek keypoint'lerinden gelir — değişen yalnız
pencerenin nereden hesaplandığıdır. Detection hedefi bulamazsa kare atlanır;
uçuşta da o karede pose zaten çalışmazdı.

**Kritik ayrıntı:** capture sırasında kullanılan detection modeli ve güven eşiği,
uçuştakiyle **birebir aynı** olmalıdır. Aksi halde aynı tuzağın başka bir yüzüne
düşülür:

```bash
export AVCI_YOLO_MODEL=$PWD/vision/models/avci_yolo.pt
export AVCI_YOLO_CONF=0.35
```

> **Not:** `tools/ucus_baslat.sh` bu modeli `avci_yolo_uzak.pt` adıyla arar;
> ikisi aynı dosyadır (MD5 eşit). Repoda `avci_yolo.pt` bulunur. Uçuş
> scriptini kullanacaksanız bir kez kopyalayın:
> `cp vision/models/avci_yolo.pt vision/models/avci_yolo_uzak.pt`

Ayrıca mesafe dağılımı takip mesafesine ağırlıklandırıldı: **12–55 m, ortanca
35 m**, karelerin %35'i 30–45 m bandında.

### Sonuç

`avci_pose_ucus.pt` (20 000 kare, 100 epoch):

| ölçüm | önceki model | yeni model |
|---|---|---|
| PnP mesafe hatası | 21.3 m | **6.8 m** |
| yaw (filtreli) | 5.4° | **3.6°** |
| kanat açıklığı oranı | 1.04 | 1.01 |

> **Not:** Bu modelin val mAP50-95'i (0.763) bir öncekinden (0.880) *düşük*
> görünür. Bu yanıltıcıdır — val setleri farklı zorluktadır. `sirali` kolay
> veride (gerçek kutudan krop), `ucus` gerçekçi veride (detection kutusundan
> krop) ölçülmüştür. İkisi **aynı gerçekçi sette** karşılaştırıldığında `ucus`
> kanatlarda daha iyidir (3.1/3.6 px vs 3.6/3.9 px).

---

## 6. Üçüncü sorun — PnP bu ölçekte matematiksel olarak çözülemiyor

Model artık sağlamdı ama rotasyon hâlâ beklenen kadar iyi değildi.

### Bulgu

Uçuşta çelişkili bir durum vardı: **yeniden-izdüşüm hatası 0.07 px** (yani
çözüm keypoint'lere kusursuz oturuyor) ama **pitch hatası 11°**. Bu, klasik
*zayıf koşullanma* (ill-conditioning) imzasıdır.

Sentetik duyarlılık testi (`tools/poz_zincir_testi.py` mantığıyla, saf geometri,
modelden bağımsız) — 38 m'de keypoint'lere **1 piksel** gürültü:

| nokta seti | yaw | pitch | roll |
|---|---|---|---|
| kanatsız (kullanılan) | 14.9° | **56.8°** | 54.4° |
| hepsi (6 nokta) | 154.9° | 29.6° | 37.7° |

1 piksel → onlarca derece. 6 serbestlik dereceli tam poz, 6 piksellik hedeften
çıkarılamıyor.

### Denenip işe yaramayan: ayna hipotezi

Model artık "görüntüde soldaki" kanadı verdiği için, bunun gövdenin hangi kanadı
olduğunu bulmak üzere iki eşleşme denenip yeniden-izdüşümü düşük olan seçildi.

| yöntem | yaw hatası |
|---|---|
| tek hipotez (sabit eşleşme) | 13.3° |
| **iki hipotez, reprojection ile seçim** | **124.5°** |

Uçak neredeyse simetrik olduğu için iki eşleşme benzer hata veriyor, seçim
yazı-turaya dönüyor. Kaldırıldı. Takipte hedef hep arkadan görüldüğü için doğru
eşleşme zaten sabittir.

---

## 7. Üçüncü çözüm — PnP yerine kanat ekseni

### Fikir

Tam 3B pozu çözmeyi bırakmak. Rotasyonun asıl ihtiyaç duyulan bileşeni **yaw**;
onu iki kanat ucunun **yönünden** çıkarmak mümkün. Bu yöntem derinliği ve ölçeği
hiç çözmez, serbestlik derecesi düşük olduğu için gürültüye dayanıklıdır.
Kanatlar en uzun bazı verir (1.28 m) ve uçuşta %100 görünürdür.

### Sonuç (3512 uçuş karesi)

| yöntem | ham | p=9 | p=15 |
|---|---|---|---|
| **kanat ekseni** | **2.6°** | 1.6° | **1.3°** |
| V-tail ekseni | 2.6° | 1.6° | 1.3° |
| PnP tam 3B | 6.8° | 4.1° | 3.6° |
| gövde ekseni (burun-kuyruk) | 34.0° | — | — |

Gövde ekseni kullanılamaz çünkü burun uçuşta örtülüdür (aşağıya bakınız).

`+90°` ofseti: model "görüntüde soldaki" kanadı verir; takipte hedef hep arkadan
görüldüğü için soldaki kanat gövdenin sol kanadıdır ve burun kanat ekseninin
90° sağındadır. Ölçüm doğruluyor: `+90°` → 2.6°, `−90°` → 177.4° (tam ters).

---

## 8. Dördüncü sorun — manevrada bozulma

Düz uçuşta 1.6°, ama hedef veya avcı dönerken 8.5°'ye çıkıyordu. Filtre pencere
büyütmek hiç işe yaramıyordu (8.5° → 8.4°), yani hata ham veriden geliyordu.

### Teşhis — eleyerek

| şüpheli | ölçüm | sonuç |
|---|---|---|
| Gazebo karesinin yaşı | **18 ms** | gecikme görüntüde değil |
| GCS işleme süresi | **26 ms** | orada da değil |
| hata ↔ hedefin dönüş hızı | +0.369 | zayıf ilişki |
| **hata ↔ avcının dönüş hızı** | **+0.517** | **güçlü ilişki** |

| avcının dönüş hızı | yaw hatası |
|---|---|
| 0–2 °/s | 1.7° |
| 5–10 °/s | 8.4° |
| 10–20 °/s | **12.3°** |

Sebep yapısal: kanat ekseni hesabı, kameranın nereye baktığını bulmak için
**avcının rotasyonunu** kullanır. Avcının telemetrisi gecikmeli geldiği için
avcı dönerken kamera yönü yanlış biliniyordu.

### Denenip işe yaramayan (1): sabit telemetri kaydırması

Offline'da telemetriyi 1 sn geri kaydırmak manevra hatasını 8.3° → 2.3°
düşürüyordu. Canlı sistemde aynı telafi uygulandı:

| | manevra hatası |
|---|---|
| offline vaat | 8.3° → 2.3° |
| **canlı sonuç** | 8.5° → **8.3°** (hiç değişmedi) |

Genel hata ise biraz arttı. Geri alındı; sebebi doğrulanmadan tekrar açılmamalı.

### Denenip işe yaramayan (2): tüm rotasyonu ileri taşımak

Açısal hızlarla avcının roll+pitch+yaw'ı 1 sn ileri taşındı:

| | roll hatası |
|---|---|
| önce | 6.5° |
| sonra | **17.3°** |

Roll tahmini çöktü. Sebebi ölçüldü: avcı bir multikopter, roll hızı ortanca
**19.3 °/s**, yaw hızı yalnız **1.0 °/s**. Hızlı ve salınımlı bir ekseni 1 sn
ileri taşımak onu tamamen bozuyor.

---

## 9. Dördüncü çözüm — yalnız yaw'ı ileri taşımak

MAVLink `ATTITUDE` mesajının taşıdığı `yawspeed` ile avcının **yalnız yaw'ı**
1 saniye ileri taşınır. Sabit zaman kaydırmasından farkı, dönüş hızına göre
ölçeklenmesidir.

| yöntem | yaw ham | filtreli | avcı dönerken |
|---|---|---|---|
| ekstrapolasyon yok | 3.9° | 2.5° | 8.1° |
| hepsi 1.0 sn | 3.5° | 2.2° | 4.8° |
| **yalnız yaw 1.0 sn** | **3.0°** | **1.9°** | **4.1°** |

1.5 sn'de tekrar kötüleşiyor (6.3°) — yani gerçek bir optimum, uydurma değil.

### Uçuşta doğrulama

| durum | önce (ham / filtreli) | sonra (ham / filtreli) |
|---|---|---|
| **avcı dönerken** | 8.8° / 8.6° | **2.7° / 1.4°** |
| **hedef dönerken** | 8.5° / 8.4° | **3.4° / 1.7°** |
| düz uçuş | 1.6° / 0.7° | 1.9° / 1.0° |

---

## 10. Çözülemeyen — pitch

Pitch **çıkarılamıyor** ve bu bir eksiklik değil, üç bağımsız sebebi var.

### (a) Kanat ekseni pitch taşımaz — geometrik kimlik

Kanat ekseni gövde çerçevesinde `(0,1,0)`. Pitch rotasyonu `Ry` tam olarak bu
eksen **etrafında** döner:

```
Ry( 0°)·(0,1,0) = (0, 1, 0)
Ry(30°)·(0,1,0) = (0, 1, 0)      ← değişmiyor
Ry(60°)·(0,1,0) = (0, 1, 0)      ← değişmiyor
```

Roll ve yaw bu ekseni değiştirir (20°'de değişim 0.347), pitch değiştirmez.
Yaw ve roll'un düzelip pitch'in düzelmemesinin birinci sebebi budur.

### (b) Pitch bilgisi olan eksen görünmüyor

Pitch yalnız gövdenin uzunlamasına ekseninde güçlüdür (burun→kuyruk, kol 0.81 m).
Takipte hedefi hep arkadan gördüğümüz için **burun örtülüdür (%0.3 görünürlük)** —
bu normaldir, occlusion doğru çalışmaktadır. Kalan en uzun kol 0.324 m ve görüş
hattı boyunca uzandığı için izdüşümde iz bırakmaz.

Altı aday gösterge denendi; gerçek pitch ile korelasyonları:

| gösterge | korelasyon |
|---|---|
| kuyruk → kanat merkezi | −0.082 |
| kuyruk → vtail merkezi | +0.177 |
| vtail → kanat merkezi | −0.120 |
| kuyruk → kanatA | −0.343 |
| piksel dikey farklar | +0.054 / +0.091 |

Hepsi sıfıra yakın. (Kıyas: kanat ekseni yaw'ı taşıdığında hata anında
6.8° → 2.6° düşmüştü.)

### (c) Tahmin edilecek sinyal zaten yok

Takipte hedefin gerçek pitch'i: ortanca **−4.6°**, standart sapma **1.4°**,
aralık [−8°, −2°]. Hedef neredeyse sabit pitch'te uçuyor.

| yöntem | pitch hatası |
|---|---|
| PnP tahmini | **12.1°** |
| uzun filtre (2.1 sn gecikmeyle) | 4.2° |
| **hiç tahmin etmemek (sabit varsaymak)** | **0.8°** |

PnP olmayan bir değişimi tahmin etmeye çalışıp gürültü üretiyor.

### Sonuç

`hedef_rotasyonu` pitch'i döndürmeye devam eder ama **`pitch_guvenilir: False`**
işaretler. Görsel güdüm hedefin pitch'ini zaten kullanmaz. Gerçekten ölçmek
gerekirse tek yol takip geometrisini değiştirip hedefi **yandan** görmektir.

---

## 11. Ölçüm tuzakları — tekrar düşülmemesi için

Bu çalışmada dört kez yanlış teşhis konuldu ve hepsi ölçümle çürütüldü. Ölçüm
yapan herkesin bilmesi gerekenler:

### (a) `/api/video_feed` overlay çizilmiş kare döndürür

GCS'in video akışına detection ve keypoint çizimleri **basılmıştır**. O akış
üzerinde tekrar detection çalıştırılırsa model kendi çizimlerini hedef sanar.
Bu yüzden bir ara şu iki **yanlış** sonuca varıldı:

- "FX 2.4× yanlış" (405 ölçüldü, gerçek 166.58)
- "kamera tilt'i pitch'e 92.9 px sızıyor"

Ham `gz` topic'i (`/iris_cam/image`) ile tekrarlandığında geometry'nin kutusu
(16.6 px) ile detection kutusu (19 px) örtüştü. Kamera içsel parametreleri
`/iris_cam/camera_info` ile doğrulandı: **FX = 166.58**, iki dünyada da aynı.

**Kural:** kamera/geometri ölçümü için ham topic kullanılmalı, video akışı değil.

### (b) GCS çözümsüz kareyi loglamıyordu

Panel deque'si yalnız başarılı kareleri tutar (`if tahmin is None: return`).
Bu yüzden "811 kayıtta %100 çözüm" sanıldı; gerçekte 428 saniyede ~8570 karenin
**%9'uydu**. Çözüm oranı ancak süre ile kıyaslanarak ölçülebilir.

Bunun için `AVCI_POZ_HAM_LOG` eklendi: her kare, çözümsüzler dahil, keypoint'leri
ve iki aracın pozuyla diske yazılır. Böylece **tek uçuştan onlarca varyant
offline denenebilir** — her deneme için yeniden uçmak gerekmez.

### (c) `target_keypoints` örtülü noktaları (0,0) döndürür

YOLO-pose kuralı gereği görünmeyen nokta `u=v=0`'dır. Bunu gerçek konum sanıp
karşılaştırınca "burun 397 px sapıyor" gibi anlamsız sonuç çıkar. Karşılaştırma
yalnız `vis > 0` olan noktalarla yapılmalıdır.

### (d) Bozuk bir bileşenin üstünde ayar taraması yanıltır

Kanatlar bozukken yapılan tilt taraması "tilt = 0 daha iyi" diyordu (pitch
25.9° → 5.3°). Veri seti düzeltilip model yeniden eğitilince aynı tarama tersini
söyledi:

| tilt | yaw | pitch | roll |
|---|---|---|---|
| 0° | 8.7° | 30.9° | 4.0° |
| **−25° (SDF'nin gerçek değeri)** | **6.5°** | **10.8°** | 4.4° |

Kamera modeli baştan doğruymuş; hatayı keypoint'ler üretiyordu.

**Kural:** önce bileşeni düzelt, sonra ayar ara.

---

## 12. Nihai sonuçlar

35–40 m takip uçuşu, `square` senaryosu, 4700+ çözülmüş kare:

| ölçüm | başlangıç | **şimdi** |
|---|---|---|
| **yaw** (ham / filtreli) | 67.2° / 65.4° | **2.5° / 1.2°** |
| yaw — avcı dönerken | 8.8° | **1.4°** |
| yaw — hedef dönerken | 8.5° | **1.7°** |
| **roll** (filtreli) | 11.1° | **3.7°** |
| pitch (filtreli) | 16.0° | 7.9° *(güvenilmez)* |
| **PnP mesafe hatası** | 21.6 m | **7.0 m** |
| çözüm üretilen kare | %9 | **%50–70** |
| takipte 1 sn üstü kesinti | 121 | **0** |
| çözümler arası süre | 340 ms | **35 ms** |

**Model kalitesi** (val, `tools/pose_kanat_olc.py`):

| ölçüm | eski | yeni |
|---|---|---|
| kanat açıklığı oranı | 0.05 | **1.01** |
| V-tail açıklığı oranı | — | **0.98** |
| keypoint hataları | ~40 px (kanatlar) | 3.0 – 5.0 px |

### Bilinen sınırlar (dürüstçe)

- **Pitch ölçülemiyor** (bölüm 10). Geometrik sınır, kod sorunu değil.
- **PnP mesafesi ±7 m.** 38 m'de %18. Bu ölçekte beklenen davranış; mesafe zaten
  telemetriden bilindiği için sistemde bu değere ihtiyaç yok.
- **Test kapsamı:** doğrulama 35–40 m bandında ve `square` senaryosunda yapıldı.
  20 m veya 50 m+ için sistemli ölçüm yok.
- **Görsel güdüm kapalıyken** ölçüldü. Pose şu an paneli besliyor, uçağı
  yönetmiyor. Güdüme bağlanırsa ~1 saniyelik telemetri gecikmesi orada canlı bir
  sorun hâline gelir.
- Detection'ın **pervane yanlış-pozitifi** duruyor: avcı yerdeyken bile karelerin
  %99.7'sinde "hedef" bulunuyor. Rotasyonu etkilemiyor (menzil süzgeciyle
  ayıklanıyor) ama duruyor.

---

## 13. Nasıl çalıştırılır

### Veri seti üretimi

```bash
# 1) Statik çekim dünyasını başlat (ayrı terminal)
export GZ_SIM_RESOURCE_PATH=$PWD/sim/gazebo_harmonic/models:$HOME/ardupilot_gazebo/models
gz sim -r sim/gazebo_harmonic/worlds/dataset_capture.sdf

# 2) Detection modeli UÇUŞTAKİYLE AYNI olmalı — bu şart
export AVCI_YOLO_MODEL=$PWD/vision/models/avci_yolo_uzak.pt
export AVCI_YOLO_CONF=0.35

# 3) Pose veri setini çek (20k ≈ 85 dakika)
python3 -m vision.capture_pose_dataset \
        --count 20000 \
        --out vision/datasets/talon_pose_ucus \
        --dist-min 12 --dist-max 55 --dist-exp 0.8 --min-box 4
```

Krop artık varsayılan olarak **detection kutusundan** alınır. Eski davranış
(geometry'nin gerçek kutusu) `--gercek-kutu-krop` ile seçilebilir, ama
**önerilmez** — bölüm 4'teki soruna geri dönülür.

### Eğitim

```bash
python3 tools/pose_sirali_egit.py tam       # 100 epoch, imgsz=192, yolo11n-pose
```

Ya da çekim bitince eğitimi otomatik başlatan zincir:

```bash
bash tools/pose_ucus_zinciri.sh
```

### Modelin sağlamlığını doğrulama

```bash
python3 tools/pose_kanat_olc.py vision/models/avci_pose.pt vision/datasets/talon_pose_ucus
```

Beklenen çıktı: **kanat açıklığı oranı ≈ 1.0**, keypoint hataları birkaç piksel.
Oran 0.05 civarındaysa piksel sıralaması uygulanmamış demektir (bölüm 2–3).

---

## 14. Dosya rehberi

| dosya | rolü |
|---|---|
| `vision/capture_pose_dataset.py` | pose veri seti üretimi — piksel sıralaması ve detection-kutusundan krop burada |
| `vision/capture_dataset.py` | detection veri seti üretimi; poz örnekleyicileri buradan gelir |
| `vision/krop.py` | eğitim ve çıkarımın **ortak** krop modülü — ikisi ayrışırsa model hata vermeden kötü çalışır |
| `vision/geometry.py` | 3B keypoint'ler, projeksiyon, occlusion (değiştirilmedi) |
| `vision/train_yolo_pose.py` | pose eğitimi |
| `vision/train_yolo.py` | detection eğitimi |
| `tools/pose_sirali_egit.py` | pose eğitim scripti (ayarlarıyla birlikte) |
| `tools/pose_ucus_zinciri.sh` | çekim bitince eğitimi otomatik başlatır |
| `tools/etiket_piksel_sirala.py` | mevcut etiketleri piksel sırasına çevirir (yeniden çekmeden) |
| `tools/pose_kanat_olc.py` | model kalitesini ölçer — kanat açıklığı oranı |

### Eğitim ayarları (`tools/pose_sirali_egit.py`)

| parametre | değer | gerekçe |
|---|---|---|
| `model` | `yolo11n-pose.pt` | 192 px kropta yeterli; kapasite sorunu ölçülmedi |
| `imgsz` | 192 | `vision/krop.KROP_BOYUT` ile **aynı olmalı** |
| `epochs` | 100 | `patience=20` ile erken durur |
| `batch` | 32 | 8 GB GPU'da sığar |
| `cache` | `"ram"` | dataloader kilitlenmesine karşı |
| `fliplr` | 0.5 | `flip_idx` doğru olduğu için güvenli |
| `pose` | 12.0 | keypoint kaybının ağırlığı |

> `imgsz` ile `KROP_BOYUT` ayrışırsa model **hata vermeden** kötü çalışır. Bu
> tuzağa detection tarafında bir kez düşüldü (model 1280'de eğitilmiş, sistem
> 640'ta çalıştırıyordu); `vision/detector.py` artık eğitim `imgsz`'ini
> ağırlıktan okuyup kendisi ayarlar.
