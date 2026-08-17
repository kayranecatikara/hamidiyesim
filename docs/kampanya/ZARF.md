# ZARF BÜYÜTMESİ — aracın manevra kısıtları kaldırıldı

> Kullanıcı kararı 2026-08-17: *"benim gerçek ortamdaki droneum bu
> simülasyondaki dronedan çok daha hızlı ve manevra kabiliyeti yüksek. Bu
> droneu sime tam entegre edemedim. O yüzden aracın herhangi bir manevra
> kısıtı falan varsa, dönüş tavanı vs., bunların hepsini istediğin gibi
> değiştirebilirsin."*

---

## 1 · NİYE — üç kampanya boyunca yanlış şeyi kısmışız

Ö5, S1, S3 ve W1/W2'nin hepsi "araç bunu yapamaz, o hâlde güdümü kıs"
mantığıyla kuruldu ve hepsi **aracı yavaşlattığı için** elendi. Oysa o
kısıtlar fizik değil, zayıf bir sim modelinin keyfi ayarlarıymış.

**Ölçülen talep vs eski aracın yeteneği:**

| büyüklük | güdümün ÖLÇÜLEN talebi | eski araç |
|---|---|---|
| yanal ivme medyan | 8.43 m/s² | tavan **8.0** (`WPNAV_ACCEL 800`) |
| yanal ivme p90 | **26.3 m/s²** | 8.0 |
| LOS dönüş hızı medyan | 42-57 °/s | ω_max **27-35 °/s** |
| LOS dönüş hızı p90 | 109-138 °/s | 27-35 °/s |

Komut, aracın dönüş tavanını **karelerin %43'ünde** aşıyordu.

## 2 · NE DEĞİŞTİ — hepsi ölçülen talepten türetildi, tahmin yok

| ne | eski | yeni | gerekçe |
|---|---|---|---|
| rotor itkisi (`LiftDrag <area>`) | 0.002 m² | **0.005 m²** | ×2.5 itki; itki/ağırlık 2.56 → ~6.4 |
| `ANGLE_MAX` | 4500 (45°) | **7000 (70°)** | atan(26.3/9.81) = 69.6° ← ölçülen p90 talep |
| `WPNAV_ACCEL` | 800 (8.0 m/s²) | **2600 (26 m/s²)** | g·tan70° = 27.0'ın hemen altı |
| `PSC_JERK_XY` | 12 | **40** | a_max büyüyünce sert jerk tavanı da büyür |
| `ATC_RAT_RLL/PIT_P,I` | 0.135 | **0.054** | itki ×2.5 → aynı PID çıkışı 2.5× tork; telafi edilmezse duruş döngüsü salınır |
| `ATC_RAT_RLL/PIT_D` | 0.0036 | **0.00144** | aynı ölçekleme |
| `ATC_ACCEL_R/P_MAX` | 110000 | **250000** | 70°'ye hızlı gitmek için |

**⚠ Duruş kazançlarının ölçeklenmesi zorunluydu.** Atalet aynı kalırken itki
2.5 katına çıkınca duruş döngüsünün etkin kazancı da 2.5 katına çıkar;
telafi edilmezse tam da çözmeye çalıştığımız salınımı ÜRETİRDİ. Uçuş bunu
doğruladı: duruş döngüsü kararlı, salınım artmadı.

## 3 · MEKANİZMA KAPISI — SONUNA KADAR AÇIK

`square`, aynı güdüm, aynı senaryo:

| | eski araç (n=5) | **yeni zarf (n=3)** |
|---|---|---|
| yatış p50 / p90 / p99 / maks | 18.2 / 33.5 / 40.6 / 44.5° | **20.4 / 50.1 / 62.7 / 68.0°** |
| yanal ivme p90 | 6.5 m/s² | **11.7 m/s²** |
| jerk p90 | 12.2 | **29.7 m/s³** |
| görsel temas (kutu oranı) | %33.7 | %33.4 — **bozulmadı** |
| medyan mesafe | 58.9 m | **53.5 m** |
| salınım ψ̇ | 0.265 | 0.266 — **değişmedi** |

Araç artık gerçekten 45°'yi aşıyor (maks **68°**), çakılmadı, görsel temasını
kaybetmedi ve duruş döngüsü salınıma girmedi.

## 4 · ⛔ KENDİ ÜRETTİĞİM GERİLEME: `MAX_ACCEL` 12 → 26

Zarf büyürken güdümün kendi kırpıcısını da büyüttüm (`Cfg.MAX_ACCEL`,
eski aracın 8 m/s²'sine göre konmuş 12 → 26). **Bu bir hataydı.**

Düz + kaçamak, n=4/kol, tür-eşli:

| kol | isabet | en yakın menzil | medyan |
|---|---|---|---|
| `MAX_ACCEL = 26` | 1/4 | 3.30 / 3.17 / 2.40 / 2.68 | **2.92 m** |
| `MAX_ACCEL = 12` | 0/4 | 1.57 / 1.82 / 1.77 / 2.08 | **1.79 m** |

**TAM AYRIŞMA, p=0.057** (n=4+4'te ulaşılabilen en küçük p) — dördü de daha
yakın. Sebep: 16 m/s'de 26 m/s², hız vektörünün **93 °/s** savrulmasına izin
verir. 12'lik sınır terminalde fiilen **yumuşatıcı** görevi görüyormuş.

→ `MAX_ACCEL` **12'ye geri alındı.**

## 5 · SONUÇ — bölünmüş, ve dürüst hâli şu

**Zarf büyütmesi TAKİBİ iyileştirdi, BİTİRİŞİ iyileştirmedi.**

| | eski araç | yeni zarf |
|---|---|---|
| kare medyan mesafe (takip) | 58.9 m | **53.5 m** ✓ |
| kare 60 m içi süre | 125 s | **146 s** ✓ |
| **düz isabet** | **4/8** | **1/8** ✗ |
| düz en yakın medyan | ~1.5 m | 1.79 m (MAX_ACCEL 12 ile) |

Aracın yeteneğini 3 katına çıkarmak, **tek başına isabeti getirmedi.** Bu,
oturumun tekrar eden bulgusunu bir kez daha doğruluyor: **takip ile bitiriş
ayrı problemler**, ve çevikliği artıran her şey (jerk 15, Ö5, şimdi zarf)
takibi düzeltip terminali bozuyor.

⚠ n: kare 3, düz 4/kol. §5.4'ün sınırında; "isabet 4/8 → 1/8" bir eğilimdir,
kesin hüküm değil.

## 6 · SIRADAKİ TEK DEĞİŞKEN — `V_TERMINAL` (henüz DENENMEDİ)

Terminalin neden bitmediğinin en olası sebebi hâlâ ölçülmemiş duruyor:

> `V_TERMINAL = 16 m/s`, hedef **15.1 m/s** → kalan kapanma **0.9 m/s**.

Bu, Ö5 kampanyasının da kalıcı bulgusuydu. **Yeni zarf bunu ilk kez
çözülebilir kılıyor:** eskiden hızı artırmak dönüşü imkânsızlaştırıyordu
(R = V²/(g·tanθ)); şimdi

| | V | yatış | dönüş yarıçapı |
|---|---|---|---|
| eski araç | 16 m/s | 45° | **26.1 m** |
| yeni zarf | 16 m/s | 70° | **9.5 m** |
| yeni zarf | **24 m/s** | 70° | **21.4 m** — hâlâ eski aracın 16 m/s'sinden DAR |

Yani yeni araç 24 m/s'de bile eski aracın 16 m/s'deki dönüşünden daha dar
dönüyor. Kapanma 0.9 → **8.9 m/s**'ye çıkar.

**Bu bir güdüm YASASI parametresidir** — CLAUDE.md §1 gereği ölçmeden önce
kullanıcı onayı alınır.
