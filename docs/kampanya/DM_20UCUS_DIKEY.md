# KAMPANYA DM — DİKEY KOMUT ÖLÇEĞİ, 20 UÇUŞ

**Tek değişken:** `AVCI_IBVS_DIKEY_KAPANMA` 0/1 · **n=10/kol**
(her kolda 5 `circle_xl` + 3 `duz`+`yatay` + 2 `duz`+`yok`, §5.9 eşit),
dönüşümlü · faz eşiği düzeltmesi **iki kolda da** yürürlükte · 2026-08-20

> Kullanıcı: *"eski ayar 4/4, bu yeni dikey şeyiyle vuruş 3/4'e düşmüş.
> Yani bir kötüleştirme var sanırım ama bunu hemen ayırt edemeyiz. Bunu tam
> analiz etmek için 20 koşu yap."*

---

## 1 · ÖNCE: FAZ EŞİĞİ DÜZELTMESİ (iki kolda da)

Kullanıcının diğer iki şikâyeti — *"faz geçişleri arasında takılma"* ve
*"geçişlerde ani fren, drone burnunu kaldırıyor"* — ile **alttan kaçırma**
aynı kökten çıktı:

| katman | eşik | işi |
|---|---|---|
| `supervisor.POSE_CONF_MIN` | **0.0** | görsel faza **girme** |
| `bbox_ibvs.CONF_MIN` | **0.35** | kutuyu **kullanma** |

Arada kalan tespitlerde supervisor "görsel" diyor, güdüm kutuyu reddediyor,
**komut donuyor**, 20 kare sonra GPS'e dönülüyor, GPS istasyona gitmek için
**fren yapıyor**.

**Kullanıcı uçuşu 20260820_170647, kare kare:** dikey ofset 37 m'den 8 m'ye
kadar **−2.7…−3.3 m'de SABİT** kaldı (komut donuk olduğu için
düzeltilmedi), son 8 metrede kapatılmaya çalışıldı, yetişmedi →
**temasta 0.57 m ALTTAN ıska**. Sonra faz VISUAL→GPS ve hız
**16.0 → 9.2 → 8.2 → 7.0** (fren).

**Düzeltme:** `POSE_CONF_MIN` 0.0 → **0.35**. Etkisi ölçüldü:

| | KUTU_YOK oranı |
|---|---|
| önce (eşik 0.0) | %28 · %43 |
| **sonra (eşik 0.35)** | **%3 – %36, medyan %16-28** |

Kör uçulan süre belirgin düştü. ⚠ Tamamen bitmedi — kontrol kolunda medyan
hâlâ %27.5.

---

## 2 · SONUÇ — n=10/kol

| ölçüt | KAPALI | AÇIK | p |
|---|---|---|---|
| **İSABET** | **9/10** | **5/10** | 0.141 (Fisher) |
| \|dikey\| temasta | **0.35 m** | 0.76 m | 0.106 |
| koşunun en yakını | **1.11 m** | 1.56 m | 0.224 |
| **\|vz\| p90** | 4.69 | **2.38** | **0.003** |
| **KUTU_YOK %** | 27.5 | **15.9** | **0.025** |
| **faz düşüşü** | **4.0** | 10.0 | **0.036** |
| ilk temasa süre | 69.6 s | 72.4 s | 0.886 |

## 3 · ⭐ SENARYO KIRILIMI — fark TAMAMEN dairede

| senaryo | KAPALI | AÇIK |
|---|---|---|
| **daire** (n=5) | **4/5** · [0.49, 0.96, 1.70, 0.84, 0.66] | **0/5** · [1.52, 2.18, 3.41, 3.47, 1.94] |
| düz+yatay (n=3) | 3/3 · [1.86, 1.72, 1.12] | **3/3** · [1.57, 1.55, 1.21] |
| düz+yok (n=2) | 2/2 · [1.10, 1.32] | **2/2** · [0.73, 1.11] |

**Dairede 0/5 — beş uçuşun beşi de ıska.** Temas anındaki dikey hata:
AÇIK [0.92, 1.72, 2.76, 2.92, 1.68] · KAPALI [0.41, 0.47, 0.24, 0.85, 0.08].

**Düz senaryolarda ikisi denk** (hatta AÇIK biraz daha yakın: 5 uçuşun
4'ünde en yakın daha küçük).

---

## 4 · ⚠ HİPOTEZİN EKSİK YERİ — bu kez tam anlaşıldı

`vz = ṙ·sin(ε)` yasası, **`d` ofsetinin SABİT olduğunu** varsayar. Türetme
şuydu: `d` metreyi `t_go = R/ṙ` sürede kapat.

**Dairede ofset sabit değil.** Hedef sürekli yatık, biz de yörüngede
dönüyoruz; LOS yükselişi sürekli değişiyor. Sabit bir ofseti kapatmaya
yetecek komut, **değişen** bir ofseti takip etmeye yetmiyor.

Klasik orantısal seyrüsefer bunu bilir ve `N = 3-5` kazancı kullanır;
bizim `ṙ·sin(ε)` fiilen `N = 1`. Düz uçuşta (ofset gerçekten sabit) N=1
yetiyor — ölçüm bunu doğruluyor: düzde 5/5 denk. Dairede yetmiyor.

**Kullanıcının ilk sezgisi doğruymuş:** *"eski ayar 4/4, yeni 3/4'e
düşmüş, bir kötüleştirme var sanırım."* 20 uçuşta 9/10 → 5/10.

---

## 5 · KARAR KURALI DENETİMİ

| # | kural | sonuç |
|---|---|---|
| 1 | isabet kötüleşmez | ✗ **9/10 → 5/10** |
| 2 | \|dikey\| temasta iyileşir | ✗ **0.35 → 0.76 m** |
| 3 | en yakın menzil kötüleşmez | ✗ **1.11 → 1.56 m** |

**Üçü de düştü. `DIKEY_KAPANMA` GİRMEZ.**

---

## 6 · AI ÖNERİSİ

**`DIKEY_KAPANMA` KAPALI kalsın** (zaten öyle, değiştirilmedi).

**Faz eşiği düzeltmesi KALSIN** — ayrı bir değişiklik, iki kolda da
yürürlükteydi, ölçülen etkisi olumlu (KUTU_YOK belirgin düştü) ve
kullanıcının iki şikâyetinin doğrudan çaresi.

⚠ **Açık kalan:** kontrol kolunda KUTU_YOK hâlâ medyan %27.5. Yani araç
zamanın dörtte birinde kör. Faz eşiği bunun bir kısmını çözdü; kalanı
tespit kalitesi (uzakta conf düşük) ve `KAYIP_M` penceresi.

**Denenmemiş ara yol:** ölçek `v_los` (18) ile `kapanma` (1.5) arasında
12 kat sıçrıyor. `N·ṙ` (N=3-5) sınanmadı. Düzde N=1 yetiyor, dairede
yetmiyor — arası işe yarayabilir. **Bu bir gözlem, öneri değil.**

---

## 7 · KULLANICININ KENDİ SINAMASI

```bash
cd ~/projects/avci_sim
bash scripts/kapat.sh && bash scripts/mkur.sh test
```

**Faz düzeltmesini görmek için:** sim terminalinde `[SUPERVISOR] GPS fazı`
satırlarını say — eskiden 70 saniyede 18 güdüm logu oluyordu. Ve geçişlerde
**burun kalkması / ani fren** azalmalı.

**`DIKEY_KAPANMA`'yı denemek için:** Panel → 🎚 AYAR KONSOLU →
**⭐ DİKEY KOMUT ÖLÇEĞİ**. Daire senaryosunda açıp kapa — açıkken aracın
dikeyde hedefi takip edemediğini göreceksin.
