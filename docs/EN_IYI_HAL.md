# EN İYİ HAL — 2026-08-17 · kullanıcı doğrulamalı yapılandırma

> Kullanıcı, `logs/kayit/ucus_20260817_205915` uçuşundan sonra:
> *"baş kısımlarda hedefin yaptığı manevralara verdiğimiz reaksiyon **aşırı
> iyi, çok çok iyi**, salınım falan da yok. Yalnız son terminal kısmında
> hedef aracı nasıl oluyorsa işte kaçırıyoruz. Yani **tüm paslar okey,
> bitiriş çok kötü.** Sistemin bu hâlini GitHub'a pushla."*

Bu dosya o uçuştaki **tam yapılandırmayı** ve **ölçülen bitiriş sorununu**
kayda geçirir. Buraya her zaman geri dönülebilir.

---

## 1 · YAPILANDIRMA — bu uçuşta yürürlükte olan tam liste

### Güdüm (`control/guidance/bbox_ibvs.py`)

| alan | değer | anlamı |
|---|---|---|
| `TERMINAL_BOYUT` | **18 px** | terminale **8.9 m**'de girilir (kullanıcı panelden 25→18 aldı) |
| `DIKEY_KAPI_M` | **2.0 m** | irtifa eşitlenmeden terminale geçilmez |
| `VZ_MAX` | **8 m/s** | dikey hız tavanı (3'ten açıldı) |
| `VZ_MAX_TERM` | **10 m/s** | terminalde dikey tavan (5'ten açıldı) |
| `V_TERMINAL` | **16 m/s** | hücum hızı (kullanıcı 20'den geri aldı) |
| `DIKEY_ROLL` | **AÇIK** | dikey kanalda roll telafisi |
| `MAX_ACCEL` | 12 m/s² | komut değişim sınırı (26 denendi, kötüydü) |
| `SONUM_T` | 0 | yatay sönümleme kapalı |
| `DONUS_A` | 0 | dönüş-farkında hız tavanı kapalı |
| `CY_NISAN` | 301 px | nişan noktası |

### Araç (`sim/ardupilot_params/avci_copter.parm`)

| parametre | değer | not |
|---|---|---|
| `ANGLE_MAX` | **7000** (70°) | zarf büyütmesi |
| `WPNAV_ACCEL` | **2600** (26 m/s²) | zarf büyütmesi |
| `PSC_JERK_XY` | **40** | bağlayıcı kısıt |
| `WPNAV_SPEED` | 2500 (25 m/s) | |
| `WPNAV_SPEED_UP/DN` | **1200 / 1000** | dikey güç |
| `WPNAV_ACCEL_Z` | **800** | dikey güç |
| `ATC_RAT_RLL/PIT_P,I` | 0.054 | taban (tarama en iyisi) |
| `ATC_RAT_RLL/PIT_D` | 0.00144 | taban |
| `ATC_ACCEL_R/P_MAX` | 250000 | |
| `MOT_THST_HOVER` | 0.39 | ⚠ gerçek hover 0.14 ama **düzeltmek kötüleştiriyor** (Faz B) |

### Model (`sim/gazebo_harmonic/models/iris_cam/model.sdf`)

Rotor `LiftDrag <area>` = **0.005 m²** (0.002'den ×2.5) → itki/ağırlık
2.56 → **7.08**.

---

## 2 · ÇALIŞAN KISIM — kullanıcının doğruladığı

Kayıttan (`ucus_20260817_205915`, 84 kare) üçüncü yaklaşma:

| kare | mesafe | dz |
|---|---|---|
| k53 | 42.0 m | −3.09 |
| k58 | 29.5 m | −4.27 |
| k62 | 20.2 m | −2.91 |
| k64 | 12.2 m | −1.53 |
| k65 | 8.6 m | **−0.89** |

44 m'den 8.6 m'ye **kesintisiz** kapatıyor ve dikey ofset −3.1'den −0.9'a
düzgün iniyor. Salınım yok. Kullanıcının "paslar okey" dediği bu.

---

## 3 · ⛔ ÇÖZÜLMEYEN — bitiriş

### 3.1 · Kayıttan: son 2 saniyede kopuş

| kare | mesafe | dz | |
|---|---|---|---|
| k79 | 9.4 m | **−0.09** | hizalama neredeyse mükemmel |
| k80 | 7.1 m | +0.57 | |
| **k81** | **5.3 m** | **+2.24** | 1 saniyede 1.7 m yükseldi |
| k82 | 5.0 m | +1.91 | |

Üç yaklaşmanın üçünde de aynı: **son ~2 saniyede dz aniden +2 m'ye fırlıyor.**

### 3.2 · Videodan: hedef kadrajdan çıkıyor

| kare | mesafe | görüntü |
|---|---|---|
| f0065 | 8.6 m | hedef kadrajda, merkeze yakın, ufuk düz — **temiz** |
| **f0067** | **4.0 m** | **hedef TAMAMEN YOK** — kadraj boş |
| f0069 | 5.8 m | hedef yok |

**Son 4 metrede araç kör uçuyor.** Kamera 25° yukarı baktığı için, araç
hedefin üstüne çıkınca hedef kadrajın altından kayıp gidiyor.

### 3.3 · 20 Hz logdan: terminal fazı 12 saniye sürüyor ve bitmiyor

Bir terminal bacağı (`bbox_ibvs_20260817_210240.csv`):

```
t=3.4  TERMINAL  menzil 5.4 m   cy=348  vz_cmd +1.13
t=6.0  TERMINAL  menzil 7.2 m   cy=235  vz_cmd −1.41
t=9.0  TERMINAL  menzil 5.2 m   cy=240  vz_cmd −0.42
t=12.0 TERMINAL  menzil 2.9 m   cy=228  vz_cmd −0.82
t=15.3 TERMINAL  menzil 1.4 m   cy=183  vz_cmd −0.76
```

**12 saniye terminalde, menzil 1.4 m'ye kadar iniyor, hiç çarpmıyor.**
`v_los` sabit 16.0 m/s, hedef 15.1 m/s → kalan kapanma 0.9 m/s.
`cy` 228-240'ta takılı (nişan 301) → hedef sürekli nişanın **üstünde**,
araç sürekli tırmanma komutu veriyor ama açığı kapatamıyor.

### 3.4 · Bulunan kod tutarsızlığı — henüz düzeltilmedi

Roll telafisi **seyir fazında var, terminal fazında YOK**:

```python
# seyir:     eps_elev = eps_elev_ham − (el_roll − el_norm)   ← telafili
# terminal:  elev_atalet = piksel_elev(cy) + iris_pitch      ← TELAFİSİZ
```

4234 terminal karesinde ölçüldü: telafili/telafisiz fark **medyan 0.64°,
p90 8.15°, maks 42.2°**. Medyan küçük ama kuyruk büyük ve terminal yatışı
p90 42.4°. Bu tek başına sorunun tamamı değil ama gerçek bir tutarsızlık.

---

## 4 · BU HÂLE NASIL DÖNÜLÜR

```bash
git checkout en-iyi-hal-20260817
bash ~/.avci_sim/mkur.sh test
```

Panelde beş düğme var; hepsi **bu dosyadaki değerlerde AÇIK/KAPALI**
konumunda başlar. ④ TERMİNALE GEÇİŞ ANI artık **varsayılan olarak 18 px**
(kullanıcının test ettiği hâl) — düğme kapatılırsa 25 px'e döner.

---

## 5 · SIRADAKİ SORU

Kullanıcının cümlesi problemi tam tarif ediyor: **"tüm paslar okey, bitiriş
çok kötü."** Ölçüm bunu doğruluyor — 44 m'den 8 m'ye kusursuz, son 4 metrede
kopuş. Çözülmesi gereken üç şey sıralı:

1. **Terminal dikey yasasında roll telafisi eksik** (§3.4) — tutarsızlık,
   düzeltilmeli.
2. **Kapanma hızı 0.9 m/s** — terminal 12 saniye sürüyor, hedefin kaçmasına
   bol vakit. `V_TERMINAL` 20 bunu 4.9 m/s yapıyordu (ölçülmüştü) ama
   kullanıcı dengeli yaklaşma için 16'yı tercih etti.
3. **Son 4 metrede hedef kadrajdan çıkıyor** — kamera 25° yukarı tilt'li;
   araç hedefin üstüne çıktığı anda görüş kayboluyor. Bu bir **kamera
   geometrisi** sorunu olabilir, güdüm değil.
