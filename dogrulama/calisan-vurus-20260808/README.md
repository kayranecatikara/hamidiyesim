# ÇALIŞAN VURUŞ — DONMUŞ DURUM (2026-08-08)

Bu dizin, **ilk gerçek fiziksel temasın (Gazebo contact)** gerçekleştiği çalışan
durumun kanıt paketidir. Kod durumu `ded9db0` commit'inde (bu snapshot commit'inin
ebeveyni); etiket: `calisan-vurus-20260808`. Buraya her an geri dönülebilir.

> Bu paket SALT KANIT/ARŞİVDİR. Kod/config/güdüm burada DEĞİŞTİRİLMEZ; tek kaynak
> depodur. Loglar `loglar/` altında (runtime `logs/` gitignore'da olduğu için kopya).

## Vuruş özeti
- Dosya: `loglar/visual_lead_20260808_142312.csv`, **t=133.32**, **menzil 0.52 m**, `gorev_state=STRIKE`
- Fiziksel temas (Gazebo contact); yakınlık yedeği DEĞİL (yedek <1.5 m olsaydı 1.28 m'de tetiklenirdi, tetiklenmedi)
- Mekanizma: tespit 0.56 m'ye kadar kesintisiz (`durum=ok`), STRIKE dalışı (vx≈17, vz≈7.4) düz kapadı, menzil monoton 1.28→0.52, fly-past yok
- Hakem logu: `loglar/kilit_20260808_142028.csv` (FSM state + kumul/kesint her karede)

## Yapılandırma damgası (vuruş koşusu)
```
ALGI=bbox,GT=off,KILIT_BYPASS=off,TRACKER=off,LOCK=off,V_KAPANMA=25.0,
V_YAKLASMA=20.0,PN_YATAY=0.6,GPS_RANGE=6.5,IVME=4.0/10.0,FLYPAST=8.0/1.5
```

## Efektif config değerleri (AVCI_GPS_RANGE=6.5)
```
SARTNAME (BLOK A, frozen):
  PENCERE_SN=10.0  KUMULATIF_KILIT_SN=5.0  KESINTISIZ_SN=3.0
  AH_HEDEF_KAPSAMA_MIN=0.9  AH_EKRAN_ORAN_MIN=0.05  FRAME_TOLERANS_ORAN=0.05
AYAR (BLOK B):
  KILIT_KAYIP_SN=2.0  V_MAX_ENGAGE=5.0  AH_ORAN_GIRIS=0.06  AH_ORAN_CIKIS=0.052
  V_KAPANMA=25.0  V_KAPANMA_MAX_ENGAGE=4.0  V_MUTLAK_MAX=20.0  ORNEK_TAVAN_SN=0.2
guidance_core.Cfg (terminal):
  TERMINAL_MENZIL=8.0  TERMINAL_SURE=0.6  VURUS_MENZIL=1.5  V_KAPANMA=25.0  V_YAKLASMA=20.0
```

## Şartname kapı doğrulaması (loglardan, salt-okuma)

**3a — TRACK_LOCK→ENGAGE, kümülatif ≥ 5.0 s ✓**
`kilit_20260808_142028.csv`, t=130.52: `TRACK_LOCK→ENGAGE  kumul=5.016  kesint=1.320  kilit=1`

**3b — ENGAGE→STRIKE, kesintisiz ≥ 3.0 s ✓**
`kilit_20260808_142028.csv`, t=132.20: `ENGAGE→STRIKE  kumul=6.568  kesint=3.002  kilit=1`

**3c — kesintisiz 3 s dilimi köprüsüz birikti ✓**
Segment [129.20, 132.20]: 82 kare, kilit=1 → 81/81 (segment içi), köprülenen kare **0**,
kesintisiz düşüş/sıfırlanma **0** (0.00→3.04 monoton). %5 frame toleransı DEVREYE GİRMEDİ —
son 3 s literal boşluksuz. (6.1.3 katı yorumu için bile temiz.)

**3d — son 3 s aktif takip + komut hedefe (6.1.3) ✓**
[130.32, 133.32]: kilit CSV bbox/tespit var 79/79 (%100), anlık_kilit 79/79.
visual_lead: durum=ok 73/74, kapanma>0 (yaklaşıyor) 72/72, |yaw_hata| medyan 0.6°,
menzil 6.06→0.52 m (sistematik azalıyor).

**3e — tek kaynak (UI = kilit CSV = FSM) ✓**
kumul/kesint üçünde de aynı `_kdurum`'dan gelir (`gcs_server.py`: FSM besleme :1574-1575,
UI köprüsü :1539, kilit CSV :1416-1417). FSM'in ayrı KilitSure'u yok. Not: `[FSM]` stdout
satırları bu koşuda dosyaya yakalanmadı (stdout tee'lenmedi); ama yapı gereği kilit
CSV'deki değerlerle birebir aynıdır (aynı kaynak).

## Tekrarlanabilirlik notu
Bu oturumda 4 STRIKE'tan yalnız 1'i temasla bitti; config, sıfır-vuruşlu 2026-08-07
oturumuyla birebir aynı. Belirleyici: terminal tespit-tutma derinliği (başarısızlar
2.7–3.0 m'de tespiti kaybedip komutu sıfırladı). Ayrıntı: hafıza `ilk-temas-vurus`.
```
STRIKE   tespit(ok) en derin   sonuç
 #1        2.96 m               durdu (tespit koptu)
 #2        2.81 m               durdu
 #3        2.74 m               durdu
 #4        0.56 m               TEMAS
```

## Geri dönüş
```
git checkout calisan-vurus-20260808      # bu donmuş duruma dön
```
