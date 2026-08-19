# Mission Planner ile Waypoint Görevi

Bu belge, çalışan SITL'e **Mission Planner** bağlayıp **hedef drone**'a
(ArduPlane, sysid 2) waypoint görevi yükleyip **AUTO** modda uçurmayı anlatır.

> Bu altyapı yalnız YENİ dosya ekler; mevcut IBVS/görsel güdüm koduna
> dokunulmadı. Interceptor (iris, sysid 5) ve GCS bağlantıları değişmedi.

---

## 1. Mimarî — portlar ve araçlar

Sim başlatıcı (`scripts/basla.sh` → `scripts/start_harmonic.sh`) iki SITL
kaldırır:

| araç | rol | sysid | MAVLink `--out` portları |
|---|---|---|---|
| ArduCopter iris | interceptor | 5 | `14541`, `14550` (+`14551`) |
| ArduPlane | **hedef drone** | 2 | `14542`, `14550` (+`14551`), **`14555`** |

- `14550` = GCS/IBVS ana yayını — **iki araç birden** buraya yayın yapar.
- **`14555` = Mission Planner için EKLENEN ayrı çıkış** — yalnız hedef
  drone (sysid 2). Mission Planner buraya bağlanınca karışıklık olmaz.
- MAVROS kullanılmıyor; GCS custom pymavlink (`control.gcs_server`).

`14555` çıkışı `scripts/start_harmonic.sh` (ArduPlane satırı) içine eklendi
(yorumla işaretli).

---

## 2. Simülasyonu başlat

```bash
cd ~/projects/hamidiyesim
bash scripts/basla.sh          # headless (Gazebo + 2 SITL + GCS), ~90 s
# durdur:  bash scripts/start_harmonic.sh stop
```

`14555` çıkışının aktif olması için sim'in **güncel scriptlerle yeniden
başlatılmış** olması gerekir (eski oturum bu portu bilmez).

---

## 3. Mission Planner'ı bağla

Mission Planner aynı Linux makinede (Mono ile) veya Windows'ta çalışabilir;
burada **aynı makine (127.0.0.1)** varsayılıyor.

1. Mission Planner'ı aç.
2. Sağ üst bağlantı tipi: **UDP** seç, **Connect**.
3. Sorulan portu **14555** gir. (MP dinleyici olur; SITL bu porta yayın yapar.)
4. Bağlanınca hedef drone'un HUD'u, konumu ve telemetrisi görünür.
   - Ayrı makinedeyse: `scripts/*.sh` içindeki `--out udp:127.0.0.1:14555`'i
     `--out udp:<MP_MAKINE_IP>:14555` yap ve o portu MP'de aç.

---

## 4. Görevi yükle

**`missions/kare_300m.waypoints`** (birincil) — home'dan başlayan
**300 m kenarlı kare, 80 m** relative irtifa (home → TAKEOFF → 4 köşe → RTL).
Koordinatlar SITL'in **canlı okunan** home'undan (sysid 2) hesaplandı.

> ⚠ **Neden 300 m?** sysid 2 bir **ArduPlane** (sabit kanat). SITL
> varsayılanlarında ~22 m/s hız + 45° yatışta dönüş yarıçapı ~50 m
> (dönüş çapı ~100 m). 40 m kenarlı kare bu çaptan küçük — uçak köşeleri
> dönemeden sarmala girer. 300 m kenar, dönüş çapının rahat üstünde.
> Eski `missions/kare_40m.waypoints` referans olarak DURUYOR (kullanma).

**Yol A — CLI (bu repo):**
```bash
# ⚠ Mission Planner KAPALIYKEN (14555'i o tutuyorsa çakışır):
python3 scripts/upload_mission.py missions/kare_300m.waypoints \
    --connect udpin:0.0.0.0:14555 --sysid 2
# Alternatif port (start_harmonic'te, iki araç paylaşımlı):
#   --connect udpin:0.0.0.0:14551 --sysid 2
```
Script kalemleri yükler ve `mission_count`'u geri okuyup doğrular
(`✓ BAŞARILI` yazmalı).

**Yol B — Mission Planner:** Plan sekmesi → **Load WP File** →
`missions/kare_300m.waypoints` → **Write WPs**.

### WP parametreleri (300 m kare için)

Canlı okunan değerler: **`WP_RADIUS = 90 m`**, **`WP_LOITER_RAD = 80 m`**.

- `WP_RADIUS = 90 m` — waypoint'e "ulaşıldı" kabul yarıçapı. Dönüş yarıçapı
  (~50 m) ile 300 m kenar arasında olduğu için **uygundur** (orbit-döngüsü
  riski yok; köşeler ~90 m yuvarlanır). Daha keskin köşe istersen ~60 m'ye
  çekilebilir ama **dönüş yarıçapının (~50 m) altına inme** — uçak köşeye
  giremeyip çember atar. *Öneri: 90 m'de bırak; keskinlik istenirse 60 m.*
- `WP_LOITER_RAD = 80 m` — yalnız LOITER kalemlerinde etkili. Bu görevde
  LOITER **yok** → **etkisiz**, dokunmaya gerek yok.

> ⚠ Bu değerler DEĞİŞTİRİLMEDİ (öneri niteliğinde). İstersen sen ayarla.

---

## 5. AUTO modda uçur

Mission Planner'da:
1. **Actions** sekmesi → **Arm/Disarm** ile ARM et (gerekiyorsa önce mode
   GUIDED/MANUAL).
2. Mod kutusundan **AUTO** seç → **Set Mode**.
3. ArduPlane AUTO'da görev 1. kalemden (TAKEOFF) başlar; kareyi uçup RTL yapar.

> Not (CLAUDE.md §0.1): uçuşları **kullanıcı** koşar. Bu belge komutları verir;
> ARM/AUTO adımını sen uygularsın.

---

## 6. Home değişirse görevi yeniden üret

Home sabit değilse (farklı world/spawn), `.waypoints` koordinatları eskir.
Yeniden üretmek için home'u canlı oku ve kareyi yeniden hesapla — koordinat
elle yazılmaz. Referans home (bu kurulum): `lat -35.3632621, lon 149.1653699`
(ArduPilot SITL varsayılan CMAC).

---

## 7. Sorun giderme

| belirti | sebep / çözüm |
|---|---|
| MP bağlanmıyor | Sim güncel scriptle yeniden başlatılmadı → `14555` yok. `bash scripts/basla.sh`. |
| MP iki araç gösteriyor | `14550`'ye bağlanmışsın. `14555`'e bağlan (yalnız sysid 2). |
| upload script takılıyor | MP 14555'i tutuyor → MP'yi kapat ya da `14551` kullan. |
| `mission_count` uyuşmuyor | Dosya bozuk (12 alan / tab) — QGC WPL 110 formatını doğrula. |
| AUTO'da kalkmıyor | ARM edilmemiş ya da EKF/GPS kilidi yok; HUD'da "GPS: 3D Fix" bekle. |
