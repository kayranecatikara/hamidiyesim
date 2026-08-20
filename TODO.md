# TODO — Avcı Sim

Tiklenebilir görev listesi. Çalıştırma ve sorun giderme için
`docs/SIMULASYON_CALISTIRMA.md`.

Güncelleme: 2026-08-02

> ## ⚑ ÖNCE BUNU OKU — deponun şu anki hali
>
> Depo **`b55953d` (son push) haline geri döndürüldü** (2026-08-02 10:55).
> Push'tan sonra yapılan 8 grup değişiklik hep birlikte uçurulmuş, hangisinin
> ne yaptığı ayırt edilememişti; bazıları işe yaradı, biri zarar verdi.
>
> **Sıradaki iş `UYGULANACAK.md`'de** — 14 madde, teker teker uygulanıp
> uçuşta ölçülecek. Yeni bir oturuma başlıyorsan ORADAN devam et.
>
> Bu dosyadaki maddeler `UYGULANACAK.md` bittikten SONRA sıraya girer.

---

## Sözlük (bu belgelerde geçen terimler)

| terim | anlamı |
|---|---|
| **kara kutu** | ArduPilot'un kendi uçuş kaydı (`~/ardupilot/logs/*.BIN`). Aracın gördüğü attitude, motor çıkışları, kontrolcü hedefleri. Bizim CSV'lerimizden bağımsız — "araç komutu uyguladı mı" sorusunun tek dürüst kaynağı. |
| **istasyon** | GPS fazının drone'a "şurada dur" dediği hayali nokta. Sabit metre DEĞİL sabit açı: hedeften `RANGE_SET` (11 m) uzakta, LOS yükselişi `ISTASYON_ELEV_DEG` (15°) → hedefin **10.63 m gerisi + 2.85 m altı**. Drone hedefi değil bu noktayı takip eder. |
| **`ok` oranı** | `visual_lead` her kamera karesine `durum` etiketi yazar. `ok` = pose modeli hedefi temiz gördü, keypoint'ler güvenilir. Diğerleri: `kpt_dusuk`, `tespit_yok`, `kor_dalis`. "%51 ok" = karelerin yarısında güdüm sağlam veriyle çalışıyor. |
| **faz** | GPS fazı (uzaktan yaklaşma, `gps_guidance`) ↔ görsel faz (terminal hücum, `visual_lead`). Geçişi `supervisor` yönetir. |
| **geçiş sayısı** | GPS→görsel kaç kez geçildi. 1 ideal; yüksek sayı görsel temasın kopup kopup kurulduğunu gösterir. |

---

## Sıradaki

- [ ] **AÇIK SORU: istasyon açısı kamera tilt'inden ayrılmalı mıydı?**
      2026-08-02'de `ISTASYON_ELEV_DEG` `CENTER_ELEV_DEG`'ten ayrıldı ve
      25° → 15° indirildi (terminal dikey ivme bütçesi). Ölçümler olumlu
      ama **kanıt karışık**: algının iyileşmesi geometrinin sonucu olabilir,
      merkez dışı kadrajlamanın kendi bedeli izole ölçülmedi. Ayrıca asıl
      alternatif (`WP_ACC_Z` 1.0 → 2.5 yükseltip istasyonu 25°'de bırakmak)
      hiç denenmedi. **Karar prosedürü, ölçütler ve rakamlar:
      `UYGULANACAK.md` → B7.** B6'dan (terminal algı) ÖNCE karara bağlanmalı.
      (`control/guidance/gps_guidance.py` → `ISTASYON_ELEV_DEG`)
- [ ] **`LOOKUP_MIN_ALT` kararı** — şu an 8 m sabit taban. Hedef yere düşüp
      sürünürken avcı 8 m'de asılı kalıyor, inemiyor. Hedefin irtifasına göre
      uyarlanmalı mı, yoksa "hedef yerdeyse görev bitti" mi sayılmalı?
      (`control/guidance/gps_guidance.py:50`)
- [ ] **`ATC_ANGLE_MAX`'i kademeli geri artır** — 45'te kaldı. 50-55 denenebilir;
      55'te yatay ivme 14 m/s². Her denemede kara kutudan motor doygunluğu ve
      toplam yaw dönüşü kontrol edilmeli. (`sim/ardupilot_params/avci_copter.parm`)
- [ ] **Video kayıt butonları** — başlat / durdur / kayıt dosyası.
      Iris kamera akışı zaten MJPEG olarak `latest_frames["iris"]` üzerinden
      servis ediliyor (`gcs_server.process_iris_frame`). Kayıt, o kareleri
      `cv2.VideoWriter` ile dosyaya yazan bir thread olur; dosyalar `logs/`
      altına zaman damgalı düşer ve `/loglar/` üzerinden zaten servis ediliyor.

## Tekrar denenecekler

Bunlar daha önce denenip GERİ ALINDI — ama o ölçümler sistemin bugünkünden
çok daha kötü olduğu dönemde alındı. O zamanki başarısızlıkların sebebi bu
fikirlerin kendisi olmayabilir; altlarındaki bozuk zemin olabilir. Aradan
düzelenler: yaw sürekli dönmesi (27 °/s → ~0), firmware parametrelerinin hiç
uygulanmaması (araç 30° eğim tavanıyla uçuyordu), dikey ıska geometrisi, ve
temiz tespit oranı (%12 → %65).

Her denemede **tek değişken** değiştirin ve eski sayıyla karşılaştırın.

- [ ] **Gerçek PN (`γ += N·Δλ`)** — klasik oransal seyrüsefer.
      *Eski sonuç:* testleri geçti (γ, λ'dan tam 2.00 kat hızlı) ama kapalı
      çevrimde ıska **0.66 m → 1.5-2.1 m'ye çıktı**. Sebep: drone hedefe
      yakınsarken LOS açısı zaten doğal olarak azalıyor; PN bunu "sıfırlanacak
      LOS hızı" sanıp yakınsamayla savaşıyor, hedef yukarıdayken γ eksiye
      (dalışa) gidiyordu — γ −22°'ye inerken λ hâlâ +1.3°'deydi.
      *Neden şimdi farklı olabilir:* o ölçümler dikey ıska geometrisi
      düzeltilmeden önce alındı; PN'in "büyük başlangıç ofsetini kapatma"
      yükü artık yok, sadece küçük sapmaları düzeltmesi gerekiyor — zaten
      tasarlandığı iş. Gerekçe `adapter_copter._dikey_pn` docstring'inde.
      *Ölçüt:* ıska 0.55 m'nin (mevcut en iyi) altına inmeli.

- [ ] **Dikey PN'i güçlendirme** (tavan 15°→30°, süre 0.4→0.6 s)
      *Eski sonuç:* PN yeni tavana da %79 oranında çakıldı — doygunluk noktası
      yukarı kaydı ama kalkmadı. `PN_LEAD_SURE`/`PN_DIKEY_MAX_DEG` şu an
      0.6/30 değerlerinde ama etkisi sınırlı.
      *Neden şimdi farklı olabilir:* doygunluğun sebebi büyük olasılıkla
      aracın ivme tavanının 5.7 m/s²'de kilitli olmasıydı (ATC_ANGLE_MAX=30°
      varsayılanı). Artık 45° → 9.8 m/s².
      *Ölçüt:* PN'in tavana çakılma oranı %79'un belirgin altına inmeli.

- [ ] **`KP_KADRAJ ≥ 1.0`** — kadraj tutma kazancını yükseltme.
      *Eski sonuç:* yüksek kazanç yakınsamayla savaşıp salınım üretti;
      1.0'da 20/24, **1.5'te 0/24 vuruş**. Tarama: 0.0 → ıska 0.59 m,
      0.5 → 0.55 m (seçilen), 1.0 → 1.59 m, 1.5 → 4.90 m.
      *Neden şimdi farklı olabilir:* o taramadaki salınımın bir kısmı yaw
      kaçağından ve tespit kopukluğundan geliyor olabilir; ikisi de düzeldi.
      *Ölçüt:* 24 denemede vuruş oranı ve ıska, yukarıdaki tabloyla kıyaslanır.

- [ ] **Yaw'ı mutlak hedefe slew etme** — kalıcı `cmd_yaw` durumu tutup
      GPS fazındaki desene benzetme (2026-08-01'de denendi).
      *Eski sonuç:* kapalı çevrim ölçümünde arıza koşulunda mevcut biçim
      1.0 tur dönerken bu biçim **7.4 tur** döndü. Mevcut biçim komutu her
      karede aracın gerçek başlığına yeniden demirliyor — bu bir güvenlik
      özelliği.
      *Neden şimdi farklı olabilir:* artık yaw kaçak kapısı var; kaçağı o
      sınırlıyorsa mutlak biçimin kararlılık avantajı kullanılabilir.
      *Ölçüt:* T44/T45 testleri geçmeli, kaçak 45°'nin altında kalmalı.


## Güdüm — açık işler

- [ ] **Görsel faza geçiş kapısı** — şu an "son 15 karenin 10'unda pose
      `conf≥0.5`" **VE** yatay mesafe < 20 m. Bağlayıcı olan pose kilidi; bbox
      20 m'de görünüyor ama pose geç kilitleniyor. Girişi tespit (bbox)
      güvenine bağlamak denenebilir — lead zaten keypoint zayıfken kendiliğinden
      sıfırlanıyor (`guidance_core.py:409`). (`supervisor.py:87-93`)
- [ ] **Lead'in yumuşak geçişi** — `kpt_dusuk`'ta sert 0'lanıyor, ~15° nişan
      zıplaması (58 geçiş, ort 10.8°, max 24.9°). (`guidance_core.process`)
- [ ] **Menzil verisi neden zıplıyor** — kapı semptomu kesti, kök neden duruyor.
      Baş şüpheli `gcs_server._frame_off` dikey kalibrasyonu (`sd = 0.0`
      varsayımı). Artık ölçülebilir: `menzil_ham_m` ile `gercek_menzil_ham_m`
      yan yana loglanıyor.
- [ ] **GPS fazında vuruş tespiti yok** — `visual_lead` dışında kimse vuruşu
      raporlamıyor. Hasar modülü artık bağımsız izliyor ama faz durumu hâlâ
      "VURULDU" demiyor. (`gps_guidance.py`)

## Simülasyon altyapısı

- [ ] **⚠ YAŞANAN PROBLEM (2026-08-20): daire çapları arasında geçince hedef
      İHA tırmanıp tırmanıp STALL ediyor ve yere çakılıyor.**
      *Kök neden:* FBWA'da elevatör irtifayı değil **pitch açısını** komut eder.
      `_daire` sabit `pitch = int(150 / cos(yatış))` = 154-171 (≈ +4° burun
      yukarı) gönderiyor ve bunu geri çeken bir geri besleme YOK — açık çevrim.
      Uçak ~1 m/s (≈65 m/dk) tırmanıyor; gaz sabit olduğu için hız payı eriyor,
      sonunda stall edip spiral dalışla düşüyor.
      *Ölçüldü (2 uçuş, `logs/gps_guidance_20260820_18*.csv` ve `..._19*.csv`):*
      1. uçuş 124 m → **508 m**, 352 s, **+1.09 m/s**, 2646 örnekte tek bir
      2 m'lik alçalma bile yok; sonra 52 s'de 508 → 87 m, düşey hız 28 m/s,
      yatay hız 0.3 m/s (= dik düşüş). 2. uçuş 23 m → 84 m → yer, uçuş yolu
      açısı −70°. Çakılan enkaz sonra Gazebo zemininin kenarından taşıp
      sonsuza düşüyor (`tgt_z` −5391 m'ye kadar) ve avcı 5 km yeraltındaki
      hedefe KİLİT kalıyor.
      *Neden çap değiştirince çıkıyor:* uçak havadayken yeni senaryo kalkışı
      atlıyor (`AIRBORNE_ALT_M`), yani tırmanış sıfırlanmıyor **birikiyor**.
      Ayrıca `gcs_server._stop_scenario_proc` senaryoyu `SIGKILL` ile öldürdüğü
      için (yakalanamaz) "kanatları düzleştir, nötr bırak" satırı hiç çalışmıyor
      — uçak 2-4 s sahipsiz, son tam yatış komutu kilitli kalıyor.
      *Düzeltme ZATEN VAR ama bu dalda değil:* `d542309` (main, 2026-08-09,
      `_irtifa_pitch` — PD'li kapalı çevrim irtifa tutucu) ve `08f8619`
      (`origin/hit_irtifa_tutucu`, 2026-08-15, aynı kazançlar + panel düğmesi,
      **12 uçuşla ölçülmüş**). `kubra-kayra`, `93ea734`'ten (kayramin hattı)
      ayrıldığı için ikisini de içermiyor; `main` sonradan birleştirdiği için
      onda var. Aynı kusur `3873344`'te (2026-08-08) dört A/B'yi birden
      geçersiz kılmıştı.
      *Yapılacak:* `_irtifa_kilitle` + `_irtifa_pitch` + 4 sabit
      (`IRTIFA_KP/KD/PITCH_MAX/OTURMA_S`) main'den elle taşınacak; çağrı
      yerleri `scenario_duz`, `_daire_sureli`, `_daire` (hold periyodu
      0.5 → 0.2 s). `elips_gorev` kendi `irtifa_hedef`'ini kullanıyor,
      DOKUNULMAZ; `square`/`aggressive` de dokunulmaz. Cherry-pick tutmaz
      (dosyalar 78+/228− ayrışmış). Ayrıca `_stop_scenario_proc` önce `SIGTERM`
      + 0.5 s beklemeli, ancak ölmezse `SIGKILL` — bu kısım hiçbir dalda yok.
      (`control/run_plane_scenario.py:342`, `control/gcs_server.py:196`)

- [ ] **Hasar modülünü arayüze bağla** — `/api/hasar` endpoint'i var, panelde
      gösterilmiyor.
- [ ] **RTF'i tam sistemde tekrar ölç** — `gcs_server` + YOLO yükü altında
      0.982 ölçüldü ama uçuş sırasında (görsel faz aktifken) ölçülmedi.

## Tamamlananlar

- [x] **GCS telemetrisi donuyor** — `mavlink_listener`'a `else` dalı; 14550'den
      gelen quadrotor paketleri `telemetry_state["iris"]`'e yazılıyor. Uçuşta
      doğrulandı.
- [x] **Parametre adları yanlıştı** — 9 parametrenin 7'si SITL'e hiç
      uygulanmıyordu (firmware SI birimine geçip yeniden adlandırmış).
      `tools/parm_denetle.py` tekrarını önlüyor.
- [x] **Kendi etrafında dönme** — kök neden: `yaw_hata` kapanmıyorken komut her
      karede bir tavan adımı daha ekliyordu (90 °/s sürekli dönme).
      `adapter_copter`'a "hata kapanmıyorsa yaw'ı sustur" kapısı eklendi
      (T44/T45). Ölçüm: seyir boyunca dönme 27 °/s → ~0 °/s, yaw takip hatası
      30.5° → 4.5°.
- [x] **`WP_YAW_BEHAVIOR` 2 → 0** — firmware, yaw komutu olmayan anlarda burnu
      gidiş yönüne çeviriyordu.
- [x] **Dikey ıska, 1. tur (GPS istasyon geometrisi)** — sabit 4.65 m ofset
      yerine `r_eff = min(menzil, RANGE_SET)`; LOS yükselişi her menzilde sabit
      (test G10, sapma 0.00°). ⚠ Bu YETMEDİ: açı sabitlendi ama istasyonun
      KENDİSİ hâlâ 25°'deydi, yani terminal yine 4.65 m tırmanmak zorundaydı.
- [x] **Dikey ıska, 2. tur (istasyon açısı ↓)** — `ISTASYON_ELEV_DEG` kamera
      tilt'inden ayrıldı, 25° → 15°: kapatılacak dikey **4.65 → 2.85 m**.
      Sebep ölçüldü: ArduPilot dikey rampası `WP_ACC_Z = 1.0 m/s²`, 4.65 m için
      3.05 s gerekiyor, terminalde 2.4-2.8 s var. Test G11 bütçeyi koruyor.
- [x] **Sahte vuruş** — `VURUS_MENZIL` 3.0 → 1.5 m. 3 m fiziksel temas değil,
      yakın geçişti.
- [x] **Sahte PnP paneli** — ground-truth'a yapay gürültü ekleyip "tahmin" diye
      gösteriyordu. Yerine gerçek görüş kestirimi + ground-truth + faz kapıları.
- [x] **Hedef telemetrisi = cevap anahtarı** — güdüme bağlanmadı, ölçüm için 10
      sütun eklendi (§8).
- [x] **Talon manuel modda kalkmıyor** — mod değiştirmek tek başına yetmiyordu;
      ARM + TAKEOFF adımları hiç yoktu. Eklendi (yerde FBWA, 15 m üstünde FBWB).
      **Uçuşta doğrulandı.**
- [ ] **Çarpışmada hedef uçmaya devam ediyor** — ⚠ TAMAMLANMADI, revert edildi.
      `gcs_server`'da hasar modülü KODU duruyor ama **varsayılan KAPALI**
      (`AVCI_HASAR=1` gerek) ve dayandığı Gazebo temas sensörü SDF'den geri
      alındı — yani şu an tetiklenemez. Tam sürümü `UYGULANACAK.md` **A5**.
