# TODO — Avcı Sim

Tiklenebilir görev listesi. Çalıştırma ve sorun giderme için
`docs/SIMULASYON_CALISTIRMA.md`.

Güncelleme: 2026-08-01

---

## Sıradaki

- [ ] **Heading titremesini doğrula** — `ATC_ANG_YAW_P` 4.5 → 3.0 yapıldı,
      uçuşta denenmedi. Ölçüt: kara kutuda `ATT.Yaw − ATT.DesYaw` std'si
      1.4-2.4°'den **<1.0°**'ye inmeli. İnmezse geri al, `ATC_RAT_YAW_FLTE`
      (2.5 Hz) denenir. (`sim/ardupilot_params/avci_copter.parm`)
- [ ] **`LOOKUP_MIN_ALT` kararı** — şu an 8 m sabit taban. Hedef yere düşüp
      sürünürken avcı 8 m'de asılı kalıyor, inemiyor. Hedefin irtifasına göre
      uyarlanmalı mı, yoksa "hedef yerdeyse görev bitti" mi sayılmalı?
      (`control/guidance/gps_guidance.py:50`)
- [ ] **Mesafeye göre hız profilini uçuşta doğrula** — sabit tavan yerine
      `v = sqrt(2·a·kalan) + hedef_hızı` geldi (uzakta 28, 15 m'de ~19,
      5 m'de ~9 m/s). Ölçüt: hedefe yetişiyor mu VE istasyonu geçip savrulmuyor
      mu? (`gps_guidance._hiz_tavani`)
- [ ] **Görsel kilidi uçuşta doğrula** — kısa kopmalarda artık GPS'e dönmüyor
      (`kilit_kor` durumu). Ölçüt: `gecis_sayisi` 1'de kalmalı; CSV'de
      `kilit_kor` kareleri görünmeli. (`AVCI_GORSEL_KILIT_SURE`, varsayılan 10 s)
- [ ] **Gerçek çarpışma tespitini uçuşta doğrula** — Gazebo contact sensörü.
      Ölçüt: ıskaladığında hedef DÜŞMEMELİ, gerçekten çarpınca düşmeli.
      Konsolda `[HASAR] Gerçek çarpışma dinleniyor:` satırı görünmeli.
- [ ] **`ATC_ANGLE_MAX`'i kademeli geri artır** — 45'te kaldı. 50-55 denenebilir;
      55'te yatay ivme 14 m/s². Her denemede kara kutudan motor doygunluğu ve
      toplam yaw dönüşü kontrol edilmeli. (`sim/ardupilot_params/avci_copter.parm`)

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
- [x] **Dikey ıska (GPS istasyon geometrisi)** — sabit 4.65 m ofset yerine
      `r_eff = min(menzil, RANGE_SET)`; LOS yükselişi her menzilde 25° (test
      G10, sapma 0.00°).
- [x] **Sahte vuruş** — `VURUS_MENZIL` 3.0 → 1.5 m. 3 m fiziksel temas değil,
      yakın geçişti.
- [x] **Sahte PnP paneli** — ground-truth'a yapay gürültü ekleyip "tahmin" diye
      gösteriyordu. Yerine gerçek görüş kestirimi + ground-truth + faz kapıları.
- [x] **Hedef telemetrisi = cevap anahtarı** — güdüme bağlanmadı, ölçüm için 10
      sütun eklendi (§8).
- [x] **Talon manuel modda kalkmıyor** — mod değiştirmek tek başına yetmiyordu;
      ARM + TAKEOFF adımları hiç yoktu. Eklendi (yerde FBWA, 15 m üstünde FBWB).
      **Uçuşta doğrulandı.**
- [x] **Çarpışmada hedef uçmaya devam ediyor** — hasar modülü. İlk sürüm
      yakınlık eşiği (<2 m) kullanıyordu ve ıskalayınca da hedefi düşürüyordu;
      GERÇEK Gazebo contact sensörüne çevrildi (yakınlık eşiği kaldırıldı).
