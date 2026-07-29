# Gövde Seçim Kararı

**Seçilen: `cand_iris`** (avci_sim iris gövdesi) → `models/interceptors/avci_net_interceptor/`

Ölçüm verisi: [`KIYAS_RAPORU.md`](KIYAS_RAPORU.md)

---

## Neden

| Kriter | `cand_iris` | En yakın rakip `cand_mrs_t650` |
|---|---|---|
| Harmonic'te temiz yükleniyor | ✅ 0 hata | ❌ 4 hata (motor eklentisi yok) |
| ArduPilot bağlı | ✅ çalışıyor | ❌ PX4 motor modeli, ArduPilot'a taşınması gerek |
| Kamera | ✅ 640×480, 125° FOV | ❌ yok (jinja'da kapalı) |
| Kütle | 1.75 kg | 3.56 kg |
| Burun +X açıklığı | 0.23 m | 0.325 m |
| Dönüştürme riski | **sıfır** | CTU'nun `MrsGazeboCommonResources_MulticopterMotorModel` C++ eklentisi ROS2 workspace'te derlenmeli, sonra ArduPilotPlugin'e çevrilmeli |

Karar tek bir gerçeğe dayanıyor: **Harmonic + ArduPilot ile bugün uçan tek aday
`cand_iris`.** MRS gövdeleri geometrik olarak daha uygun (t650/m690 daha büyük
burun açıklığı, daha yüksek taşıma payı) ama beşi de motor eklentisi olmadan
yüklendi — yani hiçbiri **uçmuyor**. Onları uçurmak = CTU'nun C++ eklentisini
derlemek + ArduPilotPlugin'e port etmek; bu, ağ mekanizmasının kendisinden daha
büyük bir iş.

`cand_iq_camera` (iq_sim) dönüşümden sonra temiz yüklendi ve kütlesi/geometrisi
`cand_iris` ile neredeyse aynı (ikisi de iris türevi) — ek bir şey getirmiyor.
Kıyas değeri: Classic→Harmonic dönüşüm hattımızın çalıştığını kanıtladı.

## Değerlendirilemeyenler (dürüst kayıt)

| Aday | Sebep |
|---|---|
| `cand_d2d_x500` | PX4'ün `x500` gz modeline bağlı; ne d2dtracker deposunda ne de klonladığımız hiçbir repoda var. Değerlendirmek için PX4-Autopilot klonlamak gerekirdi. MRS x500 (`cand_mrs_x500`) aynı gövdenin muadili olduğu için ikame edildi. |
| `cand_mrs_*` (5 adet) | Geometri/kütle ölçüldü ama **uçuş doğrulanmadı** — `MrsGazeboCommonResources_MulticopterMotorModel` paylaşımlı kütüphanesi yok. Tablodaki RTF değerleri bu yüzden `cand_iris`/`cand_iq_camera` ile kıyaslanamaz (motorları dönmüyor, fizik yükü hafif). |
| `Strix-Interceptor` | Aday bile olamadı: depoda hiç model/SDF yok (12 dosya, 1 cpp + 2 py + README). DEFCON31 demosu; ağ değil RF spoofing tabanlı, simülasyonu hiç yapılmamış. |
| `uav_simulator` (Zhefan-Xu) | İç mekan navigasyon odaklı, `px4_iris.sdf` iris'in Classic kopyası — `cand_iris`'e göre hiçbir üstünlüğü yok, aday listesine alınmadı. |
| `UAVProjectileCatcher` | Gövde adayı değil; değeri `ME4232_Final_Report.pdf` içindeki yakalama/kesişim matematiği. |

## Taşıma payı bütçesi — dikkat edilecek nokta

Ticari referanslar (`ticari_referanslar/README.md`) Fortem F700 için 2.3 kg
faydalı yük veriyor, ama F700 çok daha büyük bir platform. **1.75 kg'lık iris'e
2.3 kg yük binmez.** Gerçekçi bütçe:

| Kalem | Hedef kütle |
|---|---|
| iris gövdesi (mevcut) | 1.75 kg |
| Taret (pan + tilt + namlu) | ≤ 0.35 kg |
| Ağ (koni aşaması) | ≤ 0.15 kg |
| **Toplam** | **≤ 2.25 kg** |

iris'in ArduPilotPlugin motor ayarı (`multiplier=838`, 4 rotor) standart iris
itkisini veriyor. 2.25 kg'da havada kalıp kalmadığı Aşama 3–4 sonunda hover
testiyle ölçülecek; tutmazsa iki seçenek var:
1. `multiplier`/lift-drag katsayılarını büyütüp daha güçlü motor modellemek
2. `cand_mrs_t650` gövdesine geçmek (3.56 kg, daha büyük burun) — bu durumda
   CTU motor eklentisini ArduPilotPlugin'e portlama işi devreye girer

Bu, kararı geri alınabilir tutuyor: taret ve ağ modelleri ayrı model dosyaları
olarak, gövdeye sadece `<include>` + joint ile bağlanacak.
