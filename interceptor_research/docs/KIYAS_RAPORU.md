# Aday Interceptor Gövdeleri — Kıyas Raporu

`scripts/12_bench.sh` (Gazebo Harmonic headless, 3000 iterasyon) +
`scripts/13_report.py` (SDF geometri analizi) tarafından üretildi.

**Ölçüm notları:**
- **RTF**: gerçek zaman faktörü. Dünya yüklenmediyse `—`.
  ⚠️ Adaylar arası doğrudan kıyaslanamaz — `cand_iris` ve `cand_iq_camera`
  çalışan ArduPilotPlugin + kamera render'ı taşıyor, MRS adaylarının motor
  eklentisi yüklenemediği için fizik yükü daha hafif.
- **Burun +X**: gövdenin +X (ileri) yönündeki en uzak collision noktası.
  Taret bunun ötesine monte edilecek. Mesh collision'lar ölçülemez → alt sınır.
- **Kütle**: `include` edilen alt modeller dahil, tüm linklerin `<inertial><mass>` toplamı.

---

| Aday | Durum | RTF | Kütle (kg) | Sınır kutusu XYZ (m) | Burun +X (m) | ArduPilot | Kamera | Hata |
|---|---|---|---|---|---|---|---|---|
| `cand_d2d_x500` | YUKLENMEDI | — | 0.062 | 0.0×0.0×0.0 | 0.0 | ❌ | ❌ | 7 |
| `cand_iq_camera` | YUKLENDI | 0.402 | 1.91 | 0.46×0.64×0.195 | 0.23 | ✅ | ✅ | 0 |
| `cand_iris` | YUKLENDI | 0.363 | 1.75 | 0.46×0.64×0.221 | 0.23 | ✅ | ✅ | 0 |
| `cand_mrs_f450` | HATALI | 0.584 | 1.704 | 0.4×0.4×0.334 | 0.2 | ❌ | ❌ | 4 |
| `cand_mrs_m690` | HATALI | 0.582 | 4.73 | 0.51×0.54×0.689 | 0.255 | ❌ | ❌ | 4 |
| `cand_mrs_naki` | HATALI | 0.585 | 7.54 | 0.94×0.94×0.434 | 0.47 | ❌ | ❌ | 8 |
| `cand_mrs_t650` | HATALI | 0.584 | 3.564 | 0.65×0.65×0.46 | 0.325 | ❌ | ❌ | 4 |
| `cand_mrs_x500` | HATALI | 0.583 | 2.006 | 0.382×0.382×0.385 | 0.191 | ❌ | ❌ | 4 |

### Adaya özel notlar

**`cand_d2d_x500`**
- Eksik bağımlılık: `Unable to find uri[model://x500]`
- 1 adet mesh collision ölçülemedi → sınır kutusu/burun değeri **alt sınır**, gerçeği daha büyük
- SDF 22 satır

**`cand_iq_camera`**
- SDF 278 satır

**`cand_iris`**
- 1 adet mesh collision ölçülemedi → sınır kutusu/burun değeri **alt sınır**, gerçeği daha büyük
- SDF 883 satır

**`cand_mrs_f450`**
- Eklenti hatası: `Failed to load system plugin [MrsGazeboCommonResources_MulticopterMotorModel]`
- SDF 935 satır

**`cand_mrs_m690`**
- Eklenti hatası: `Failed to load system plugin [MrsGazeboCommonResources_MulticopterMotorModel]`
- SDF 717 satır

**`cand_mrs_naki`**
- Eklenti hatası: `Failed to load system plugin [MrsGazeboCommonResources_MulticopterMotorModel]`
- SDF 1063 satır

**`cand_mrs_t650`**
- Eklenti hatası: `Failed to load system plugin [MrsGazeboCommonResources_MulticopterMotorModel]`
- SDF 904 satır

**`cand_mrs_x500`**
- Eklenti hatası: `Failed to load system plugin [MrsGazeboCommonResources_MulticopterMotorModel]`
- SDF 1265 satır

---

## Sonuç

Karar `docs/SECIM_KARARI.md` dosyasında.
