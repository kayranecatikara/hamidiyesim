# AVCI SİM — Dokümantasyon

> ⚠ **2026-08-10 — BU BÖLÜMLERİN BİR KISMI KEYPOINT DÖNEMİNE AİT.**
> Görsel güdüm 2026-08-06'da keypoint zincirinden çıktı; aktif yasa artık
> `control/guidance/bbox_ibvs.py` (yalnız tespit kutusu). Kamera↔güdüm köprüsü
> KARE köprüsüdür (`wait_new_frame` / `kayit["det"]`). Aşağıda keypoint'lerden,
> `pose_detector`'dan ya da `process(pose, ...)` imzasından söz eden her yer
> TARİHSELDİR → [`POSEA_GERI_DONMEK_ISTERSENIZ/`](../POSEA_GERI_DONMEK_ISTERSENIZ/README.md).
> Canlı davranış için kaynak kodu esastır.


> **Depo:** `~/projects/avci_sim` (hamidiyesim) · **Dal:** `main`
> **Son güncelleme:** 30 Temmuz 2026 · **Kapsam:** temizlik sonrası güncel kod ağacı
>
> Bu klasör iki iş yapar: (1) projenin **yapısını** anlatır, (2) **her kod
> dosyasının her fonksiyonunu** — ne yaptığı, nasıl çalıştığı, neden öyle
> tasarlandığı — açıklar.
>
> **Toplam ~5.700 satır.** İleride HTML siteye dönüştürülecek: dosya ağacına
> tıklayınca açılan detay paneline bu içerik girecek.

---

## Okuma sırası

### Genel bakış
| # | Dosya | Satır | İçerik |
|---|-------|------:|--------|
| 00 | **00_ICINDEKILER.md** | 79 | Bu dosya |
| 01 | [01_PROJE_YAPISI.md](01_PROJE_YAPISI.md) | 228 | Klasör ağacı, dosya envanteri, satır sayıları, gitignore |
| 02 | [02_MIMARI_VE_VERI_AKISI.md](02_MIMARI_VE_VERI_AKISI.md) | 230 | Süreç haritası, portlar, kare akışı, görev akışı, koordinat çerçeveleri |

### Kod açıklamaları (fonksiyon bazında)
| # | Dosya | Satır | Kapsadığı kod |
|---|-------|------:|---------------|
| 10 | [10_KOD_control_cekirdek.md](10_KOD_control_cekirdek.md) | 828 | `mav_common.py` · `drone_functions.py` · `plane_functions.py` · `plane_patterns.py` |
| 11 | [11_KOD_control_senaryo_demo.md](11_KOD_control_senaryo_demo.md) | 717 | `run_plane_scenario.py` · `arm_diag.py` · `control/demos/` |
| 12 | [12_KOD_guidance.md](12_KOD_guidance.md) | **1384** | ★ `control/guidance/` — 6 dosya, **projenin kalbi** |
| 13 | [13_KOD_gcs_server.md](13_KOD_gcs_server.md) | 495 | `gcs_server.py` — API, kamera, telemetri, görev |
| 14 | [14_KOD_gcs_ui.md](14_KOD_gcs_ui.md) | 361 | `gcs_ui/` — HTML, JS, CSS |
| 15 | [15_KOD_vision.md](15_KOD_vision.md) | 700 | `vision/` — projeksiyon, tespit, veri, eğitim |
| 16 | [16_KOD_sim_varliklari.md](16_KOD_sim_varliklari.md) | 212 | `sim/` — world, model, parametre |
| 17 | [17_KOD_scripts_tools_tests.md](17_KOD_scripts_tools_tests.md) | 296 | `scripts/` · `tests/` · `tools/` · `logs/` · kök dosyalar |

### Kayıt
| # | Dosya | Satır | İçerik |
|---|-------|------:|--------|
| 90 | [90_TEMIZLIK_KAYDI.md](90_TEMIZLIK_KAYDI.md) | 197 | Temizlikte ne silindi/taşındı/düzeltildi + commit bilgisi |

---

## Kod açıklamalarının şablonu

Her fonksiyon şu başlıklarla anlatılır (hepsi her fonksiyonda olmayabilir):

- **İmza** — parametreler ve dönüş değeri
- **Ne yapar** — dışarıdan bakışla görevi
- **Nasıl çalışır** — kritik kod parçası + adım adım akış
- **Neden böyle** — tasarım kararı ve gerekçesi (çoğu koddaki yorumlardan
  veya gerçek uçuş loglarından çıkarıldı)

Kritik algoritma parçaları **kod bloğu olarak gömülü**; gerisi anlatım ve tablo.

---

## 30 saniyede proje

Bir **avcı multikopter** (iris, ArduCopter) sabit kanatlı bir **hedef İHA'yı**
(mini Talon, ArduPlane) hava-havada tespit edip kovalar ve kamikaze müdahale
eder. Her iki araç da **Gazebo Harmonic** fiziğinde **ArduPilot SITL** ile
gerçekten uçar.

Güdüm **iki fazlıdır**:

1. **GPS fazı** — hedefi kameranın tam ortasına ve tespitin güvenilir
   çalıştığı ~10-11 m bandına oturtur. Amacı vuruş değil, **kadraj merkezleme**.
2. **Görsel faz (bbox IBVS)** — yalnız tespit kutusundan çıkarılan
   yönelimle **menzilden bağımsız** öne nişan alır, kesme rotasında kapanır,
   terminal kör dalışla vurur.

Geçişi bir **supervisor** yönetir; görsel temas kesilirse GPS'e döner, GPS
karıştırılırsa (jamming) görsele geçer. Her şey tek bir web arayüzünden
(`http://localhost:8000`) yönetilir.

---

## Tek satırlık dosya haritası

**Beyin (güdüm):** `guidance_core.py` (matematik) → `adapter_copter.py` (komut)
→ `visual_lead.py` (görsel döngü) · `gps_guidance.py` (GPS döngüsü) ·
`supervisor.py` (geçiş) · `common.py` (MAVLink çıkışı)

**Gözler (görü):** `geometry.py` (projeksiyon) → `capture_*.py` (veri) →
`train_yolo.py` (eğitim) → `detector.py` (çıkarım) →
`detection_state.py` (paylaşım)

**Kaslar (kontrol):** `mav_common.py` (MAVLink) → `drone_functions.py` (iris) /
`plane_functions.py` + `plane_patterns.py` (Talon) → `run_plane_scenario.py`

**Yüz (arayüz):** `gcs_server.py` (API + kamera + görev) → `gcs_ui/` (web)

**Sahne (simülasyon):** `sim/gazebo_harmonic/` (dünya + modeller) +
`sim/ardupilot_params/` (uçuş parametreleri)

---

## Nereden başlamalı?

| Amaç | Bakılacak yer |
|------|---------------|
| Sistemi çalıştırmak | `docs/SIMULASYON_CALISTIRMA.md` (depoda) |
| Mimariyi anlamak | [02_MIMARI_VE_VERI_AKISI.md](02_MIMARI_VE_VERI_AKISI.md) |
| Güdümü anlamak | [12_KOD_guidance.md](12_KOD_guidance.md) — supervisor → gps_guidance → guidance_core |
| Arayüz / API | [13_KOD_gcs_server.md](13_KOD_gcs_server.md) → [14_KOD_gcs_ui.md](14_KOD_gcs_ui.md) |
| Görüntü işleme | [15_KOD_vision.md](15_KOD_vision.md) — geometry → detector |
| Uçuşu analiz etmek | [17_KOD_scripts_tools_tests.md](17_KOD_scripts_tools_tests.md) — CSV formatları + `gps_log_viz.py` |
| Kod değiştirmeden önce | `python3 -m tests.test_visual_lead` ve `python3 -m tests.test_gps_guidance` |
