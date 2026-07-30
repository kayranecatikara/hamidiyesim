# Proje Yapısı ve Kod Açıklamaları

Bu klasör projenin **fonksiyon bazında** kod dokümantasyonunu ve onu tek dosyalık
bir siteye çeviren üreteci içerir.

**Siteyi açmak için:** [`site/index.html`](site/index.html) — çift tıklayın.
Tek dosya, kendine yeten (internet/CDN gerektirmez), 580 KB.

---

## İçerik

### Genel bakış
| Dosya | İçerik |
|-------|--------|
| [00_ICINDEKILER.md](00_ICINDEKILER.md) | Okuma sırası, 30 saniyede proje, dosya haritası |
| [01_PROJE_YAPISI.md](01_PROJE_YAPISI.md) | Klasör ağacı, dosya envanteri, satır sayıları, gitignore |
| [02_MIMARI_VE_VERI_AKISI.md](02_MIMARI_VE_VERI_AKISI.md) | Süreç haritası, portlar, kare akışı, görev akışı, koordinat çerçeveleri |

### Kod açıklamaları
| Dosya | Kapsadığı kod |
|-------|---------------|
| [10_KOD_control_cekirdek.md](10_KOD_control_cekirdek.md) | `mav_common.py` · `drone_functions.py` · `plane_functions.py` · `plane_patterns.py` |
| [11_KOD_control_senaryo_demo.md](11_KOD_control_senaryo_demo.md) | `run_plane_scenario.py` · `arm_diag.py` · `control/demos/` |
| [12_KOD_guidance.md](12_KOD_guidance.md) | ★ `control/guidance/` — hibrit güdüm, projenin kalbi |
| [13_KOD_gcs_server.md](13_KOD_gcs_server.md) | `gcs_server.py` — API, kamera, telemetri, görev |
| [14_KOD_gcs_ui.md](14_KOD_gcs_ui.md) | `gcs_ui/` — HTML, JS, CSS |
| [15_KOD_vision.md](15_KOD_vision.md) | `vision/` — projeksiyon, tespit, veri toplama, eğitim |
| [16_KOD_sim_varliklari.md](16_KOD_sim_varliklari.md) | `sim/` — world, model, ArduPilot parametreleri |
| [17_KOD_scripts_tools_tests.md](17_KOD_scripts_tools_tests.md) | `scripts/` · `tests/` · `tools/` · `logs/` · kök dosyalar |

### Kayıt
| Dosya | İçerik |
|-------|--------|
| [90_TEMIZLIK_KAYDI.md](90_TEMIZLIK_KAYDI.md) | 30 Temmuz 2026 temizliğinde ne silindi/taşındı/düzeltildi |

**Toplam ~5.800 satır.**

---

## Açıklama şablonu

Her fonksiyon şu başlıklarla anlatılır:

- **İmza** — parametreler ve dönüş değeri
- **Ne yapar** — dışarıdan bakışla görevi
- **Nasıl çalışır** — kritik kod parçası + adım adım akış
- **Neden böyle** — tasarım kararı ve gerekçesi

Gerekçelerin çoğu koddaki yorumlardan ve gerçek uçuş loglarından çıkarıldı —
ör. `supervisor.GATE_MENZIL = 20` ayarının sebebi 2026-07-24 logundaki
"uzakta devralınca hedef hemen kaçtı" gözlemidir.

---

## Siteyi yeniden üretmek

Markdown dosyalarını düzenledikten sonra:

```bash
cd dokumantasyon/build
python3 build_site.py
```

Çıktı: `dokumantasyon/site/index.html`

Harici bağımlılık **yoktur** — markdown dönüştürücüsü de dahil her şey
`build/` altında.

| Dosya | İşlevi |
|-------|--------|
| `build/build_site.py` | Gezinti ağacını kurar, markdown'ı panellere böler, arama indeksini üretir |
| `build/md2html.py` | Bağımlılıksız markdown → HTML (başlık, tablo, kod, alıntı, liste, satır içi) |
| `build/shell.html` | Sayfa kabuğu: CSS token sistemi, gezinti, arama, tema |

`build_site.py` içindeki `TREE` sabiti gezinti ağacını tanımlar — yeni bir kod
dosyası dokümante edilirse oraya bir satır eklenir.

---

## Site özellikleri

- **Proje ağacı** — sol rayda, alt sistem renk kodlu (kontrol / güdüm / arayüz / görü / simülasyon)
- **Fonksiyon indeksi** — sağ rayda, kaydırma takipli
- **Arama** — `/` tuşu; önce fonksiyon/dosya adlarında, sonra gövde metninde
- **Derin bağlantı** — `#bölüm/fonksiyon` biçiminde adres
- **Açık/koyu tema** — sistem tercihini izler, düğmeyle değiştirilebilir
- **Mobil** — çekmece menü
