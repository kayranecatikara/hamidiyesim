# Güdüm algoritması anlık görüntüsü — 2026-08-24 12:07

Bu dizin, **algoritma değişikliklerine başlamadan ÖNCEKİ** çalışan hâlin
birebir kopyasıdır. İstenildiğinde tek komutla geri dönülür.

## Neyi kaydettim

| dosya | ne |
|---|---|
| `dosyalar/guidance/` | tüm güdüm çekirdeği (`bbox_ibvs.py` = görsel güdüm, `gps_guidance.py`, `visual_lead.py`, `guidance_core.py`, `frpn*.py`, `hedef_kestirim.py`, `kurtarma.py`, `supervisor.py`, `adapter_copter.py`, `common.py`) |
| `dosyalar/ayar_konsolu.py` | 56 canlı ayarın tanımı ve sınırları |
| `dosyalar/gcs_server.py` | panel sunucusu (`_OZELLIKLER` düğme listesi burada) |
| `dosyalar/run_plane_scenario.py` | hedef senaryoları (`kare_gorev` dahil) |
| `dosyalar/gcs_ui/` | panel arayüzü (index.html, script.js) |
| `dosyalar/ardupilot_params/avci_copter.parm` | avcı araç parametreleri |
| `calisma_agaci.patch` | `git diff` — HEAD'e göre commit'lenmemiş tüm fark |
| `git_head.txt` | temel alınan commit |
| `SHA256SUMS.txt` | her dosyanın bütünlük damgası |

## Temel alınan nokta

- commit: `f64e4fe` (2026-08-20) "README: yol adi avci_sim'e geri alindi…"
- üstüne commit'lenmemiş `kalkis_kare_inis` entegrasyonu (kare_gorev) duruyor
  — o da `calisma_agaci.patch` içinde.

## Geri alma

```bash
bash yedek/algoritma_2026-08-24_1207/GERI_AL.sh
```

Sorar, "evet" yazınca yukarıdaki dosyaları eski hâline yazar.
Sonra simi yeniden kur:

```bash
bash scripts/kapat.sh && AVCI_TEMIZ=1 bash scripts/mkur.sh m
```

## Bütünlük kontrolü

```bash
cd yedek/algoritma_2026-08-24_1207 && sha256sum -c SHA256SUMS.txt --quiet && echo SAGLAM
```
