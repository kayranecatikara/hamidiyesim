# Güdüm algoritması anlık görüntüsü — 2026-08-28 14:06
## "Ö-KF ÖNCESİ" — görüntü gecikmesi telafisi eklenmeden önceki çalışan hâl

Bu dizin, **Ö-KF (görüntü gecikmesi telafisi) eklenmeden ÖNCEKİ** hâlin
birebir kopyasıdır. Tek komutla geri dönülür.

## Neden alındı

Kullanıcı isteği (2026-08-28): *"böyle iyi bir düzeltme yapma sen bunu
(önceki halini de saklayıp başka bir yerde) commitle"*.

Ö-KF, kutu ölçümünü güdüme vermeden önce gecikmeli bir Kalman süzgecinden
geçiriyor. Ölçülen bir kusuru var (aşağıda) ve kullanıcı bu hâliyle
saklanmasını istedi — düzeltme yapılmadan. Geri dönüş yolu bu dizindir.

## Temel alınan nokta

- commit: `147d57d` (2026-08-24) "yedek: 2026-08-24 algoritma anlik goruntusu…"
- `calisma_agaci.patch` = o commit'e göre commit'lenmemiş fark (Ö-KF'nin kendisi)

## Neyi kaydettim

| dosya | ne |
|---|---|
| `dosyalar/guidance/` | tüm güdüm çekirdeği — **`gecikme_kf.py` YOK** (Ö-KF öncesi) |
| `dosyalar/ayar_konsolu.py` | 64 canlı ayarın tanımı ve sınırları |
| `dosyalar/gcs_server.py` | panel sunucusu |
| `dosyalar/run_plane_scenario.py` | hedef senaryoları |
| `dosyalar/gcs_ui/` | panel arayüzü |
| `dosyalar/tests/test_bbox_ibvs.py` | 69 kabul bekçisi (Ö-KF testleri hariç) |
| `dosyalar/UYGULANACAK.md` | Ö-KF maddesi eklenmeden önceki hâli |
| `dosyalar/ardupilot_params/` | avcı araç parametreleri |
| `calisma_agaci.patch` | HEAD'e göre commit'lenmemiş tüm fark |
| `git_head.txt` · `git_status.txt` · `SHA256SUMS.txt` | temel nokta ve bütünlük |

## Geri alma

```bash
bash yedek/algoritma_2026-08-28_1406_gecikme_oncesi/GERI_AL.sh
```

Betik `gecikme_kf.py` ve `test_gecikme_kf.py` dosyalarını da **siler**
(CLAUDE.md §5.12: silinen özellik tamamen silinir, artık bırakılmaz), sonra
`tests/test_bbox_ibvs.py`'yi koşup doğrular.

## ⚠ Bu yedeğin ALINDIĞI andaki Ö-KF durumu — bilinen kusur

Ö-KF **bir uçuş yaptı ve VURDU** (2026-08-28 13:47, `mini_talon` sağ kanadına
fiziksel temas). Mekanizma kapısı geçti: `kf_durum = AKTIF` %98.6, ölçülen
gecikme medyan 104 ms.

**AMA ölçülen bir kusuru var, düzeltilmedi:** süzgecin uyguladığı düzeltme
(`kf_dcx`) kare kare işaret değiştiriyor — saniyede 4.6-5.9 kez. Ham kutu
merkezi ise çok sakin (0.17-0.87/s, kare kare 1-1.5 px). Yani titreşimi
süzgeç ekliyor. Genliği orta menzilde küçük (1.3-1.5 px) ama temas fazında
9.2 px'e çıkıyor.

**Sebep:** `KF_HEDEF_IVME = 15 m/s²` (süreç gürültüsü Q) bilerek cömert
seçildi — hedef manevra yapınca süzgeç geride kalmasın diye. Cömert Q =
"ölçüme çok güven" = her ölçümde kestirimi sertçe çekmek = salınan düzeltme.

**Aday çare (UYGULANMADI, kullanıcı kararı):** `AVCI_IBVS_KF_Q=6`.
Ödünleşim: titreşim düşer, manevrada geri kalma artar.

⚠ Ö-KF'nin **iyileştirdiği kanıtlanmadı** — n=1, eşli kontrol kolu yok
(CLAUDE.md §5.4). Kanıt için kol başına ≥4 koşuluk A/B gerekiyor.
