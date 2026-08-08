# Otonom Uçuş Testi — Claude'a uçurt, görüntü + log ile doğrulat

> Kısayol: Claude Code içinde **`/ucus-testi`** yaz. Gerisini Claude yapar.
> Bu doküman yöntemin ne olduğunu, neden var olduğunu ve parçalarının elle
> nasıl kullanılacağını anlatır.

## Ne bu?

Güdüm değişikliklerini test etmenin insansız yolu. Claude:

1. **Simi kendisi başlatır** (Gazebo + 2 SITL + gcs_server) ve EKF'in
   oturmasını loglardan bekler.
2. **Uçuşu kendisi yönetir** — kalkış, takip, senaryo seçimi; hepsi panelin
   kullandığı HTTP API'siyle (`/api/command/...`). "Yanlış senaryo açıldı mı,
   gaz farklı mıydı" belirsizliği olmaz; her koşu birebir tekrarlanabilir.
3. **Saniyede 1 kamera karesi + o anki panel telemetrisini** kaydeder
   (`tools/ucus_kaydi.py`) ve kareleri **gözle** inceler: hedef kadrajın
   neresinde, takip nasıl, manevrada ne oluyor.
4. **CSV ile çapraz kontrol eder** (`tools/ucus_analiz.py`): görüntünün
   söylediğiyle logun söylediği çelişiyorsa bunu raporlar.
5. Uçuşun tamamını izlenebilir **video** yapar (ffmpeg) — insan da doğrular.

## Neden var?

İki insan-faktörü hatası bu projede pahalıya maloldu:

- **Panel +8 m hatası**: arayüz aylarca her mesafeye 8 m ekledi; bütün uçuş
  gözlemleri şişik okundu. Görüntü↔log çapraz kontrolü olsaydı ilk gün
  yakalanırdı.
- **"En iyi an" okuması**: panelden elle not alınan değer dağılımın en iyi
  anı çıkabiliyor ("kare kenarı 14 m" notu; aynı uçuşun dürüst medyanı
  ~22 m'ydi). Otomatik kayıt her saniyeyi eşit sayar.

## Kurallar (yöntemin değeri bunlara bağlı)

- **Her koşuda TEK değişken.** Değişiklik + eski davranış (kill-switch env
  değişkeni) aynı oturumda A/B uçurulur.
- **Kıyas hep AYNI segmentasyonla.** İki uçuşu karşılaştırırken ikisine de
  `tools/ucus_analiz.py` uygulanır; elle okunmuş eski notlarla medyan
  karşılaştırılmaz.
- **Görüntü ve log birlikte.** Sadece sayı ya da sadece video yetmez;
  ikisinin tutarlılığı da raporun parçası.
- Sonuç `UYGULANACAK.md`'deki ilgili maddenin altına işlenir.

## Parçaları elle kullanmak

Sim + gcs_server çalışırken:

```bash
# 1) Kayıt (ör. 420 saniye):
python3 tools/ucus_kaydi.py /tmp/ucus_kayit 420
#    → /tmp/ucus_kayit/frames/f0001.jpg...  ve meta.csv (kare ↔ panel eşi)

# 2) Uçuşu API'yle yönet (panel butonlarının birebir karşılığı):
curl -X POST http://127.0.0.1:8000/api/command/plane/scenario/circle   # uçak kalkar + daire
curl -X POST http://127.0.0.1:8000/api/command/iris/start_chase        # drone kalkar + takip
curl -X POST http://127.0.0.1:8000/api/command/plane/scenario/square   # desen değiştir
curl -X POST http://127.0.0.1:8000/api/command/iris/stop_chase
curl -X POST http://127.0.0.1:8000/api/command/plane/stop_scenario
#    Durum: /api/chase_status  /api/scenario_status  /api/debug/telem
#    Senaryolar: square, circle (⌀55 referans), circle_xl, circle_l, circle_s, aggressive

# 3) Analiz (rejim ayrımlı özet; kıyas için iki CSV verilebilir):
python3 tools/ucus_analiz.py logs/gps_guidance_YYYYMMDD_HHMMSS.csv [eski.csv]

# 4) Video:
ffmpeg -framerate 5 -i /tmp/ucus_kayit/frames/f%04d.jpg -c:v libx264 \
       -pix_fmt yuv420p logs/ucus.mp4
```

## Claude'un otomasyonuna özgü tuzaklar (öğrenildi, tekrarlama)

- **`pkill -f` kendi kabuğunu öldürür**: Claude komutları `bash -c "..."`
  ile koşar, desen kendi komut satırıyla eşleşir. Çözüm köşeli parantez
  numarası: `pkill -f 'gz [s]im'` ("gz sim"i öldürür, kendini öldürmez).
- **MAVProxy TTY'siz hemen çıkar**: arka planda `sim_vehicle.py`yi
  `script -qfec "..." /dev/null` ile sarmala (sahte TTY).
- **EKF beklenmeden kalkış yapılmaz**: iki SITL çıktısında da
  `EKF3 IMU0 is using GPS` görülmeli.
- **Kod doğrulaması**: gcs_server açılışında beklenen banner'lar kontrol
  edilir (ör. `[GPS] istasyon yükselişi DİNAMİK ...`) — yoksa test edilen
  kod çalışmıyor demektir.

## İlk kullanım: 2026-08-08

Dinamik istasyon yükselişi (`403131f`) bu yöntemle doğrulandı: 402 kare +
7224 satır CSV; kadraj dikey sapması dönüşte −23°→−9.4°, düzde −10°→−3°,
menzil değişmedi. Aynı koşuda yöntem, tabandaki "kare kenarı 14 m"
değerinin en-iyi-an okuması olduğunu da ortaya çıkardı (dürüst medyan ~22 m,
eski uçuşta da öyleymiş). Video: `logs/ucus_20260808_1212_kamera.mp4`.
