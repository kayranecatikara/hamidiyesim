# ESP32 + RadioMaster Ranger Micro — CRSF kare görevi

Bu bileşen, mevcut operasyon arayüzünün temel mantığına dokunmadan bağımsız bir donanım yer istasyonu ekler. ESP32, Ranger Micro'nun **handset/CRSF girişine** 16 adet RC kanalı yollar ve kendi Wi‑Fi erişim noktasında basit operasyon ekranı sunar.

## Önemli sınır

Bu, MAVLink görev planı değildir. CRSF `RC_CHANNELS_PACKED (0x16)` paketleriyle sanal kumanda kolu üretir. Betaflight/INAV tarafında roll–pitch komutları açı/hız isteğidir; GPS/optik konum geri beslemesi yoksa rüzgâr ve trim hataları nedeniyle tam geometrik kare veya başlangıç noktasına hassas dönüş garanti edilemez. Hassas kare için INAV waypoint veya uçuş kontrolcüsünün ayrı telemetri/MAVLink bağlantısı kullanılmalıdır.

Kod otomatik kalkış yapmaz. Yalnız uygun stabilize/irtifa-tutma modu seçilmiş ve havada olan araç için tasarlanmıştır.

## Donanım

| ESP32 DevKit V1 | Ranger Micro tarafı | Not |
|---|---|---|
| GPIO17 / TX2 | CRSF RX / module input | Veri ESP32 → modül |
| GPIO16 / RX2 | CRSF TX / telemetry output | Veri modül → ESP32 |
| GND | GND | Ortak toprak zorunlu |
| GPIO27 | GND üzerinden acil buton | `INPUT_PULLUP`, basınca DISARM |
| GPIO26 | GND üzerinden anahtarlı ARM izni | Kapalı devre olmadan ARM reddedilir |

Ranger Micro'yu ESP32'nin güç pininden beslemeyin. Anten takılı olmalıdır. Modül için üreticinin belirttiği XT30 giriş aralığı **6–16.8 V**'tur. Modül-bay konektör revizyonunuzdaki pin isimlerini kendi RadioMaster şemanızdan doğrulamadan kablo bağlamayın. ESP32 yalnız 3.3 V mantık toleranslıdır; ölçtüğünüz UART seviyesi farklıysa seviye dönüştürücü kullanın.

## Arduino IDE

1. Boards Manager URL: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. Boards Manager'dan `esp32 by Espressif Systems` paketini kurun.
3. Board: **DOIT ESP32 DEVKIT V1** (yoksa **ESP32 Dev Module**).
4. Upload Speed: `921600` sorun çıkarırsa `460800` veya `115200`.
5. Flash Frequency: `80 MHz`; Partition Scheme: `Default 4MB with spiffs`.
6. `elrs_square_gcs.ino` dosyasını açıp yükleyin. Harici Arduino kütüphanesi gerekmez.
7. Telefon/bilgisayardan `AVCI-ELRS-GCS` ağına bağlanın; varsayılan parola kaynak kodundadır. `http://192.168.4.1` adresini açın.

## Uçuş kontrolcüsü / ELRS

- TX ve RX aynı ELRS ana sürümünde ve aynı bind phrase/regulatory domain ile bağlı olmalıdır.
- Alıcı çıkışı **CRSF** olmalıdır; FC UART'ında Serial RX, receiver protocol olarak CRSF seçilir.
- Kanal sırası `AETR`: CH1 Roll, CH2 Pitch, CH3 Throttle, CH4 Yaw.
- Betaflight/INAV Modes ekranında AUX1=ARM, AUX2=ANGLE veya doğrulanmış irtifa-tutma modu olacak şekilde aralıkları ayarlayın.
- FC failsafe davranışını `DROP/LAND` gibi güvenli bir seçeneğe ayarlayıp **pervaneler sökülüyken** RF kesme testini yapın.

## Emniyet ve test sırası

1. Pervaneleri sökün; Ranger antenini takın.
2. ARM izin anahtarını açık bırakın. ESP32 açıldığında Receiver sekmesinde tüm eksenlerin nötr, throttle ve AUX1'in düşük olduğunu doğrulayın.
3. Yönleri kontrol edin: ileri safhada pitch düşmeli, sağ safhada roll yükselmelidir. FC'nizde tersse yalnız ilgili `+/- MOVE_DEFLECTION_US` işaretini değiştirin.
4. E‑STOP'un AUX1'i ve throttle'ı anında düşürdüğünü doğrulayın.
5. Modül kapatıldığında FC failsafe'inin beklendiği gibi çalıştığını doğrulayın.
6. İlk enerjili testi bağlı/tethered test düzeneğinde, geniş güvenlik alanında ve düşük `MOVE_DEFLECTION_US` / kısa `LEG_DURATION_MS` ile yapın.

## Ağ topolojisi — AP + STA

Varsayılan davranış: ESP32 yalnız kendi erişim noktasını (`AVCI-ELRS-GCS`,
`192.168.4.1`) yayınlar. Bu, altyapı olmayan sahada çalışır ama bir bedeli
vardır: operatör dizüstü bu ağa bağlıyken **başka bir ağda olamaz**.
`gcs_server` ayrı bir makinede koşuyorsa video ve telemetri kesilir.
(Aynı makinede koşuyorsa `localhost` çalışmayı sürdürür.)

Kaynakta `STA_SSID` / `STA_PASSWORD` doldurulursa sketch `WIFI_AP_STA`
moduna geçer:

- **AP kapanmaz** — saha yedeği olarak yayında kalır.
- ESP32 aynı anda mevcut ağa istemci olarak katılır ve ikinci bir IP alır.
- Operatör dizüstü tek ağda kalır; hem `gcs_server`'a hem ESP32'ye erişir.
- Bağlantı koparsa `STA_RETRY_MS` (15 sn) aralıklarla yeniden denenir;
  deneme bloklamaz, CRSF kare üretimi kesintiye uğramaz.

`STA_SSID` boş bırakılırsa davranış **eskisinin birebir aynısıdır**.

Arayüz, ESP32'nin hangi yoldan erişildiğini `/api/status` içindeki
`ap_ip` / `sta_ip` alanlarından okur ve ELRS rozetinin üzerine yazar.

## `/api/status` alanları

| alan | anlam |
|---|---|
| `state` / `state_code` | görev durumu (insan / makine okunur) |
| `armed`, `mission_active`, `arm_permit`, `estop` | emniyet kilitleri |
| `lq`, `rssi_dbm`, `telemetry_age_ms` | CRSF LINK_STATISTICS'ten gelen link ölçümleri |
| **`frame_hz`** | **ÖLÇÜLEN** CRSF kare hızı (aşağıya bakın) |
| `frame_skips` | zamanında gönderilemeyen kare sayısı (teşhis) |
| `uptime_s` | açılıştan beri geçen saniye |
| `ap_ip`, `ap_clients`, `sta_enabled`, `sta_connected`, `sta_ip`, `sta_rssi` | ağ durumu |
| `channels_us` | 16 kanalın µs değerleri |

### `frame_hz` artık ölçülüyor

Önceki sürüm bu alanda sabit `50` döndürüyordu — bu bir ölçüm değil, niyet
beyanıydı. `loop()` bir HTTP isteği veya UART okuması yüzünden gecikirse
gerçek hız 50'nin altına düşer ve arayüz bunu **asla göremezdi**.

Şimdi gönderilen kareler sayılıp 1 saniyelik pencerede gerçek hıza çevriliyor.
İlk pencere dolana kadar `0` döner; arayüz bunu `ÖLÇÜLÜYOR` diye gösterir,
`50` uydurmaz. 45 Hz'in altına düşen değer arayüzde kırmızıya döner —
ESP32'nin yükünü gizlemek yerine görünür kılar.

Ayrıca kare zamanlaması düzeltildi: `loop()` uzun süre takılırsa birikmiş
kareler artık arka arkaya **patlatılmıyor**; iki periyottan fazla geri
kalınmışsa faz yeniden senkronlanıyor ve kaçan kareler `frame_skips`
sayacında görünür oluyor.

## Ayarlanabilir sabitler

- `HOVER_THROTTLE_US`: aracın hover değeri; varsayılan 1500 evrensel değildir.
- `MOVE_DEFLECTION_US`: hareket şiddeti; varsayılan ±220 µs.
- `LEG_DURATION_MS`: her kenar; varsayılan 3 saniye.
- `CORNER_PAUSE_MS`: kenarlar arası nötr bekleme; varsayılan 0.5 saniye.
- `COMMAND_WATCHDOG_MS`: arayüz heartbeat zaman aşımı. Görev sırasında tarayıcı bağlantısı kesilirse eksenler nötre alınır; araç havadayken motor kesilmemesi için ARM korunur. Fiziksel E‑STOP ve kırmızı `ACİL DISARM` ise throttle/AUX1'i düşürür.

## Protokol özeti

Sketch CRSF'yi doğrudan üretir: `0xEE, length=24, type=0x16`, 16×11 bit kanal ve `0xD5` polinomlu CRC8. PWM-benzeri 1000–2000 µs aralığı CRSF 172–1811 aralığına çevrilir. Çıkış 400000 baud, 8N1 ve **hedef** 50 Hz'dir; gerçekleşen hız `/api/status` → `frame_hz` alanından okunur.
