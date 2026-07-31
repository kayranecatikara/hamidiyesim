# 14 — `gcs_ui/` Web Arayüzü

**Yol:** `control/gcs_ui/` · **2.187 satır** (HTML 258 + JS 990 + CSS 939)
**Rol:** "Avcı Operasyon Merkezi" — taktik saha ekranı.
**Servis:** `gcs_server` statik dosya olarak sunar → `http://localhost:8000`

---
---

# `index.html` (258 satır)

## Panel yerleşimi

| Panel | Konum | İçerik |
|-------|-------|--------|
| **GÖREV VE KONTROL** | Sol | Uçuş senaryoları, manuel mod + joystick, gaz ayarı, GPS karıştırma, takip modu, video parazit, PnP poz tahmini |
| **Video** | Orta | Kamera akışı + parazit katmanı, AVCI DRONE / HEDEF İHA sekmeleri |
| **TELEMETRİ & ALGORİTMA** | Sağ | Bağlantı durumu, gecikme/kayıp/heartbeat, araç modları, 3B konum grafiği |
| **MISSION LOG** | Alt | Olay zaman çizelgesi |

## Kritik DOM öğeleri

```html
<!-- Video: MJPEG doğrudan <img> ile, üstünde parazit canvas'ı -->
<img id="fpv-stream" src="/api/video_feed/plane" width="1280" height="720">
<canvas id="video-noise-canvas" class="video-noise-canvas"></canvas>

<!-- 3B konum grafiği -->
<canvas id="pos3d-canvas"></canvas>
```

**Video neden `<img>`:** `gcs_server` MJPEG akışını
`multipart/x-mixed-replace` olarak yollar. Tarayıcı bunu `<img>` ile doğrudan
tüketir — JavaScript, WebRTC veya video codec'i gerekmez. Parazit efekti üstteki
canvas katmanına çizilir, akışı bozmadan.

## Kontrol öğeleri

| ID | İşlev |
|----|-------|
| `btn-scn-square/circle/aggressive` | Senaryo butonları |
| `btn-plane-manual` | Manuel moda geç |
| `joystick-base`, `joystick-knob` | Sanal joystick |
| `js-x`, `js-y`, `js-thr` | Joystick sayısal göstergeleri |
| `plane-thr-slider`, `plane-thr-value` | Gaz kaydırıcısı |
| `gps-jam-slider`, `gps-jam-value`, `gps-jam-status` | GPS karıştırma |
| `btn-chase`, `chase-status`, `chase-dist` | Takip modu |
| `conn-dot`, `conn-text` | Bağlantı göstergesi |
| `net-latency`, `net-loss`, `net-hb` | Ağ metrikleri |
| `mode-target`, `mode-hunter` | Araç modları |

---
---

# `script.js` (990 satır)

## Bağlantı katmanı

### `connectWebSocket()`

```javascript
const ws = new WebSocket(`ws://${window.location.host}/ws`);

ws.onopen = () => {
    connDot.className = 'dot green';
    connText.textContent = 'LINK ACTIVE';
    addLog('NET', 'Local Gazebo Sim Bağlantısı Kuruldu', 'info');
};

ws.onclose = () => {
    connDot.className = 'dot red';
    connText.textContent = 'LINK LOST';
    addLog('NET', 'Bağlantı Koptu! Yeniden bağlanılıyor...', 'crit');
    setTimeout(connectWebSocket, 1000);        // ← otomatik yeniden bağlanma
};

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    lastHbTime = Date.now();
    updateTelemetry(data);
};
```

**`onclose` içinde özyinelemeli `setTimeout`:** `gcs_server` yeniden başlatılırsa
arayüz kendini toparlar. Kullanıcının sayfayı yenilemesi gerekmez.

### Heartbeat izleyici

```javascript
setInterval(() => {
    const diff_sec = ((Date.now() - lastHbTime) / 1000).toFixed(1);
    netHb.textContent = diff_sec;
    if (diff_sec > 2.0 && connDot.classList.contains('green')) {
        addLog('NET', 'Veri akışı gecikmesi: ' + diff_sec + 's', 'warn');
    }
}, 500);
```

WebSocket açık görünse bile **veri akmıyor** olabilir (sunucu takıldı, thread
öldü). Bu sayaç son mesajdan bu yana geçen süreyi gösterir — sessiz arızayı
yakalar.

---

## Telemetri dağıtımı

### `updateTelemetry(data)`

```javascript
if (data.iris)  updateAvci(data.iris);
if (data.plane) updateHedef(data.plane);
if (data.iris)  recordTrail('iris', data.iris);
if (data.plane) recordTrail('plane', data.plane);
if (data.iris && data.plane) updateChasePositions(data.plane, data.iris);
```

Tek giriş noktası, beş tüketici. Her biri kendi paneline yazar.

### `updateAvci(drone)` / `updateHedef(plane)`
İlgili panele konum, hız, irtifa, mod ve ARM durumunu yazar.

---

## 3B konum grafiği

```javascript
const P3D_TRAIL_MAX = 350;            // ~35 sn iz @ 10 Hz
const p3dTrails = { iris: [], plane: [] };
let p3dAzim = -0.8;                   // radyan — sürükleyerek değişir
let p3dElev = 1.0;                    // 0.15 (yandan) .. 1.5 (tepeden)
const p3dView = { cx: 0, cy: 0, cz: 0, span: 40, init: false };
```

Kod yorumu:
> *"NED telemetri (x=Kuzey, y=Doğu, z=Aşağı) → dünya (E, N, YUKARI=-z). Harici
> kütüphane yok: ortografik projeksiyon + azimut/yükseliş döndürme."*

### `recordTrail(name, v)`

```javascript
if (v.x === 0 && v.y === 0 && v.z === 0) return;  // telemetri henüz yok
```

Sıfır telemetriyi ele — başlangıçta gelen boş paketler grafiği origin'e
çakmasın diye.

İz `P3D_TRAIL_MAX` uzunluğunda halka tampon (ring buffer).

### `p3dNiceStep(raw)`
Izgara aralığını "güzel" bir sayıya yuvarlar (1, 2, 5, 10, 20, 50...). Eksen
etiketleri 37.4 m gibi değil, 40 m gibi görünür.

### `p3dRender()`
Canvas'a çizim: ızgara, iki iz, mevcut konumlar, eksen etiketleri.
`p3dView` yumuşatması *"grafik zıplamasın"* diye — otomatik çerçeveleme ani
sıçrama yapmaz.

**Neden harici kütüphane yok:** Three.js gibi bir bağımlılık eklemek yerine
~150 satır ortografik projeksiyon yazılmış. Arayüz tamamen self-contained kalır.

---

## Kamera sekmeleri

### `switchCamera(vehicle)`

```javascript
document.getElementById('fpv-stream').src = `/api/video_feed/${vehicle}`;
```

`<img>` `src`'sini değiştirmek yeni MJPEG akışı başlatır, eskisi kapanır.

---

## Senaryo kontrolü

### `startScenario(name)` / `stopScenario()` / `markScenarioButtons()`

```javascript
async function startScenario(name) {
    await fetch(`/api/command/plane/scenario/${name}`, { method: 'POST' });
    markScenarioButtons();
}
```

`markScenarioButtons()` aktif senaryonun butonunu vurgular. Durum
`/api/scenario_status` **polling**'iyle senkron tutulur — sayfa yenilense bile
doğru buton işaretli kalır.

**Neden WebSocket değil polling:** Senaryo durumu seyrek değişir ve idempotent
okuma yeterlidir. Telemetri gibi sürekli akan veri için WebSocket, durum
sorguları için polling — doğru araç ayrımı.

---

## Manuel kontrol

### Durum değişkenleri

```javascript
let manualActive = false;
let mAil = 0, mElv = 0, mThr = 0;      // yumuşatılmış komut değerleri
let jsAil = 0, jsElv = 0;              // joystick ham girdisi
let isDragging = false;
const keysDown = {};                    // klavye durumu
```

### `enterManualMode()` / `exitManualMode()`

```javascript
await fetch('/api/command/plane/start_manual', { method: 'POST' });
manualActive = true;
setInterval(manualTick, 50);            // 20 Hz iç döngü
```

### `joystickEventPos(e)` / `setKnob(ail, elv)`

Fare/dokunma konumunu `-1..+1` aralığına çevirir ve topuzu konumlandırır.
`setKnob` hem joystick hem klavye girdisini yansıtır — kullanıcı hangi yöntemi
kullanırsa kullansın aynı görsel geri bildirimi alır.

### `manualTick()` — kontrol döngüsü (20 Hz)

```javascript
// Hedef yüzey komutu: joystick sürükleniyorsa joystick, değilse klavye
let tAil, tElv;
if (isDragging) {
    tAil = jsAil;  tElv = jsElv;
} else {
    tAil = (keysDown['d'] ? 1 : 0) - (keysDown['a'] ? 1 : 0);
    tElv = (keysDown['w'] ? 1 : 0) - (keysDown['s'] ? 1 : 0);
}

// Yumuşatma: ani PWM sıçraması yerine ~0.3s'de hedefe ulaşır
mAil += (tAil - mAil) * 0.25;
mElv += (tElv - mElv) * 0.25;
if (tAil === 0 && Math.abs(mAil) < 0.02) mAil = 0;    // sıfıra tam otur
if (tElv === 0 && Math.abs(mElv) < 0.02) mElv = 0;

// L/I: kalıcı gaz seviyesi — basılı tutuldukça artar/azalır
if (keysDown['l']) mThr = Math.min(100, mThr + 1);
if (keysDown['i']) mThr = Math.max(0, mThr - 1);

// PWM'e çevir — FBWA: tam sapma = maks yatış/pitch açı hedefi
const aileron  = Math.round(1500 + mAil * 450);
const elevator = Math.round(1500 + mElv * 450);   // yüksek PWM = burun yukarı
const thr      = Math.round(1000 + mThr * 10);

// Sunucuya 10 Hz gönder (iç döngü 20 Hz — bir atlayarak)
manualSendTick++;
if (manualSendTick % 2) return;
fetch('/api/command/plane/manual', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({ aileron, elevator, throttle: thr })
}).catch(() => {});      // Sessiz hata
```

### Beş tasarım kararı

**1. Üstel yumuşatma (`* 0.25`):** Klavye tuşu ikili (0 veya 1). Ham gönderilse
PWM 1500 → 1950 arası anında sıçrar, uçak sarsılır. `0.25` katsayısı ~0.3 sn'de
hedefe ulaşır — analog kumanda hissi verir.

**2. Sıfıra tam oturma:** Üstel yumuşatma asla tam 0'a ulaşmaz (sonsuza kadar
yaklaşır). `< 0.02` eşiğinde sıfırlanır, yoksa uçak sürekli çok hafif yatık
kalır.

**3. Gaz kalıcı, yüzeyler geçici:** `L`/`I` gaz **seviyesini** değiştirir (bırakınca
kalır); `W/A/S/D` yüzey **komutu** verir (bırakınca nötre döner). Gerçek RC
kumandasındaki mantık: gaz kolu yerinde kalır, çubuklar yaya döner.

**4. 20 Hz iç döngü, 10 Hz gönderim:** `manualSendTick % 2` ile bir atlanır.
Yumuşatma ve görsel geri bildirim 20 Hz'te akıcı, ağ trafiği 10 Hz'te makul.

**5. Sessiz `catch`:** Tek bir isteğin başarısız olması kontrol döngüsünü
durdurmaz. Sonraki tick tekrar dener.

### Kaydırıcı senkronu

```javascript
if (planeThrSlider && parseInt(planeThrSlider.value, 10) !== Math.round(mThr)) {
    planeThrSlider.value = Math.round(mThr);
    planeThrValue.textContent = Math.round(mThr) + '%';
}
```

`L`/`I` ile gaz değiştirilince kaydırıcı da hareket eder. Yorum: *"sürüklerken
değer zaten aynı — çakışmaz."*

---

## Takip modu

### `btnChase` dinleyicisi

```javascript
btnChase.addEventListener('click', async () => {
    if (!chaseActive) {
        await fetch('/api/command/iris/start_chase', { method: 'POST' });
        startChaseStatusPolling();
    } else {
        await fetch('/api/command/iris/stop_chase', { method: 'POST' });
        stopChaseStatusPolling();
    }
});
```

### `startChaseStatusPolling()` / `stopChaseStatusPolling()`

`/api/chase_status` düzenli sorgulanır → `supervisor.status` alanları
(`faz`, `gecis_sayisi`, `kilit_sayac`) arayüze yansır. Kullanıcı **hangi fazda**
olduğunu ve **kaç kez geçiş** yapıldığını canlı görür.

### `updateChasePositions(plane, iris)`
İki araç arası mesafeyi hesaplayıp gösterir.

---

## Log ve HUD

### `addLog(source, message, level)`

```javascript
addLog('NET', 'Bağlantı Koptu!', 'crit');
```

| Seviye | Kullanım |
|--------|----------|
| `info` | Normal olay |
| `warn` | Dikkat (gecikme, uyarı) |
| `crit` | Kritik (bağlantı kaybı) |

Mission Log paneline zaman damgalı satır ekler.

### `animateHUD()` / `updateMissionStatus()`
Nişangâh animasyonu ve görev durumu göstergesi.

---
---

# `style.css` (939 satır)

Askeri/taktik temalı tasarım.

## Ana bileşen grupları

| Grup | İçerik |
|------|--------|
| **CSS değişkenleri** | `--success-green`, `--danger-red`, `--warning-amber`, panel renkleri |
| **Panel çerçeveleri** | Köşe aksanları, başlık şeritleri, ayırıcılar |
| **Göstergeler** | `.dot.green/.red`, durum rozetleri, sayısal göstergeler |
| **Joystick** | `#joystick-base` dairesel taban, `#joystick-knob` sürüklenebilir topuz |
| **Kaydırıcılar** | Gaz, GPS karıştırma, video parazit — özel `range` stilleri |
| **Video katmanı** | `.video-noise-canvas` mutlak konumlu üst katman |
| **Mission Log** | Zaman çizelgesi satırları, seviye renkleri |
| **Duyarlı yerleşim** | Grid tabanlı üç sütun, dar ekranda yığılma |

**Renk kodlaması tutarlı:** yeşil = sağlıklı/aktif, kırmızı = hata/kayıp,
amber = uyarı. `script.js` bu sınıfları JS tarafından ekleyip çıkarır
(`connDot.className = 'dot green'`).
