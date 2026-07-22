// DOM Elements
const connDot = document.getElementById('conn-dot');
const connText = document.getElementById('conn-text');
const netLatency = document.getElementById('net-latency');
const netLoss = document.getElementById('net-loss');
const netHb = document.getElementById('net-hb');
const eventLog = document.getElementById('event-log');

// Panels
const modeTarget = document.getElementById('mode-target');
const modeHunter = document.getElementById('mode-hunter');
const targetControls = document.getElementById('target-controls');
const hunterControls = document.getElementById('hunter-controls');

// Kamera stream'ini değiştiren fonksiyon
// MJPEG multipart stream tarayıcıda süresiz bağlantı açar,
// src değiştirmek her zaman eski bağlantıyı kapatmaz.
// Bu yüzden eski img'yi DOM'dan silip yenisini oluşturuyoruz.
function switchCamera(vehicle) {
    const container = document.querySelector('.fpv-container');
    const oldImg = document.getElementById('fpv-stream');
    if (oldImg) {
        oldImg.src = '';  // eski stream bağlantısını kes
        oldImg.remove();  // DOM'dan kaldır
    }
    const newImg = document.createElement('img');
    newImg.id = 'fpv-stream';
    newImg.alt = 'Video Yükleniyor...';
    newImg.style.cssText = 'object-fit: cover; width: 100%; height: 100%;';
    // cache-buster ile tarayıcıyı yeni bağlantı açmaya zorla
    newImg.src = `/api/video_feed/${vehicle}?t=${Date.now()}`;
    // HUD overlay'in önüne (arkasına) ekle
    const hudOverlay = container.querySelector('.hud-overlay');
    container.insertBefore(newImg, hudOverlay);
}

// Radio Button UI Toggle
modeTarget.addEventListener('change', () => {
    if(modeTarget.checked) {
        targetControls.classList.add('active-section');
        hunterControls.classList.remove('active-section');
        switchCamera('plane');
        addLog('SYS', 'UI Modu Değiştirildi: HEDEF İHA Seçili');
    }
});

modeHunter.addEventListener('change', () => {
    if(modeHunter.checked) {
        hunterControls.classList.add('active-section');
        targetControls.classList.remove('active-section');
        switchCamera('iris');
        addLog('SYS', 'UI Modu Değiştirildi: AVCI DRONE Seçili');
    }
});

// Helper: Add Event Log
function addLog(source, message, level='info') {
    const el = document.createElement('div');
    el.className = `log-entry ${level}`;

    const d  = new Date();
    const hh = d.getHours().toString().padStart(2,'0');
    const mm = d.getMinutes().toString().padStart(2,'0');
    const ss = d.getSeconds().toString().padStart(2,'0');
    const ms = d.getMilliseconds().toString().padStart(3,'0');
    const t  = `${hh}:${mm}:${ss}.${ms}`;

    el.innerHTML = `
        <span class="log-time">[${t}]</span>
        <span class="log-src">[${source}]</span>
        <span class="log-msg">${message}</span>
    `;

    // Kullanıcı manuel scroll yaptıysa onu bozma
    // Eğer zaten en alttaysa (veya kullanıcı hiç scroll yapmadıysa) → auto-scroll
    const isAtBottom = eventLog.scrollHeight - eventLog.scrollTop - eventLog.clientHeight < 40;

    eventLog.appendChild(el); // en yeni ALTA

    if (isAtBottom) {
        eventLog.scrollTop = eventLog.scrollHeight;
    }

    // Max 100 satır tut
    while (eventLog.children.length > 100) {
        eventLog.removeChild(eventLog.firstChild);
    }
}

// Initial Log
addLog('SYS', 'Y.K.İ. Başlatıldı. Sistem dinleniyor...', 'info');

// === WebSocket ====
let lastHbTime = Date.now();

function connectWebSocket() {
    const wsUrl = `ws://${window.location.host}/ws`;
    const ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        connDot.className = 'dot green';
        connText.textContent = 'LINK ACTIVE';
        connText.style.color = 'var(--success-green)';
        addLog('NET', 'Local Gazebo Sim Bağlantısı Kuruldu', 'info');
        netLatency.textContent = '12';
        netLoss.textContent = '0.0';
    };

    ws.onclose = () => {
        connDot.className = 'dot red';
        connText.textContent = 'LINK LOST';
        connText.style.color = 'var(--danger-red)';
        addLog('NET', 'Bağlantı Koptu! Yeniden bağlanılıyor...', 'crit');
        setTimeout(connectWebSocket, 1000);
    };

    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        lastHbTime = Date.now();
        updateTelemetry(data);
    };
}

// Heartbeat updater
setInterval(() => {
    const diff_sec = ((Date.now() - lastHbTime) / 1000).toFixed(1);
    netHb.textContent = diff_sec;
    if(diff_sec > 2.0 && connDot.classList.contains('green')) {
        addLog('NET', 'Veri akışı gecikmesi: ' + diff_sec + 's', 'warn');
    }
}, 500);

// Update Telemetry Panel
function updateTelemetry(data) {
    if(data.iris) updateAvci(data.iris);
    if(data.plane) updateHedef(data.plane);
    // Chase modunda konum bilgilerini güncelle
    if(data.iris && data.plane) updateChasePositions(data.plane, data.iris);
}

function updateAvci(drone) {
    const spd = (drone.speed !== undefined) ? drone.speed.toFixed(1) + ' m/s' : '--';
    document.getElementById('tele-hunter-speed').textContent = spd;
    document.getElementById('tele-hunter-alt').textContent  = (-drone.z).toFixed(1) + ' m';
    document.getElementById('tele-hunter-pos').textContent  = `${drone.x.toFixed(1)}, ${drone.y.toFixed(1)}`;
    document.getElementById('tele-hunter-hdg').textContent  = `${drone.yaw.toFixed(0)}°`;
}

function updateHedef(plane) {
    // Sağ panel — HEDEF İHA METRİKLERİ
    const spd = document.getElementById('tele-plane-speed');
    const alt = document.getElementById('tele-plane-alt');
    const pos = document.getElementById('tele-plane-pos');
    const hdg = document.getElementById('tele-plane-hdg');
    if (spd) spd.textContent = (plane.speed !== undefined ? plane.speed.toFixed(1) : '0.0') + ' m/s';
    if (alt) alt.textContent = (-plane.z).toFixed(1) + ' m';
    if (pos) pos.textContent = `${plane.x.toFixed(1)}, ${plane.y.toFixed(1)}`;
    if (hdg) hdg.textContent = `${plane.yaw.toFixed(0)}°`;
}

// === GÖREV DURUMU — KİLİT & TERMİNAL MOD ===
function updateMissionStatus() {
    fetch('/api/chase_status')
        .then(r => r.json())
        .then(d => {
            const lockEl = document.getElementById('tele-lock-status');
            const termEl = document.getElementById('tele-terminal-status');
            if (!lockEl || !termEl) return;

            if (!d.active) {
                lockEl.textContent = 'YOK';
                lockEl.className = 'val red';
                termEl.textContent = 'BEKLEMEDE';
                termEl.className = 'val warning';
                return;
            }

            const dist = d.distance;

            // Lock: <15m ise KİLİTLENDİ
            if (dist < 15) {
                lockEl.textContent = 'KİLİTLENDİ';
                lockEl.className = 'val green';
            } else if (dist < 30) {
                lockEl.textContent = 'YAKLAŞIYOR';
                lockEl.className = 'val warning';
            } else {
                lockEl.textContent = 'ARAMA';
                lockEl.className = 'val red';
            }

            // Terminal: <5m ise TERMİNAL AKTİF
            if (dist < 5) {
                termEl.textContent = '⚡ TERMİNAL AKTİF';
                termEl.className = 'val red';
            } else if (dist < 10) {
                termEl.textContent = 'VURUŞ HAZIRLIĞI';
                termEl.className = 'val warning';
            } else {
                termEl.textContent = 'TAKİP';
                termEl.className = 'val';
            }
        })
        .catch(() => {});
}
setInterval(updateMissionStatus, 500);


// === FPV HUD Mockup Drawing (DOM Manipulation over REAL VIDEO) ===
let timeSec = 0;

function animateHUD() {
    // Sahte tracking animasyonu devre dışı - gerçek algoritma entegrasyonu bekleniyor
    const tBox = document.getElementById('target-box');
    if (tBox) tBox.classList.add('hidden');
    // Status bar temiz kalsın
    const hudStatus = document.getElementById('hud-status');
    if (hudStatus) { hudStatus.textContent = ''; hudStatus.className = 'lock-status'; }
}


// === Command Buttons ===
function sendCommand(endpoint, logMsg) {
    addLog('CMD', `Sunucuya iletiliyor: ${logMsg}`, 'info');
    fetch(`/api/command/${endpoint}`, { method: 'POST' })
        .then(res => res.json())
        .then(data => {
            if(data.status === 'success') addLog('SYS', `Görev Kabul Edildi: ${logMsg}`, 'success');
            else addLog('ERR', `Hata: ${data.message}`, 'crit');
        })
        .catch(err => addLog('ERR', `Bağlantı Hatası: ${err}`, 'crit'));
}

document.getElementById('btn-plane-square').addEventListener('click', () => {
    sendCommand('plane/square', 'Kare Çiz Görevi');
    // Hız slider bloğunu göster
    const speedBlock = document.getElementById('plane-speed-block');
    if (speedBlock) speedBlock.classList.remove('hidden');
});

// === UÇAK THROTTLE SLIDER ===
const planeThrSlider = document.getElementById('plane-thr-slider');
const planeThrValue  = document.getElementById('plane-thr-value');
let planeThrTimeout  = null;

if (planeThrSlider) {
    planeThrSlider.addEventListener('input', () => {
        const pct = parseInt(planeThrSlider.value, 10);
        planeThrValue.textContent = pct + '%';

        // Debounce: 100ms bekle, sonra POST et
        clearTimeout(planeThrTimeout);
        planeThrTimeout = setTimeout(() => {
            const throttleVal = Math.round(pct * 10); // 0-100 → 0-1000
            fetch('/api/plane_throttle', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({throttle: throttleVal})
            }).catch(() => {});
        }, 100);
    });
}

// === MANUEL MOD & JOYSTICK ===
const btnManual = document.getElementById('btn-plane-manual');
const manualBlock = document.getElementById('manual-control-block');
const joystickBase = document.getElementById('joystick-base');
const joystickKnob = document.getElementById('joystick-knob');

let manualActive = false;
let jsX = 0; // -1..1 (roll / aileron)
let jsY = 0; // -1..1 (pitch / elevator)
let throttle = 0; // 0..100
let isDragging = false;
let sendInterval = null;

btnManual.addEventListener('click', async () => {
    if (!manualActive) {
        // Manuel mod AÇ
        btnManual.textContent = '⏳ BAĞLANIYOR...';
        btnManual.disabled = true;
        addLog('SYS', 'Kare scripti durduruluyor, Plane MANUAL moda alınıyor...', 'warn');
        
        try {
            const res = await fetch('/api/command/plane/start_manual', { method: 'POST' });
            const data = await res.json();
            if (data.status === 'success') {
                manualActive = true;
                manualBlock.classList.remove('hidden');
                btnManual.textContent = '✖ MANUEL KAPAT';
                btnManual.style.borderLeftColor = 'var(--danger-red)';
                addLog('SYS', 'Manuel Mod AKTİF! Joystick ve W tuşu aktive edildi.', 'warn');
                sendInterval = setInterval(sendManualCommand, 100); // 10 Hz
            } else {
                addLog('ERR', 'Manuel mod başlatılamadı: ' + data.message, 'crit');
            }
        } catch(e) {
            addLog('ERR', 'Bağlantı hatası: ' + e, 'crit');
        }
        btnManual.disabled = false;
    } else {
        // Manuel mod KAPAT
        manualActive = false;
        manualBlock.classList.add('hidden');
        btnManual.textContent = 'MANUEL MOD';
        btnManual.style.borderLeftColor = '';
        clearInterval(sendInterval);
        jsX = 0; jsY = 0; throttle = 0;
        updateJoystickUI(0, 0);
        addLog('SYS', 'Manuel Mod KAPALI. Throttle sıfırlandı.', 'info');
        fetch('/api/command/plane/stop_manual', { method: 'POST' }).catch(() => {});
    }
});

// Joystick Geometry
function getJoystickPos(e) {
    const rect = joystickBase.getBoundingClientRect();
    const cx = rect.left + rect.width  / 2;
    const cy = rect.top  + rect.height / 2;
    const r  = rect.width / 2;
    let dx = ((e.clientX || e.touches[0].clientX) - cx);
    let dy = ((e.clientY || e.touches[0].clientY) - cy);
    // Sınırla
    const dist = Math.sqrt(dx*dx + dy*dy);
    if (dist > r) { dx = dx/dist*r; dy = dy/dist*r; }
    return { dx, dy, r };
}

function updateJoystickUI(dx, dy) {
    const r = joystickBase.getBoundingClientRect().width / 2;
    joystickKnob.style.left = `calc(50% + ${dx}px)`;
    joystickKnob.style.top  = `calc(50% + ${dy}px)`;
    jsX = +(dx / r).toFixed(3);  // -1..1
    jsY = -(dy / r).toFixed(3);  // -1..1 (y ekranı ters)
    document.getElementById('js-x').textContent = jsX.toFixed(2);
    document.getElementById('js-y').textContent = jsY.toFixed(2);
}

joystickBase.addEventListener('mousedown', (e) => {
    if (!manualActive) return;
    isDragging = true;
    joystickKnob.classList.add('active');
    const pos = getJoystickPos(e);
    updateJoystickUI(pos.dx, pos.dy);
});

window.addEventListener('mousemove', (e) => {
    if (!isDragging || !manualActive) return;
    const pos = getJoystickPos(e);
    updateJoystickUI(pos.dx, pos.dy);
});

window.addEventListener('mouseup', () => {
    if (!isDragging) return;
    isDragging = false;
    joystickKnob.classList.remove('active');
    // Joystick merkeze dön
    updateJoystickUI(0, 0);
});

// Touch support
joystickBase.addEventListener('touchstart', (e) => {
    if (!manualActive) return;
    isDragging = true;
    joystickKnob.classList.add('active');
    const pos = getJoystickPos(e);
    updateJoystickUI(pos.dx, pos.dy);
}, { passive: true });

window.addEventListener('touchmove', (e) => {
    if (!isDragging || !manualActive) return;
    const pos = getJoystickPos(e);
    updateJoystickUI(pos.dx, pos.dy);
}, { passive: true });

window.addEventListener('touchend', () => {
    if (!isDragging) return;
    isDragging = false;
    joystickKnob.classList.remove('active');
    updateJoystickUI(0, 0);
});

// W key Throttle
const keysDown = {};

window.addEventListener('keydown', (e) => {
    if (!manualActive) return;
    keysDown[e.key.toLowerCase()] = true;
    if (e.key.toLowerCase() === 'w') {
        throttle = Math.min(100, throttle + 5);
        document.getElementById('js-thr').textContent = throttle;
    }
    if (e.key.toLowerCase() === 's') {  // S ile gaz kıs
        throttle = Math.max(0, throttle - 5);
        document.getElementById('js-thr').textContent = throttle;
    }
});

window.addEventListener('keyup', (e) => {
    keysDown[e.key.toLowerCase()] = false;
});

function sendManualCommand() {
    if (!manualActive) return;
    // MAVLink PWM aralığı: 1000-2000, merkez 1500
    const aileron  = Math.round(1500 + jsX * 500);   // Roll
    const elevator = Math.round(1500 + jsY * 500);   // Pitch
    const thr      = Math.round(1000 + throttle * 10); // Throttle (0%=1000, 100%=2000)
    fetch('/api/command/plane/manual', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ aileron, elevator, throttle: thr })
    }).catch(() => {}); // Sessiz hata
}

// =====================================================
// CHASE MODE (TAKİP MODU)
// =====================================================
let chaseActive = false;
const btnChase = document.getElementById('btn-chase');

btnChase.addEventListener('click', async () => {
    if (!chaseActive) {
        // START CHASE
        btnChase.textContent = '⏳ KALKIŞ YAPILIYOR...';
        btnChase.disabled = true;
        addLog('CMD', 'Takip modu başlatılıyor — Iris kalkış yapacak...', 'warn');

        try {
            const res = await fetch('/api/command/iris/start_chase', { method: 'POST' });
            const data = await res.json();
            if (data.status === 'success') {
                chaseActive = true;
                btnChase.textContent = '⛔ GÖREVİ DURDUR';
                btnChase.className = 't-btn btn-danger block-btn';
                document.getElementById('chase-status').textContent = 'AKTİF';
                document.getElementById('chase-status').className = 'val green';
                addLog('SYS', '✓ Takip modu aktif! Drone hedef İHA\'yı takip ediyor.', 'success');
                // Mesafe güncelleme döngüsünü başlat
                startChaseStatusPolling();
            } else {
                addLog('ERR', 'Takip başlatılamadı: ' + data.message, 'crit');
            }
        } catch(e) {
            addLog('ERR', 'Chase bağlantı hatası: ' + e, 'crit');
        }
        btnChase.disabled = false;
    } else {
        // STOP CHASE
        chaseActive = false;
        btnChase.textContent = '🚀 GÖREVİ BAŞLAT';
        btnChase.className = 't-btn btn-success block-btn';
        document.getElementById('chase-status').textContent = 'DURDURULDU';
        document.getElementById('chase-status').className = 'val warning';
        addLog('SYS', 'Takip modu durduruldu. Drone hover\'a geçiyor.', 'info');
        fetch('/api/command/iris/stop_chase', { method: 'POST' }).catch(() => {});
        stopChaseStatusPolling();
    }
});

// Chase mesafe takibi (1 Hz polling)
let chaseStatusInterval = null;

function startChaseStatusPolling() {
    stopChaseStatusPolling(); // Öncekini temizle
    chaseStatusInterval = setInterval(async () => {
        try {
            const res = await fetch('/api/chase_status');
            const data = await res.json();
            if (data.active) {
                document.getElementById('chase-dist').textContent = (data.distance + 8).toFixed(1) + ' m';
            } else {
                // Backend chase bitti
                if (chaseActive) {
                    chaseActive = false;
                    btnChase.textContent = '🚀 GÖREVİ BAŞLAT';
                    btnChase.className = 't-btn btn-success block-btn';
                    document.getElementById('chase-status').textContent = 'BİTTİ';
                    document.getElementById('chase-status').className = 'val warning';
                    stopChaseStatusPolling();
                }
            }
        } catch(e) { /* sessiz */ }
    }, 1000);
}

function stopChaseStatusPolling() {
    if (chaseStatusInterval) {
        clearInterval(chaseStatusInterval);
        chaseStatusInterval = null;
    }
}

// Chase pozisyon bilgisi — artık sağ paneldeki veriler updateAvci/updateHedef
// ile beslendiğinden bu gereksiz, ama çağrı referansı bozulmasın diye bırakıyoruz
function updateChasePositions(plane, iris) {
    // artık sol panelde konum satırı yok — noop
}

// Init
window.onload = () => {
    connectWebSocket();
    animateHUD();

    // === GPS KARIŞTIRMA SLIDER (DOM hazır olduğunda bağlan) ===
    const slider = document.getElementById('gps-jam-slider');
    const valEl  = document.getElementById('gps-jam-value');
    const statEl = document.getElementById('gps-jam-status');
    let debounce = null;

    if (slider) {
        slider.addEventListener('input', () => {
            const val = parseInt(slider.value);
            valEl.textContent = val + '%';

            // Slider track dolurma
            slider.style.setProperty('--fill', val + '%');

            if (val === 0) {
                valEl.className = 'gps-jam-value green';
                statEl.innerHTML = '<span class="dot green"></span> GPS Sinyali Normal';
            } else if (val < 30) {
                valEl.className = 'gps-jam-value yellow';
                statEl.innerHTML = '<span class="dot yellow"></span> Hafif Parazit &mdash; &plusmn;' + Math.round(val * 0.2) + 'm gürültü';
            } else if (val < 70) {
                valEl.className = 'gps-jam-value orange';
                statEl.innerHTML = '<span class="dot orange"></span> Orta Kariştirma &mdash; veri dalgalanıyor';
            } else if (val < 100) {
                valEl.className = 'gps-jam-value red';
                statEl.innerHTML = '<span class="dot red"></span> Şiddetli Kariştirma &mdash; spoofing aktif';
            } else {
                valEl.className = 'gps-jam-value red blink';
                statEl.innerHTML = '<span class="dot red blink"></span> GPS KAYBI &mdash; veri donmuş!';
            }

            clearTimeout(debounce);
            debounce = setTimeout(() => {
                fetch('/api/gps_noise', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({ level: val / 100.0 })
                }).then(r => r.json()).then(() => {
                    addLog('GPS', 'Kariştirma: %' + val, val > 50 ? 'crit' : val > 0 ? 'warn' : 'info');
                }).catch(() => {});
            }, 120);
        });
    }

    // === VIDEO PARAZİT SLIDER ===
    const vSlider = document.getElementById('video-noise-slider');
    const vValEl  = document.getElementById('video-noise-value');
    const vStatEl = document.getElementById('video-noise-status');
    let vDebounce = null;
    let videoNoiseLevel = 0;

    if (vSlider) {
        vSlider.addEventListener('input', () => {
            const val = parseInt(vSlider.value);
            videoNoiseLevel = val / 100.0;
            vValEl.textContent = val + '%';

            if (val === 0) {
                vValEl.className = 'gps-jam-value green';
                vStatEl.innerHTML = '<span class="dot green"></span>&nbsp;Temiz Analog Sinyal';
            } else if (val < 25) {
                vValEl.className = 'gps-jam-value yellow';
                vStatEl.innerHTML = '<span class="dot yellow"></span>&nbsp;Hafif Parazit — gürültü başlıyor';
            } else if (val < 60) {
                vValEl.className = 'gps-jam-value orange';
                vStatEl.innerHTML = '<span class="dot orange"></span>&nbsp;Orta Parazit — görüntü bozuluyor';
            } else if (val < 100) {
                vValEl.className = 'gps-jam-value red';
                vStatEl.innerHTML = '<span class="dot red"></span>&nbsp;Şiddetli Parazit — sinyal zayıf!';
            } else {
                vValEl.className = 'gps-jam-value red blink';
                vStatEl.innerHTML = '<span class="dot red blink"></span>&nbsp;SINYAL YOK — ekran siyah!';
            }

            clearTimeout(vDebounce);
            vDebounce = setTimeout(() => {
                fetch('/api/video_noise', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({ level: videoNoiseLevel })
                }).then(r => r.json()).then(() => {
                    addLog('VID', 'Video parazit: %' + val, val > 50 ? 'crit' : val > 0 ? 'warn' : 'info');
                }).catch(() => {});
            }, 120);
        });
    }

    // === VIDEO NOISE CANVAS ANİMASYONU (tarama çizgileri efekti) ===
    const noiseCanvas = document.getElementById('video-noise-canvas');
    function resizeNoiseCanvas() {
        if (!noiseCanvas) return;
        const parent = noiseCanvas.parentElement;
        noiseCanvas.width  = parent.offsetWidth  || 640;
        noiseCanvas.height = parent.offsetHeight || 360;
    }
    resizeNoiseCanvas();
    window.addEventListener('resize', resizeNoiseCanvas);

    function drawNoise() {
        if (!noiseCanvas) return requestAnimationFrame(drawNoise);
        const ctx = noiseCanvas.getContext('2d');
        const W = noiseCanvas.width, H = noiseCanvas.height;
        const lvl = videoNoiseLevel;

        ctx.clearRect(0, 0, W, H);

        if (lvl >= 1.0) {
            ctx.fillStyle = '#000';
            ctx.fillRect(0, 0, W, H);
            requestAnimationFrame(drawNoise);
            return;
        }
        if (lvl <= 0) {
            requestAnimationFrame(drawNoise);
            return;
        }

        // Rastgele piksel gürültüsü
        if (lvl > 0.05) {
            const imgData = ctx.createImageData(W, H);
            const d = imgData.data;
            const prob = lvl * 0.35;
            for (let i = 0; i < d.length; i += 4) {
                if (Math.random() < prob) {
                    const v = Math.random() * 255 | 0;
                    d[i] = v; d[i+1] = v; d[i+2] = v;
                    d[i+3] = Math.min(255, lvl * 230 | 0);
                }
            }
            ctx.putImageData(imgData, 0, 0);
        }

        // Yatay tarama çizgileri (analog CRT efekti)
        if (lvl > 0.25) {
            const nLines = Math.floor(lvl * 20);
            for (let i = 0; i < nLines; i++) {
                const y = Math.random() * H;
                const lh = Math.random() * lvl * 4 + 1;
                const r = Math.random()*255|0, g = Math.random()*255|0, b = Math.random()*255|0;
                ctx.fillStyle = `rgba(${r},${g},${b},${lvl * 0.85})`;
                ctx.fillRect(0, y, W, lh);
            }
        }

        // Üst karartma (yüksek parazitte solar)
        if (lvl > 0.65) {
            const alpha = (lvl - 0.65) * 2.2;
            ctx.fillStyle = `rgba(0,0,0,${Math.min(alpha, 0.92)})`;
            ctx.fillRect(0, 0, W, H);
        }

        requestAnimationFrame(drawNoise);
    }
    drawNoise();

    // === PnP POSE TAHMİNİ — Kamera tabanlı telemetri güncelleme ===
    const pnpDist   = document.getElementById('pnp-dist');
    const pnpSpeed  = document.getElementById('pnp-speed');
    const pnpPos    = document.getElementById('pnp-pos');
    const pnpAccel  = document.getElementById('pnp-accel');
    const pnpYaw    = document.getElementById('pnp-yaw');
    const pnpModel  = document.getElementById('pnp-model-status');

    function updatePnPTelemetry() {
        fetch('/api/telemetry/pnp')
            .then(r => r.json())
            .then(d => {
                if (d && d.active) {
                    pnpDist.textContent  = d.distance.toFixed(1) + ' m';
                    pnpSpeed.textContent = d.speed.toFixed(1) + ' m/s';
                    pnpPos.textContent   = d.x.toFixed(1) + ', ' + d.y.toFixed(1) + ', ' + d.z.toFixed(1);
                    pnpAccel.textContent = d.accel.toFixed(2) + ' m/s²';
                    pnpYaw.textContent   = d.yaw.toFixed(1) + '°';

                    pnpModel.textContent = 'AKTİF';
                    pnpModel.className   = 'val green';

                    pnpDist.className  = d.distance < 10 ? 'val red' : d.distance < 30 ? 'val warning' : 'val green';
                    pnpSpeed.className = 'val';
                } else {
                    // Model henüz aktif değil
                    pnpModel.textContent = 'BEKLEMEDE';
                    pnpModel.className   = 'val warning';
                }
            })
            .catch(() => {
                // API endpoint henüz yok — pose modeli gelince aktifleşecek
                pnpModel.textContent = 'BEKLEMEDE';
                pnpModel.className   = 'val warning';
            });
    }
    setInterval(updatePnPTelemetry, 1000);

};
