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
    // 3B konum grafiği izleri
    if(data.iris) recordTrail('iris', data.iris);
    if(data.plane) recordTrail('plane', data.plane);
    // Chase modunda konum bilgilerini güncelle
    if(data.iris && data.plane) updateChasePositions(data.plane, data.iris);
}

// =====================================================
// 3B KONUM İZLEME — sağ panel alt grafik
// NED telemetri (x=Kuzey, y=Doğu, z=Aşağı) → dünya (E, N, YUKARI=-z).
// Harici kütüphane yok: ortografik projeksiyon + azimut/yükseliş döndürme.
// =====================================================
const p3dCanvas = document.getElementById('pos3d-canvas');
const P3D_TRAIL_MAX = 350;            // ~35 sn iz @ 10 Hz
const p3dTrails = { iris: [], plane: [] };
let p3dAzim = -0.8;                   // radyan — sürükleyerek değişir
let p3dElev = 1.0;                    // 0.15 (yandan) .. 1.5 (tepeden)
let p3dDrag = null;
// Otomatik çerçeveleme yumuşatması (grafik zıplamasın)
const p3dView = { cx: 0, cy: 0, cz: 0, span: 40, init: false };

function recordTrail(name, v) {
    if (v.x === 0 && v.y === 0 && v.z === 0) return;  // telemetri henüz yok
    const arr = p3dTrails[name];
    const last = arr[arr.length - 1];
    if (last && last.x === v.x && last.y === v.y && last.z === v.z) return;
    arr.push({ x: v.x, y: v.y, z: v.z });
    if (arr.length > P3D_TRAIL_MAX) arr.shift();
}

function p3dNiceStep(raw) {
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const n = raw / mag;
    return (n < 1.5 ? 1 : n < 3.5 ? 2 : n < 7.5 ? 5 : 10) * mag;
}

function p3dRender() {
    requestAnimationFrame(p3dRender);
    if (!p3dCanvas) return;
    const dpr = window.devicePixelRatio || 1;
    const w = p3dCanvas.clientWidth, h = p3dCanvas.clientHeight;
    if (w === 0 || h === 0) return;
    if (p3dCanvas.width !== Math.round(w * dpr)) {
        p3dCanvas.width = Math.round(w * dpr);
        p3dCanvas.height = Math.round(h * dpr);
    }
    const ctx = p3dCanvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const all = p3dTrails.iris.concat(p3dTrails.plane);
    if (all.length === 0) {
        ctx.fillStyle = '#475569';
        ctx.font = '11px Roboto Mono, monospace';
        ctx.textAlign = 'center';
        ctx.fillText('TELEMETRİ BEKLENİYOR...', w / 2, h / 2);
        return;
    }

    // ---- Hedef çerçeve: tüm noktaları kapsa (dünya: E=y, N=x, U=-z) ----
    let mnE = 1e9, mxE = -1e9, mnN = 1e9, mxN = -1e9, mnU = 1e9, mxU = -1e9;
    for (const p of all) {
        const E = p.y, N = p.x, U = -p.z;
        if (E < mnE) mnE = E; if (E > mxE) mxE = E;
        if (N < mnN) mnN = N; if (N > mxN) mxN = N;
        if (U < mnU) mnU = U; if (U > mxU) mxU = U;
    }
    mnU = Math.min(mnU, 0);                       // zemin hep görünsün
    const tgtCx = (mnE + mxE) / 2, tgtCy = (mnN + mxN) / 2, tgtCz = (mnU + mxU) / 2;
    const tgtSpan = Math.max(30, mxE - mnE, mxN - mnN, mxU - mnU) * 1.15;
    if (!p3dView.init) {
        p3dView.cx = tgtCx; p3dView.cy = tgtCy; p3dView.cz = tgtCz;
        p3dView.span = tgtSpan; p3dView.init = true;
    } else {
        const a = 0.06;                           // yumuşak takip
        p3dView.cx += (tgtCx - p3dView.cx) * a;
        p3dView.cy += (tgtCy - p3dView.cy) * a;
        p3dView.cz += (tgtCz - p3dView.cz) * a;
        p3dView.span += (tgtSpan - p3dView.span) * a;
    }

    const scale = Math.min(w, h) * 0.72 / p3dView.span;
    const cosA = Math.cos(p3dAzim), sinA = Math.sin(p3dAzim);
    const cosE = Math.cos(p3dElev), sinE = Math.sin(p3dElev);
    const scx = w / 2, scy = h / 2 + h * 0.06;

    // NED nokta → ekran. Dünya eksenleri: X=E(doğu) Y=N(kuzey) Z=U(yukarı)
    function proj(ned) {
        const X = ned.y - p3dView.cx;
        const Y = ned.x - p3dView.cy;
        const Z = -ned.z - p3dView.cz;
        const x1 = X * cosA - Y * sinA;
        const y1 = X * sinA + Y * cosA;
        return {
            x: scx + x1 * scale,
            y: scy - (y1 * sinE + Z * cosE) * scale,
        };
    }
    const projW = (E, N, U) => proj({ x: N, y: E, z: -U });

    // ---- Zemin ızgarası (U=0) ----
    const step = p3dNiceStep(p3dView.span / 4);
    const gE0 = Math.floor((p3dView.cx - p3dView.span / 2) / step) * step;
    const gE1 = Math.ceil((p3dView.cx + p3dView.span / 2) / step) * step;
    const gN0 = Math.floor((p3dView.cy - p3dView.span / 2) / step) * step;
    const gN1 = Math.ceil((p3dView.cy + p3dView.span / 2) / step) * step;
    ctx.strokeStyle = 'rgba(42, 49, 61, 0.9)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let E = gE0; E <= gE1 + 0.001; E += step) {
        const a1 = projW(E, gN0, 0), a2 = projW(E, gN1, 0);
        ctx.moveTo(a1.x, a1.y); ctx.lineTo(a2.x, a2.y);
    }
    for (let N = gN0; N <= gN1 + 0.001; N += step) {
        const a1 = projW(gE0, N, 0), a2 = projW(gE1, N, 0);
        ctx.moveTo(a1.x, a1.y); ctx.lineTo(a2.x, a2.y);
    }
    ctx.stroke();

    // Eksen okları + etiketler (ızgara köşesinden)
    const axLen = step;
    const o = projW(gE0, gN0, 0);
    ctx.font = '9px Roboto Mono, monospace';
    ctx.textAlign = 'center';
    const axes = [
        { p: projW(gE0 + axLen, gN0, 0), label: 'D', color: '#798696' },  // doğu
        { p: projW(gE0, gN0 + axLen, 0), label: 'K', color: '#798696' },  // kuzey
        { p: projW(gE0, gN0, axLen),     label: 'İRT', color: '#3b82f6' },
    ];
    for (const ax of axes) {
        ctx.strokeStyle = ax.color; ctx.fillStyle = ax.color;
        ctx.beginPath(); ctx.moveTo(o.x, o.y); ctx.lineTo(ax.p.x, ax.p.y); ctx.stroke();
        ctx.fillText(ax.label, ax.p.x, ax.p.y - 3);
    }
    // Izgara adım bilgisi
    ctx.fillStyle = '#475569';
    ctx.textAlign = 'left';
    ctx.fillText(`ızgara: ${step}m`, 6, h - 6);

    // ---- İzler + araçlar ----
    const vehicles = [
        { key: 'iris',  color: '16, 185, 129', label: 'AVCI' },
        { key: 'plane', color: '239, 68, 68',  label: 'HEDEF' },
    ];
    for (const v of vehicles) {
        const tr = p3dTrails[v.key];
        if (tr.length === 0) continue;
        // İz: eskiden yeniye solarak
        for (let i = 1; i < tr.length; i++) {
            const alpha = Math.pow(i / tr.length, 1.4) * 0.85;
            const p1 = proj(tr[i - 1]), p2 = proj(tr[i]);
            ctx.strokeStyle = `rgba(${v.color}, ${alpha.toFixed(3)})`;
            ctx.lineWidth = 1.4;
            ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
        }
        // Güncel nokta: zemine dikme (derinlik algısı) + dot + etiket
        const cur = tr[tr.length - 1];
        const pc = proj(cur);
        const pg = projW(cur.y, cur.x, 0);
        ctx.strokeStyle = `rgba(${v.color}, 0.35)`;
        ctx.setLineDash([3, 3]);
        ctx.beginPath(); ctx.moveTo(pc.x, pc.y); ctx.lineTo(pg.x, pg.y); ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = `rgba(${v.color}, 0.5)`;
        ctx.beginPath(); ctx.arc(pg.x, pg.y, 2, 0, 2 * Math.PI); ctx.fill();
        ctx.fillStyle = `rgb(${v.color})`;
        ctx.beginPath(); ctx.arc(pc.x, pc.y, 4, 0, 2 * Math.PI); ctx.fill();
        ctx.textAlign = 'left';
        ctx.font = '9px Roboto Mono, monospace';
        ctx.fillText(`${v.label} ${(-cur.z).toFixed(0)}m`, pc.x + 7, pc.y - 4);
    }
}

// Sürükleyerek döndürme
if (p3dCanvas) {
    p3dCanvas.addEventListener('mousedown', (e) => {
        p3dDrag = { x: e.clientX, y: e.clientY };
        e.preventDefault();
    });
    window.addEventListener('mousemove', (e) => {
        if (!p3dDrag) return;
        p3dAzim += (e.clientX - p3dDrag.x) * 0.01;
        p3dElev = Math.max(0.15, Math.min(1.5, p3dElev + (e.clientY - p3dDrag.y) * 0.008));
        p3dDrag = { x: e.clientX, y: e.clientY };
    });
    window.addEventListener('mouseup', () => { p3dDrag = null; });
    p3dRender();   // çizim döngüsünü başlat
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

// === UÇUŞ SENARYOLARI (kare / daire / agresif) ===
// Her buton: takeoff + desen. Aktif senaryonun butonuna tekrar basmak durdurur.
// Manuel mod butonu ise uçuşu klavye kontrolüne devralır.
const SCN_LABELS = {
    square:     '▢ KARE ÇİZ',
    circle:     '◯ DAİRE ÇİZ',
    aggressive: '⚡ AGRESİF UÇUŞ',
};
const scnButtons = {
    square:     document.getElementById('btn-scn-square'),
    circle:     document.getElementById('btn-scn-circle'),
    aggressive: document.getElementById('btn-scn-aggressive'),
};
let activeScenario = null;

function markScenarioButtons() {
    for (const [name, btn] of Object.entries(scnButtons)) {
        if (!btn) continue;
        if (name === activeScenario) {
            btn.classList.add('scn-active');
            btn.textContent = '⛔ DURDUR — ' + SCN_LABELS[name].slice(2);
        } else {
            btn.classList.remove('scn-active');
            btn.textContent = SCN_LABELS[name];
        }
    }
    // Hız slider'ı yalnızca bir senaryo uçarken anlamlı
    const speedBlock = document.getElementById('plane-speed-block');
    if (speedBlock) speedBlock.classList.toggle('hidden', !activeScenario);
}

async function startScenario(name) {
    if (manualActive) await exitManualMode();
    addLog('CMD', 'Senaryo başlatılıyor: ' + SCN_LABELS[name] + ' (takeoff + desen)', 'info');
    try {
        const res = await fetch('/api/command/plane/scenario/' + name, { method: 'POST' });
        const data = await res.json();
        if (data.status === 'success') {
            activeScenario = name;
            addLog('SYS', '✓ ' + SCN_LABELS[name] + ' aktif — araç kalkış yapıp desene başlayacak.', 'success');
        } else {
            addLog('ERR', 'Senaryo hatası: ' + data.message, 'crit');
        }
    } catch (e) {
        addLog('ERR', 'Bağlantı hatası: ' + e, 'crit');
    }
    markScenarioButtons();
}

async function stopScenario() {
    if (activeScenario) addLog('SYS', 'Senaryo durduruluyor: ' + SCN_LABELS[activeScenario], 'warn');
    activeScenario = null;
    markScenarioButtons();
    try { await fetch('/api/command/plane/stop_scenario', { method: 'POST' }); } catch (e) {}
}

for (const [name, btn] of Object.entries(scnButtons)) {
    if (!btn) continue;
    btn.addEventListener('click', () => {
        if (activeScenario === name) stopScenario();
        else startScenario(name);
    });
}

// Senaryo süreci arka planda kendi kendine sonlanırsa butonları senkronize et
setInterval(async () => {
    try {
        const res = await fetch('/api/scenario_status');
        const d = await res.json();
        const backend = d.active ? d.name : null;
        if (backend !== activeScenario) {
            activeScenario = backend;
            markScenarioButtons();
        }
    } catch (e) { /* sessiz */ }
}, 2000);

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

// === MANUEL MOD — JOYSTICK (mouse) + KLAVYE (W/S: pitch, A/D: roll, L/I: gaz) ===
// Basıldığında aktif senaryo durur, uçuş FBWA'da devralınır.
const btnManual = document.getElementById('btn-plane-manual');
const manualBlock = document.getElementById('manual-control-block');
const joystickBase = document.getElementById('joystick-base');
const joystickKnob = document.getElementById('joystick-knob');

let manualActive = false;
let keysDown = {};
let mAil = 0;      // -1..1 yumuşatılmış roll komutu
let mElv = 0;      // -1..1 yumuşatılmış pitch komutu
let mThr = 60;     // % gaz — cruise'dan başlar (havada devralınca stall olmasın)
let manualLoop = null;
let manualSendTick = 0;
let isDragging = false;
let jsAil = 0;     // -1..1 joystick hedefi (sürükleme sırasında)
let jsElv = 0;

btnManual.addEventListener('click', async () => {
    if (!manualActive) await enterManualMode();
    else await exitManualMode();
});

async function enterManualMode() {
    btnManual.textContent = '⏳ BAĞLANIYOR...';
    btnManual.disabled = true;
    addLog('SYS', 'Aktif senaryo durduruluyor, uçuş devralınıyor (FBWA)...', 'warn');
    activeScenario = null;
    markScenarioButtons();

    try {
        const res = await fetch('/api/command/plane/start_manual', { method: 'POST' });
        const data = await res.json();
        if (data.status === 'success') {
            manualActive = true;
            keysDown = {}; mAil = 0; mElv = 0; mThr = 60;
            isDragging = false; jsAil = 0; jsElv = 0;
            setKnob(0, 0);
            manualBlock.classList.remove('hidden');
            btnManual.textContent = '✖ MANUEL KAPAT';
            btnManual.style.borderLeftColor = 'var(--danger-red)';
            addLog('SYS', 'Manuel Mod AKTİF — W/S: pitch, A/D: roll, L: hızlan, I: yavaşla', 'warn');
            manualLoop = setInterval(manualTick, 50); // 20 Hz iç döngü
        } else {
            btnManual.textContent = '🕹 MANUEL MOD';
            addLog('ERR', 'Manuel mod başlatılamadı: ' + data.message, 'crit');
        }
    } catch(e) {
        btnManual.textContent = '🕹 MANUEL MOD';
        addLog('ERR', 'Bağlantı hatası: ' + e, 'crit');
    }
    btnManual.disabled = false;
}

async function exitManualMode() {
    manualActive = false;
    clearInterval(manualLoop);
    manualLoop = null;
    isDragging = false; jsAil = 0; jsElv = 0;
    joystickKnob.classList.remove('active');
    setKnob(0, 0);
    manualBlock.classList.add('hidden');
    btnManual.textContent = '🕹 MANUEL MOD';
    btnManual.style.borderLeftColor = '';
    addLog('SYS', 'Manuel Mod KAPALI.', 'info');
    try { await fetch('/api/command/plane/stop_manual', { method: 'POST' }); } catch(e) {}
}

// --- Joystick (mouse/touch) ---
function joystickEventPos(e) {
    const rect = joystickBase.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    const r = rect.width / 2;
    const px = (e.clientX !== undefined) ? e.clientX : e.touches[0].clientX;
    const py = (e.clientY !== undefined) ? e.clientY : e.touches[0].clientY;
    let dx = px - cx, dy = py - cy;
    const dist = Math.sqrt(dx * dx + dy * dy);
    if (dist > r) { dx = dx / dist * r; dy = dy / dist * r; }
    jsAil = +(dx / r).toFixed(3);   // -1..1
    jsElv = -(dy / r).toFixed(3);   // -1..1 (ekran y'si ters: yukarı = burun yukarı)
}

function setKnob(ail, elv) {
    const r = joystickBase.getBoundingClientRect().width / 2;
    joystickKnob.style.left = `calc(50% + ${ail * r}px)`;
    joystickKnob.style.top  = `calc(50% + ${-elv * r}px)`;
}

joystickBase.addEventListener('mousedown', (e) => {
    if (!manualActive) return;
    isDragging = true;
    joystickKnob.classList.add('active');
    joystickEventPos(e);
});
window.addEventListener('mousemove', (e) => {
    if (!isDragging || !manualActive) return;
    joystickEventPos(e);
});
window.addEventListener('mouseup', () => {
    if (!isDragging) return;
    isDragging = false;
    jsAil = 0; jsElv = 0;           // bırakınca merkeze dön
    joystickKnob.classList.remove('active');
});
joystickBase.addEventListener('touchstart', (e) => {
    if (!manualActive) return;
    isDragging = true;
    joystickKnob.classList.add('active');
    joystickEventPos(e);
}, { passive: true });
window.addEventListener('touchmove', (e) => {
    if (!isDragging || !manualActive) return;
    joystickEventPos(e);
}, { passive: true });
window.addEventListener('touchend', () => {
    if (!isDragging) return;
    isDragging = false;
    jsAil = 0; jsElv = 0;
    joystickKnob.classList.remove('active');
});

// --- Klavye ---
const MANUAL_KEYS = ['w', 'a', 's', 'd', 'l', 'i'];

window.addEventListener('keydown', (e) => {
    if (!manualActive) return;
    const k = e.key.toLowerCase();
    if (MANUAL_KEYS.includes(k)) {
        keysDown[k] = true;
        e.preventDefault();
    }
});

window.addEventListener('keyup', (e) => {
    keysDown[e.key.toLowerCase()] = false;
});

function manualTick() {
    if (!manualActive) return;
    // Hedef yüzey komutu: joystick sürükleniyorsa joystick, değilse klavye
    let tAil, tElv;
    if (isDragging) {
        tAil = jsAil;
        tElv = jsElv;
    } else {
        tAil = (keysDown['d'] ? 1 : 0) - (keysDown['a'] ? 1 : 0);
        tElv = (keysDown['w'] ? 1 : 0) - (keysDown['s'] ? 1 : 0);
    }
    // Yumuşatma: ani PWM sıçraması yerine ~0.3s'de hedefe ulaşır
    mAil += (tAil - mAil) * 0.25;
    mElv += (tElv - mElv) * 0.25;
    if (tAil === 0 && Math.abs(mAil) < 0.02) mAil = 0;
    if (tElv === 0 && Math.abs(mElv) < 0.02) mElv = 0;
    // L/I: kalıcı gaz seviyesi — basılı tutuldukça artar/azalır
    if (keysDown['l']) mThr = Math.min(100, mThr + 1);
    if (keysDown['i']) mThr = Math.max(0, mThr - 1);

    setKnob(mAil, mElv);   // topuz hem joystick hem klavye girişini yansıtır
    document.getElementById('js-x').textContent = mAil.toFixed(2);
    document.getElementById('js-y').textContent = mElv.toFixed(2);
    document.getElementById('js-thr').textContent = Math.round(mThr);

    // PWM'e çevir — FBWA: tam sapma = maks yatış/pitch açı hedefi
    const aileron  = Math.round(1500 + mAil * 450);
    const elevator = Math.round(1500 + mElv * 450);   // yüksek PWM = burun yukarı (SITL'de doğrulandı)
    const thr      = Math.round(1000 + mThr * 10);

    // Sunucuya 10 Hz gönder (iç döngü 20 Hz — bir atlayarak)
    manualSendTick++;
    if (manualSendTick % 2) return;
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
