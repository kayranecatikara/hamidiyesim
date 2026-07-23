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

// === CANLI KAMERA — WebSocket base64 hattı ===
// MJPEG <img> tarayıcıda takılıp donabiliyor. Bunun yerine kareleri WebSocket
// ile base64 alıp KALICI <img>'in src'sini data-URI ile güncelliyoruz.
// requestAnimationFrame ekrana çizimi tarayıcının repaint'iyle senkronlar →
// her yeni kare anında görünür, buffer'da eski kare takılmaz.
let _videoWs = null;
let _videoCam = 'plane';
let _pendingFrame = null;
let _rafQueued = false;

function _drawPendingFrame() {
    _rafQueued = false;
    if (_pendingFrame == null) return;
    const img = document.getElementById('fpv-stream');
    if (img) img.src = 'data:image/jpeg;base64,' + _pendingFrame;
    _pendingFrame = null;
}

function switchCamera(vehicle) {
    _videoCam = vehicle;
    // önceki video WS'ini kapat (sekme değişimi)
    if (_videoWs) {
        try { _videoWs.onclose = null; _videoWs.close(); } catch (e) {}
        _videoWs = null;
    }
    const proto = (location.protocol === 'https:') ? 'wss' : 'ws';
    const ws = new WebSocket(`${proto}://${location.host}/ws/video/${vehicle}`);
    _videoWs = ws;
    ws.onmessage = (e) => {
        _pendingFrame = e.data;            // yalnızca EN SON kareyi tut
        if (!_rafQueued) { _rafQueued = true; requestAnimationFrame(_drawPendingFrame); }
    };
    ws.onclose = () => {
        // beklenmedik kapanma → aynı kamera için yeniden bağlan (OFFLINE'a düşme)
        if (_videoWs === ws && _videoCam === vehicle) {
            setTimeout(() => { if (_videoCam === vehicle) switchCamera(vehicle); }, 800);
        }
    };
    ws.onerror = () => { try { ws.close(); } catch (e) {} };
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
    // Minimap — simülasyon GPS/pozisyon (NED x,y,z)
    if(data.iris && data.plane) drawMinimap(data.iris, data.plane);
}

// =====================================================
// KONUM 3B HARİTASI — X/Y/Z eksenli ortografik "turntable" izdüşüm.
// (HAMIDIYE_AVCI_DRONE referans web/index.html'den uyarlandı; veri beslemesi
//  bizim sim telemetrimize (data.iris=AVCI, data.plane=HEDEF) bağlandı.)
// SAĞ/SOL-SÜRÜKLE döndür · TEKERLEK zoom · ÇİFT-TIK sıfırla.
// =====================================================
const _mm$ = (id) => document.getElementById(id);
let mmAvci = [], mmHam = [], mmSonYaw = null;
const MM_AZ0 = -90, MM_EL0 = 90;   // ilk görünüş: TOP-DOWN (el=90 → X-Y üstten). Sürükle → Z görünür
let mmAz = MM_AZ0, mmEl = MM_EL0, mmZoom = 1;
// STABİL çerçeveleme durumu (yumuşak takip — küçülme/kayma önlenir)
let mmCx = null, mmCy = null, mmCz = null, mmHalf = null;
const MM_TRAIL = 200;                        // iz uzunluğu (~20 sn) — kısa tutulur
const MM_FIXED_HALF = 180;   // SABİT ölçek (m) — ±180m gösterir; tekerlekle zoom

// updateTelemetry buradan besler
function drawMinimap(iris, plane) {
    // NED z (aşağı+) → irtifa (yukarı+) = -z. Işınlanma/restart'ta izi sıfırla.
    if (iris && iris.x != null) {
        const last = mmAvci[mmAvci.length - 1];
        if (last && Math.hypot(iris.x - last[0], iris.y - last[1]) > 800) mmAvci = [];
        mmAvci.push([iris.x, iris.y, -(iris.z || 0)]);
        if (mmAvci.length > MM_TRAIL) mmAvci.shift();
    }
    if (plane && plane.x != null) {
        const last = mmHam[mmHam.length - 1];
        if (last && Math.hypot(plane.x - last[0], plane.y - last[1]) > 800) mmHam = [];
        mmHam.push([plane.x, plane.y, -(plane.z || 0)]);
        if (mmHam.length > MM_TRAIL) mmHam.shift();
    }
    mmSonYaw = iris ? iris.yaw : null;
    mmCiz();
    // alt bilgi satırı
    const setTxt = (id, v) => { const el = _mm$(id); if (el) el.textContent = v; };
    if (iris)  setTxt('mm-iris-alt',  (-iris.z).toFixed(0) + 'm');
    if (plane) setTxt('mm-plane-alt', (-plane.z).toFixed(0) + 'm');
    if (iris && plane) {
        const dx = plane.x - iris.x, dy = plane.y - iris.y, dz = plane.z - iris.z;
        setTxt('mm-dist', Math.sqrt(dx * dx + dy * dy + dz * dz).toFixed(0) + 'm');
        setTxt('mm-scale', 'sağ-sürükle döndür');
    }
}

function mmCiz() {
    const cv = _mm$('minimap'); if (!cv) return;
    const wrap = cv.parentElement, cw = wrap.clientWidth, ch = wrap.clientHeight;
    if (!cw || !ch) return;
    if (cv.width !== cw) cv.width = cw;
    if (cv.height !== ch) cv.height = ch;
    const ctx = cv.getContext('2d'); ctx.clearRect(0, 0, cw, ch);
    ctx.fillStyle = '#06080c'; ctx.fillRect(0, 0, cw, ch);
    const a = mmAvci.length ? mmAvci[mmAvci.length - 1] : null;
    const h = mmHam.length ? mmHam[mmHam.length - 1] : null;
    if (!a && !h) {
        ctx.fillStyle = '#55627a'; ctx.font = '11px monospace';
        ctx.fillText('harita: veri bekleniyor…', 10, 20); return;
    }
    // ---- ORİGİN SABİT + SABİT ÖLÇEK (takip YOK, kayma YOK, küçülme YOK) ----
    // Merkez daima HOME/ORİGİN (0,0,0)'dadır → görünüm HİÇ kaymaz/oynamaz.
    // Ölçek sabittir → araç uzağa uçsa da küçülmez (gerekirse tekerlekle zoom).
    // Araçlar bu sabit koordinat sisteminin içinde hareket eder.
    const cxm = 0, cym = 0, czm = 0;
    const half = MM_FIXED_HALF;                            // SABİT — küçülme/kayma yok
    // ortografik turntable (Z yukarı)
    const A = mmAz * Math.PI / 180, E = mmEl * Math.PI / 180;
    const cA = Math.cos(A), sA = Math.sin(A), cE = Math.cos(E), sE = Math.sin(E);
    const pad = 16, s = (Math.min(cw, ch) / 2 - pad) / half * mmZoom;
    const P = (x, y, z) => { const px = x - cxm, py = y - cym, pz = z - czm; return [cw / 2 + (py * cA - px * sA) * s, ch / 2 - (pz * cE - (px * cA + py * sA) * sE) * s]; };
    // zemin ızgarası z0 = 0 (yer / home seviyesi)
    const z0 = 0, hedef = 48 / s, p10 = Math.pow(10, Math.floor(Math.log10(hedef)));
    let adim = 10 * p10; for (const m of [1, 2, 5]) { if (m * p10 >= hedef) { adim = m * p10; break; } }
    ctx.strokeStyle = 'rgba(255,255,255,.06)'; ctx.lineWidth = 1; ctx.beginPath();
    for (let gx = Math.ceil((cxm - half) / adim) * adim, n = 0; gx <= cxm + half && n < 80; gx += adim, n++) { const a = P(gx, cym - half, z0), b = P(gx, cym + half, z0); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); }
    for (let gy = Math.ceil((cym - half) / adim) * adim, n = 0; gy <= cym + half && n < 80; gy += adim, n++) { const a = P(cxm - half, gy, z0), b = P(cxm + half, gy, z0); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); }
    ctx.stroke();
    // 3 eksen + X/Y/Z harfleri (X/Y zeminde, Z yukarı)
    const eksen = (x2, y2, z2, renk, ad) => { const o = P(cxm, cym, z0), u = P(x2, y2, z2); ctx.strokeStyle = renk; ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(u[0], u[1]); ctx.stroke(); ctx.fillStyle = renk; ctx.font = '10px monospace'; ctx.fillText(ad, u[0] + 3, u[1] - 2); };
    ctx.lineWidth = 1;
    eksen(cxm + half, cym, z0, 'rgba(255,110,110,.6)', 'X');
    eksen(cxm, cym + half, z0, 'rgba(110,255,170,.5)', 'Y');
    eksen(cxm, cym, z0 + half, 'rgba(110,170,255,.7)', 'Z');
    // son noktadan zemine kesikli irtifa sarkıtı
    const sap = (iz, renk) => { if (!iz.length) return; const p = iz[iz.length - 1]; const a = P(p[0], p[1], p[2]), b = P(p[0], p[1], z0); if ((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) < 4) return; ctx.strokeStyle = renk; ctx.lineWidth = 1; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); ctx.setLineDash([]); };
    sap(mmAvci, 'rgba(37,224,160,.3)'); sap(mmHam, 'rgba(255,93,93,.25)');
    // izler
    const ciz = (iz, renk) => { if (iz.length < 2) return; ctx.strokeStyle = renk; ctx.lineWidth = 1.5; ctx.beginPath(); iz.forEach((p, i) => { const q = P(p[0], p[1], p[2]); i ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1]); }); ctx.stroke(); };
    ciz(mmHam, 'rgba(255,93,93,.4)');
    ciz(mmAvci, 'rgba(37,224,160,.6)');
    // noktalar
    const nokta = (iz, renk, r) => { const p = iz[iz.length - 1], q = P(p[0], p[1], p[2]); ctx.fillStyle = renk; ctx.beginPath(); ctx.arc(q[0], q[1], r, 0, 6.2832); ctx.fill(); };
    if (mmHam.length) nokta(mmHam, '#ff5d5d', 4);
    if (mmAvci.length) {
        const p = mmAvci[mmAvci.length - 1], q = P(p[0], p[1], p[2]);
        nokta(mmAvci, '#25e0a0', 5);
        if (mmSonYaw != null) { const a = mmSonYaw * Math.PI / 180, L = 14 / s; const b = P(p[0] + Math.cos(a) * L, p[1] + Math.sin(a) * L, p[2]); ctx.strokeStyle = '#25e0a0'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(q[0], q[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); }
    }
    // efsane + ölçek
    ctx.font = '10px monospace';
    ctx.fillStyle = '#25e0a0'; ctx.fillText('● AVCI', 8, 14);
    ctx.fillStyle = '#ff5d5d'; ctx.fillText('● HEDEF', 58, 14);
    ctx.fillStyle = '#55627a'; ctx.fillText('~' + Math.round(2 * half / mmZoom) + ' m', cw - 62, ch - 8);
}
function mmZum(k) { mmZoom = Math.min(8, Math.max(0.2, mmZoom * k)); mmCiz(); }
(function () {  // sürükle döndür + tekerlek zoom + çift-tık sıfırla
    const cv = _mm$('minimap'); if (!cv) return;
    const wrap = cv.parentElement; let srk = false, lx = 0, ly = 0, srkSonT = 0;
    cv.addEventListener('pointerdown', e => { srk = true; lx = e.clientX; ly = e.clientY; try { cv.setPointerCapture(e.pointerId); } catch (_) {} e.preventDefault(); });
    cv.addEventListener('pointermove', e => { if (!srk) return; mmAz -= (e.clientX - lx) * 0.4; mmEl -= (e.clientY - ly) * 0.4; mmEl = Math.min(90, Math.max(5, mmEl)); lx = e.clientX; ly = e.clientY; mmCiz(); });
    const birak = () => { if (srk) srkSonT = Date.now(); srk = false; };
    cv.addEventListener('pointerup', birak); cv.addEventListener('pointercancel', birak); cv.addEventListener('pointerleave', birak);
    if (wrap) wrap.addEventListener('contextmenu', e => e.preventDefault());
    window.addEventListener('contextmenu', e => { if (Date.now() - srkSonT < 250) e.preventDefault(); }, true);
    cv.addEventListener('wheel', e => { e.preventDefault(); mmZum(e.deltaY < 0 ? 1.12 : 1 / 1.12); }, { passive: false });
    cv.addEventListener('dblclick', () => { mmAz = MM_AZ0; mmEl = MM_EL0; mmZoom = 1; mmCx = mmCy = mmCz = mmHalf = null; mmCiz(); });
})();

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

// =====================================================
// HEDEF İHA — 4 UÇUŞ SENARYOSU
//   🔵 Daire  = tam otomatik dairesel devriye (X-Y çember, sabit Z)
//   🟧 Kare   = MANUEL uçuş (W/S/A/D + joystick)
//   🔺 Üçgen  = tam otomatik geometrik 3B rota (X-Y-Z)
//   🔲 [R]    = otonom rastgele / kaçış (3B gelişigüzel manevra)
// Backend her modu başlatırken diğerlerini otomatik durdurur; burada sadece
// görsel vurgu + yerel durum senkronu yapıyoruz.
// =====================================================
// Her senaryoyu tetikleyen buton(lar): ŞEKİL çizimi + YAZI butonu birlikte
// (aynı senaryo iki yerden de kontrol edilir, ikisi de vurgulanır).
// Daire / Kare / Üçgen = ŞEKİL çizimleri, hepsi OTOMATİK uçuş.
// Manuel + Rastgele = "AKTİF SENARYO" altındaki YAZI butonları.
const _scnButtons = {
    circle:   ['btn-plane-circle'],     // ○ Daire  (otomatik)
    square:   ['btn-plane-square'],     // □ Kare   (otomatik)
    triangle: ['btn-plane-triangle'],   // △ Üçgen  (otomatik)
    manual:   ['btn-mode-manual'],      // "Manuel Uçuş" yazısı
    random:   ['btn-mode-random'],      // "Rastgele Uçuş" yazısı
};
let activeScenario = null;   // 'circle' | 'square' | 'triangle' | 'manual' | 'random' | null
const _scnLabels = {
    circle:   'OTOMATİK UÇUŞ (daire)',
    square:   'OTOMATİK UÇUŞ (kare)',
    triangle: 'OTOMATİK UÇUŞ (üçgen)',
    manual:   'MANUEL UÇUŞ',
    random:   'RASTGELE UÇUŞ',
};

function setScenarioHighlight(name) {
    activeScenario = name;
    for (const k in _scnButtons) {
        _scnButtons[k].forEach(id => {
            const b = document.getElementById(id);
            if (!b) return;
            if (k === name) {
                b.classList.add('active-scn');
                b.style.borderLeftColor = 'var(--danger-red)';
            } else {
                b.classList.remove('active-scn');
                b.style.borderLeftColor = '';
            }
        });
    }
    const st = document.getElementById('plane-random-status');
    if (st) {
        if (name) { st.textContent = _scnLabels[name]; st.className = 'val green'; }
        else      { st.textContent = 'BEKLEMEDE';      st.className = 'val warning'; }
    }
}

// Geometrik otomatik modlar (Daire, Üçgen) — başlat/durdur toggle
async function toggleGeoScenario(name, startEp, stopEp, logName) {
    if (activeScenario === name) {
        fetch(`/api/command/plane/${stopEp}`, { method: 'POST' }).catch(() => {});
        setScenarioHighlight(null);
        addLog('SYS', `${logName} durduruldu (LOITER).`, 'info');
        return;
    }
    // Başka mod (manuel/random dahil) açıksa yerel UI'yı da kapat
    teardownManualUI(false);
    planeRandomActive = false;
    addLog('CMD', `${logName} başlatılıyor...`, 'info');
    try {
        const res = await fetch(`/api/command/plane/${startEp}`, { method: 'POST' });
        const data = await res.json();
        if (data.status === 'success') {
            setScenarioHighlight(name);
            addLog('SYS', `${logName} AKTİF (otomatik).`, 'success');
        } else {
            addLog('ERR', `Hata: ${data.message}`, 'crit');
        }
    } catch (e) {
        addLog('ERR', 'Bağlantı hatası: ' + e, 'crit');
    }
}

// Geometrik ŞEKİLLER — hepsi otomatik uçuş (Daire ○, Kare □, Üçgen △)
const _geoBtns = [
    { id: 'btn-plane-circle',   scn: 'circle',   ep: 'circle',   stop: 'stop_circle',   log: '🔵 Otomatik daire' },
    { id: 'btn-plane-square',   scn: 'square',   ep: 'square',   stop: 'stop_square',   log: '🟧 Otomatik kare' },
    { id: 'btn-plane-triangle', scn: 'triangle', ep: 'triangle', stop: 'stop_triangle', log: '🔺 Otomatik üçgen' },
];
_geoBtns.forEach(g => {
    const b = document.getElementById(g.id);
    if (b) b.addEventListener('click',
        () => toggleGeoScenario(g.scn, g.ep, g.stop, g.log));
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

// === "MANUEL UÇUŞ" = MANUEL MOD & JOYSTICK ===
// Manuel yalnızca "Manuel Uçuş" yazı butonundan tetiklenir (şekiller otomatik).
const manualBtns = ['btn-mode-manual']
    .map(id => document.getElementById(id)).filter(Boolean);
const manualBlock = document.getElementById('manual-control-block');
const joystickBase = document.getElementById('joystick-base');
const joystickKnob = document.getElementById('joystick-knob');

let manualActive = false;
let jsX = 0; // -1..1 (roll / aileron)
let jsY = 0; // -1..1 (pitch / elevator)
let throttle = 0; // 0..100
let isDragging = false;
let sendInterval = null;

// Manuel UI'yı yerel olarak kapat (başka moda geçişte çağrılır).
// postStop=true → backend'e stop_manual da gönder (tam duruş / iniş).
function teardownManualUI(postStop) {
    if (!manualActive) return;
    manualActive = false;
    if (manualBlock) manualBlock.classList.add('hidden');
    clearInterval(sendInterval);
    jsX = 0; jsY = 0; throttle = 0;
    updateJoystickUI(0, 0);
    if (postStop) fetch('/api/command/plane/stop_manual', { method: 'POST' }).catch(() => {});
}

async function toggleManual() {
    if (!manualActive) {
        // MANUEL mod AÇ
        manualBtns.forEach(b => b.disabled = true);
        planeRandomActive = false;   // diğer modların yerel bayrağını temizle
        addLog('SYS', 'MANUEL uçuş seçildi → Plane MANUAL moda alınıyor...', 'warn');
        try {
            const res = await fetch('/api/command/plane/start_manual', { method: 'POST' });
            const data = await res.json();
            if (data.status === 'success') {
                manualActive = true;
                manualBlock.classList.remove('hidden');
                setScenarioHighlight('manual');
                // SORUNSUZ AUTO→MANUEL: manuele CRUISE gazla gir (uçak süzülüp
                // düşmesin). W/S ile buradan azalt/artır. Yerdeyken de zararsız
                // (kalkış için gaz gerekir); backend havada değilse yok sayar.
                throttle = 60;
                const thrEl = document.getElementById('js-thr');
                if (thrEl) thrEl.textContent = throttle;
                addLog('SYS', '🟧 Manuel Mod AKTİF! Gaz %60 (cruise). W/S=gaz, A/D=roll.', 'warn');
                sendInterval = setInterval(sendManualCommand, 100); // 10 Hz
            } else {
                addLog('ERR', 'Manuel mod başlatılamadı: ' + data.message, 'crit');
            }
        } catch(e) {
            addLog('ERR', 'Bağlantı hatası: ' + e, 'crit');
        }
        manualBtns.forEach(b => b.disabled = false);
    } else {
        // Manuel mod KAPAT (tam duruş → iniş/disarm)
        teardownManualUI(true);
        setScenarioHighlight(null);
        addLog('SYS', 'Manuel Mod KAPALI. Throttle sıfırlandı.', 'info');
    }
}
manualBtns.forEach(b => b.addEventListener('click', toggleManual));

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

// === KLAVYE İLE MANUEL UÇUŞ KONTROLÜ ===
//   W / S            → Gaz (throttle) artır / azalt
//   A / D  veya  ← / →  → Roll  (aileron)
//   ↑ / ↓            → Pitch (elevator)
// Yön tuşu bırakılınca ilgili eksen otomatik merkeze döner (self-centering).
// Not: ok tuşları için preventDefault() sayfanın kaymasını engeller.
const ROLL_KEYS  = { 'a': -1, 'd': 1, 'arrowleft': -1, 'arrowright': 1 };
const PITCH_KEYS = { 'arrowup': 1, 'arrowdown': -1 };
const heldKeys = new Set();

// Basılı yön tuşlarından jsX (roll) / jsY (pitch) hesapla ve joystick UI'ına yansıt
function applyKeyboardAxes() {
    let roll = 0, pitch = 0;
    heldKeys.forEach((k) => {
        if (k in ROLL_KEYS)  roll  = ROLL_KEYS[k];
        if (k in PITCH_KEYS) pitch = PITCH_KEYS[k];
    });
    const r = (joystickBase.getBoundingClientRect().width / 2) || 1;
    // updateJoystickUI(dx,dy): jsX=dx/r, jsY=-(dy/r) → dx=roll*r, dy=-pitch*r
    updateJoystickUI(roll * r, -pitch * r);
}

window.addEventListener('keydown', (e) => {
    const kk = (e.key || '').toLowerCase();

    // AVCI (iris) manuel uçuş aktifse tuşları iris'e yönlendir (öncelik)
    if (irisManualActive) {
        if (kk in IRIS_THR || kk in IRIS_YAW || kk in IRIS_PITCH || kk in IRIS_ROLL) {
            irisHeld.add(kk);
            e.preventDefault();
        }
        return;
    }

    if (!manualActive) return;
    const k = (e.key || '').toLowerCase();

    // Gaz kontrolü (kademeli)
    if (k === 'w' || k === 's') {
        throttle = (k === 'w')
            ? Math.min(100, throttle + 5)
            : Math.max(0, throttle - 5);
        document.getElementById('js-thr').textContent = throttle;
        e.preventDefault();
        return;
    }

    // Roll / Pitch (basılı tutuldukça tam kırım)
    if (k in ROLL_KEYS || k in PITCH_KEYS) {
        if (!heldKeys.has(k)) {
            heldKeys.add(k);
            applyKeyboardAxes();
        }
        e.preventDefault();   // ok tuşlarının sayfayı kaydırmasını engelle
    }
});

window.addEventListener('keyup', (e) => {
    const k = (e.key || '').toLowerCase();

    // AVCI (iris) manuel uçuş aktifse iris tuş durumunu temizle
    if (irisManualActive) {
        if (irisHeld.has(k)) {
            irisHeld.delete(k);
            e.preventDefault();
        }
        return;
    }

    if (k in ROLL_KEYS || k in PITCH_KEYS) {
        heldKeys.delete(k);
        applyKeyboardAxes();  // tuş bırakıldı → eksen merkeze döner
        e.preventDefault();
    }
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
// AVCI (IRIS) MANUEL UÇUŞ — GUIDED + velocity (SET_POSITION_TARGET_LOCAL_NED)
// =====================================================
// Multikopter olduğu için: buton → backend GUIDED'a alır, ARM eder, ~3 m'ye
// kaldırır (TAKEOFF) ve 10 Hz hız akışı başlatır. Klavye tuşları hız (m/s)
// hedeflerine çevrilir; tuş bırakılınca 0 → drone asılı kalır (hover).
let irisManualActive = false;
let irisSendInterval = null;
const IRIS_SPEED  = 3.0;    // m/s yatay (ileri/geri/sağ/sol)
const IRIS_VSPEED = 2.0;    // m/s dikey (yüksel/alçal)
const IRIS_YAWRATE = 0.6;   // rad/s (dönüş)
const irisHeld = new Set();
// Fare joystick'inden gelen yatay hız bileşeni (-1..1)
let irisJoyX = 0;   // sağ(+)/sol(-)  → vy
let irisJoyY = 0;   // ileri(+)/geri(-) → vx  (ekranda yukarı = ileri)
let irisJoyDragging = false;

// Multikopter tuş haritası (konvansiyonel drone düzeni):
//   W/S = GAZ/İRTİFA (Vz) — GUIDED irtifayı kilitlediği için dikey hız ile değişir
//   A/D = YAW (dönüş)
//   ok ↑/↓ = PITCH (ileri/geri, vx) · ok ←/→ = ROLL (yana, vy)
const IRIS_THR   = { 'w': -1, 's': 1 };                 // vz: W=yüksel(NED negatif), S=alçal
const IRIS_YAW   = { 'a': -1, 'd': 1 };                 // yaw_rate: A=sol, D=sağ
const IRIS_PITCH = { 'arrowup': 1, 'arrowdown': -1 };   // vx: ileri/geri
const IRIS_ROLL  = { 'arrowleft': -1, 'arrowright': 1 };// vy: sol/sağ

const btnIrisManual = document.getElementById('btn-iris-manual');
const irisManualBlock = document.getElementById('iris-manual-block');

btnIrisManual.addEventListener('click', async () => {
    if (!irisManualActive) {
        if (chaseActive) { addLog('ERR', 'Önce Takip görevini durdurun.', 'crit'); return; }
        // Rastgele dans çalışıyorsa: backend onu durdurup manuel devralır;
        // burada dans UI durumunu sıfırlıyoruz (tutarlılık).
        if (randomActive) {
            randomActive = false;
            if (btnIrisRandom) {
                btnIrisRandom.textContent = '🎲 RASTGELE UÇUŞ';
                btnIrisRandom.style.borderLeftColor = '';
            }
            const rs = document.getElementById('iris-random-status');
            if (rs) { rs.textContent = 'BEKLEMEDE'; rs.className = 'val warning'; }
            addLog('SYS', 'Rastgele dans → Manuele geçiliyor...', 'warn');
        }
        btnIrisManual.textContent = '⏳ KALKIŞ...';
        btnIrisManual.disabled = true;
        addLog('SYS', 'Avcı GUIDED moda alınıyor, kalkış yapılıyor (~3 m)...', 'warn');
        try {
            const res = await fetch('/api/command/iris/start_manual', { method: 'POST' });
            const data = await res.json();
            if (data.status === 'success') {
                irisManualActive = true;
                irisManualBlock.classList.remove('hidden');
                btnIrisManual.textContent = '✖ MANUEL UÇUŞ DURDUR';
                btnIrisManual.style.borderLeftColor = 'var(--danger-red)';
                document.getElementById('iris-manual-status').textContent = 'AKTİF';
                addLog('SYS', 'Avcı Manuel Uçuş AKTİF! WASD + ok tuşlarıyla uçur.', 'warn');
                irisSendInterval = setInterval(sendIrisManualCommand, 100); // 10 Hz akış
            } else {
                addLog('ERR', 'Avcı manuel başlatılamadı: ' + (data.message || ''), 'crit');
            }
        } catch (e) {
            addLog('ERR', 'Bağlantı hatası: ' + e, 'crit');
        }
        btnIrisManual.disabled = false;
    } else {
        irisManualActive = false;
        irisHeld.clear();
        clearInterval(irisSendInterval);
        irisManualBlock.classList.add('hidden');
        btnIrisManual.textContent = '🚁 MANUEL UÇUŞ BAŞLAT';
        btnIrisManual.style.borderLeftColor = '';
        document.getElementById('iris-manual-status').textContent = 'BEKLEMEDE';
        addLog('SYS', 'Avcı Manuel Uçuş KAPALI — iniş yapılıyor.', 'info');
        fetch('/api/command/iris/stop_manual', { method: 'POST' }).catch(() => {});
    }
});

// --- Fare joystick'i: iris-joystick-base üzerinde sürükle → yatay hız ---
const irisJoyBase = document.getElementById('iris-joystick-base');
const irisJoyKnob = document.getElementById('iris-joystick-knob');
function irisJoyUpdate(dx, dy) {
    const r = (irisJoyBase.getBoundingClientRect().width / 2) || 1;
    irisJoyKnob.style.left = `calc(50% + ${dx}px)`;
    irisJoyKnob.style.top  = `calc(50% + ${dy}px)`;
    irisJoyX = +(dx / r).toFixed(3);    // sağ = +
    irisJoyY = -(dy / r).toFixed(3);    // ekranda yukarı = ileri (+)
}
function irisJoyPos(e) {
    const rect = irisJoyBase.getBoundingClientRect();
    const cx = rect.left + rect.width / 2, cy = rect.top + rect.height / 2, r = rect.width / 2;
    let dx = ((e.clientX ?? e.touches[0].clientX) - cx);
    let dy = ((e.clientY ?? e.touches[0].clientY) - cy);
    const d = Math.hypot(dx, dy);
    if (d > r) { dx = dx / d * r; dy = dy / d * r; }
    return { dx, dy };
}
if (irisJoyBase) {
    irisJoyBase.addEventListener('mousedown', (e) => {
        if (!irisManualActive) return;
        irisJoyDragging = true; irisJoyKnob.classList.add('active');
        const p = irisJoyPos(e); irisJoyUpdate(p.dx, p.dy);
    });
    window.addEventListener('mousemove', (e) => {
        if (!irisJoyDragging || !irisManualActive) return;
        const p = irisJoyPos(e); irisJoyUpdate(p.dx, p.dy);
    });
    window.addEventListener('mouseup', () => {
        if (!irisJoyDragging) return;
        irisJoyDragging = false; irisJoyKnob.classList.remove('active');
        irisJoyUpdate(0, 0);   // merkeze dön → hover
    });
    irisJoyBase.addEventListener('touchstart', (e) => {
        if (!irisManualActive) return;
        irisJoyDragging = true; const p = irisJoyPos(e); irisJoyUpdate(p.dx, p.dy);
    }, { passive: true });
    window.addEventListener('touchmove', (e) => {
        if (!irisJoyDragging || !irisManualActive) return;
        const p = irisJoyPos(e); irisJoyUpdate(p.dx, p.dy);
    }, { passive: true });
    window.addEventListener('touchend', () => {
        if (!irisJoyDragging) return;
        irisJoyDragging = false; irisJoyUpdate(0, 0);
    });
}

// Basılı tuşlar + fare joystick'inden hız hesapla, 10 Hz akışla gönder (watchdog)
function sendIrisManualCommand() {
    if (!irisManualActive) return;
    let vx = 0, vy = 0, vz = 0, yaw = 0;
    irisHeld.forEach((k) => {
        if (k in IRIS_THR)   vz  = IRIS_THR[k]   * IRIS_VSPEED;  // W/S = gaz/irtifa (Vz)
        if (k in IRIS_YAW)   yaw = IRIS_YAW[k]   * IRIS_YAWRATE; // A/D = dönüş
        if (k in IRIS_PITCH) vx  = IRIS_PITCH[k] * IRIS_SPEED;   // ok ↑/↓ = ileri/geri
        if (k in IRIS_ROLL)  vy  = IRIS_ROLL[k]  * IRIS_SPEED;   // ok ←/→ = yana
    });
    // Fare joystick'i (yatay) klavye ile birleşir, [-IRIS_SPEED, IRIS_SPEED] sınırlı
    const cl = (v) => Math.max(-IRIS_SPEED, Math.min(IRIS_SPEED, v));
    vx = cl(vx + irisJoyY * IRIS_SPEED);
    vy = cl(vy + irisJoyX * IRIS_SPEED);
    document.getElementById('iris-vx').textContent  = vx.toFixed(1);
    document.getElementById('iris-vy').textContent  = vy.toFixed(1);
    document.getElementById('iris-vz').textContent  = vz.toFixed(1);
    document.getElementById('iris-yaw').textContent = yaw.toFixed(1);
    fetch('/api/command/iris/manual', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ vx, vy, vz, yaw_rate: yaw })
    }).catch(() => {});
}

// =====================================================
// RASTGELE DANS (AVCI) — daireler + sağ-sol gelişigüzel
// Klavyede R tuşu (veya buton) ile aç/kapat. Backend otonom uçurur.
// =====================================================
let randomActive = false;
const btnIrisRandom = document.getElementById('btn-iris-random');

async function toggleRandomDance() {
    if (!randomActive) {
        if (chaseActive) {
            addLog('ERR', 'Önce Takip görevini kapatın.', 'crit');
            return;
        }
        // Manuel uçuş aktifse: KESİNTİSİZ dansa geç. Backend manueli indirmeden
        // LOITER'a alıp devralır; burada sadece manuel UI durumunu sıfırlıyoruz.
        if (irisManualActive) {
            irisManualActive = false;
            irisHeld.clear();
            clearInterval(irisSendInterval);
            if (irisManualBlock) irisManualBlock.classList.add('hidden');
            if (btnIrisManual) {
                btnIrisManual.textContent = '🚁 MANUEL UÇUŞ BAŞLAT';
                btnIrisManual.style.borderLeftColor = '';
            }
            const ms = document.getElementById('iris-manual-status');
            if (ms) { ms.textContent = 'BEKLEMEDE'; ms.className = 'val warning'; }
            addLog('SYS', 'Manuel → Rastgele dansa KESİNTİSİZ geçiliyor...', 'warn');
        } else {
            addLog('SYS', '🎲 Rastgele dans başlatılıyor — Avcı kalkıp daireler çizecek...', 'warn');
        }
        try {
            const res = await fetch('/api/command/iris/start_random', { method: 'POST' });
            const data = await res.json();
            if (data.status === 'success') {
                randomActive = true;
                if (btnIrisRandom) {
                    btnIrisRandom.textContent = '✖ RASTGELE DURDUR';
                    btnIrisRandom.style.borderLeftColor = 'var(--danger-red)';
                }
                const st = document.getElementById('iris-random-status');
                if (st) { st.textContent = 'AKTİF'; st.className = 'val green'; }
                addLog('SYS', '🎲 Rastgele dans AKTİF! Daireler + sağ-sol. (R ile durdur)', 'success');
            } else {
                addLog('ERR', 'Başlatılamadı: ' + (data.message || ''), 'crit');
            }
        } catch (e) {
            addLog('ERR', 'Bağlantı hatası: ' + e, 'crit');
        }
    } else {
        randomActive = false;
        if (btnIrisRandom) {
            btnIrisRandom.textContent = '🎲 RASTGELE UÇUŞ';
            btnIrisRandom.style.borderLeftColor = '';
        }
        const st = document.getElementById('iris-random-status');
        if (st) { st.textContent = 'BEKLEMEDE'; st.className = 'val warning'; }
        addLog('SYS', 'Rastgele dans kapatıldı — iniş yapılıyor.', 'info');
        fetch('/api/command/iris/stop_random', { method: 'POST' }).catch(() => {});
    }
}

if (btnIrisRandom) btnIrisRandom.addEventListener('click', toggleRandomDance);
// (Iris dansı artık yalnızca buton ile — R tuşu HEDEF İHA rastgele uçuşuna atandı.)

// =====================================================
// HEDEF İHA RASTGELE / KAÇIŞ UÇUŞU — buton + R tuşu
// =====================================================
let planeRandomActive = false;
const btnPlaneRandom = document.getElementById('btn-mode-random');   // "Rastgele Uçuş" yazı butonu

async function togglePlaneRandom() {
    if (!planeRandomActive) {
        // Başka mod (manuel) açıksa yerel UI'yı kapat (backend zaten durduracak)
        teardownManualUI(false);
        addLog('SYS', '🔲 [R] Otonom rastgele/kaçış uçuşu başlatılıyor...', 'warn');
        try {
            const res = await fetch('/api/command/plane/start_random', { method: 'POST' });
            const data = await res.json();
            if (data.status === 'success') {
                planeRandomActive = true;
                setScenarioHighlight('random');
                addLog('SYS', '🔲 Hedef İHA 3B gelişigüzel kaçıyor! (R ile durdur)', 'success');
            } else {
                addLog('ERR', 'Başlatılamadı: ' + (data.message || ''), 'crit');
            }
        } catch (e) {
            addLog('ERR', 'Bağlantı hatası: ' + e, 'crit');
        }
    } else {
        planeRandomActive = false;
        setScenarioHighlight(null);
        addLog('SYS', 'Rastgele/kaçış uçuşu durduruldu (LOITER).', 'info');
        fetch('/api/command/plane/stop_random', { method: 'POST' }).catch(() => {});
    }
}
if (btnPlaneRandom) btnPlaneRandom.addEventListener('click', togglePlaneRandom);

// Klavye: R tuşu HEDEF İHA rastgele/kaçış uçuşunu aç/kapat (input alanında değilken)
window.addEventListener('keydown', (e) => {
    const k = (e.key || '').toLowerCase();
    if (k === 'r' && !e.repeat) {
        const tag = (document.activeElement && document.activeElement.tagName) || '';
        if (tag === 'INPUT' || tag === 'TEXTAREA') return;   // slider/metin girişini bozma
        togglePlaneRandom();
        e.preventDefault();
    }
});

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
    // Başlangıç kamerası: seçili sekmeye göre WebSocket video hattını başlat
    switchCamera((modeHunter && modeHunter.checked) ? 'iris' : 'plane');
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
