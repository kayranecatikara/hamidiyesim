/* ══════════════════════════════════════════════════════════════════════
   AVCI GCS — Taktik Saha Ekranı (canlı)

   Bu dosyada SİMÜLE VERİ YOKTUR. Ekrandaki her sayı gcs_server'dan gelir:
     ws://.../ws              → iris + plane telemetrisi (10 Hz)
     /api/video_feed/{iris|plane} → MJPEG (YOLO/pose overlay'i sunucuda çizili)
     /api/chase_status        → görev + supervisor fazı + gps_guidance durumu
     /api/telemetry/pnp       → görüş hattı (tespit/pose/menzil) + faz kapıları
     /api/scenario_status     → senaryo süreci yaşıyor mu (buton senkronu)
     /api/hasar               → gerçek Gazebo teması (imha)
   Komutlar klasik arayüzle AYNI uçlara gider; PWM eşlemesi de birebir aynıdır.
   ══════════════════════════════════════════════════════════════════════ */
(() => {
const $ = id => document.getElementById(id);
const reduced = matchMedia('(prefers-reduced-motion: reduce)').matches;
const monoFont = 'ui-monospace,Menlo,Consolas,monospace';
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

// ══ OLAY KAYDI ═════════════════════════════════════════════════════════
const logBody = $('logBody');
function tstamp(){
  const d = new Date();
  return [d.getHours(), d.getMinutes(), d.getSeconds()]
    .map(n => String(n).padStart(2, '0')).join(':');
}
function addLog(cls, tag, msg){
  const atBottom = logBody.scrollHeight - logBody.scrollTop - logBody.clientHeight < 40;
  const row = document.createElement('div');
  row.className = 'logrow ' + cls;
  row.innerHTML = '<span class="tm"></span><span class="tag"></span><span class="msg"></span>';
  row.children[0].textContent = tstamp();
  row.children[1].textContent = tag;
  row.children[2].textContent = msg;          // textContent: sunucu metni HTML olarak yorumlanmasın
  logBody.appendChild(row);
  while (logBody.children.length > 200) logBody.removeChild(logBody.firstElementChild);
  if (atBottom) logBody.scrollTop = logBody.scrollHeight;
}
addLog('sys', 'SYS', 'Y.K.İ taktik ekranı başlatıldı — sunucu dinleniyor.');

// ══ DURUM ══════════════════════════════════════════════════════════════
const st = {
  tab: 'hedef',              // hangi aracın kamerası/kontrolü açık
  scenario: null,            // square | circle | aggressive | null
  manual: false,
  mission: false,            // chase (hibrit güdüm) aktif mi
  telem: { iris: null, plane: null },
  range: null,               // iris ↔ plane 3B menzil (m)
  rangeRate: null,           // m/s (negatif = yaklaşıyor)
  pnp: null,
  faz: null,                 // GPS | VISUAL | VURULDU | DURDU
  imha: false,
};
const SCN_LBL = { square: 'KARE', circle: 'DAİRE', aggressive: 'AGRESİF' };

// ══ KAMERA ─ MJPEG akışı ═══════════════════════════════════════════════
// MJPEG multipart bağlantısı süresiz açık kalır; src değiştirmek eski
// bağlantıyı her tarayıcıda kapatmaz. Bu yüzden <img> DOM'dan silinip
// yeniden kurulur (klasik arayüzdeki switchCamera ile aynı gerekçe).
function switchCamera(vehicle){
  const wrap = $('fpvwrap');
  const old = $('fpvImg');
  if (old){ old.src = ''; old.remove(); }
  const img = document.createElement('img');
  img.id = 'fpvImg';
  img.alt = '';
  // MJPEG akışında 'load' ancak akış BİTİNCE tetiklenir; ilk karenin gelip
  // gelmediği naturalWidth'ten anlaşılır (frame döngüsü yokluyor).
  img.src = `/api/video_feed/${vehicle}?t=${Date.now()}`;
  wrap.insertBefore(img, wrap.firstChild);
  $('noFeed').hidden = false;
}

function setTab(t){
  st.tab = t;
  $('segT').setAttribute('aria-pressed', String(t === 'hedef'));
  $('segA').setAttribute('aria-pressed', String(t === 'avci'));
  $('viewHedef').hidden = t !== 'hedef';
  $('viewAvci').hidden  = t !== 'avci';
  $('camTag').textContent = t === 'hedef' ? 'CAM · HEDEF İHA' : 'CAM · AVCI';
  $('oVeh').textContent = t === 'hedef' ? 'HEDEF' : 'AVCI';
  $('manBadge').hidden = !(t === 'hedef' && st.manual);
  switchCamera(t === 'hedef' ? 'plane' : 'iris');
  addLog('sys', 'SYS', t === 'hedef' ? 'Kamera: HEDEF İHA (Talon burun)' : 'Kamera: AVCI DRONE (iris)');
}
$('segT').addEventListener('click', () => { if (st.tab !== 'hedef') setTab('hedef'); });
$('segA').addEventListener('click', () => { if (st.tab !== 'avci')  setTab('avci'); });

// ══ WEBSOCKET TELEMETRİ ════════════════════════════════════════════════
let lastMsg = 0, msgCount = 0, wsOpen = false;
function connectWS(){
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onopen = () => {
    wsOpen = true;
    $('connBadge').classList.add('on');
    $('connTxt').textContent = 'ONLINE';
    addLog('sys', 'NET', 'Telemetri bağlantısı kuruldu.');
  };
  ws.onclose = () => {
    wsOpen = false;
    $('connBadge').classList.remove('on');
    $('connTxt').textContent = 'BAĞLANTI YOK';
    addLog('err', 'NET', 'Bağlantı koptu — yeniden bağlanılıyor.');
    setTimeout(connectWS, 1000);
  };
  ws.onerror = () => { try { ws.close(); } catch (e) {} };
  ws.onmessage = ev => {
    lastMsg = performance.now(); msgCount++;
    let d; try { d = JSON.parse(ev.data); } catch (e) { return; }
    onTelemetry(d);
  };
}

function alive(v){ return v && !(v.x === 0 && v.y === 0 && v.z === 0); }

let lastRangeT = 0, lastRange = null;
function onTelemetry(d){
  if (d.iris)  st.telem.iris  = d.iris;
  if (d.plane) st.telem.plane = d.plane;
  const ir = st.telem.iris, pl = st.telem.plane;

  if (alive(ir) && alive(pl)){
    // Menzil: ÖNCE sunucunun zaman hizalı gz ölçümü (sim_truth), yoksa
    // telemetriden hesapla. Telemetri farkı iki ayrı akışın hizasız farkı ve
    // loglarda karelerin %37'sinde donuk çıkıyor — 0.4 s donma 25 m/s'te 10 m.
    const rGz = (st.pnp && st.pnp.gercek_menzil_kaynak === 'gz')
                ? st.pnp.gercek_menzil : null;
    const r = (rGz !== null && rGz !== undefined)
              ? rGz : Math.hypot(pl.x - ir.x, pl.y - ir.y, pl.z - ir.z);
    const now = performance.now();
    if (lastRange !== null && now > lastRangeT){
      const rate = (r - lastRange) / ((now - lastRangeT) / 1000);
      st.rangeRate = st.rangeRate === null ? rate : st.rangeRate * 0.8 + rate * 0.2;
    }
    lastRange = r; lastRangeT = now;
    st.range = r;
  }
  if (alive(ir)) pushTrail(world.aTrail, ir);
  if (alive(pl)) pushTrail(world.tTrail, pl);
  renderTelemetryPanels();
}

// NED (x=Kuzey, y=Doğu, z=Aşağı) → sahne dünyası (e=Doğu, n=Kuzey, u=Yukarı)
const toWorld = v => ({ e: v.y, n: v.x, u: -v.z });

function renderTelemetryPanels(){
  const ir = st.telem.iris, pl = st.telem.plane;
  const fmt = (v, s) => (v === undefined || v === null) ? '--' : v.toFixed(1) + (s || '');
  if (pl){
    $('tHiz').textContent  = fmt(pl.speed, ' m/s');
    $('tIrt').textContent  = fmt(-pl.z, ' m');
    $('tPos').textContent  = `${pl.x.toFixed(1)}, ${pl.y.toFixed(1)}`;
    $('tHead').textContent = `${Math.round(pl.yaw)}°`;
  }
  if (ir){
    $('aHiz').textContent  = fmt(ir.speed, ' m/s');
    $('aIrt').textContent  = fmt(-ir.z, ' m');
    $('aPos').textContent  = `${ir.x.toFixed(1)}, ${ir.y.toFixed(1)}`;
    $('aHead').textContent = `${Math.round(ir.yaw)}°`;
  }
  // FPV üstü telemetri — seçili kameranın aracı
  const v = st.tab === 'hedef' ? pl : ir;
  if (v){
    $('oAlt').textContent = (-v.z).toFixed(0);
    $('oSpd').textContent = (v.speed ?? 0).toFixed(1);
    $('oHdg').textContent = String(Math.round((v.yaw + 360) % 360)).padStart(3, '0');
  }
  if (st.range !== null){
    $('oRng').textContent = st.range.toFixed(0);
    $('sRng').textContent = st.range.toFixed(0);
    $('posRng').textContent = 'MENZİL ' + st.range.toFixed(0) + ' m';
    if (st.rangeRate !== null){
      const r = st.rangeRate;
      $('sRngCap').textContent = (r < -0.3 ? 'yaklaşıyor · ' : r > 0.3 ? 'uzaklaşıyor · ' : 'sabit · ')
        + r.toFixed(1) + ' m/s';
    }
  }
}

// Bağlantı sağlığı: HB = son mesajdan bu yana geçen süre, loss = beklenen
// 10 Hz'in kaçını aldık, latency = hafif bir GET'in gidiş-dönüş süresi.
setInterval(() => {
  const hb = lastMsg ? (performance.now() - lastMsg) / 1000 : null;
  $('hHb').textContent = hb === null ? '--' : hb.toFixed(1) + 's';
  const beklenen = 10;                       // sunucu 0.1 s'de bir yolluyor
  const loss = wsOpen ? clamp(100 * (1 - msgCount / beklenen), 0, 100) : 100;
  $('hLoss').textContent = loss.toFixed(1) + '%';
  msgCount = 0;
}, 1000);

setInterval(async () => {
  const t0 = performance.now();
  try {
    await fetch('/api/scenario_status', { cache: 'no-store' });
    $('hLat').textContent = Math.round(performance.now() - t0) + 'ms';
  } catch (e) { $('hLat').textContent = '--'; }
}, 3000);

// ══ 3B KONUM İZLEME ════════════════════════════════════════════════════
// Maketin gezinme mantığı (yörünge / kaydırma / yakınlaştırma) korunmuştur;
// çizilen noktalar GERÇEK NED telemetrisidir, ölçek veriye göre oturur.
function mkCanvas(id){
  const c = $(id), cx = c.getContext('2d');
  function size(){
    const r = c.getBoundingClientRect();
    const dpr = Math.min(devicePixelRatio || 1, 2);
    c.width = Math.max(1, r.width * dpr); c.height = Math.max(1, r.height * dpr);
    cx.setTransform(dpr, 0, 0, dpr, 0, 0);
    c._w = r.width; c._h = r.height;
  }
  size(); addEventListener('resize', size);
  return { c, cx, size };
}
const SC = mkCanvas('scene');
const HALF = Math.PI / 2;
const TRAIL_MAX = 400;                       // ~40 s iz @ 10 Hz
let camAz = -0.7, camEl = 0.6, camAzT = -0.7, camElT = 0.6;
let scZoom = 1, scPanX = 0, scPanY = 0, panMode = false;
let dragS = false, lx = 0, ly = 0;

const world = {
  tTrail: [], aTrail: [],
  view: { cx: 0, cy: 0, span: 60, zMax: 60, init: false },
};
function pushTrail(arr, v){
  const p = toWorld(v);
  const last = arr[arr.length - 1];
  if (last && last.e === p.e && last.n === p.n && last.u === p.u) return;
  arr.push(p);
  if (arr.length > TRAIL_MAX) arr.shift();
}

function clearActiveView(){
  document.querySelectorAll('.viewbtns button[data-view]').forEach(x => x.classList.remove('on'));
}
SC.c.addEventListener('contextmenu', e => e.preventDefault());
SC.c.addEventListener('pointerdown', e => {
  dragS = true; lx = e.clientX; ly = e.clientY;
  try { SC.c.setPointerCapture(e.pointerId); } catch (_) {}
  e.preventDefault();
});
SC.c.addEventListener('pointermove', e => {
  if (!dragS) return;
  const dx = e.clientX - lx, dy = e.clientY - ly; lx = e.clientX; ly = e.clientY;
  if (panMode || e.shiftKey || e.buttons === 2){ scPanX += dx; scPanY += dy; }
  else {
    camAzT = camAz += dx * 0.01;
    camElT = camEl = clamp(camEl - dy * 0.008, 0, HALF);
    clearActiveView();
  }
});
SC.c.addEventListener('pointerup', () => { dragS = false; });
SC.c.addEventListener('pointercancel', () => { dragS = false; });
SC.c.addEventListener('wheel', e => {
  e.preventDefault();
  scZoom = clamp(scZoom * (e.deltaY < 0 ? 1.12 : 0.89), 0.3, 4);
}, { passive: false });
SC.c.addEventListener('dblclick', () => {
  scPanX = 0; scPanY = 0; scZoom = 1; camAzT = -0.7; camElT = 0.6; clearActiveView();
  const ib = document.querySelector('.viewbtns button[data-view="iso"]');
  if (ib) ib.classList.add('on');
});
function setView(v){
  if (v === 'iso'){ camAzT = -0.7; camElT = 0.6; }
  else if (v === 'top')  { camAzT = 0;    camElT = HALF; }   // Kuzey-Doğu düzlemi
  else if (v === 'front'){ camAzT = 0;    camElT = 0; }      // Doğu-İrtifa
  else if (v === 'side') { camAzT = HALF; camElT = 0; }      // Kuzey-İrtifa
  scPanX = 0; scPanY = 0; scZoom = 1;
}
document.querySelectorAll('.viewbtns button[data-view]').forEach(b =>
  b.addEventListener('click', () => { clearActiveView(); b.classList.add('on'); setView(b.dataset.view); }));
$('panBtn').addEventListener('click', () => {
  panMode = !panMode;
  $('panBtn').classList.toggle('on', panMode);
});

function niceStep(raw){
  const mag = Math.pow(10, Math.floor(Math.log10(Math.max(raw, 1e-6))));
  const n = raw / mag;
  return (n < 1.5 ? 1 : n < 3.5 ? 2 : n < 7.5 ? 5 : 10) * mag;
}

function drawScene(){
  const c = SC.cx, w = SC.c._w, h = SC.c._h;
  if (!w || !h) return;
  c.clearRect(0, 0, w, h);

  const cur = {
    tgt:  alive(st.telem.plane) ? toWorld(st.telem.plane) : null,
    avci: alive(st.telem.iris)  ? toWorld(st.telem.iris)  : null,
  };
  const pts = world.tTrail.concat(world.aTrail);
  if (!pts.length){
    c.fillStyle = '#475569'; c.font = '11px ' + monoFont; c.textAlign = 'center';
    c.fillText('TELEMETRİ BEKLENİYOR...', w / 2, h / 2); c.textAlign = 'left';
    return;
  }

  // ── Otomatik çerçeveleme: tüm izler ekrana sığsın, yumuşak takip ──
  let mnE = 1e9, mxE = -1e9, mnN = 1e9, mxN = -1e9, mxU = 0;
  for (const p of pts){
    if (p.e < mnE) mnE = p.e; if (p.e > mxE) mxE = p.e;
    if (p.n < mnN) mnN = p.n; if (p.n > mxN) mxN = p.n;
    if (p.u > mxU) mxU = p.u;
  }
  const tCx = (mnE + mxE) / 2, tCy = (mnN + mxN) / 2;
  const tSpan = Math.max(40, mxE - mnE, mxN - mnN, mxU * 1.3) * 1.2;
  const V = world.view;
  if (!V.init){ V.cx = tCx; V.cy = tCy; V.span = tSpan; V.zMax = Math.max(40, mxU * 1.2); V.init = true; }
  else {
    const a = 0.06;
    V.cx += (tCx - V.cx) * a; V.cy += (tCy - V.cy) * a; V.span += (tSpan - V.span) * a;
    V.zMax += (Math.max(40, mxU * 1.2) - V.zMax) * a;
  }

  const s = (Math.min(w, h) * 0.62 / V.span) * scZoom;
  const ox = w * 0.5 + scPanX, oy = h * 0.62 + scPanY;
  const ca = Math.cos(camAz), sa = Math.sin(camAz), cp = Math.cos(camEl), sp = Math.sin(camEl);
  // Ortografik izdüşüm — yatay ve dikey AYNI ölçekte (geometri bozulmasın)
  const P3 = (e, n, u) => {
    const rx = e - V.cx, ry = n - V.cy;
    const x1 = rx * ca - ry * sa, y1 = rx * sa + ry * ca;
    const y2 = y1 * cp - u * sp, z2 = y1 * sp + u * cp;
    return { x: ox + x1 * s, y: oy - z2 * s, d: y2 };
  };
  const showZ = camEl < 1.30;

  // ── Zemin ızgarası (u = 0) ──
  const cell = niceStep(V.span / 5);
  const ext = Math.min(4000, V.span * 1.6);
  const e0 = Math.floor((V.cx - ext / 2) / cell) * cell, e1 = V.cx + ext / 2;
  const n0 = Math.floor((V.cy - ext / 2) / cell) * cell, n1 = V.cy + ext / 2;
  c.strokeStyle = 'rgba(63,216,192,0.10)'; c.lineWidth = 1;
  for (let g = e0; g <= e1; g += cell){
    const a = P3(g, n0, 0), b = P3(g, n1, 0);
    c.beginPath(); c.moveTo(a.x, a.y); c.lineTo(b.x, b.y); c.stroke();
  }
  for (let g = n0; g <= n1; g += cell){
    const a = P3(e0, g, 0), b = P3(e1, g, 0);
    c.beginPath(); c.moveTo(a.x, a.y); c.lineTo(b.x, b.y); c.stroke();
  }
  c.fillStyle = '#54646f'; c.font = '9px ' + monoFont;
  c.textAlign = 'right'; c.fillText('kare = ' + cell + ' m', w - 12, h - 22); c.textAlign = 'left';

  // ── Eksenler + irtifa kulesi (EKF orijini = iris kalkış noktası) ──
  const zMax = V.zMax, zStep = niceStep(zMax / 4);
  const O = P3(0, 0, 0), AE = P3(cell, 0, 0), AN = P3(0, cell, 0), ZT = P3(0, 0, zMax);
  c.strokeStyle = 'rgba(120,150,170,.5)'; c.lineWidth = 1.4;
  c.beginPath(); c.moveTo(O.x, O.y); c.lineTo(AE.x, AE.y); c.stroke();
  c.beginPath(); c.moveTo(O.x, O.y); c.lineTo(AN.x, AN.y); c.stroke();
  if (showZ){
    c.strokeStyle = 'rgba(150,170,185,.4)'; c.setLineDash([4, 4]);
    c.beginPath(); c.moveTo(O.x, O.y); c.lineTo(ZT.x, ZT.y); c.stroke(); c.setLineDash([]);
  }
  c.fillStyle = '#7f93a2'; c.font = '10px ' + monoFont;
  c.fillText('D', AE.x + 4, AE.y + 4); c.fillText('K', AN.x - 12, AN.y + 8);
  if (showZ){
    c.fillText('İRT', ZT.x + 4, ZT.y);
    c.fillStyle = '#54646f'; c.font = '9px ' + monoFont;
    for (let a = zStep; a <= zMax + 1; a += zStep){
      const p = P3(0, 0, a);
      c.fillRect(p.x - 2, p.y - 1, 4, 2);
      c.fillText(Math.round(a) + 'm', p.x + 6, p.y + 3);
    }
  }
  c.fillStyle = '#F4B740'; c.shadowColor = '#F4B740'; c.shadowBlur = 8;
  c.beginPath(); c.arc(O.x, O.y, 4, 0, 7); c.fill(); c.shadowBlur = 0;
  c.font = '9px ' + monoFont; c.fillText('BAŞLANGIÇ', O.x + 7, O.y + 13);

  // ── İzler ──
  const trail = (arr, rgb) => {
    for (let i = 1; i < arr.length; i++){
      const a = P3(arr[i - 1].e, arr[i - 1].n, arr[i - 1].u), b = P3(arr[i].e, arr[i].n, arr[i].u);
      c.strokeStyle = rgb + (i / arr.length * 0.55).toFixed(3) + ')';
      c.lineWidth = 1.6;
      c.beginPath(); c.moveTo(a.x, a.y); c.lineTo(b.x, b.y); c.stroke();
    }
  };
  trail(world.tTrail, 'rgba(255,77,77,');
  trail(world.aTrail, 'rgba(55,224,107,');

  // ── Kilit çizgisi (avcı → hedef) ──
  if (cur.tgt && cur.avci){
    const at = P3(cur.tgt.e, cur.tgt.n, cur.tgt.u), aa = P3(cur.avci.e, cur.avci.n, cur.avci.u);
    c.strokeStyle = st.imha ? 'rgba(255,77,77,.9)' : 'rgba(255,77,77,.5)';
    c.setLineDash([4, 4]); c.lineWidth = 1;
    c.beginPath(); c.moveTo(aa.x, aa.y); c.lineTo(at.x, at.y); c.stroke(); c.setLineDash([]);
    if (st.range !== null){
      const mx = (at.x + aa.x) / 2, my = (at.y + aa.y) / 2;
      c.fillStyle = '#ff8a8a'; c.font = '9px ' + monoFont;
      c.fillText(st.range.toFixed(1) + ' m', mx + 6, my - 4);
    }
  }

  // ── Araçlar (uzaktakini önce çiz) ──
  const drones = [];
  if (cur.tgt)  drones.push({ ...cur.tgt,  col: '#FF4D4D', ad: 'HEDEF' });
  if (cur.avci) drones.push({ ...cur.avci, col: '#37E06B', ad: 'AVCI' });
  drones.sort((A, B) => P3(A.e, A.n, 0).d - P3(B.e, B.n, 0).d);
  for (const d of drones){
    const g = P3(d.e, d.n, 0), a = P3(d.e, d.n, d.u);
    c.globalAlpha = .45; c.strokeStyle = d.col; c.setLineDash([3, 3]); c.lineWidth = 1;
    c.beginPath(); c.moveTo(g.x, g.y); c.lineTo(a.x, a.y); c.stroke(); c.setLineDash([]);
    c.beginPath(); c.ellipse(g.x, g.y, 8, 4, 0, 0, 7); c.stroke(); c.globalAlpha = 1;
    c.fillStyle = d.col; c.shadowColor = d.col; c.shadowBlur = 11;
    c.beginPath(); c.arc(a.x, a.y, 5.5, 0, 7); c.fill(); c.shadowBlur = 0;
    c.font = '9px ' + monoFont;
    c.fillText(`${d.ad} ${d.u.toFixed(0)}m`, a.x + 8, a.y - 4);
  }
}

// ══ SENARYOLAR (kare / daire / agresif) ════════════════════════════════
const scnBtns = [...document.querySelectorAll('#scn [data-scn]')];
function markScenario(){
  scnBtns.forEach(b => b.setAttribute('aria-pressed', String(b.dataset.scn === st.scenario)));
  const lbl = st.scenario ? SCN_LBL[st.scenario] : null;
  scnBtns.forEach(b => {
    const span = b.querySelector('span');
    const base = { square: 'Kare Çiz', circle: 'Daire Çiz', aggressive: 'Agresif Uçuş' }[b.dataset.scn];
    span.textContent = (b.dataset.scn === st.scenario) ? 'Durdur — ' + base : base;
  });
  $('oMode').textContent = st.manual ? 'MANUEL' : (lbl || 'BEKLEME');
}
async function startScenario(name){
  if (st.manual) await exitManual();
  addLog('sys', 'CMD', SCN_LBL[name] + ' senaryosu gönderiliyor (kalkış + desen)...');
  scnBtns.forEach(b => b.disabled = true);
  try {
    const r = await (await fetch('/api/command/plane/scenario/' + name, { method: 'POST' })).json();
    if (r.status === 'success'){
      st.scenario = name;
      addLog('sys', 'SYS', '✓ ' + SCN_LBL[name] + ' aktif — hedef kalkıp deseni uçacak.');
    } else addLog('err', 'HATA', 'Senaryo reddedildi: ' + r.message);
  } catch (e){ addLog('err', 'HATA', 'Bağlantı hatası: ' + e); }
  scnBtns.forEach(b => b.disabled = false);
  markScenario();
}
async function stopScenario(){
  if (st.scenario) addLog('sys', 'SYS', SCN_LBL[st.scenario] + ' senaryosu durduruluyor.');
  st.scenario = null; markScenario();
  try { await fetch('/api/command/plane/stop_scenario', { method: 'POST' }); } catch (e) {}
}
scnBtns.forEach(b => b.addEventListener('click', () => {
  if (st.scenario === b.dataset.scn) stopScenario(); else startScenario(b.dataset.scn);
}));
// Senaryo süreci kendi kendine biterse butonlar sunucuyla senkron kalsın
setInterval(async () => {
  try {
    const d = await (await fetch('/api/scenario_status')).json();
    const backend = d.active ? d.name : null;
    if (backend !== st.scenario){ st.scenario = backend; markScenario(); }
  } catch (e) {}
}, 2000);

// ══ MANUEL MOD ═════════════════════════════════════════════════════════
// Yüzey/gaz → PWM eşlemesi klasik arayüzle BİREBİR aynı:
//   aileron/elevator = 1500 ± 450   (FBWA: tam sapma = maks açı hedefi)
//   throttle         = 1000 + %gaz × 10
const manBtn = $('manBtn'), stick = $('stick'), knob = $('knob');
const MAX_R = 57;
let mAil = 0, mElv = 0, mThr = 60;            // yumuşatılmış komutlar (-1..1, %)
let jsAil = 0, jsElv = 0, dragging = false;
let manualLoop = null, sendTick = 0;
const keys = { w: false, a: false, s: false, d: false, l: false, i: false };

function drawKnob(){
  knob.style.transform = `translate(calc(-50% + ${mAil * MAX_R}px), calc(-50% + ${-mElv * MAX_R}px))`;
  $('rRoll').textContent  = mAil.toFixed(2);
  $('rPitch').textContent = mElv.toFixed(2);
  $('rThr').textContent   = Math.round(mThr) + '%';
  stick.setAttribute('aria-valuetext', `roll ${mAil.toFixed(2)}, pitch ${mElv.toFixed(2)}`);
}
function fromPointer(cx, cy){
  const r = stick.getBoundingClientRect();
  let dx = cx - (r.left + r.width / 2), dy = cy - (r.top + r.height / 2);
  const dist = Math.hypot(dx, dy);
  if (dist > MAX_R){ dx = dx / dist * MAX_R; dy = dy / dist * MAX_R; }
  jsAil = dx / MAX_R; jsElv = -dy / MAX_R;
}
stick.addEventListener('pointerdown', e => {
  if (!st.manual) return;
  dragging = true;
  try { stick.setPointerCapture(e.pointerId); } catch (_) {}
  fromPointer(e.clientX, e.clientY); e.preventDefault();
});
stick.addEventListener('pointermove', e => { if (dragging) fromPointer(e.clientX, e.clientY); });
const release = () => { if (!dragging) return; dragging = false; jsAil = 0; jsElv = 0; };
stick.addEventListener('pointerup', release);
stick.addEventListener('pointercancel', release);

addEventListener('keydown', e => {
  if (!st.manual) return;
  const k = e.key.toLowerCase();
  if (k in keys){ keys[k] = true; e.preventDefault(); }
});
addEventListener('keyup', e => { const k = e.key.toLowerCase(); if (k in keys) keys[k] = false; });

function manualTick(){
  if (!st.manual) return;
  let tAil, tElv;
  if (dragging){ tAil = jsAil; tElv = jsElv; }
  else {
    tAil = (keys.d ? 1 : 0) - (keys.a ? 1 : 0);
    tElv = (keys.w ? 1 : 0) - (keys.s ? 1 : 0);
  }
  mAil += (tAil - mAil) * 0.25;               // ~0.3 s'de hedefe: PWM sıçraması olmasın
  mElv += (tElv - mElv) * 0.25;
  if (tAil === 0 && Math.abs(mAil) < 0.02) mAil = 0;
  if (tElv === 0 && Math.abs(mElv) < 0.02) mElv = 0;
  if (keys.l) mThr = Math.min(100, mThr + 1); // L/I: kalıcı gaz seviyesi
  if (keys.i) mThr = Math.max(0, mThr - 1);
  drawKnob();
  syncThrSlider(Math.round(mThr));

  sendTick++;
  if (sendTick % 2) return;                   // iç döngü 20 Hz → sunucuya 10 Hz
  fetch('/api/command/plane/manual', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      aileron:  Math.round(1500 + mAil * 450),
      elevator: Math.round(1500 + mElv * 450),
      throttle: Math.round(1000 + mThr * 10),
    }),
  }).catch(() => {});
}

async function enterManual(){
  manBtn.disabled = true;
  $('manLbl').textContent = 'Bağlanıyor...';
  addLog('sys', 'SYS', 'Aktif senaryo durduruluyor, uçuş devralınıyor (yerde FBWA → havada FBWB)...');
  st.scenario = null; markScenario();
  try {
    const r = await (await fetch('/api/command/plane/start_manual', { method: 'POST' })).json();
    if (r.status === 'success'){
      st.manual = true;
      for (const k in keys) keys[k] = false;
      mAil = 0; mElv = 0; jsAil = 0; jsElv = 0; dragging = false;
      mThr = parseInt($('thrSl').value, 10) || 60;   // kullanıcının ayarladığı gazla devam
      drawKnob();
      $('manwrap').hidden = false;
      manBtn.classList.add('open'); manBtn.setAttribute('aria-pressed', 'true');
      $('manLbl').textContent = 'Manuel Kapat';
      $('manBadge').hidden = st.tab !== 'hedef';
      addLog('sys', 'SYS', 'Manuel mod AKTİF — W/S pitch, A/D roll, L hızlan, I yavaşla.');
      manualLoop = setInterval(manualTick, 50);
    } else {
      $('manLbl').textContent = 'Manuel Mod';
      addLog('err', 'HATA', 'Manuel mod başlatılamadı: ' + r.message);
    }
  } catch (e){
    $('manLbl').textContent = 'Manuel Mod';
    addLog('err', 'HATA', 'Bağlantı hatası: ' + e);
  }
  manBtn.disabled = false;
  markScenario();
}
async function exitManual(){
  if (!st.manual) return;
  st.manual = false;
  clearInterval(manualLoop); manualLoop = null;
  dragging = false; jsAil = 0; jsElv = 0; mAil = 0; mElv = 0;
  drawKnob();
  $('manwrap').hidden = true;
  manBtn.classList.remove('open'); manBtn.setAttribute('aria-pressed', 'false');
  $('manLbl').textContent = 'Manuel Mod';
  $('manBadge').hidden = true;
  addLog('sys', 'SYS', 'Manuel mod kapatıldı.');
  markScenario();
  try { await fetch('/api/command/plane/stop_manual', { method: 'POST' }); } catch (e) {}
}
manBtn.addEventListener('click', () => { st.manual ? exitManual() : enterManual(); });

// ══ GAZ AYARI ══════════════════════════════════════════════════════════
// Her modda geçerli: senaryo modunda hedefin gazını, manuel modda doğrudan
// gazı sürer. Açılışta sunucudaki GERÇEK değer okunur.
const thrSl = $('thrSl');
let thrTimer = null;
function syncThrSlider(pct){
  if (parseInt(thrSl.value, 10) === pct) return;
  thrSl.value = pct; $('thrV').textContent = pct + '%';
}
fetch('/api/plane_throttle').then(r => r.json()).then(d => {
  const pct = Math.round((d.throttle ?? 600) / 10);
  thrSl.value = pct; $('thrV').textContent = pct + '%'; mThr = pct; drawKnob();
}).catch(() => {});
thrSl.addEventListener('input', () => {
  const pct = parseInt(thrSl.value, 10);
  $('thrV').textContent = pct + '%';
  if (st.manual) mThr = pct;
  clearTimeout(thrTimer);
  thrTimer = setTimeout(() => {
    fetch('/api/plane_throttle', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ throttle: Math.round(pct * 10) }),   // 0-100 → 0-1000
    }).catch(() => {});
  }, 100);
});

// ══ GPS KARIŞTIRMA ═════════════════════════════════════════════════════
const GPS_WORDS = ['GPS Sinyali Normal', 'Hafif Parazit', 'Orta Düzey Karışım',
                   'Şiddetli Karışım', 'GPS KAYBI — Görsel Seyir'];
const gpsSl = $('gpsSl');
let gpsTimer = null;
gpsSl.addEventListener('input', () => {
  const val = parseInt(gpsSl.value, 10);
  $('gpsV').textContent = val + '%';
  const i = Math.min(4, Math.floor(val / 20));
  const col = i >= 4 ? 'var(--red)' : i >= 2 ? 'var(--amber)' : 'var(--green)';
  const dot = $('gpsSig').querySelector('.d');
  dot.style.background = col; dot.style.boxShadow = '0 0 7px ' + col;
  $('gpsSigTxt').textContent = GPS_WORDS[i];
  $('gpsV').style.color = col;
  clearTimeout(gpsTimer);
  gpsTimer = setTimeout(() => {
    fetch('/api/gps_noise', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ level: val / 100 }),
    }).then(() => addLog('gps', 'GPS', 'Karıştırma seviyesi: %' + val)).catch(() => {});
  }, 120);
});
fetch('/api/gps_noise').then(r => r.json()).then(d => {
  gpsSl.value = Math.round((d.level || 0) * 100);
  gpsSl.dispatchEvent(new Event('input'));
  clearTimeout(gpsTimer);                       // açılışta okuduğumuzu geri yazma
}).catch(() => {});

// ══ VİDEO PARAZİT ══════════════════════════════════════════════════════
// Sunucu paraziti JPEG'e uyguluyor; buradaki tuval yalnız tarama-çizgisi
// (analog CRT) efektini üstüne bindirir.
const VID_WORDS = ['Temiz Analog Sinyal', 'Hafif Parazit', 'Orta Parazit',
                   'Şiddetli Parazit', 'SİNYAL YOK'];
const vidSl = $('vidSl');
let vidTimer = null, vidLevel = 0;
vidSl.addEventListener('input', () => {
  const val = parseInt(vidSl.value, 10);
  vidLevel = val / 100;
  $('vidV').textContent = val + '%';
  const i = Math.min(4, Math.floor(val / 20));
  const col = i >= 4 ? 'var(--red)' : i >= 2 ? 'var(--amber)' : 'var(--green)';
  const dot = $('vidSig').querySelector('.d');
  dot.style.background = col; dot.style.boxShadow = '0 0 7px ' + col;
  $('vidSigTxt').textContent = VID_WORDS[i];
  $('vidV').style.color = col;
  clearTimeout(vidTimer);
  vidTimer = setTimeout(() => {
    fetch('/api/video_noise', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ level: vidLevel }),
    }).then(() => addLog('sys', 'VID', 'Video parazit: %' + val)).catch(() => {});
  }, 120);
});
fetch('/api/video_noise').then(r => r.json()).then(d => {
  vidSl.value = Math.round((d.level || 0) * 100);
  vidSl.dispatchEvent(new Event('input'));
  clearTimeout(vidTimer);
}).catch(() => {});

const noiseCv = $('noiseCv');
function drawNoise(){
  const parent = noiseCv.parentElement;
  const W = parent.clientWidth, H = parent.clientHeight;
  if (!W || !H) return;
  if (noiseCv.width !== W || noiseCv.height !== H){ noiseCv.width = W; noiseCv.height = H; }
  const ctx = noiseCv.getContext('2d');
  ctx.clearRect(0, 0, W, H);
  const lvl = vidLevel;
  if (lvl <= 0) return;
  if (lvl >= 1){ ctx.fillStyle = '#000'; ctx.fillRect(0, 0, W, H); return; }
  if (lvl > 0.25){                                   // yatay tarama çizgileri
    const n = Math.floor(lvl * 20);
    for (let i = 0; i < n; i++){
      const y = Math.random() * H, lh = Math.random() * lvl * 4 + 1;
      const r = Math.random() * 255 | 0, g = Math.random() * 255 | 0, b = Math.random() * 255 | 0;
      ctx.fillStyle = `rgba(${r},${g},${b},${lvl * 0.85})`;
      ctx.fillRect(0, y, W, lh);
    }
  }
  if (lvl > 0.65){                                   // yüksek parazitte karartma
    ctx.fillStyle = `rgba(0,0,0,${Math.min((lvl - 0.65) * 2.2, 0.92)})`;
    ctx.fillRect(0, 0, W, H);
  }
}

// ══ GÖREV (hibrit güdüm / chase) ═══════════════════════════════════════
const startBtn = $('startBtn');
startBtn.addEventListener('click', async () => {
  if (!st.mission){
    startBtn.disabled = true;
    $('startLbl').textContent = 'Kalkış yapılıyor...';
    addLog('sys', 'CMD', 'Takip görevi başlatılıyor — avcı kalkacak, hibrit güdüm devreye girecek.');
    try {
      const r = await (await fetch('/api/command/iris/start_chase', { method: 'POST' })).json();
      if (r.status === 'success'){
        setMission(true);
        addLog('sys', 'SYS', '✓ Görev aktif — avcı hedefi takip ediyor.');
      } else {
        $('startLbl').textContent = 'Görevi Başlat';
        addLog('err', 'HATA', 'Görev başlatılamadı: ' + r.message);
      }
    } catch (e){
      $('startLbl').textContent = 'Görevi Başlat';
      addLog('err', 'HATA', 'Bağlantı hatası: ' + e);
    }
    startBtn.disabled = false;
  } else {
    setMission(false);
    addLog('sys', 'SYS', 'Görev durduruldu — avcı hover\'a geçiyor.');
    fetch('/api/command/iris/stop_chase', { method: 'POST' }).catch(() => {});
  }
});
function setMission(on){
  st.mission = on;
  startBtn.classList.toggle('on', on);
  $('startLbl').textContent = on ? 'Görevi Durdur' : 'Görevi Başlat';
}

// ══ DURUM YOKLAMA (chase / pnp / hasar) ════════════════════════════════
const G_ICONS = {
  gps:    '<path d="M12 21s-6-5.7-6-10a6 6 0 0 1 12 0c0 4.3-6 10-6 10z"/><circle cx="12" cy="11" r="2.3"/>',
  vision: '<path d="M2 12s3.6-7 10-7 10 7 10 7-3.6 7-10 7S2 12 2 12z"/><circle cx="12" cy="12" r="3"/>',
  hit:    '<path d="M12 3v6m0 6v6m-9-9h6m6 0h6"/><circle cx="12" cy="12" r="3"/>',
  idle:   '<circle cx="12" cy="12" r="8"/><path d="M12 8v4l3 2"/>',
};
let lastFaz = null, lastImha = false, lastLockTxt = null;

async function pollChase(){
  try {
    const d = await (await fetch('/api/chase_status')).json();
    if (d.active !== st.mission) setMission(d.active);
    st.faz = (d.active && d.supervisor) ? d.supervisor.faz : null;
  } catch (e){ st.faz = null; }
}
async function pollPnp(){
  try { st.pnp = await (await fetch('/api/telemetry/pnp')).json(); }
  catch (e){ st.pnp = null; }
}
async function pollHasar(){
  try {
    const d = await (await fetch('/api/hasar')).json();
    st.imha = !!d.imha;
    if (st.imha && !lastImha){
      lastImha = true;
      addLog('guide', 'VURUŞ', `✷ HEDEF İMHA — temas menzili ${d.menzil ?? '?'} m (${d.temas ?? 'gazebo teması'})`);
    } else if (!st.imha) lastImha = false;
  } catch (e){}
}

function renderStatus(){
  const p = st.pnp;
  const faz = st.faz;

  // ── Güdüm tipi: supervisor fazı ne diyorsa o ──
  //   GPS fazı    → detection modeli, hedefin GPS pozuna göre kadraj merkezleme
  //   Görsel faz  → pose modeli, kameradan lead pursuit
  let key, main, model, tel, mdl, col, cap;
  if (st.imha || faz === 'VURULDU'){
    key = 'hit'; main = 'HEDEF VURULDU'; model = 'GÖREV TAMAM';
    tel = 'GÖRSEL'; mdl = 'POSE'; col = 'var(--red)'; cap = 'fiziksel temas doğrulandı';
  } else if (faz === 'VISUAL'){
    key = 'vision'; main = 'GÖRSEL GÜDÜM'; model = 'POSE MODELİ';
    tel = 'GÖRSEL'; mdl = 'POSE'; col = 'var(--green)'; cap = 'pose modeli · lead pursuit';
  } else if (faz === 'GPS'){
    key = 'gps'; main = 'GPS GÜDÜM'; model = 'DETECTION MODELİ';
    tel = 'GPS'; mdl = 'DETECTION'; col = 'var(--amber)'; cap = 'detection modeli · kadraj merkezleme';
  } else {
    key = 'idle'; main = 'GÜDÜM BEKLEMEDE'; model = 'MODEL PASİF';
    tel = '—'; mdl = 'PASİF'; col = 'var(--muted)'; cap = 'güdüm bekliyor';
  }
  $('guideBadge').className = 'guidebadge ' + key;
  $('guideIcon').innerHTML = G_ICONS[key];
  $('guideTxt').innerHTML = '';
  $('guideTxt').append(main + ' ');
  const sub = document.createElement('span'); sub.className = 'sub'; sub.textContent = model;
  $('guideTxt').append(sub);
  const ag = $('aGuide'); ag.textContent = tel; ag.style.color = col;
  const am = $('aModel'); am.textContent = mdl; am.style.color = col;

  // ── Kaçıncı GPS→görsel GEÇİŞİ (eski arayüzden geri getirildi) ──
  // 1 ideal. Yüksek sayı görsel temasın kopup kopup yeniden kurulduğunu,
  // yani fazın gidip geldiğini gösterir — 5+ sorunlu sayılır.
  const gec = p && p.gecis_sayisi ? p.gecis_sayisi : 0;
  const ge = $('aGecis');
  if (ge){
    ge.textContent = gec ? `${gec}.` : '—';
    ge.className = 'tv' + (gec === 0 ? ' muted' : gec <= 2 ? ' green' : gec <= 4 ? '' : ' red');
  }
  $('sTermCap').textContent = cap + (gec ? ` · ${gec}. geçiş` : '');
  if (faz && faz !== lastFaz){
    lastFaz = faz;
    if (faz === 'GPS') addLog('gps', 'GÜDÜM', 'GPS fazı — detection modeliyle kadraj merkezleme.');
    else if (faz === 'VISUAL') addLog('vision', 'GÜDÜM', 'Görsel temas oturdu → POSE modeli, lead pursuit devrede.');
    else if (faz === 'VURULDU') addLog('guide', 'GÜDÜM', 'Terminal vuruş — hedefe temas.');
    else if (faz === 'DURDU') addLog('sys', 'GÜDÜM', 'Güdüm durdu.');
  }

  // ── Terminal mod kartı ──
  const termTxt = st.imha ? 'VURULDU'
    : faz === 'VISUAL' ? 'GÖRSEL FAZ'
    : faz === 'GPS' ? 'GPS FAZI'
    : faz === 'DURDU' ? 'DURDU'
    : st.mission ? 'TAKİP' : 'HAZIR';
  $('sTerm').textContent = termTxt;
  $('tTerm').textContent = termTxt;
  $('aTerm').textContent = termTxt;
  $('aTerm').className = 'tv' + (st.mission ? ' green' : ' muted');

  // ── Kilit durumu: görüş hattının GERÇEK çıktısından ──
  let lockTxt, lockCls, lockCap;
  if (st.imha){ lockTxt = 'VURULDU'; lockCls = 'hit'; lockCap = 'hedef imha edildi'; }
  else if (!st.mission){ lockTxt = 'BEKLEMEDE'; lockCls = 'idle'; lockCap = 'görev başlatılmadı'; }
  else if (p && p.pose_var){
    lockTxt = 'KİLİT'; lockCls = '';
    lockCap = `pose ${p.pose_conf ?? '--'}${p.kanat_gorunur ? '' : ' · kanat yok'}`;
  } else if (p && p.tespit_var){
    lockTxt = 'TESPİT'; lockCls = 'searching'; lockCap = `detection ${p.tespit_conf ?? '--'}`;
  } else { lockTxt = 'ARANIYOR'; lockCls = 'searching'; lockCap = 'hedef kadrajda yok'; }
  $('lockBadge').className = 'lockbadge ' + lockCls;
  $('lockBadge').textContent = lockTxt;
  $('sLock').textContent = lockTxt;
  $('sLockCap').textContent = lockCap;
  $('tLock').textContent = lockTxt;
  $('aLock').textContent = lockTxt;
  $('aLock').className = 'tv' + (lockTxt === 'KİLİT' ? ' green' : lockTxt === 'BEKLEMEDE' ? ' muted' : ' amber');
  if (st.mission && lockTxt !== lastLockTxt){
    lastLockTxt = lockTxt;
    if (lockTxt === 'KİLİT') addLog('vision', 'GÖRÜŞ', 'Pose kilidi kuruldu — hedef 6 keypoint ile görülüyor.');
    else if (lockTxt === 'ARANIYOR') addLog('gps', 'GÖRÜŞ', 'Görsel temas yok — hedef kadrajda değil.');
  }
  if (!st.mission) lastLockTxt = null;

  // ── FPV köşe verileri + görüş hattı paneli ──
  const yaz = (id, txt, cls) => { const e = $(id); e.textContent = txt; e.className = 'tv ' + (cls || ''); };
  if (p){
    $('oConf').textContent = p.tespit_var ? p.tespit_conf : '--';
    $('oPose').textContent = p.pose_var ? p.pose_conf : '--';
    $('aPct').textContent = p.kilit_n ? Math.round(100 * p.kilit_sayac / p.kilit_n) : 0;
    yaz('pDet', p.tespit_var
        ? `VAR ${p.tespit_conf}${p.det_px != null ? ` · ${p.det_px} px` : ''}`
        : 'YOK', p.tespit_var ? 'green' : 'red');
    // "Pose YOK" niye yok? Model kapalı mı, hedef çok mu küçük — ekran söylesin.
    // Gövde 0.81 m / FX≈167 px: 10 px ≈ 13.5 m. Altında pose keypoint bulamaz.
    let poseTxt, poseCls;
    if (p.pose_var){
      poseTxt = `VAR ${p.pose_conf}${p.kanat_gorunur ? '' : ' (kanat yok)'}`;
      poseCls = p.kanat_gorunur ? 'green' : 'amber';
    } else if (p.pose_model_var === false){
      poseTxt = 'MODEL YÜKLÜ DEĞİL'; poseCls = 'red';
    } else if (p.tespit_var && p.det_px != null && p.det_px < 10){
      poseTxt = `hedef çok küçük (${p.det_px} px)`; poseCls = 'amber';
    } else {
      poseTxt = 'YOK'; poseCls = 'red';
    }
    yaz('pPose', poseTxt, poseCls);
    yaz('pDist', p.gorus_menzil != null ? p.gorus_menzil.toFixed(1) + ' m' : '--',
        p.gorus_menzil == null ? '' : p.gorus_menzil < 10 ? 'red' : p.gorus_menzil < 30 ? 'amber' : 'green');
    yaz('pTruth', p.gercek_menzil != null ? p.gercek_menzil.toFixed(1) + ' m' : '--', '');
    if (p.menzil_hata != null){
      const h = p.menzil_hata;
      yaz('pErr', (h >= 0 ? '+' : '') + h.toFixed(1) + ' m',
          Math.abs(h) < 3 ? 'green' : Math.abs(h) < 8 ? 'amber' : 'red');
    } else yaz('pErr', '--', '');
    yaz('pLock', `${p.kilit_sayac}/${p.kilit_n} (son ${p.kilit_pencere} kare)`, p.poz_kapi_ok ? 'green' : 'amber');
    yaz('pDh', p.d_h != null ? `${p.d_h.toFixed(1)} / ${p.gate_menzil} m` : '--', p.menzil_kapi_ok ? 'green' : 'amber');
    yaz('pBlock', p.engel, p.engel === '—' ? 'green' : 'red');
  } else {
    yaz('pBlock', 'API YOK', 'red');
  }
}

// ══ PANEL DÜZENİ (büyüt / daralt) ══════════════════════════════════════
const appEl = document.querySelector('.app');
let posBig = false;
$('posExpand').addEventListener('click', () => {
  posBig = !posBig;
  if (posBig){ $('centerCol').appendChild($('posPanel')); appEl.classList.add('split');
               $('posExpand').textContent = '⤡'; $('posExpand').title = 'Küçült'; }
  else { $('rightCol').appendChild($('posPanel')); appEl.classList.remove('split');
         $('posExpand').textContent = '⤢'; $('posExpand').title = 'Büyüt'; }
  requestAnimationFrame(() => dispatchEvent(new Event('resize')));
});
let ctrlOpen = true;
$('ctrlToggle').addEventListener('click', () => {
  ctrlOpen = !ctrlOpen;
  $('ctrlBody').hidden = !ctrlOpen;
  appEl.classList.toggle('ctrl-collapsed', !ctrlOpen);
  $('ctrlToggle').textContent = ctrlOpen ? '⯆' : '⯈';
  $('ctrlToggle').title = ctrlOpen ? 'Paneli kapat' : 'Paneli aç';
  requestAnimationFrame(() => dispatchEvent(new Event('resize')));
});

// ══ ÇİZİM DÖNGÜSÜ ══════════════════════════════════════════════════════
function frame(now){
  camAz += (camAzT - camAz) * 0.18;
  camEl += (camElT - camEl) * 0.18;
  drawScene();
  drawNoise();
  const img = $('fpvImg');
  $('noFeed').hidden = !!(img && img.naturalWidth > 0);
  // (Aşağı yukarı süzülen yeşil "tarama" çizgisi KALDIRILDI — sinüsle
  //  hareket eden salt dekoratif bir öğeydi, hiçbir veriyi göstermiyordu.)
  requestAnimationFrame(frame);
}

// ══ AÇILIŞ ═════════════════════════════════════════════════════════════
markScenario();
drawKnob();
setTab('hedef');
connectWS();
pollChase(); pollPnp(); pollHasar();
setInterval(pollChase, 500);
setInterval(pollPnp, 700);
setInterval(pollHasar, 1000);
setInterval(renderStatus, 250);
renderStatus();
requestAnimationFrame(frame);
})();
