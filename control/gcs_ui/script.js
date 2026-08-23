/* ══════════════════════════════════════════════════════════════════════
   AVCI GCS — Taktik Saha Ekranı (canlı)

   Bu dosyada SİMÜLE VERİ YOKTUR. Ekrandaki her sayı gcs_server'dan gelir:
     ws://.../ws              → iris + plane telemetrisi (10 Hz)
     /api/video_feed/{iris|plane} → MJPEG (YOLO kutu overlay'i sunucuda çizili)
     /api/chase_status        → görev + supervisor fazı + gps_guidance durumu
     /api/telemetry/pnp       → görüş hattı (tespit/kutu/menzil) + faz kapıları
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
// GÖREV KAYDI paneli KALDIRILDI. addLog 33 yerden çağrılıyor; çağrıları tek
// tek sökmek yerine kapı burada kapatıldı. Paneli geri istersen: index.html'e
// logbody bölümünü ekle, aşağıdaki erken dönüşü sil — başka değişiklik gerekmez.
function addLog(cls, tag, msg){
  if (!logBody) return;
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
const SCN_LBL = {
  duz: 'DÜZ', square: 'KARE', circle: 'DAİRE', aggressive: 'AGRESİF',
  elips_gorev: 'ELİPS',
  circle_xl: 'DAİRE ⌀96', circle_l: 'DAİRE ⌀71', circle_s: 'DAİRE ⌀41',
};

// ══ KAMERA ─ MJPEG akışı ═══════════════════════════════════════════════
// MJPEG multipart bağlantısı süresiz açık kalır; src değiştirmek eski
// bağlantıyı her tarayıcıda kapatmaz. Bu yüzden <img> DOM'dan silinip
// yeniden kurulur (klasik arayüzdeki switchCamera ile aynı gerekçe).
function switchCamera(vehicle){
  // Video, 4:3 sahneye (fpvStage) eklenir — kilit paneli ayrı kolonda kalır.
  const wrap = $('fpvStage') || $('fpvwrap');
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

// SEKME YALNIZ SOLDAKİ KONTROL PANELİNİ DEĞİŞTİRİR — kameralara DOKUNMAZ.
// Eskiden sekme değişince ana ekrandaki kamera da değişiyordu ve "artık ana
// ekranda olan" görüşün ayrı penceresi KAPATILIYORDU: Hedef İHA sekmesine
// geçilince kullanıcının açtığı Talon penceresi kayboluyordu (aynı şey ters
// yönde de oluyordu). Artık FPV ekranı SABİT avcı drone kamerasıdır
// (bkz. ANA_KEY) ve Talon burun kamerası kendi TL penceresinde kalır — beş
// pencerenin hiçbiri sekme değişiminden etkilenmez.
function setTab(t){
  st.tab = t;
  $('segT').setAttribute('aria-pressed', String(t === 'hedef'));
  $('segA').setAttribute('aria-pressed', String(t === 'avci'));
  $('viewHedef').hidden = t !== 'hedef';
  $('viewAvci').hidden  = t !== 'avci';
  addLog('sys', 'SYS', t === 'hedef' ? 'Kontrol: HEDEF İHA' : 'Kontrol: AVCI DRONE');
}
$('segT').addEventListener('click', () => { if (st.tab !== 'hedef') setTab('hedef'); });
$('segA').addEventListener('click', () => { if (st.tab !== 'avci')  setTab('avci'); });

// ══ KAMERA GÖRÜŞLERİ ─ pencereler ══════════════════════════════════════
// Üst çubuktaki daireler dört görüşün TAMAMINI yönetir. Hangi daire ne yapar,
// SEKMEDEN BAĞIMSIZDIR (2026-08-20 — sekme değişiminde pencere kapanması):
//   • AV (iris) = ANA görüş: FPV takip ekranını pencereye alır / panele geri
//     koyar. Ana ekranın kamerası her zaman budur, sekme onu değiştirmez.
//   • TL / AVD / TLD: kendi taşınabilir-boyutlandırılabilir penceresini
//     açar/kapatır.
// Böylece dört görüş de aynı anda izlenebilir, hiçbiri iki kez çizilmez ve
// hiçbiri sekme değiştirince kaybolmaz.
// Akışlar sunucuda: /api/video_feed/{iris|plane|iris_chase|talon_chase}
const CAMS = [
  { key: 'iris',        kod: 'AV',  ad: 'Avcı Drone Kamerası',
    alt: 'iris ön kamera — tespit/kilit overlay’i sunucuda çizili' },
  { key: 'plane',       kod: 'TL',  ad: 'Talon Kamerası',
    alt: 'hedef İHA burun kamerası — ham' },
  { key: 'iris_chase',  kod: 'AVD', ad: 'Gazebo · Avcı Dış Görüş',
    alt: 'avcıyı arkadan/üstten gösteren sahne kamerası — ham' },
  { key: 'talon_chase', kod: 'TLD', ad: 'Gazebo · Talon Dış Görüş',
    alt: 'talonu arkadan/üstten gösteren sahne kamerası — ham' },
];
// Dış görüş kameraları SDF'teki chase_camera sensörlerinden gelir ve YALNIZ
// Gazebo Harmonic (gz-transport) yolunda yayınlanır. Boş kalmasının iki tipik
// sebebi var, ikisi de kurulumla ilgili — "bağlantı koptu" değil:
//   1) Gazebo, sensörler eklenmeden ÖNCE başlatılmış (model belleğe alınmış),
//   2) ROS 2 (Classic) yolu kullanılıyor, o köprü bu topic'leri yayınlamıyor.
const CHASE_IPUCU = 'Gazebo, dış görüş sensörleriyle yeniden başlatılmalı ' +
                    '(Harmonic + AVCI_GZ_CAMERA=1)';

const camWins = new Map();          // key → {win, img, nofeed, dot, ad}
let camZ = 60;

// ANA sahnedeki (FPV takip ekranı) görüş SABİTTİR: avcı drone kamerası.
// Sekmeye bağlı DEĞİL — bkz. setTab'daki gerekçe. AV dairesi bu yüzden her
// zaman "takip ekranını pencereye al / panele geri koy" demektir.
const ANA_KEY = 'iris';
function camAnaKey(){ return ANA_KEY; }

// ══ PENCERE DÜZENİ — varsayılan yerleşim + hatırlama ═══════════════════
// Beş pencerenin (AV takip ekranı · KNM konum izleme · TL/AVD/TLD kameralar)
// açılış GENİŞLİĞİ, YÜKSEKLİĞİ ve KONUMU tek yerden gelir. Sebebi: her oturumda
// beşini de elle yerleştirmek gerekiyordu — kamera pencereleri sağ üst köşeden
// 26 px'lik kademeyle açıldığı için üst üste biniyorlardı.
//
// Yerleşim ORTA SÜTUNA (#centerCol) göre ORANLA hesaplanır, piksel sabitiyle
// değil: sol panel katlansa da, ekran boyu değişse de düzen kendini yeniden
// ölçer. Oranlar kullanıcının elle kurduğu düzenden alındı (2026-08-20 ekran
// görüntüsü):
//     ┌───────────────────────┬─────────────────┐  üst sıra = alanın %64'ü
//     │   AV  (takip ekranı)  │   KNM (konum)   │  genişlik %52 | %48
//     ├───────────────────────┼────────┬────────┤
//     │   AVD (avcı dış)      │   TL   │  TLD   │  alt sıra = kalan
//     └───────────────────────┴────────┴────────┘
//
// 2026-08-22 (kullanıcı isteği): AVD tam AV'nin ALTINDA — ikisi de AVCI
// aracına bakar, alt alta durunca aynı aracın iç/dış görüşü yan yana okunur.
// TL ve TLD YAN YANA — ikisi de HEDEF Talon'a ait (burun kamerası + dış
// görüş). Önceki düzende (TL | AVD | TLD) iki aracın görüşleri birbirine
// karışıyordu.
//
// Kullanıcı bir pencereyi TAŞIR ya da BOYUTLANDIRIRSA yalnız o pencerenin
// geometrisi localStorage'a yazılır ve sonraki açılışta o kullanılır.
// Dokunulmayan pencereler varsayılanda kalır (böylece ekran boyu değişince
// kendilerini yeniden ölçerler). Üst çubuktaki ⟲ dairesi kaydı siler.
// ⚠ SÜRÜM v2 (2026-08-22): alt sıra düzeni değişti. v1 kaydı taşıyan
// pencerelere yol açıyordu (eski kademeli konumlar kaydedilmişti);
// anahtarı yükseltmek o kaydı otomatik geçersiz kılar.
const DUZEN_ANAHTAR = 'avci.pencere.duzen.v2';
const FPV_KEY   = ANA_KEY;   // takip ekranı — kamera penceresi DEĞİL, panelin kendisi
const POS_DOT   = '__pos';   // konum izleme paneli
const DUZEN_DOT = '__duzen'; // "düzeni sıfırla" dairesi (kamera değil)
const PEN_BOSLUK = 8;        // pencereler arası boşluk — .main grid gap'iyle aynı
// Alt sıradaki üç küçük pencerenin anahtarları (SOLDAN SAĞA):
//   iris_chase (AVD) = avcı dış görüş — AV'nin altında
//   plane (TL) + talon_chase (TLD) = Talon'un iki kamerası, yan yana
const DUZEN_ALT = ['iris_chase', 'plane', 'talon_chase'];

// Orta sütunun ekrandaki dikdörtgeni. Takip ekranı pencereye alınınca bu sütun
// boşalır ama grid genişliğini/yüksekliğini KORUR (grid-template-columns'ta
// minmax(0,1fr), align-items:stretch) — ölçüm bu yüzden güvenilir. Yine de
// güvenlik kapısı var: dar ekranda (.main tek sütuna düşer, <=1080 px) ya da
// ölçüm alınamazsa tüm görüntü alanı kullanılır.
function ortaAlan(){
  const r = $('centerCol')?.getBoundingClientRect();
  if (!r || r.width < 360 || r.height < 260){
    const ust = document.querySelector('.topbar')?.getBoundingClientRect().bottom ?? 80;
    return { x: PEN_BOSLUK, y: ust + PEN_BOSLUK,
             w: innerWidth - 2 * PEN_BOSLUK, h: innerHeight - ust - 2 * PEN_BOSLUK };
  }
  return { x: r.left, y: r.top, w: r.width, h: r.height };
}

// Beş pencerenin VARSAYILAN geometrisi — orta alanın oranlarından.
function varsayilanDuzen(){
  const a = ortaAlan(), g = PEN_BOSLUK;
  const ustH = Math.round((a.h - g) * 0.64);          // üst sıra: AV + KNM
  const altH = a.h - g - ustH;                        // kalan: AVD + TL + TLD
  const avW  = Math.round((a.w - g) * 0.52);          // AV sütunu
  const sagW = a.w - avW - g;                         // KNM sütunu
  const tlW  = Math.round((sagW - g) / 2);            // Talon'un iki kamerası
  const altT = a.y + ustH + g;
  const d = {};
  d[FPV_KEY]      = { l: a.x,                    t: a.y,  w: avW, h: ustH };
  d[POS_DOT]      = { l: a.x + avW + g,          t: a.y,  w: sagW, h: ustH };
  // AVD tam AV'nin altında ve AYNI GENİŞLİKTE — aynı araca bakan iki görüş.
  d['iris_chase'] = { l: a.x,                    t: altT, w: avW, h: altH };
  // TL + TLD yan yana, KNM sütununu paylaşır — ikisi de Talon'a ait.
  d['plane']       = { l: a.x + avW + g,         t: altT, w: tlW, h: altH };
  d['talon_chase'] = { l: a.x + avW + g + tlW + g, t: altT, w: sagW - tlW - g, h: altH };
  return d;
}

// localStorage okuma/yazma — depo kapalıysa (gizli sekme, kota) sessizce
// varsayılana düşer; düzen bir konfor özelliğidir, uçuşu etkilemez.
function duzenOku(){
  try { return JSON.parse(localStorage.getItem(DUZEN_ANAHTAR)) || {}; }
  catch { return {}; }
}
function duzenYaz(o){
  try { localStorage.setItem(DUZEN_ANAHTAR, JSON.stringify(o)); } catch {}
}

// Bir pencerenin AÇILIŞ geometrisi: kullanıcının kaydettiği varsa o, yoksa
// varsayılan. Her iki durumda da ekrana kelepçelenir — kayıt daha büyük bir
// ekranda kurulmuş olabilir, orada geçerli konum burada dışarıda kalırdı.
function duzenGeo(key){
  const say = v => typeof v === 'number' && isFinite(v);
  let g = (duzenOku().geo || {})[key];
  if (!(g && say(g.l) && say(g.t) && say(g.w) && say(g.h))) g = varsayilanDuzen()[key];
  if (!g) g = { l: 40, t: 100, w: 420, h: 354 };      // bilinmeyen anahtar — olmamalı
  const w = clamp(Math.round(g.w), CW_MIN_W, Math.max(CW_MIN_W, innerWidth  - 16));
  const h = clamp(Math.round(g.h), CW_MIN_H, Math.max(CW_MIN_H, innerHeight - 16));
  return { w, h,
           l: clamp(Math.round(g.l), 0, Math.max(0, innerWidth  - w)),
           t: clamp(Math.round(g.t), 0, Math.max(0, innerHeight - h)) };
}

function geoUygula(el, g){
  el.style.left = g.l + 'px';  el.style.top    = g.t + 'px';
  el.style.width = g.w + 'px'; el.style.height = g.h + 'px';
}

// Bir DOM öğesi hangi pencere? (geometri kaydında anahtar olarak kullanılır)
function pencereAnahtari(el){
  if (el.classList.contains('fpvpanel')) return FPV_KEY;
  if (el.id === 'posPanel') return POS_DOT;
  for (const [k, w] of camWins) if (w.win === el) return k;
  return null;
}

// YALNIZ taşınan/boyutlandırılan pencerenin geometrisi yazılır. Hepsini birden
// yazmak, dokunulmamış pencereleri de "elle ayarlanmış" sayardı; o zaman
// varsayılan yerleşim ekran boyu değiştiğinde bir daha uygulanamazdı.
function duzenGeoKaydet(el){
  const key = pencereAnahtari(el);
  if (!key || el.classList.contains('max')) return;   // tam ekran hâli kaydedilmez
  const r = el.getBoundingClientRect();
  const o = duzenOku();
  (o.geo = o.geo || {})[key] = { l: Math.round(r.left), t: Math.round(r.top),
                                 w: Math.round(r.width), h: Math.round(r.height) };
  duzenYaz(o);
}

// Hangi pencereler AÇIK — bir sonraki oturum aynı takımla açılsın. Kapattığın
// pencere kapalı gelir; hiç kayıt yoksa BEŞİ de açılır (varsayılan düzen).
function duzenAcikKaydet(){
  const o = duzenOku();
  const a = {};
  a[FPV_KEY] = !!document.querySelector('.fpvpanel.pencere');
  a[POS_DOT] = !!document.querySelector('#posPanel.pencere');
  for (const k of DUZEN_ALT) a[k] = camWins.has(k);
  o.acik = a;
  duzenYaz(o);
}

// AÇILIŞ DÜZENİ — sayfa yüklenince çağrılır (bkz. dosya sonu).
function duzenAcilista(){
  const acik = duzenOku().acik;
  const iste = k => (acik ? !!acik[k] : true);        // kayıt yoksa hepsi açık
  if (iste(FPV_KEY)) setFpvPencere(true);
  if (iste(POS_DOT)) posPen.ayarla(true);
  for (const k of DUZEN_ALT) if (iste(k)) openCamWin(k);
}

// ⟲ — kaydı at, beş pencereyi de varsayılan yerine oturt. Elle dağıtılmış bir
// düzeni tek tıkla toparlamanın yolu; ekran boyu değişince de bunu kullan.
function duzenSifirla(){
  duzenYaz({});                                       // kayıt gitti → duzenGeo varsayılanı verir
  if (!fpvPen.pencereMi()) setFpvPencere(true); else geoUygula(fpvPanel, duzenGeo(FPV_KEY));
  if (!posPen.pencereMi()) posPen.ayarla(true);  else geoUygula(posPanel, duzenGeo(POS_DOT));
  for (const k of DUZEN_ALT){
    if (!camWins.has(k)) { openCamWin(k); continue; }
    const w = camWins.get(k).win;
    camMax(w, false);                                 // büyütülmüşse önce eski hâline
    geoUygula(w, duzenGeo(k));
  }
  duzenAcikKaydet();
  requestAnimationFrame(() => dispatchEvent(new Event('resize')));
  addLog('sys', 'SYS', 'Pencere düzeni varsayılana döndürüldü.');
}


// Dairelerin görünümü tek yerden kurulur: hangisi ana ekran, hangisinin
// penceresi açık, başlıkta ne yazacak.
function camDotDurum(){
  for (const b of camDock.children){
    // DÜZEN dairesi (⟲) bir görüş değil, komut düğmesidir: durumu yok.
    if (b.dataset.cam === DUZEN_DOT) continue;
    // KONUM dairesi CAMS'te yok (kamera değil) — kendi kuralıyla işlenir.
    // Anahtar burada düz metin: bu döngü POS_DOT sabiti ilklenmeden de
    // çalışabilir ve sabite dokunmak TDZ hatası verirdi.
    if (b.dataset.cam === '__pos'){
      const acik = !!document.querySelector('#posPanel.pencere');
      b.setAttribute('aria-pressed', String(acik));
      b.title = 'Konum İzleme — ' + (acik ? 'basınca panele geri döner' : 'basınca pencereye alınır');
      b.setAttribute('aria-label', b.title);
      continue;
    }
    const c = CAMS.find(x => x.key === b.dataset.cam);
    const ana = c.key === camAnaKey();
    // .fpvpanel DOM'dan okunuyor: camDotDurum açılışta, CAMS döngüsünün hemen
    // ardından çalışıyor ve FPV blokundaki const'lar (fpvPanel/fpvPencereMi)
    // o an henüz ilklenmemiş oluyor — onlara dokunmak TDZ hatası verirdi.
    const pencerede = !!document.querySelector('.fpvpanel.pencere');
    const acik = ana ? pencerede : camWins.has(c.key);
    b.classList.toggle('ana', ana);
    b.setAttribute('aria-pressed', String(acik));
    b.title = ana
      ? `${c.ad} — ANA EKRAN` + (acik ? ' · basınca panele geri döner' : ' · basınca pencereye alınır')
      : `${c.ad} — ${c.alt}` + (acik ? ' · basınca pencere kapanır' : ' · basınca pencerede açılır');
    b.setAttribute('aria-label', ana
      ? `${c.ad} — takip ekranını ${acik ? 'panele geri koy' : 'pencereye al'}`
      : `${c.ad} penceresi`);
  }
}

function camBringFront(key){
  const w = camWins.get(key);
  if (!w) return;
  w.win.style.zIndex = ++camZ;
  for (const o of camWins.values()) o.win.classList.toggle('front', o === w);
}

const CW_MIN_W = 220, CW_MIN_H = 180;   // altına inilemeyen pencere boyutu

// Büyült / eski boyuta dön. Eski geometri pencerenin üzerinde saklanır, böylece
// küçültünce kullanıcının kendi ayarladığı boyut geri gelir.
function camMax(win, buyut){
  if (buyut){
    win._eski = { left: win.style.left, top: win.style.top,
                  width: win.style.width, height: win.style.height };
    win.style.left = '8px';
    win.style.top  = '8px';
    win.style.width  = (innerWidth  - 16) + 'px';
    win.style.height = (innerHeight - 16) + 'px';
  } else if (win._eski){
    Object.assign(win.style, win._eski);
  }
  win.classList.toggle('max', !!buyut);
  const b = win.querySelector('.cw-max');
  if (b){
    b.textContent = buyut ? '❐' : '▢';
    b.title = buyut ? 'Eski boyuta dön' : 'Tam ekran büyüt';
    b.setAttribute('aria-label', b.title);
  }
}

// Sürükleme ve boyutlandırma tek kalıp: pointer capture ile — imleç pencere
// dışına taşsa da olaylar gelmeye devam eder (manuel çubuktaki yaklaşımla aynı).
// mode: 'move' | yön dizgesi ('n','s','e','w','ne','nw','se','sw').
// izin: opsiyonel kapı — false dönerse sürükleme başlamaz. Ana FPV paneli için
// gerekli: o panel yalnız PENCERE modunda taşınır, grid içindeyken tutamakları
// sessiz kalmalı (kamera pencereleri bu parametreyi vermez, davranışı aynı).
// kelepce: opsiyonel taşıma sınırı (L,T,w,h) → {L,T}. Verilmezse "pencere
// tamamen ekranda kalır" kuralı işler; büyük FPV paneli için yetersiz kaldığı
// için orada masaüstü kuralı geçiliyor (bkz. çağrı yeri).
function camDrag(win, handle, mode, izin, kelepce){
  let sx = 0, sy = 0, x0 = 0, y0 = 0, w0 = 0, h0 = 0, on = false;
  handle.addEventListener('pointerdown', e => {
    if (e.button !== 0) return;
    if (izin && !izin(e)) return;
    // Büyütülmüş pencereyi sürüklemek/boyutlandırmak onu eski boyutuna
    // döndürür — "kilitlendi" hissi vermesin diye.
    if (win.classList.contains('max')) camMax(win, false);
    on = true;
    sx = e.clientX; sy = e.clientY;
    const r = win.getBoundingClientRect();
    x0 = r.left; y0 = r.top; w0 = r.width; h0 = r.height;
    win.classList.add('busy', mode === 'move' ? 'dragging' : 'resizing');
    handle.setPointerCapture(e.pointerId);
    e.preventDefault();
  });
  handle.addEventListener('pointermove', e => {
    if (!on) return;
    const dx = e.clientX - sx, dy = e.clientY - sy;
    if (mode === 'move'){
      let L = x0 + dx, T = y0 + dy;
      if (kelepce) ({ L, T } = kelepce(L, T, w0, h0));
      else {
        // Pencere tamamen ekranda kalır — kaybolup geri getirilememesin.
        L = clamp(L, 0, Math.max(0, innerWidth  - w0));
        T = clamp(T, 0, Math.max(0, innerHeight - h0));
      }
      win.style.left = L + 'px';
      win.style.top  = T + 'px';
      return;
    }
    // Boyutlandırma sekiz yönden. Sol/üst kenar çekilirken pencere hem
    // küçülür hem KAYAR; karşı kenar sabit kalsın diye left/top da güncellenir.
    let L = x0, T = y0, W = w0, H = h0;
    if (mode.includes('e')) W = clamp(w0 + dx, CW_MIN_W, Math.max(CW_MIN_W, innerWidth  - x0));
    if (mode.includes('s')) H = clamp(h0 + dy, CW_MIN_H, Math.max(CW_MIN_H, innerHeight - y0));
    if (mode.includes('w')){
      W = clamp(w0 - dx, CW_MIN_W, x0 + w0);   // sol kenar ekranın dışına taşmasın
      L = x0 + w0 - W;
    }
    if (mode.includes('n')){
      H = clamp(h0 - dy, CW_MIN_H, y0 + h0);
      T = y0 + h0 - H;
    }
    win.style.width = W + 'px'; win.style.height = H + 'px';
    win.style.left  = L + 'px'; win.style.top    = T + 'px';
  });
  const bitir = e => {
    if (!on) return;
    on = false;
    win.classList.remove('busy', 'dragging', 'resizing');
    if (e && handle.hasPointerCapture?.(e.pointerId)) handle.releasePointerCapture(e.pointerId);
    // Elle ayarlanan yer/boyut bir dahaki açılışta da geçerli olsun.
    duzenGeoKaydet(win);
  };
  handle.addEventListener('pointerup', bitir);
  handle.addEventListener('pointercancel', bitir);
}

function openCamWin(key){
  if (camWins.has(key)){ camBringFront(key); return; }
  const c = CAMS.find(x => x.key === key);
  if (!c) return;

  const win = document.createElement('div');
  win.className = 'camwin';
  // Boyut ve konum DÜZEN'den gelir (bkz. PENCERE DÜZENİ bölümü): kullanıcının
  // kaydettiği geometri varsa o, yoksa orta sütunun alt sırasındaki yeri.
  // Eskiden sabit 420 px'lik pencere sağ üstten kademeli açılıyor ve üçü üst
  // üste biniyordu — her açılışta elle dağıtmak gerekiyordu.
  geoUygula(win, duzenGeo(key));
  // Sekiz boyutlandırma tutamacı: dört kenar + dört köşe. Sağ alt köşe ayrıca
  // görünür bir işaret taşır (.cw-grip), çünkü kenarlar görünmezdir ve
  // pencerenin boyutlandırılabildiğinin tek görsel ipucu odur.
  const YONLER = ['n', 's', 'e', 'w', 'ne', 'nw', 'sw'];
  win.innerHTML =
    '<div class="cw-head"><span class="cw-title"></span>' +
    '<button class="cw-max" type="button" title="Tam ekran büyüt" aria-label="Tam ekran büyüt">▢</button>' +
    '<button class="cw-x" type="button" title="Kapat" aria-label="Kamera penceresini kapat">×</button></div>' +
    '<div class="cw-body"><div class="cw-nofeed"></div></div>' +
    YONLER.map(y => `<div class="cw-rs ${y}" data-yon="${y}"></div>`).join('') +
    '<div class="cw-grip cw-rs se" data-yon="se" title="Köşeden boyutlandır"></div>';
  win.querySelector('.cw-title').textContent = c.ad;

  const nofeed = win.querySelector('.cw-nofeed');
  nofeed.textContent = key.endsWith('_chase')
    ? 'GÖRÜNTÜ BEKLENİYOR — ' + CHASE_IPUCU
    : 'GÖRÜNTÜ BEKLENİYOR';

  // MJPEG: <img> ana sahnedeki switchCamera ile aynı kuralla kurulur/yıkılır —
  // src değiştirmek multipart bağlantısını her tarayıcıda kapatmaz, o yüzden
  // kapanışta element DOM'dan silinir (bkz. closeCamWin).
  const body = win.querySelector('.cw-body');
  const img = document.createElement('img');
  img.alt = '';
  img.src = `/api/video_feed/${key}?t=${Date.now()}`;
  body.insertBefore(img, body.firstChild);

  const dot = camDock.querySelector(`[data-cam="${key}"]`);
  win.querySelector('.cw-x').addEventListener('click', () => closeCamWin(key));
  win.querySelector('.cw-max').addEventListener('click',
    () => camMax(win, !win.classList.contains('max')));
  // Başlığa çift tıklamak da büyütür/küçültür (alışılmış pencere davranışı).
  win.querySelector('.cw-head').addEventListener('dblclick',
    () => camMax(win, !win.classList.contains('max')));
  win.addEventListener('pointerdown', () => camBringFront(key));
  camDrag(win, win.querySelector('.cw-head'), 'move');
  for (const h of win.querySelectorAll('.cw-rs')) camDrag(win, h, h.dataset.yon);

  document.body.appendChild(win);
  camWins.set(key, { win, img, nofeed, dot, ad: c.ad });
  camDotDurum();
  camBringFront(key);
  duzenAcikKaydet();
  addLog('sys', 'SYS', `Kamera penceresi açıldı: ${c.ad}`);
}

function closeCamWin(key){
  const w = camWins.get(key);
  if (!w) return;
  w.img.src = '';                    // MJPEG bağlantısını bırak (element de siliniyor)
  w.img.remove();
  w.win.remove();
  camWins.delete(key);
  w.dot.classList.remove('live');
  camDotDurum();
  duzenAcikKaydet();
  addLog('sys', 'SYS', `Kamera penceresi kapatıldı: ${w.ad}`);
}

const camDock = $('camDock');
for (const c of CAMS){
  const b = document.createElement('button');
  b.type = 'button';
  b.className = 'camdot';
  b.dataset.cam = c.key;
  b.textContent = c.kod;
  // ANA ekrandaki görüşün dairesi (varsayılan sekmede AV — Avcı Drone Kamerası)
  // TAKİP EKRANINI pencereye alır / panele geri koyar. Diğer daireler eskisi
  // gibi kendi kamera penceresini açar/kapatır — böylece dört dairenin dördü de
  // "bu görüşü pencerede aç" demek oluyor.
  b.addEventListener('click', () => {
    if (c.key === camAnaKey())        setFpvPencere(!document.querySelector('.fpvpanel').classList.contains('pencere'));
    else if (camWins.has(c.key))      closeCamWin(c.key);
    else                              openCamWin(c.key);
  });
  camDock.appendChild(b);
}
// 5. DAİRE — KONUM İZLEME. CAMS'e eklenmedi bilerek: orada olsaydı openCamWin
// /camAnaKey/video_feed onu bir kamera sanardı. Aynı .camdot görünümünü
// paylaşır, ayrı anahtarla (POS_DOT) ayırt edilir.
{
  const b = document.createElement('button');
  b.type = 'button';
  b.className = 'camdot';
  b.dataset.cam = POS_DOT;
  b.textContent = 'KNM';
  b.addEventListener('click', () => posPen.ayarla(!posPen.pencereMi()));
  camDock.appendChild(b);
}
// 6. DAİRE — DÜZENİ SIFIRLA. Kamera değil, komut düğmesi: elle dağıtılmış beş
// pencereyi varsayılan yerleşime döndürür (kapalı olanları da açar).
{
  const b = document.createElement('button');
  b.type = 'button';
  b.className = 'camdot duzen';
  b.dataset.cam = DUZEN_DOT;
  b.textContent = '⟲';
  b.title = 'Pencere düzenini varsayılana döndür — beş pencere de yerine oturur';
  b.setAttribute('aria-label', b.title);
  b.addEventListener('click', duzenSifirla);
  camDock.appendChild(b);
}
camDotDurum();   // açılıştaki durum (ana ekran dairesi dolu başlar)

// Pencere küçülünce dışarıda kalan kamera pencerelerini içeri çek.
addEventListener('resize', () => {
  for (const { win } of camWins.values()){
    const r = win.getBoundingClientRect();
    win.style.left = clamp(r.left, 0, Math.max(0, innerWidth  - r.width))  + 'px';
    win.style.top  = clamp(r.top,  0, Math.max(0, innerHeight - r.height)) + 'px';
  }
});

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
    // NOT: st.rangeRate şu an EKRANDA GÖSTERİLMİYOR — tek tüketicisi silinen
    // "Hedef Mesafe" kartının "yaklaşıyor/uzaklaşıyor" alt satırıydı. Hesap
    // bilerek bırakıldı: yaklaşma/uzaklaşma işaretini üreten tek yer burası ve
    // maliyeti iki çarpma. Gösterecek yeni bir alan olursa hazır.
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
  // FPV üstü telemetri — FPV ekranı SABİT avcı kamerası olduğu için hep iris
  const v = ir;
  if (v){
    $('oAlt').textContent = (-v.z).toFixed(0);
    $('oSpd').textContent = (v.speed ?? 0).toFixed(1);
    $('oHdg').textContent = String(Math.round((v.yaw + 360) % 360)).padStart(3, '0');
  }
  if (st.range !== null){
    $('oRng').textContent = st.range.toFixed(0);
    $('posRng').textContent = 'MENZİL ' + st.range.toFixed(0) + ' m';
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
  // Konum paneli yalnız pencere modunda görünür; gizliyken çizim boşa CPU.
  if (document.getElementById('posPanel')?.hidden) return;
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
// #cap = daire çapı varyantları; aynı senaryo API'sini kullanırlar, o yüzden
// #scn ile tek liste hâlinde yönetilirler.
const scnBtns = [...document.querySelectorAll('#scn [data-scn], #cap [data-scn]')];
// Etiketler HTML'den okunur (elle yazılmış eşleme yerine) — yeni bir senaryo
// butonu eklendiğinde burayı güncellemeyi unutmak "undefined" yazdırıyordu.
scnBtns.forEach(b => { b.dataset.base = b.querySelector('span').textContent; });
function markScenario(){
  scnBtns.forEach(b => b.setAttribute('aria-pressed', String(b.dataset.scn === st.scenario)));
  const lbl = st.scenario ? SCN_LBL[st.scenario] : null;
  scnBtns.forEach(b => {
    const span = b.querySelector('span');
    const base = b.dataset.base;
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
    // NOT: manuel KALKIŞ FAZI göstergesi çıkarıldı (2026-08-20). Onu besleyen
    // `manuel_faz` alanını /api/scenario_status bu dalda YAYINLAMIYOR
    // (hit_irtifa_tutucu dalındaki manuel-kalkış eklentisiyle geliyordu) ve
    // burada start_manual havada devralıyor — kalkış fazı hiç oluşmuyor.
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
  addLog('sys', 'SYS', 'Aktif senaryo durduruluyor, uçuş devralınıyor. '
        + 'Uçak yerdeyse önce KENDİ KALKIŞINI yapar.');
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
      $('manBadge').hidden = false;   // rozet manuel moda bağlı, sekmeye değil
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
    // Güdüm modu vurgusu SUNUCUDAKİ gerçek modu izler; buton kendi kararına
    // güvenmez (görev sırasında mod değişebiliyor).
    if (d.mode) modVurgula(d.mode);
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
  //   Görsel faz  → tespit kutusu, kameradan saf takip (bbox IBVS)
  let key, main, model, tel, mdl, col;
  if (st.imha || faz === 'VURULDU'){
    key = 'hit'; main = 'HEDEF VURULDU'; model = 'GÖREV TAMAM';
    tel = 'GÖRSEL'; mdl = 'KUTU'; col = 'var(--red)';
  } else if (faz === 'VISUAL'){
    key = 'vision'; main = 'GÖRSEL GÜDÜM'; model = 'TESPİT KUTUSU';
    tel = 'GÖRSEL'; mdl = 'KUTU'; col = 'var(--green)';
  } else if (faz === 'GPS'){
    key = 'gps'; main = 'GPS GÜDÜM'; model = 'DETECTION MODELİ';
    tel = 'GPS'; mdl = 'DETECTION'; col = 'var(--amber)';
  } else {
    key = 'idle'; main = 'GÜDÜM BEKLEMEDE'; model = 'MODEL PASİF';
    tel = '—'; mdl = 'PASİF'; col = 'var(--muted)';
  }
  // ── Kaçıncı GPS→görsel GEÇİŞİ (eski arayüzden geri getirildi) ──
  // 1 ideal. Yüksek sayı görsel temasın kopup kopup yeniden kurulduğunu,
  // yani fazın gidip geldiğini gösterir — 5+ sorunlu sayılır.
  // FAZIN YANINDA gösterilir: "hangi fazdayım" ile "kaçıncı kez bu faza
  // girdim" birlikte okunmalı, ayrı yerlerde işe yaramıyor.
  const gec = p && p.gecis_sayisi ? p.gecis_sayisi : 0;

  $('guideBadge').className = 'guidebadge ' + key;
  $('guideIcon').innerHTML = G_ICONS[key];
  $('guideTxt').innerHTML = '';
  $('guideTxt').append(main + ' ');
  const sub = document.createElement('span'); sub.className = 'sub';
  sub.textContent = model;
  $('guideTxt').append(sub);
  const ag = $('aGuide'); ag.textContent = tel; ag.style.color = col;
  const am = $('aModel'); am.textContent = mdl; am.style.color = col;

  const ge = $('aGecis');
  if (ge){
    ge.textContent = gec ? `${gec}.` : '—';
    ge.className = 'tv' + (gec === 0 ? ' muted' : gec <= 2 ? ' green' : gec <= 4 ? '' : ' red');
  }
  if (faz && faz !== lastFaz){
    lastFaz = faz;
    if (faz === 'GPS') addLog('gps', 'GÜDÜM', 'GPS fazı — detection modeliyle kadraj merkezleme.');
    else if (faz === 'VISUAL') addLog('vision', 'GÜDÜM', 'Görsel temas oturdu → IBVS, tespit kutusuyla takip devrede.');
    else if (faz === 'VURULDU') addLog('guide', 'GÜDÜM', 'Terminal vuruş — hedefe temas.');
    else if (faz === 'DURDU') addLog('sys', 'GÜDÜM', 'Güdüm durdu.');
  }

  // ── Terminal mod metni (sağ paneldeki tTerm + avcı satırı aTerm) ──
  const termTxt = st.imha ? 'VURULDU'
    : faz === 'VISUAL' ? 'GÖRSEL FAZ'
    : faz === 'GPS' ? 'GPS FAZI'
    : faz === 'DURDU' ? 'DURDU'
    : st.mission ? 'TAKİP' : 'HAZIR';
  $('tTerm').textContent = termTxt;
  $('aTerm').textContent = termTxt;
  $('aTerm').className = 'tv' + (st.mission ? ' green' : ' muted');

  // ── Kilit durumu: görüş hattının GERÇEK çıktısından ──
  let lockTxt, lockCls;
  if (st.imha){ lockTxt = 'VURULDU'; lockCls = 'hit'; }
  else if (!st.mission){ lockTxt = 'BEKLEMEDE'; lockCls = 'idle'; }
  // KİLİT ile TESPİT'in ayrımı: iki dalın koşulu da `p.tespit_var` idi, yani
  // "TESPİT" hiç görünemiyordu ve tek bir kutu bile "KİLİT" yazdırıyordu.
  // Doğrusu: KİLİT = görsel güdüm fiilen devrede (supervisor VISUAL fazı),
  // TESPİT = kutu var ama faz kapıları henüz açılmadı (hâlâ GPS güdümü).
  else if (p && p.tespit_var && st.faz === 'VISUAL'){
    lockTxt = 'KİLİT'; lockCls = '';
  } else if (p && p.tespit_var){
    lockTxt = 'TESPİT'; lockCls = 'searching';
  } else { lockTxt = 'ARANIYOR'; lockCls = 'searching'; }
  $('lockBadge').className = 'lockbadge ' + lockCls;
  $('lockBadge').textContent = lockTxt;
  $('tLock').textContent = lockTxt;
  $('aLock').textContent = lockTxt;
  $('aLock').className = 'tv' + (lockTxt === 'KİLİT' ? ' green' : lockTxt === 'BEKLEMEDE' ? ' muted' : ' amber');
  if (st.mission && lockTxt !== lastLockTxt){
    lastLockTxt = lockTxt;
    if (lockTxt === 'KİLİT') addLog('vision', 'GÖRÜŞ', 'Görsel kilit kuruldu — hedef tespit kutusuyla görülüyor.');
    else if (lockTxt === 'ARANIYOR') addLog('gps', 'GÖRÜŞ', 'Görsel temas yok — hedef kadrajda değil.');
  }
  if (!st.mission) lastLockTxt = null;

  // ── FPV köşe verileri + görüş hattı paneli ──
  const yaz = (id, txt, cls) => { const e = $(id); e.textContent = txt; e.className = 'tv ' + (cls || ''); };
  if (p){
    $('oConf').textContent = p.tespit_var ? p.tespit_conf : '--';
    // KUTU = kutunun uzun kenarı (px). Eskiden buraya da CONF yazılıyordu —
    // iki köşe aynı sayıyı gösteriyordu ve etiket yalan söylüyordu. Kutu
    // boyutu asıl yakınlık ölçütü (CLAUDE.md §5.3), ayrı gösterilmeli.
    $('oKutu').textContent = (p.tespit_var && p.det_px != null)
                             ? p.det_px + ' px' : '--';
    $('aPct').textContent = p.kilit_n ? Math.round(100 * p.kilit_sayac / p.kilit_n) : 0;
    yaz('pDet', p.tespit_var
        ? `VAR ${p.tespit_conf}${p.det_px != null ? ` · ${p.det_px} px` : ''}`
        : 'YOK', p.tespit_var ? 'green' : 'red');
    // Poz + kanat ekseni: menzil kestiriminin (pDist) kaynağı.
    yaz('pPose', p.pose_var
        ? `VAR ${p.pose_conf}${p.kanat_gorunur ? ' · kanat' : ' · kanat yok'}`
        : 'YOK', p.pose_var ? (p.kanat_gorunur ? 'green' : 'amber') : 'red');
    // ── FAZ + kaçıncı geçiş (tek satır: "GPS · 3. geçiş") ──
    // Tespit/kilit satırları kaldırıldı; aynı bilgi zaten "Engel" satırında
    // ("GÖRSEL KİLİT" / "MENZİL KAPISI") daha doğrudan veriliyor.
    const fazAd = st.imha || st.faz === 'VURULDU' ? 'VURULDU'
                : st.faz === 'VISUAL' ? 'GÖRSEL'
                : st.faz === 'GPS' ? 'GPS'
                : st.mission ? 'BEKLİYOR' : '—';
    const gecN = p.gecis_sayisi || 0;
    yaz('pFaz', fazAd + (gecN ? ` · ${gecN}. geçiş` : ''),
        st.faz === 'VURULDU' || st.faz === 'VISUAL' ? 'green'
        : st.faz === 'GPS' ? 'amber' : '');
    yaz('pDist', p.gorus_menzil != null ? p.gorus_menzil.toFixed(1) + ' m' : '--',
        p.gorus_menzil == null ? '' : p.gorus_menzil < 10 ? 'red' : p.gorus_menzil < 30 ? 'amber' : 'green');
    yaz('pTruth', p.gercek_menzil != null ? p.gercek_menzil.toFixed(1) + ' m' : '--', '');
    if (p.menzil_hata != null){
      const h = p.menzil_hata;
      yaz('pErr', (h >= 0 ? '+' : '') + h.toFixed(1) + ' m',
          Math.abs(h) < 3 ? 'green' : Math.abs(h) < 8 ? 'amber' : 'red');
    } else yaz('pErr', '--', '');
    yaz('pDh', p.d_h != null ? `${p.d_h.toFixed(1)} / ${p.gate_menzil} m` : '--', p.menzil_kapi_ok ? 'green' : 'amber');
    yaz('pBlock', p.engel, p.engel === '—' ? 'green' : 'red');

    // ── SOL KİLİTLENME PANELİ — HEPSİ /api/telemetry/pnp'den ──
    // (Taşınırken marj / kümülatif 5 s / kesintisiz / tespit doğrulandı /
    //  menzil ref satırları çıkarıldı: pnp'nin `kilit` ve `menzil_tutucu`
    //  sözlükleri bu dalın sunucusunda YOK, hepsi kalıcı "--" gösteriyordu.
    //  Yerlerine supervisor'ın gerçekten yayınladığı faz kapıları kondu.)
    const setk = (id, txt, cls) => { const e = $(id); if (!e) return; e.textContent = txt; if (cls !== undefined) e.className = 'kp-v ' + cls; };
    setk('kpFaz', fazAd);
    // Kilit sayacı: supervisor KILIT_N ardışık poz kilidi sayar; dolunca
    // GPS→görsel geçişinin poz kapısı açılır.
    const ks = p.kilit_sayac ?? 0, kn = p.kilit_n ?? 0;
    setk('kpKilit', kn ? `${ks} / ${kn}` : '-- / --', p.poz_kapi_ok ? 'ok' : '');
    const kbar = $('kpKilitBar');
    if (kbar) kbar.style.width = (kn ? Math.max(0, Math.min(100, 100 * ks / kn)) : 0) + '%';
    setk('kpConf', p.tespit_var && p.tespit_conf != null ? p.tespit_conf.toFixed(2) : '--',
         p.tespit_var ? 'ok' : 'no');
    setk('kpPoz', p.poz_kapi_ok ? 'AÇIK' : 'kapalı', p.poz_kapi_ok ? 'ok' : 'no');
    setk('kpMenzil',
         p.d_h != null ? `${p.d_h.toFixed(1)} / ${p.gate_menzil} m` : '--',
         p.menzil_kapi_ok ? 'ok' : 'no');
  } else {
    yaz('pBlock', 'API YOK', 'red');
  }
}

// ══ PANEL DÜZENİ (büyüt / daralt) ══════════════════════════════════════
const appEl = document.querySelector('.app');
let ctrlOpen = true;
$('ctrlToggle').addEventListener('click', () => {
  ctrlOpen = !ctrlOpen;
  $('ctrlBody').hidden = !ctrlOpen;
  appEl.classList.toggle('ctrl-collapsed', !ctrlOpen);
  $('ctrlToggle').textContent = ctrlOpen ? '⯆' : '⯈';
  $('ctrlToggle').title = ctrlOpen ? 'Paneli kapat' : 'Paneli aç';
  requestAnimationFrame(() => dispatchEvent(new Event('resize')));
});

const fpvPanel = document.querySelector('.fpvpanel');
const posPanel = $('posPanel');

// ══ PANELİ PENCEREYE AL — ORTAK ALTYAPI ════════════════════════════════
// Bir panel, kamera pencereleriyle aynı davranışa geçer: ekranda istenen yere
// taşınır, sekiz tutamaktan boyutlandırılır. Fark, panelin DOM'da YERİNDE
// kalması — yalnız position:fixed'e geçer; böylece içindeki <img> (MJPEG) ve
// canvas'lar yeniden kurulmaz, akış/çizim kopmaz.
// Hem FPV takip ekranı hem Konum İzleme bunu kullanır; panele özgü işler
// (kilit panelini taşımak, yer tutucuyu göstermek) hook'larla verilir.
//
function pencereKur(panel, o){
  const pencereMi = () => panel.classList.contains('pencere');
  const olcuEl = o.olcuEl || panel;
  function ayarla(ac){
    if (ac === pencereMi()) return;
    if (ac){
      o.girerken?.();
      // Açılış geometrisi iki yoldan biriyle:
      //  • o.acilis() varsa onun verdiği BELİRLİ pencere boyutu/konumu. Dar
      //    sütunlardaki paneller için şart: panel zaten ekrana sığdığı için
      //    "bulunduğu yerden" açılış onu AYNI yerde AYNI boyutta bırakır ve
      //    kullanıcı pencereye geçtiğini anlamaz (konum panelinde birebir bu oldu).
      //  • yoksa panelin bulunduğu yerden, ama ekrandan belirgin KÜÇÜK: grid'deki
      //    panel neredeyse tam ekran yüksekliğinde olabiliyor; birebir aynı
      //    boyutta açılsa taşıma payı 20-30 px'e düşer ve "taşınamıyor" hissi
      //    verir (tarayıcı testinde birebir bu çıktı).
      let g;
      if (o.acilis){
        g = o.acilis();
      } else {
        const r = panel.getBoundingClientRect();
        const w = clamp(Math.round(Math.min(r.width,  innerWidth  * o.enOran)),  CW_MIN_W, Math.max(CW_MIN_W, innerWidth  - 16));
        const h = clamp(Math.round(Math.min(r.height, innerHeight * o.boyOran)), CW_MIN_H, Math.max(CW_MIN_H, innerHeight - 16));
        g = { w, h,
              l: clamp(Math.round(r.left), 0, Math.max(0, innerWidth  - w)),
              t: clamp(Math.round(r.top),  0, Math.max(0, innerHeight - h)) };
      }
      panel.style.width  = g.w + 'px';
      panel.style.height = g.h + 'px';
      panel.style.left   = g.l + 'px';
      panel.style.top    = g.t + 'px';
      panel.classList.add('pencere');
      panel.style.zIndex = ++camZ;                    // kamera pencereleriyle ortak z sırası
    } else {
      o.cikarken?.();
      // Sütuna dönüş: pencere modunun BÜTÜN kalıntıları temizlenir, yoksa panel
      // yerinde inline width/height ile yanlış boyutta oturur.
      panel.classList.remove('pencere', 'busy', 'dragging', 'resizing', 'front');
      for (const k of ['left', 'top', 'width', 'height', 'zIndex']) panel.style[k] = '';
    }
    o.sonra?.(ac);
    camDotDurum();                                    // daireler bu modun düğmesi
    duzenAcikKaydet();                                // açık pencere takımı hatırlansın
    requestAnimationFrame(() => dispatchEvent(new Event('resize')));
    addLog('sys', 'SYS', `${o.ad} ${ac ? 'pencereye alındı' : 'panele geri kondu'}.`);
  }
  // Taşıma + sekiz yönlü boyutlandırma: kamera pencerelerinin AYNI camDrag'i.
  // İzin kapısı, panel sütunundayken başlığa/tutamaklara basmanın hiçbir şey
  // yapmamasını sağlar.
  // Başlıktaki düğmeler sürükleme başlatmasın: pointerdown'da preventDefault
  // çağrıldığı için düğmenin kendi click'i düşebiliyor (konum panelinin
  // başlığında ⤢ düğmesi var).
  const tasinabilir = e => pencereMi() && !e.target.closest('button');
  // Taşıma sınırı MASAÜSTÜ kuralı: pencere kenarlardan taşabilir, ama üstten
  // taşmaz ve en az PEN_GORUNUR kadar genişliği ekranda kalır → başlık her
  // zaman yakalanabilir, pencere kaybolamaz. Kamera pencerelerinin "tamamen
  // ekranda kal" kuralı burada YETMİYOR: paneller büyük olduğu için (713 px /
  // 914 px viewport) dikey pay 201 px'e düşüyordu ve tarayıcı testinde pencere
  // aşağı taşınamıyordu. Boyutlandırma tutamakları kendi sınırında kalır.
  const PEN_GORUNUR = 150, PEN_BASLIK = 44;
  camDrag(panel, panel.querySelector(o.basSec), 'move', tasinabilir, (L, T, w) => ({
    L: clamp(L, -(w - PEN_GORUNUR), innerWidth - PEN_GORUNUR),
    T: clamp(T, 0, Math.max(0, innerHeight - PEN_BASLIK)),
  }));
  for (const h of panel.querySelectorAll('.cw-rs')) camDrag(panel, h, h.dataset.yon, pencereMi);
  // Öne al — kamera pencereleri ve diğer panel penceresiyle ortak z sırası.
  panel.addEventListener('pointerdown', () => {
    if (!pencereMi()) return;
    panel.style.zIndex = ++camZ;
    panel.classList.add('front');
    for (const { win } of camWins.values()) win.classList.remove('front');
    for (const d of document.querySelectorAll('.pencere')) if (d !== panel) d.classList.remove('front');
  });
  // Tarayıcı penceresi küçülünce erişilemez yere düşmesin. Taşımadaki AYNI
  // kelepçe — tam kapsama zorlansaydı kasten kenardan taşırılan pencere her
  // ekran boyu değişiminde geri sıçrardı.
  addEventListener('resize', () => {
    if (!pencereMi()) return;
    const r = panel.getBoundingClientRect();
    panel.style.left = clamp(r.left, -(r.width - PEN_GORUNUR), innerWidth - PEN_GORUNUR) + 'px';
    panel.style.top  = clamp(r.top, 0, Math.max(0, innerHeight - PEN_BASLIK)) + 'px';
  });
  // Boyut her değiştiğinde (sürükleyerek de) 'resize' yayınla: parazit ve mini
  // harita canvas'ları kendilerini YALNIZ bu olayda ölçüyor (bkz. mkCanvas), ve
  // camDrag olay yayınlamıyor — bu gözlemci olmadan canvas'lar bulanık kalırdı.
  {
    let sonOlcu = '';
    new ResizeObserver(() => {
      const r = olcuEl.getBoundingClientRect();
      const k = Math.round(r.width) + 'x' + Math.round(r.height);
      if (k === sonOlcu) return;                      // aynı ölçüde tekrar yayınlamayalım
      sonOlcu = k;
      requestAnimationFrame(() => dispatchEvent(new Event('resize')));
    }).observe(olcuEl);
  }
  // Esc: pencere kenara çekilmiş olsa bile panele dönmenin klavye yolu.
  addEventListener('keydown', e => { if (e.key === 'Escape' && pencereMi()) ayarla(false); });
  return { ayarla, pencereMi };
}

// ── TAKİP EKRANI (FPV) ──
// Başlıkta düğme YOK (▢ ve ◉ kaldırıldı); tetikleyici üst çubuktaki AV dairesi.
const fpvDock = $('fpvDock');
const fpvPen = pencereKur(fpvPanel, {
  ad: 'Takip ekranı', basSec: '#fpvHead',
  olcuEl: $('fpvwrap'),
  // Açılış geometrisi DÜZEN'den: orta sütunun ÜST SIRASINDA, solda, alanın
  // %52 genişliği. Beş pencerenin en büyüğü budur (KNM ile birlikte) — bkz.
  // PENCERE DÜZENİ bölümündeki şema.
  acilis: () => duzenGeo(FPV_KEY),
  // Kilit paneli artık SOL SÜTUNDA (Avcı Drone sekmesi) — pencere moduyla
  // taşınmıyor, hep aynı yerde duruyor.
  sonra: ac => { fpvDock.hidden = !ac; },
});
const setFpvPencere = fpvPen.ayarla, fpvPencereMi = fpvPen.pencereMi;
$('fpvDockBtn').addEventListener('click', () => setFpvPencere(false));

// ── KONUM İZLEME ──
// Sütunda YER KAPLAMAZ: HTML'de hidden duruyor, yalnız üst çubuktaki KNM
// dairesine basılınca pencere olarak açılır, kapanınca yeniden gizlenir.
const posPen = pencereKur(posPanel, {
  ad: 'Konum izleme', basSec: '.phead',
  olcuEl: posPanel.querySelector('.scenewrap'),
  // Açılış geometrisi DÜZEN'den: orta sütunun ÜST SIRASINDA, sağda, alanın
  // %48 genişliği. "Bulunduğu yerden" açılsaydı sağ sütundaki 288 px'lik panel
  // aynı yerde aynı boyutta kalır, pencereye geçtiği ANLAŞILMAZDI.
  acilis: () => duzenGeo(POS_DOT),
  // Panel sütunda hidden duruyor; pencere açılırken görünür olmalı ki
  // getBoundingClientRect/canvas ölçümü sıfır çıkmasın.
  girerken: () => { posPanel.hidden = false; },
  cikarken: () => { posPanel.hidden = true; },
});

// ══ ÇİZİM DÖNGÜSÜ ══════════════════════════════════════════════════════
function frame(now){
  camAz += (camAzT - camAz) * 0.18;
  camEl += (camElT - camEl) * 0.18;
  drawScene();
  drawNoise();
  const img = $('fpvImg');
  $('noFeed').hidden = !!(img && img.naturalWidth > 0);
  // Kamera pencereleri: MJPEG'de 'load' akış bitince tetiklendiği için ilk
  // karenin gelip gelmediği burada da naturalWidth ile yoklanır (ana sahnedeki
  // ile aynı gerekçe).
  for (const { img: ci, nofeed } of camWins.values()) nofeed.hidden = ci.naturalWidth > 0;
  // Dairedeki yeşil halka = o görüşten kare akıyor. Ana ekrandaki görüş için
  // ölçüt fpvImg, diğerleri için kendi pencerelerinin <img>'i.
  for (const b of camDock.children){
    const key = b.dataset.cam;
    const el = key === camAnaKey() ? img : camWins.get(key)?.img;
    b.classList.toggle('live', !!(el && el.naturalWidth > 0));
  }
  // (Aşağı yukarı süzülen yeşil "tarama" çizgisi KALDIRILDI — sinüsle
  //  hareket eden salt dekoratif bir öğeydi, hiçbir veriyi göstermiyordu.)
  requestAnimationFrame(frame);
}

// ══ GÜDÜM MODU (GPS / GÖRSEL / HİBRİT) ═════════════════════════════════
// Basınca sunucuya yazılır; vurgu HER ZAMAN sunucudan dönen GERÇEK modla
// güncellenir (görev sırasında da geçerli — chase thread aktif fazı kırıp
// yeni modu kurar). Sunucu varsayılanı "hybrid".
const MOD_AD = { gps: 'GPS', visual: 'GÖRSEL', hybrid: 'HİBRİT' };
// ⚠ SEÇİCİ #modGrup'a KİLİTLİ. Kaçamak testinin tür/tetik düğmeleri de
// .mod-btn sınıfını kullanıyor; genel `.mod-btn` seçicisi, mod vurgusu her
// yoklandığında (500 ms) onların 'aktif' sınıfını da siliyordu — kullanıcı
// kaçamak seçimini yaptıktan yarım saniye sonra vurgu kayboluyordu.
const modBtns = [...document.querySelectorAll('#modGrup .mod-btn')];
function modVurgula(mode){
  modBtns.forEach(b => b.classList.toggle('aktif', b.dataset.mode === mode));
}
function modNot(txt){ const e = $('modNot'); if (e) e.textContent = txt || ''; }
modBtns.forEach(btn => {
  btn.addEventListener('click', () => {
    fetch('/api/guidance_mode', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode: btn.dataset.mode })
    }).then(r => r.json()).then(d => {
      if (d.status === 'success' && d.mode){
        modVurgula(d.mode); modNot('');
        addLog('sys', 'MOD', `Güdüm modu: ${MOD_AD[d.mode] || d.mode}`);
      } else {
        if (d.mode) modVurgula(d.mode);       // sunucudaki GERÇEK mod neyse o
        modNot(d.message || 'Güdüm modu değiştirilemedi');
        addLog('err', 'MOD', d.message || 'Güdüm modu değiştirilemedi.');
      }
    }).catch(() => modNot('Sunucuya ulaşılamadı'));
  });
});
// Açılışta GERÇEK modu oku; görsel faz kapalıysa (AVCI_GORSEL=off) GÖRSEL ve
// HİBRİT'i kilitle ve SEBEBİNİ yaz — sessizce çalışmayan düğme en kötüsü.
fetch('/api/guidance_mode').then(r => r.json()).then(d => {
  modVurgula(d.mode);
  if (!d.gorsel_acik){
    modBtns.forEach(b => { if (b.dataset.mode !== 'gps') b.disabled = true; });
    modNot('Görsel faz kapalı — sunucuyu AVCI_GORSEL=on ile başlatın');
  }
}).catch(() => {});

// ══ UÇUŞ KAYDI (⏺ video + durum) ═══════════════════════════════════════
// Durum HER ZAMAN sunucudan okunur — buton yerel bayrağa güvenirse, sunucu
// yeniden başlatılınca arayüz "kayıtta" görünüp aslında hiçbir şey yazmaz.
const kayitBtn = $('kayitBtn');
let kayitAktif = false;

async function kayitTazele(){
  try {
    const d = await (await fetch('/api/kayit/durum')).json();
    kayitAktif = !!d.aktif;
    if (!kayitBtn) return;
    kayitBtn.setAttribute('aria-pressed', kayitAktif ? 'true' : 'false');
    $('kayitLbl').textContent = kayitAktif ? 'Kaydı Durdur' : 'Video Kaydı Al';
    $('kayitNot').textContent = kayitAktif
      ? `${d.kare} kare · ${Math.round(d.gecen_s)} s` : '';
  } catch (e){ /* sunucu yok — sessiz geç, 2 s sonra yine denenir */ }
}

if (kayitBtn){
  kayitBtn.addEventListener('click', async () => {
    const yol = kayitAktif ? '/api/kayit/dur' : '/api/kayit/basla';
    try {
      const d = await (await fetch(yol, { method: 'POST' })).json();
      if (d.status === 'success'){
        addLog('sys', 'KAYIT', kayitAktif
          ? `Kayıt durdu — ${d.kare} kare → ${d.dizin}`
          : `Kayıt başladı → ${d.dizin}`);
      } else addLog('err', 'KAYIT', d.message || 'Kayıt komutu reddedildi.');
    } catch (e){ addLog('err', 'KAYIT', 'Kayıt isteği başarısız: ' + e); }
    kayitTazele();
  });
  setInterval(kayitTazele, 2000);
}

// ══ (ÇIKARILDI) GÜDÜM ÖZELLİKLERİ + İRTİFA TUTUCU ═════════════════════
// Bu iki bölüm 2026-08-20'de arayüz taşınırken SİLİNDİ; sunucu uçları bu
// dalda yok:
//   · /api/gudum_ozellikleri — deney paneli 2026-08-19'da kaldırıldı,
//     yerine 🎚 AYAR KONSOLU geldi (/api/ayarlar, aşağıda).
//   · /api/senaryo_ayar + control/senaryo_cfg.py — yalnız
//     hit_irtifa_tutucu dalında var.
// Geri istenirse sunucu tarafıyla BİRLİKTE gelmeli (CLAUDE.md §5.12).

// ══ KAÇAMAK TESTİ — panelden tek düğmeyle ══════════════════════════════
// Hedef düz uçar, drone kuyruk yaklaşması kurar, mesafe eşiğe inince hedef
// seçilen kaçamağı yapar. Kareler kaydedilir; bitince vuruş KONTROLLÜ/ŞANS
// diye sınıflandırılır (CLAUDE.md §3.3 + §4).
const KAC_TURLER = [
  ['yok', 'YOK (taban)'], ['yatay', 'YATAY'], ['capraz', 'ÇAPRAZ'],
  ['dikey_yukari', 'TIRMAN'], ['dikey_asagi', 'DALIŞ'], ['hizlan', 'HIZLAN'],
];
const KAC_TETIKLER = [[8, '8 m'], [15, '15 m'], [25, '25 m']];
let kacTur = 'yatay', kacTetik = 8;

function kacSecimCiz(){
  const t = $('kac-tur'), m = $('kac-tetik');
  if (!t || !m) return;
  t.innerHTML = ''; m.innerHTML = '';
  KAC_TURLER.forEach(([v, ad]) => {
    const b = document.createElement('button');
    b.className = 'mod-btn' + (v === kacTur ? ' aktif' : '');
    b.textContent = ad;
    b.addEventListener('click', () => { kacTur = v; kacSecimCiz(); });
    t.appendChild(b);
  });
  KAC_TETIKLER.forEach(([v, ad]) => {
    const b = document.createElement('button');
    b.className = 'mod-btn' + (v === kacTetik ? ' aktif' : '');
    b.textContent = ad;
    b.addEventListener('click', () => { kacTetik = v; kacSecimCiz(); });
    m.appendChild(b);
  });
}

function kacBasla(){
  fetch('/api/kacamak/basla', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ kacamak: kacTur, tetik_m: kacTetik, kayit_s: 240 })
  }).then(r => r.json()).then(d => {
    if (d.status !== 'success'){
      const el = $('kac-durum');
      if (el) el.textContent = 'HATA: ' + (d.message || '');
      addLog('err', 'KAÇAMAK', d.message || 'başlatılamadı');
    } else {
      addLog('sys', 'KAÇAMAK', `${kacTur}, tetik ${kacTetik} m — başladı`);
    }
    kacDurumYenile();
  }).catch(() => {});
}

function kacDur(){
  fetch('/api/kacamak/durdur', { method: 'POST' })
    .then(() => kacDurumYenile()).catch(() => {});
}

function kacSonucCiz(s){
  const el = $('kac-sonuc');
  if (!el) return;
  if (!s){ el.innerHTML = ''; return; }
  const vur = s.imha ? '<span class="kac-basari">✓ İSABET</span>'
                     : '<span class="kac-iska">✗ ıska</span>';
  let sinif = '';
  if (s.sinif === 'KONTROLLÜ') sinif = '<span class="kac-basari">KONTROLLÜ</span>';
  else if (s.sinif === 'ŞANS') sinif = '<span class="kac-sans">ŞANS</span>';
  let h = `${vur} &nbsp; en yakın <b>${s.en_yakin ?? '—'} m</b>`;
  if (sinif) h += ` &nbsp; vuruş: ${sinif}`;
  h += `<br>salınım <b>${s.cx_salinim ?? '—'}</b>/s &nbsp; yatış p90 <b>${s.roll_p90 ?? '—'}°</b>`;
  if (s.gerekce) h += `<br><span class="kac-olcut">${s.gerekce}</span>`;
  Object.entries(s.olcut || {}).forEach(([ad, [ok, det]]) => {
    h += `<br><span class="kac-olcut">${ok ? '✓' : '✗'} ${ad} — ${det}</span>`;
  });
  el.innerHTML = h;
}

function kacDurumYenile(){
  fetch('/api/kacamak/durum').then(r => r.json()).then(d => {
    const el = $('kac-durum'), b = $('kac-basla'), s = $('kac-dur');
    if (b) b.disabled = d.kosuyor;
    if (s) s.disabled = !d.kosuyor;
    if (el){
      el.className = 'kacamak-durum' + (d.kosuyor ? ' kosuyor' : '');
      el.textContent = d.kosuyor
        ? `KOŞUYOR — ${d.kacamak}, tetik ${d.tetik} m, ${d.gecen_s || 0} s\n`
          + (d.satirlar || []).slice(-3).join('\n')
        : 'hazır';
    }
    kacSonucCiz(d.sonuc);
    const g = $('kac-gecmis');
    if (g){
      g.innerHTML = (d.gecmis || []).slice(0, 5).map(x =>
        `${x.imha ? '✓' : '✗'} ${x.ad} — ${x.en_yakin} m, salınım ${x.cx_salinim ?? '—'}/s`
      ).join('<br>');
    }
  }).catch(() => {});
}

(function kacKur(){
  kacSecimCiz();
  const b = $('kac-basla'), s = $('kac-dur');
  if (b) b.addEventListener('click', kacBasla);
  if (s) s.addEventListener('click', kacDur);
  kacDurumYenile();
  setInterval(kacDurumYenile, 2000);
})();

/* ══ AYAR KONSOLU ═══════════════════════════════════════════════════════
   Klasik arayüzden TAŞINDI (2026-08-18'de eklenmişti). Kullanıcı isteği:
   "arayüze bir buton koy, bu butona basınca bir panel açılsın ve bu panelden
   sistemdeki tüm tune edilmesi gereken şeylerin parametrelerin katsayılarını
   slidebarlardan ayarlayabileyim... her şeyin de ne işe yaradığı, neyi
   kontrol ettiği, neyi artırıp neyi azalttığı bilinsin."

   Liste sunucudan gelir (/api/ayarlar); control/ayar_konsolu.py AYARLAR
   listesine satır eklemek yeterli — burası kendiliğinden büyür.

   ⚠ Kaydırırken HER piksel için istek atmıyoruz: 'input' olayında yalnız
   ekran güncellenir, sunucuya 'change' olayında (fare bırakılınca) yazılır.
   Güdüm 20 Hz koşuyor; saniyede 60 POST paneli de güdümü de boğar. */
let _ayarVeri = null;
const _ayarAcikBilgi = new Set();

function ayarKatman(){ return $('ayar-katman'); }
function ayarAc(){ ayarKatman().classList.remove('gizli'); ayarYenile(); }
function ayarKapat(){ ayarKatman().classList.add('gizli'); }

function ayarYenile(){
  fetch('/api/ayarlar').then(r => r.json())
    .then(d => { _ayarVeri = d; ayarCiz(); })
    .catch(() => { $('ayar-govde').innerHTML =
      '<div class="ozellik-bos">sunucuya ulaşılamadı</div>'; });
}

function ayarCiz(){
  const govde = $('ayar-govde');
  if (!_ayarVeri) return;
  const ara = ($('ayar-ara').value || '').trim().toLowerCase();
  const yalnizDegisen = $('ayar-suz-degisen').checked;
  govde.innerHTML = '';
  let gosterilen = 0;

  // "kaç tanesini oynattım" rozeti — tuning sırasında en çok gereken bilgi
  const degisenSayi = _ayarVeri.ayarlar.filter(a => a.degisti).length;
  const rozet = $('ayar-degisen-sayi');
  rozet.textContent = degisenSayi;
  rozet.classList.toggle('dolu', degisenSayi > 0);

  _ayarVeri.gruplar.forEach(g => {
    const uyan = _ayarVeri.ayarlar.filter(a => a.grup === g.kod &&
      (!yalnizDegisen || a.degisti) &&
      (!ara || (a.etiket + ' ' + a.alan + ' ' + (a.ne_yapar || ''))
        .toLowerCase().includes(ara)));
    if (!uyan.length) return;
    const kutu = document.createElement('div');
    kutu.className = 'ayar-grup';
    const bas = document.createElement('div');
    bas.className = 'ayar-grup-bas';
    bas.textContent = g.baslik;
    kutu.appendChild(bas);
    if (g.aciklama){
      const not = document.createElement('div');
      not.className = 'ayar-grup-not';
      not.textContent = g.aciklama;
      kutu.appendChild(not);
    }
    uyan.forEach(a => { kutu.appendChild(ayarSatir(a)); gosterilen++; });
    govde.appendChild(kutu);
  });

  if (!gosterilen){
    govde.innerHTML = '<div class="ozellik-bos">' +
      (yalnizDegisen && !degisenSayi
        ? 'hiçbir ayar açılış değerinden farklı değil'
        : 'eşleşen ayar yok') + '</div>';
  }
}

function ayarSatir(a){
  const s = document.createElement('div');
  s.className = 'ayar-satir' + (a.degisti ? ' degisti' : '');

  const et = document.createElement('div');
  et.className = 'ayar-etiket';
  const ad = document.createElement('div');
  ad.className = 'ayar-etiket-ad';
  ad.textContent = a.etiket;
  const alan = document.createElement('div');
  alan.className = 'ayar-etiket-alan';
  alan.textContent = a.alan + (a.tip === 'param' ? '  (ARAÇ)' : '');
  et.append(ad, alan);
  s.appendChild(et);

  if (a.tip === 'secim'){
    // Metin seçeneği (ör. menzil ölçüsü: carpim | kosegen) — her seçenek bir
    // düğme. kayramin_super_gudumu 98f7c61'den geldi; sunucu seçenekleri
    // /api/ayarlar'da `secenekler` alanıyla yolluyor.
    const kutu = document.createElement('div');
    kutu.className = 'ayar-secim';
    (a.secenekler || []).forEach(sec => {
      const b = document.createElement('button');
      b.className = 'ayar-bool' + (a.deger === sec ? ' acik' : '');
      b.textContent = sec;
      b.addEventListener('click', () => ayarYaz(a.ad, sec));
      kutu.appendChild(b);
    });
    s.appendChild(kutu);
    s.appendChild(document.createElement('div'));
  } else if (a.tip === 'bool'){
    s.appendChild(document.createElement('div'));   // orta sütun boş: ızgara bozulmasın
    const sag = document.createElement('div');
    sag.className = 'ayar-sag';
    const b = document.createElement('button');
    b.className = 'ayar-bool' + (a.deger ? ' acik' : '');
    b.textContent = a.deger ? 'AÇIK' : 'KAPALI';
    b.addEventListener('click', () => ayarYaz(a.ad, !a.deger));
    sag.appendChild(b);
    s.appendChild(sag);
  } else if (a.deger === null){
    const bos = document.createElement('div');
    bos.className = 'ayar-etiket-alan';
    bos.textContent = 'araç henüz cevap vermedi';
    s.appendChild(bos);
    s.appendChild(document.createElement('div'));
  } else {
    const r = document.createElement('input');
    r.type = 'range'; r.className = 'ayar-kaydir';
    r.min = a.min; r.max = a.maks; r.step = a.adim; r.value = a.deger;
    const sag = document.createElement('div');
    sag.className = 'ayar-sag';
    const n = document.createElement('input');
    n.type = 'number'; n.className = 'ayar-sayi';
    n.min = a.min; n.max = a.maks; n.step = a.adim; n.value = a.deger;
    const br = document.createElement('span');
    br.className = 'ayar-birim';
    br.textContent = a.birim || '';
    sag.append(n, br);
    // sürüklerken YALNIZ ekran; sunucuya bırakınca yazılır
    r.addEventListener('input', () => { n.value = r.value; });
    r.addEventListener('change', () => ayarYaz(a.ad, parseFloat(r.value)));
    n.addEventListener('change', () => { r.value = n.value; ayarYaz(a.ad, parseFloat(n.value)); });
    s.append(r, sag);
  }

  const ib = document.createElement('button');
  ib.className = 'ayar-bilgi-btn';
  ib.textContent = '?';
  ib.title = 'ne işe yarar / neyi artırır-azaltır';
  s.appendChild(ib);

  const bilgi = document.createElement('div');
  bilgi.className = 'ayar-bilgi' + (_ayarAcikBilgi.has(a.ad) ? '' : ' gizli');
  const p1 = document.createElement('p');
  p1.textContent = a.ne_yapar || '';
  bilgi.appendChild(p1);
  if (a.artarsa){
    const y = document.createElement('span');
    y.className = 'yon artar';
    y.textContent = (a.tip === 'bool' ? '▲ AÇIK: ' : '▲ ARTARSA: ') + a.artarsa;
    bilgi.appendChild(y);
  }
  if (a.azalirsa){
    const y = document.createElement('span');
    y.className = 'yon azalir';
    y.textContent = (a.tip === 'bool' ? '▼ KAPALI: ' : '▼ AZALIRSA: ') + a.azalirsa;
    bilgi.appendChild(y);
  }
  if (a.varsayilan !== null && a.varsayilan !== undefined){
    const v = document.createElement('span');
    v.className = 'yon vars';
    v.textContent = 'açılış değeri: ' + a.varsayilan +
      (a.tip !== 'bool' ? `   ·   aralık ${a.min} … ${a.maks}` : '');
    bilgi.appendChild(v);
  }
  ib.addEventListener('click', () => {
    const kapali = bilgi.classList.toggle('gizli');
    if (kapali) _ayarAcikBilgi.delete(a.ad); else _ayarAcikBilgi.add(a.ad);
  });
  s.appendChild(bilgi);
  return s;
}

function ayarYaz(ad, deger){
  fetch('/api/ayarlar', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ ad: ad, deger: deger })
  }).then(r => r.json()).then(d => {
    if (d.status === 'error') alert('Ayar yazılamadı: ' + d.message);
    if (d.ayarlar){ _ayarVeri.ayarlar = d.ayarlar; ayarCiz(); }
    addLog('sys', 'AYAR', `${ad} = ${deger}`);
  }).catch(() => alert('Sunucuya ulaşılamadı'));
}

function ayarSifirla(){
  if (!confirm('Tüm güdüm ayarları açılış değerlerine dönecek.\n' +
               'ARAÇ parametreleri (ANGLE_MAX, PSC_JERK_XY…) etkilenmez.\nDevam?')) return;
  fetch('/api/ayarlar/sifirla', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ grup: null })
  }).then(r => r.json())
    .then(d => { if (d.ayarlar){ _ayarVeri.ayarlar = d.ayarlar; ayarCiz(); } });
}

/* Değiştirdiklerini panoya al — "ne denediğini" yapıştırabilmek ve aynı
   ayarı sonra tek komutla geri getirebilmek için. */
function ayarKopyala(){
  if (!_ayarVeri) return;
  const d = _ayarVeri.ayarlar.filter(a => a.degisti);
  if (!d.length){ alert('Hiçbir ayar açılış değerinden farklı değil.'); return; }
  const grupAd = {};
  _ayarVeri.gruplar.forEach(g => { grupAd[g.kod] = g.baslik; });
  let t = '# AYAR KONSOLU — değiştirilenler (' + new Date().toLocaleString('tr-TR') + ')\n';
  t += '# ' + d.length + ' ayar açılış değerinden farklı\n\n';
  let sonGrup = null;
  d.forEach(a => {
    if (a.grup !== sonGrup){ t += '## ' + grupAd[a.grup] + '\n'; sonGrup = a.grup; }
    t += `${a.alan.padEnd(22)} ${String(a.varsayilan).padStart(8)} -> ` +
         `${String(a.deger).padStart(8)}   ${a.birim || ''}  (${a.etiket})\n`;
  });
  t += '\n# AYNI AYARI GERİ GETİRMEK İÇİN (sim ayaktayken yapıştır):\n';
  d.forEach(a => {
    const v = (a.tip === 'bool') ? String(!!a.deger) : a.deger;
    t += `curl -s -X POST localhost:8000/api/ayarlar -H 'Content-Type: application/json' ` +
         `-d '{"ad":"${a.ad}","deger":${v}}' >/dev/null\n`;
  });
  const bitir = ok => alert(ok ? d.length + ' ayar panoya kopyalandı.'
    : 'Pano kullanılamadı — metin konsola yazıldı (F12 → Console).');
  if (navigator.clipboard && navigator.clipboard.writeText){
    navigator.clipboard.writeText(t).then(() => bitir(true),
      () => { console.log(t); bitir(false); });
  } else { console.log(t); bitir(false); }
}

(function ayarKonsoluBagla(){
  const ac = $('ayar-ac');
  if (!ac) return;
  ac.addEventListener('click', ayarAc);
  $('ayar-kapat').addEventListener('click', ayarKapat);
  $('ayar-sifirla').addEventListener('click', ayarSifirla);
  $('ayar-ara').addEventListener('input', ayarCiz);
  $('ayar-suz-degisen').addEventListener('change', ayarCiz);
  $('ayar-kopyala').addEventListener('click', ayarKopyala);
  // katmanın boşluğuna tıklayınca kapan (kutunun içine tıklayınca DEĞİL)
  ayarKatman().addEventListener('click', e => { if (e.target === ayarKatman()) ayarKapat(); });
  document.addEventListener('keydown', e => {
    if (e.key === 'Escape' && !ayarKatman().classList.contains('gizli')) ayarKapat();
  });
})();

// ══ AÇILIŞ ═════════════════════════════════════════════════════════════
markScenario();
kayitTazele();
drawKnob();
setTab('avci');          // soldaki kontrol paneli avcı drone sekmesiyle açılır
switchCamera(ANA_KEY);   // FPV ekranı SABİT: avcı drone kamerası (sekme değiştirmez)
duzenAcilista();         // beş pencere varsayılan/kayıtlı yerleşimine oturur
connectWS();
pollChase(); pollPnp(); pollHasar();
setInterval(pollChase, 500);
setInterval(pollPnp, 700);
setInterval(pollHasar, 1000);
setInterval(renderStatus, 250);
renderStatus();
requestAnimationFrame(frame);
})();
