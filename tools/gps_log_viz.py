#!/usr/bin/env python3
"""
gps_log_viz.py — GPS güdüm CSV loglarını tek-dosya interaktif HTML panele çevirir.

Panelde her uçuş için: kuşbakışı yörünge (drone vs hedef), kamera nişangâhı
(hedefin u,v izi), menzil d_h zaman serisi, kadraj açıları (elev/yaw) ve
veriden türetilen otomatik yorum. Çıktı tamamen kendine yeten (self-contained)
tek HTML — internet/CDN gerektirmez, tarayıcıda açılır.

Kullanım:
  python3 tools/gps_log_viz.py                     # en yeni 6 log
  python3 tools/gps_log_viz.py --last 8            # en yeni 8 log
  python3 tools/gps_log_viz.py logs/a.csv logs/b.csv
  python3 tools/gps_log_viz.py --last 4 -o rapor.html --open

Log formatı ve kolon anlamları: docs/GPS_LOGGING.md
"""
import argparse
import csv
import glob
import json
import os
import sys
import webbrowser

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_LOG_DIR = os.path.join(_ROOT, "logs")
_HEDEF_NOKTA = 600   # uçuş başına HTML'e gömülecek yaklaşık nokta sayısı (downsample)


def _fnum(satir, alan):
    try:
        return float(satir.get(alan, ""))
    except (TypeError, ValueError):
        return None


def _log_yukle(yol):
    """Tek CSV'yi oku, ~_HEDEF_NOKTA'ya indir, panel için nokta listesi döndür."""
    with open(yol) as f:
        satirlar = list(csv.DictReader(f))
    if not satirlar:
        return None
    n_orig = len(satirlar)
    adim = max(1, n_orig // _HEDEF_NOKTA)
    satirlar = satirlar[::adim]
    t0 = _fnum(satirlar[0], "t") or 0.0
    noktalar = []
    for r in satirlar:
        t = _fnum(r, "t")
        noktalar.append({
            "t": round(t - t0, 2) if t is not None else None,
            "dh": _fnum(r, "d_h"),
            "durum": r.get("durum"),
            "yaw": _fnum(r, "kadraj_yaw_deg"),
            "elev": _fnum(r, "kadraj_elev_deg"),
            "u": _fnum(r, "u_px"),
            "v": _fnum(r, "v_px"),
            "ix": _fnum(r, "iris_x"), "iy": _fnum(r, "iris_y"),
            "tx": _fnum(r, "tgt_x"), "ty": _fnum(r, "tgt_y"),
        })
    stamp = os.path.basename(yol).replace("gps_guidance_", "").replace(".csv", "")
    return {"stamp": stamp, "etiket": stamp, "n_orig": n_orig, "pts": noktalar}


def _loglari_sec(args):
    if args.logs:
        return args.logs
    hepsi = sorted(glob.glob(os.path.join(_LOG_DIR, "gps_guidance_*.csv")),
                   key=os.path.getmtime, reverse=True)
    return list(reversed(hepsi[:args.last]))   # eskiden yeniye


def html_uret(ucuslar):
    data_js = json.dumps(ucuslar, separators=(",", ":"))
    return _SABLON.replace("__DATA__", data_js)


def main(argv=None):
    ap = argparse.ArgumentParser(description="GPS güdüm loglarını HTML panele çevirir.")
    ap.add_argument("logs", nargs="*", help="CSV log dosyaları (boşsa en yeni --last kadarı)")
    ap.add_argument("--last", type=int, default=6, help="dosya verilmezse en yeni kaç log (varsayılan 6)")
    ap.add_argument("-o", "--out", default=os.path.join(_LOG_DIR, "gps_log_panel.html"),
                    help="çıktı HTML yolu")
    ap.add_argument("--open", action="store_true", help="oluşturunca tarayıcıda aç")
    args = ap.parse_args(argv)

    yollar = _loglari_sec(args)
    if not yollar:
        print("Log bulunamadı.", file=sys.stderr)
        return 1
    ucuslar = []
    for y in yollar:
        u = _log_yukle(y)
        if u:
            ucuslar.append(u)
            print(f"  yüklendi: {os.path.basename(y)}  ({u['n_orig']} kare → {len(u['pts'])} nokta)")
    if not ucuslar:
        print("Geçerli veri yok.", file=sys.stderr)
        return 1

    html = html_uret(ucuslar)
    with open(args.out, "w") as f:
        f.write(html)
    print(f"\nPanel yazıldı: {args.out}  ({len(html)//1024} KB, {len(ucuslar)} uçuş)")
    if args.open:
        webbrowser.open("file://" + os.path.abspath(args.out))
    return 0


_SABLON = r'''<!doctype html><html lang="tr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>GPS Güdüm — Log Paneli</title>
<style>
:root{--bg:#0e1417;--panel:#151d21;--panel2:#1b262b;--line:#25343a;--ink:#dfe8e6;--ink-dim:#8fa39f;
  --accent:#35e0c9;--tgt:#f2a541;--good:#4ade80;--warn:#fbbf24;--bad:#f87171;--grid:rgba(120,150,150,.12);}
@media (prefers-color-scheme: light){:root{--bg:#eef2f1;--panel:#fff;--panel2:#f2f6f5;--line:#d3ddda;
  --ink:#17241f;--ink-dim:#5c6d68;--accent:#0f9e8c;--tgt:#c9741a;--grid:rgba(20,60,55,.10);}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font-family:ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;line-height:1.5;-webkit-font-smoothing:antialiased}
.mono{font-family:ui-monospace,"SF Mono",Menlo,Consolas,monospace;font-variant-numeric:tabular-nums}
.wrap{max-width:1200px;margin:0 auto;padding:28px 22px 60px}
h1{font-size:21px;margin:0 0 4px;text-wrap:balance}
.sub{color:var(--ink-dim);font-size:13px;max-width:74ch}
.eyebrow{font-size:11px;letter-spacing:.16em;text-transform:uppercase;color:var(--accent);font-weight:600;margin-bottom:10px}
.flights{display:flex;flex-wrap:wrap;gap:8px;margin:20px 0 22px}
.fbtn{background:var(--panel);border:1px solid var(--line);color:var(--ink-dim);padding:8px 13px;border-radius:8px;cursor:pointer;font-size:12.5px;display:flex;flex-direction:column;gap:2px}
.fbtn:hover{border-color:var(--accent);color:var(--ink)}
.fbtn.on{background:var(--panel2);border-color:var(--accent);color:var(--ink)}
.fbtn b{font-size:12px;font-weight:600}.fbtn span{font-size:10.5px}
.fbtn:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
.kpis{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin-bottom:20px}
.kpi{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:14px 16px;position:relative;overflow:hidden}
.kpi .lab{font-size:11px;letter-spacing:.08em;text-transform:uppercase;color:var(--ink-dim)}
.kpi .val{font-size:25px;font-weight:600;margin-top:6px}
.kpi .note{font-size:11.5px;color:var(--ink-dim);margin-top:2px}
.kpi .stripe{position:absolute;left:0;top:0;bottom:0;width:3px;background:var(--accent)}
.kpi.good .stripe{background:var(--good)}.kpi.warn .stripe{background:var(--warn)}.kpi.bad .stripe{background:var(--bad)}
.kpi .val.good{color:var(--good)}.kpi .val.warn{color:var(--warn)}.kpi .val.bad{color:var(--bad)}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:18px;align-items:start;margin-bottom:18px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:16px 16px 12px}
.card h2{font-size:13px;margin:0 0 3px}.card .cap{font-size:11.5px;color:var(--ink-dim);margin:0 0 12px}
canvas{width:100%;display:block}
.legend{display:flex;gap:14px;flex-wrap:wrap;font-size:11.5px;color:var(--ink-dim);margin-top:10px}
.legend i{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:5px;vertical-align:-1px}
.charts{display:flex;flex-direction:column;gap:14px}
.verdict{background:var(--panel2);border:1px solid var(--line);border-left:3px solid var(--accent);border-radius:8px;padding:13px 16px;font-size:13px;margin-top:6px}
.verdict b{color:var(--accent)}
@media(max-width:860px){.grid2{grid-template-columns:1fr}.kpis{grid-template-columns:repeat(2,1fr)}}
</style></head><body>
<div class="wrap">
<div class="eyebrow">Avcı İHA · GPS Güdüm Telemetrisi</div>
<h1>GPS Güdüm — Log Paneli</h1>
<p class="sub">Her uçuşta drone'un hedefi nasıl kovaladığı. Yörünge = yukarıdan bakış;
nişangâh = hedefin kamera karesindeki yeri (ideal: merkez, elev 25°); d_h = hedefe yatay mesafe
(kadraj istasyonu ~11 m).</p>
<div class="flights" id="flights"></div>
<div class="kpis" id="kpis"></div>
<div class="grid2">
  <div class="card"><h2>Kuşbakışı yörünge · N-E</h2>
    <p class="cap">Turuncu = hedef, camgöbeği = drone. ● başlangıç, ▲ son. Eş ölçek.</p>
    <canvas id="traj"></canvas>
    <div class="legend"><span><i style="background:var(--tgt)"></i>hedef</span><span><i style="background:var(--accent)"></i>drone</span></div>
  </div>
  <div class="card"><h2>Kamera nişangâhı · 640×480</h2>
    <p class="cap">Hedefin (u,v) izi — koyudan parlağa = zaman. Artı = merkez (320,240).</p>
    <canvas id="reticle"></canvas>
    <div class="legend"><span><i style="background:var(--good)"></i>KILIT</span><span><i style="background:var(--warn)"></i>ARAMA</span><span><i style="background:var(--bad)"></i>kadraj dışı</span></div>
  </div>
</div>
<div class="charts">
  <div class="card"><h2>Menzil d_h — hedefe yatay mesafe</h2>
    <p class="cap">Yeşil çizgi = kadraj istasyonu 11 m. Yaklaşınca iner, orbit'te asılı kalır.</p>
    <canvas id="range"></canvas></div>
  <div class="card"><h2>Kadraj açıları — elev (dikey) &amp; yaw (yatay)</h2>
    <p class="cap">elev hedefi 25° (camgöbeği kesikli), yaw hedefi 0° (turuncu kesikli).</p>
    <canvas id="angles"></canvas>
    <div class="legend"><span><i style="background:var(--accent)"></i>elev</span><span><i style="background:var(--tgt)"></i>yaw</span></div></div>
</div>
<div class="verdict" id="verdict"></div>
</div>
<script>
const DATA=__DATA__;
const css=k=>getComputedStyle(document.documentElement).getPropertyValue(k).trim();
let cur=0;
function stats(f){
  const near=f.pts.filter(p=>p.dh!=null&&p.dh<20&&p.durum==='KILIT'&&p.yaw!=null&&Math.abs(p.u)<2000);
  const base=near.length?near:f.pts.filter(p=>p.yaw!=null&&Math.abs(p.u)<2000);
  const avg=(a,g)=>a.length?a.reduce((x,p)=>x+g(p),0)/a.length:NaN;
  const dhs=f.pts.filter(p=>p.dh!=null).map(p=>p.dh);
  return{near:near.length,lockN:f.pts.filter(p=>p.durum==='KILIT').length,
    yaw:avg(base,p=>Math.abs(p.yaw)),elev:avg(base,p=>p.elev),elevErr:avg(base,p=>Math.abs(p.elev-25)),
    dhAvg:avg(f.pts.filter(p=>p.dh!=null),p=>p.dh),dhMin:dhs.length?Math.min(...dhs):NaN,
    u:avg(base,p=>p.u),v:avg(base,p=>p.v)};
}
const cls=(v,g,w)=>v<=g?'good':v<=w?'warn':'bad';
function renderFlights(){const el=document.getElementById('flights');el.innerHTML='';
  DATA.forEach((f,i)=>{const b=document.createElement('button');b.className='fbtn'+(i===cur?' on':'');
    b.setAttribute('aria-pressed',i===cur);
    b.innerHTML='<b>'+f.etiket+'</b><span class="mono">'+f.n_orig+' kare</span>';
    b.onclick=()=>{cur=i;draw();};el.appendChild(b);});}
function renderKpis(){const s=stats(DATA[cur]);
  const cards=[
    {lab:'En yakın mesafe',val:isNaN(s.dhMin)?'—':s.dhMin.toFixed(1)+' m',note:'istasyon 11 m',c:cls(s.dhMin,14,30)},
    {lab:'Ort. menzil d_h',val:isNaN(s.dhAvg)?'—':s.dhAvg.toFixed(0)+' m',note:s.dhAvg>40?'hiç yaklaşamadı':'yaklaştı',c:cls(s.dhAvg,20,45)},
    {lab:'Yatay |yaw|',val:isNaN(s.yaw)?'—':s.yaw.toFixed(1)+'°',note:'u_px '+(isNaN(s.u)?'—':Math.round(s.u))+' / 320',c:cls(s.yaw,5,12)},
    {lab:'KILIT kare',val:s.lockN,note:s.lockN?'görsel devir bandı':'kilitlenemedi',c:s.lockN>20?'good':'bad'},
  ];
  document.getElementById('kpis').innerHTML=cards.map(c=>'<div class="kpi '+c.c+'"><div class="stripe"></div><div class="lab">'+c.lab+'</div><div class="val '+c.c+'">'+c.val+'</div><div class="note">'+c.note+'</div></div>').join('');}
function fit(id,ratio){const c=document.getElementById(id),r=c.getBoundingClientRect(),d=window.devicePixelRatio||1;
  const h=ratio?r.width*ratio:parseInt(getComputedStyle(c).height)||160;c.width=r.width*d;c.height=h*d;
  const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);return{x,w:r.width,h};}
function drawTraj(){const{x,w,h}=fit('traj',0.85);x.clearRect(0,0,w,h);x.fillStyle=css('--panel2');x.fillRect(0,0,w,h);
  const P=DATA[cur].pts.filter(p=>p.ix!=null&&p.tx!=null);if(!P.length)return;
  let xs=[],ys=[];P.forEach(p=>{xs.push(p.ix,p.tx);ys.push(p.iy,p.ty);});
  const emin=Math.min(...ys),emax=Math.max(...ys),nmin=Math.min(...xs),nmax=Math.max(...xs);
  const pad=26,span=Math.max(emax-emin,nmax-nmin)*1.08||1,ecx=(emin+emax)/2,ncx=(nmin+nmax)/2;
  const sc=(Math.min(w,h)-2*pad)/span,PX=e=>w/2+(e-ecx)*sc,PY=n=>h/2-(n-ncx)*sc;
  x.strokeStyle=css('--grid');x.lineWidth=1;
  for(let g=-250;g<=250;g+=50){x.beginPath();x.moveTo(PX(ecx+g),0);x.lineTo(PX(ecx+g),h);x.stroke();
    x.beginPath();x.moveTo(0,PY(ncx+g));x.lineTo(w,PY(ncx+g));x.stroke();}
  const path=(g,col)=>{x.strokeStyle=col;x.lineWidth=2;x.beginPath();
    P.forEach((p,i)=>{const a=g(p);i?x.lineTo(PX(a[0]),PY(a[1])):x.moveTo(PX(a[0]),PY(a[1]));});x.stroke();};
  path(p=>[p.ty,p.tx],css('--tgt'));path(p=>[p.iy,p.ix],css('--accent'));
  const mk=(e,n,col,tri)=>{x.fillStyle=col;x.beginPath();
    if(tri){x.moveTo(PX(e),PY(n)-5);x.lineTo(PX(e)-4.5,PY(n)+4);x.lineTo(PX(e)+4.5,PY(n)+4);}else x.arc(PX(e),PY(n),4,0,7);
    x.closePath();x.fill();};
  mk(P[0].ty,P[0].tx,css('--tgt'),0);mk(P.at(-1).ty,P.at(-1).tx,css('--tgt'),1);
  mk(P[0].iy,P[0].ix,css('--accent'),0);mk(P.at(-1).iy,P.at(-1).ix,css('--accent'),1);
  x.strokeStyle=css('--line');x.strokeRect(.5,.5,w-1,h-1);
  x.fillStyle=css('--ink-dim');x.font='10px ui-monospace,monospace';x.fillText('E →',w-30,h-8);x.fillText('N ↑',8,16);
  x.strokeStyle=css('--ink-dim');x.beginPath();x.moveTo(14,h-16);x.lineTo(14+50*sc,h-16);x.stroke();x.fillText('50 m',16,h-20);}
function drawReticle(){const{x,w,h}=fit('reticle',0.75);const sx=w/640,sy=h/480;
  x.clearRect(0,0,w,h);x.fillStyle=css('--panel2');x.fillRect(0,0,w,h);x.strokeStyle=css('--grid');
  for(let g=0;g<=640;g+=80){x.beginPath();x.moveTo(g*sx,0);x.lineTo(g*sx,h);x.stroke();}
  for(let g=0;g<=480;g+=80){x.beginPath();x.moveTo(0,g*sy);x.lineTo(w,g*sy);x.stroke();}
  const CX=320*sx,CY=240*sy;x.strokeStyle=css('--accent');x.lineWidth=1.4;
  x.beginPath();x.moveTo(CX-16,CY);x.lineTo(CX+16,CY);x.moveTo(CX,CY-16);x.lineTo(CX,CY+16);x.stroke();
  x.globalAlpha=.5;x.beginPath();x.arc(CX,CY,30,0,7);x.stroke();x.globalAlpha=1;
  const pts=DATA[cur].pts.filter(p=>p.u!=null&&p.v!=null);
  pts.forEach((p,i)=>{let u=p.u,v=p.v,off=false;
    if(u<0||u>640||v<0||v>480){off=true;u=Math.max(0,Math.min(640,u));v=Math.max(0,Math.min(480,v));}
    x.globalAlpha=.15+.85*(i/pts.length);
    x.fillStyle=off?css('--bad'):(p.durum==='KILIT'?css('--good'):css('--warn'));
    x.beginPath();x.arc(u*sx,v*sy,off?2.4:3.1,0,7);x.fill();});
  x.globalAlpha=1;x.strokeStyle=css('--line');x.strokeRect(.5,.5,w-1,h-1);}
function drawRange(){const{x,w,h}=fit('range',0.17);x.clearRect(0,0,w,h);
  const pad={l:40,r:10,t:10,b:18},P=DATA[cur].pts.filter(p=>p.t!=null&&p.dh!=null);if(!P.length)return;
  const T=P.map(p=>p.t),tmin=Math.min(...T),tmax=Math.max(...T),dmax=Math.max(30,...P.map(p=>p.dh));
  const X=t=>pad.l+(t-tmin)/(tmax-tmin||1)*(w-pad.l-pad.r),Y=v=>pad.t+(1-v/dmax)*(h-pad.t-pad.b);
  x.strokeStyle=css('--grid');x.fillStyle=css('--ink-dim');x.font='10px ui-monospace,monospace';
  [0,Math.round(dmax/2),Math.round(dmax)].forEach(g=>{x.beginPath();x.moveTo(pad.l,Y(g));x.lineTo(w-pad.r,Y(g));x.stroke();x.fillText(g+'m',4,Y(g)+3);});
  x.setLineDash([4,4]);x.strokeStyle=css('--good');x.globalAlpha=.7;x.beginPath();x.moveTo(pad.l,Y(11));x.lineTo(w-pad.r,Y(11));x.stroke();x.setLineDash([]);x.globalAlpha=1;
  x.beginPath();P.forEach((p,i)=>{const px=X(p.t),py=Y(p.dh);i?x.lineTo(px,py):x.moveTo(px,py);});
  const g=x.createLinearGradient(0,pad.t,0,h);g.addColorStop(0,css('--accent')+'55');g.addColorStop(1,css('--accent')+'08');
  x.lineTo(X(P.at(-1).t),h-pad.b);x.lineTo(X(P[0].t),h-pad.b);x.closePath();x.fillStyle=g;x.fill();
  x.strokeStyle=css('--accent');x.lineWidth=1.6;x.beginPath();P.forEach((p,i)=>{const px=X(p.t),py=Y(p.dh);i?x.lineTo(px,py):x.moveTo(px,py);});x.stroke();}
function drawAngles(){const{x,w,h}=fit('angles',0.22);x.clearRect(0,0,w,h);
  const pad={l:40,r:10,t:12,b:18},P=DATA[cur].pts.filter(p=>p.t!=null);if(!P.length)return;
  const T=P.map(p=>p.t),tmin=Math.min(...T),tmax=Math.max(...T),ymin=-40,ymax=60;
  const X=t=>pad.l+(t-tmin)/(tmax-tmin||1)*(w-pad.l-pad.r),Y=v=>pad.t+(ymax-v)/(ymax-ymin)*(h-pad.t-pad.b);
  x.fillStyle=css('--ink-dim');x.font='10px ui-monospace,monospace';
  [-40,-20,0,20,25,40,60].forEach(g=>{x.strokeStyle=(g===25||g===0)?css('--line'):css('--grid');
    x.beginPath();x.moveTo(pad.l,Y(g));x.lineTo(w-pad.r,Y(g));x.stroke();x.fillText(g+'°',4,Y(g)+3);});
  x.setLineDash([4,4]);x.strokeStyle=css('--accent');x.globalAlpha=.6;x.beginPath();x.moveTo(pad.l,Y(25));x.lineTo(w-pad.r,Y(25));x.stroke();
  x.strokeStyle=css('--tgt');x.beginPath();x.moveTo(pad.l,Y(0));x.lineTo(w-pad.r,Y(0));x.stroke();x.setLineDash([]);x.globalAlpha=1;
  const line=(g,col)=>{x.strokeStyle=col;x.lineWidth=1.5;x.beginPath();let st=false;
    P.forEach(p=>{let v=g(p);if(v==null||isNaN(v)){st=false;return;}v=Math.max(ymin,Math.min(ymax,v));
      const px=X(p.t),py=Y(v);st?x.lineTo(px,py):x.moveTo(px,py);st=true;});x.stroke();};
  line(p=>p.elev,css('--accent'));line(p=>p.yaw,css('--tgt'));}
function verdict(){const s=stats(DATA[cur]),el=document.getElementById('verdict');let m;
  if(s.lockN===0&&s.dhAvg>40){
    m='<b>Hiç yaklaşamadı:</b> ort d_h <b class="mono">'+s.dhAvg.toFixed(0)+' m</b> (en yakın '+
      s.dhMin.toFixed(0)+' m), 0 KILIT karesi → görsel devir yok. Yörüngeye bak: drone hedefin '+
      '<b>dışında daha büyük bir yay</b> çiziyorsa, saf kuyruk-takibi dönen hedefte içeri kesemiyor demektir.';
  }else if(s.lockN>0){
    const yon=s.v>240?'merkezin biraz altında':'merkeze yakın';
    m='<b>Kadraja girdi:</b> en yakın <b class="mono">'+s.dhMin.toFixed(1)+' m</b>, yatay |yaw| ≈ <b class="mono">'+
      s.yaw.toFixed(1)+'°</b> (u_px '+Math.round(s.u)+'/320), elev ≈ <b class="mono">'+s.elev.toFixed(1)+
      '°</b> (hedef 25° → hedef '+yon+'). <b class="mono">'+s.lockN+'</b> KILIT karesi.';
  }else{
    m='<b>Yaklaştı ama kilitlenemedi:</b> en yakın <b class="mono">'+s.dhMin.toFixed(1)+
      ' m</b>, KILIT yok. Nişangâhta hedefin merkezden ne kadar saptığına bak.';
  }
  el.innerHTML=m;}
function draw(){renderFlights();renderKpis();drawTraj();drawReticle();drawRange();drawAngles();verdict();}
window.addEventListener('resize',()=>{drawTraj();drawReticle();drawRange();drawAngles();});
draw();
</script></body></html>'''


if __name__ == "__main__":
    sys.exit(main())
