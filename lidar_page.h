#pragma once

// LiDAR viewer web page. Served at http://<device>/ in station mode.
// Polls /lidar/scan ~12 Hz and renders the floor polygon, extruded walls,
// raw points (speed-coloured), and smiley-faced person markers on a canvas.
// Reads /lidar/scan's "persons" array — detection runs server-side in
// personTask() so the page only paints.

static const char LIDAR_PAGE_HTML[] = R"rawhtml(<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>LiDAR</title><style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:sans-serif;background:#050510;color:#e8eaff;min-height:100vh;display:flex;flex-direction:column;align-items:center;padding:10px;gap:8px}
h1{font-size:1.2rem;letter-spacing:3px;color:#7cf0ff;margin:4px 0}
#hud{font-size:.85rem;color:#a8b0d0;display:flex;gap:14px;flex-wrap:wrap;justify-content:center}
#hud b{color:#7cf0ff}
canvas{background:#02050d;border:1px solid #1c2848;border-radius:8px;max-width:100%;height:auto;touch-action:none}
.row{display:flex;gap:8px;align-items:center;font-size:.85rem;color:#a8b0d0;flex-wrap:wrap;justify-content:center}
button{background:#142036;border:1px solid #2a3a60;color:#cfe1ff;padding:5px 12px;border-radius:6px;cursor:pointer}
button.on{background:#1d3a6e;border-color:#7cf0ff;color:#7cf0ff}
a{color:#7cf0ff;font-size:.8rem}
#ppllist{font-size:.8rem;color:#cfe1ff;background:#0a1530;border:1px solid #1c2848;border-radius:6px;padding:6px 12px;min-width:240px;text-align:left;font-family:monospace;white-space:pre;line-height:1.4em}
</style></head><body>
<h1>LiDAR LD19</h1>
<div id='hud'><span>RPM <b id='rpm'>-</b></span><span>Points <b id='np'>-</b></span><span>Area <b id='area'>-</b> m&sup2;</span><span>FPS <b id='fps'>-</b></span><span>Ppl <b id='ppl'>-</b></span></div>
<canvas id='c'></canvas>
<div class='row'>
  <button id='pplBtn' class='on'>People</button>
  <button id='raw'>Raw</button>
  <button id='hold'>Hold</button>
  <button id='mirror'>Mirror</button>
  <button id='record'>Record</button>
  <button id='mapBtn'>Map</button>
</div>
<div id='ppllist'>People detection off</div>
<a href='/sokoban'>Sokoban &rarr;</a>
<script>
var c=document.getElementById('c'),ctx=c.getContext('2d');
var dpr=Math.min(window.devicePixelRatio||1,2);
var paused=false,rawOn=false,frames=0,lastFps=performance.now();
// Fixed world rectangle: x in [-4,+4] m left..right, y in [0,5] m sensor..forward
var WX0=-4,WX1=4,WY0=0,WY1=5;
var prevX=new Float32Array(360),prevY=new Float32Array(360),prevTime=new Float64Array(360);
var prevSeen=new Uint8Array(360),speedBins=new Float32Array(360);
var personsOn=true,lastPersons=[],mirror=false;
var recording=false, recordedData=[];
var mapOn=false, refMap=null, refBbox=null, mapFetching=false, refMapN=0, poseTrail=[], lastPose=null;
var SENSOR_ROT_DEG=270; // physical front sits at sensor angle 270°
var WALL_CSS=70;   // wall extrusion height in CSS px
var WALL_PX=WALL_CSS*dpr;
function sz(){
  var availW=window.innerWidth-20, availH=window.innerHeight-260;
  var aspect=(WX1-WX0)/(WY1-WY0); // floor width:height (world units)
  var floorW=Math.min(availW, (availH-WALL_CSS)*aspect, 900);
  if(floorW<200)floorW=200;
  var floorH=floorW/aspect;
  var h=floorH+WALL_CSS;
  c.style.width=floorW+'px';c.style.height=h+'px';
  c.width=floorW*dpr;c.height=h*dpr;
}
sz();window.addEventListener('resize',sz);
document.getElementById('hold').onclick=function(){
  paused=!paused;this.innerText=paused?'Run':'Hold';
};
document.getElementById('raw').onclick=function(){
  rawOn=!rawOn;this.classList.toggle('on',rawOn);
};
document.getElementById('pplBtn').onclick=function(){
  personsOn=!personsOn;this.classList.toggle('on',personsOn);
};
document.getElementById('mirror').onclick=function(){
  mirror=!mirror;this.classList.toggle('on',mirror);
};
document.getElementById('record').onclick=function(){
  recording=!recording;
  this.classList.toggle('on',recording);
  this.innerText=recording?'Stop':'Record';
  if(!recording){
    var a=document.createElement('a');
    a.href=URL.createObjectURL(new Blob([JSON.stringify(recordedData)],{type:'application/json'}));
    a.download='lidar_data.json';
    a.click();
    recordedData=[];
  }
};
function fetchMap(){
  if(mapFetching) return;
  mapFetching=true;
  fetch('/lidar/map',{cache:'no-store'}).then(function(r){return r.json();}).then(function(m){
    refMap=m.pts; refMapN=m.n;
    if(refMap.length){
      var xs=refMap.map(function(p){return p[0];}),ys=refMap.map(function(p){return p[1];});
      var pad=0.5;
      refBbox=[Math.min.apply(null,xs)-pad,Math.min.apply(null,ys)-pad,
               Math.max.apply(null,xs)+pad,Math.max.apply(null,ys)+pad];
    }
    mapFetching=false;
  }).catch(function(){mapFetching=false;});
}
document.getElementById('mapBtn').onclick=function(){
  mapOn=!mapOn;this.classList.toggle('on',mapOn);
  if(mapOn){
    // Fresh start: clear local cache, ask device to re-seed, then poll for
    // the new map. Until seeding finishes (~4 s) lastPose.valid stays false
    // and drawMap shows the "Seeding…" message.
    refMap=null;refBbox=null;refMapN=0;poseTrail=[];lastPose=null;
    fetch('/lidar/map/reset',{method:'POST'}).catch(function(){});
  }
};
function drawMap(d){
  // Map-frame render. Reference map = grey dots, current scan (transformed by
  // pose) = red, robot = blue dot + heading line, trail = green.
  var w=c.width,h=c.height;
  ctx.fillStyle='#02050d';ctx.fillRect(0,0,w,h);
  if(!refMap||!refBbox||refMap.length===0){
    ctx.fillStyle='#a8b0d0';ctx.font=(14*dpr)+'px sans-serif';
    var msg = (lastPose && !lastPose.valid) ? 'Seeding map (hold still)…' : 'Loading map…';
    ctx.fillText(msg,20*dpr,30*dpr);
    return;
  }
  var bx0=refBbox[0],by0=refBbox[1],bx1=refBbox[2],by1=refBbox[3];
  var bw=bx1-bx0,bh=by1-by0;
  var sx=w/bw,sy=h/bh,sc=Math.min(sx,sy);
  var ox=(w-bw*sc)/2-bx0*sc, oy=(h-bh*sc)/2-by0*sc;
  function mx(x){return x*sc+ox;}
  function my(y){return h-(y*sc+oy);} // flip y so +y is up
  ctx.fillStyle='#3a4060';
  for(var i=0;i<refMap.length;i++){
    var p=refMap[i];
    ctx.fillRect(mx(p[0])-1,my(p[1])-1,2,2);
  }
  // Current scan transformed into world frame using lastPose
  if(lastPose && lastPose.valid && d && d.pts){
    var cx=Math.cos(lastPose.th),sy2=Math.sin(lastPose.th);
    ctx.fillStyle='rgba(228,68,68,0.85)';
    for(var i=0;i<d.pts.length;i+=2){
      var q=d.pts[i];
      var dmm=q[1]; if(dmm<100||dmm>=12000) continue; // LD19 rated max range (mm)
      var a=q[0]/100*Math.PI/180;
      var lx=Math.cos(a)*dmm/1000, ly=Math.sin(a)*dmm/1000;
      var wx=cx*lx-sy2*ly+lastPose.x;
      var wy=sy2*lx+cx*ly+lastPose.y;
      ctx.beginPath();ctx.arc(mx(wx),my(wy),1.5*dpr,0,Math.PI*2);ctx.fill();
    }
  }
  // Trail
  if(poseTrail.length>1){
    ctx.strokeStyle='#4a8';ctx.lineWidth=1.5*dpr;
    ctx.beginPath();
    for(var i=0;i<poseTrail.length;i++){
      var p=poseTrail[i];
      if(i===0) ctx.moveTo(mx(p[0]),my(p[1])); else ctx.lineTo(mx(p[0]),my(p[1]));
    }
    ctx.stroke();
  }
  // Robot
  if(lastPose && lastPose.valid){
    var rx=mx(lastPose.x),ry=my(lastPose.y);
    ctx.fillStyle='#27f';
    ctx.beginPath();ctx.arc(rx,ry,6*dpr,0,Math.PI*2);ctx.fill();
    ctx.strokeStyle='#27f';ctx.lineWidth=2*dpr;
    var hx=rx+Math.cos(lastPose.th)*20*dpr;
    var hy=ry-Math.sin(lastPose.th)*20*dpr;
    ctx.beginPath();ctx.moveTo(rx,ry);ctx.lineTo(hx,hy);ctx.stroke();
  }
  // Status overlay
  ctx.fillStyle='#a8b0d0';ctx.font=(12*dpr)+'px monospace';
  var status='map: '+refMapN+' pts';
  if(lastPose){
    status+='  pose: '+(lastPose.valid?'tracking':'not locked');
    if(lastPose.valid){
      status+='  err='+lastPose.err.toFixed(3)+'m  in='+lastPose.in;
    }
  }
  ctx.fillText(status,8*dpr,16*dpr);
}
function w2x(x){return (x-WX0)/(WX1-WX0)*c.width;}
function w2y(y){return WALL_PX + (y-WY0)/(WY1-WY0)*(c.height-WALL_PX);}
function drawSmiley(px,py,r){
  ctx.fillStyle='rgba(0,0,0,0.45)';ctx.beginPath();ctx.ellipse(px,py+r*0.95,r*0.75,r*0.22,0,0,Math.PI*2);ctx.fill();
  ctx.fillStyle='#ffd54f';ctx.beginPath();ctx.arc(px,py,r,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle='#3e2723';ctx.lineWidth=Math.max(1,r*0.09);ctx.stroke();
  ctx.fillStyle='#3e2723';
  ctx.beginPath();ctx.arc(px-r*0.35,py-r*0.18,r*0.13,0,Math.PI*2);ctx.fill();
  ctx.beginPath();ctx.arc(px+r*0.35,py-r*0.18,r*0.13,0,Math.PI*2);ctx.fill();
  ctx.beginPath();ctx.arc(px,py+r*0.05,r*0.45,0.15*Math.PI,0.85*Math.PI);ctx.stroke();
}
function draw(d){
  if(recording) recordedData.push(d);
  if(d&&typeof d.area==='number')document.getElementById('area').innerText=d.area.toFixed(2);
  var w=c.width,h=c.height;
  ctx.fillStyle='#02050d';ctx.fillRect(0,0,w,h);
  if(!d||!d.pts){document.getElementById('np').innerText='0';return;}
  // Build per-degree bin array (display frame, 0..359). Take min distance per bin.
  var bins=new Float32Array(360);
  for(var i=0;i<d.pts.length;i++){
    var q=d.pts[i];
    if(q[1]<=0) continue;
    var ra=q[0]-SENSOR_ROT_DEG*100;if(ra<0)ra+=36000;
    var bin=Math.floor(ra/100)%360;if(bin<0)bin+=360;
    if(bins[bin]===0 || q[1]<bins[bin]) bins[bin]=q[1];
  }
  // Fill gaps with average of nearest valid neighbors so missing bins (90° dead zone, etc.) interpolate cleanly
  for(var k=0;k<360;k++){
    if(bins[k]>0) continue;
    var L=0,R=0;
    for(var s=1;s<=180;s++){
      if(L===0){var li=(k-s+360)%360; if(bins[li]>0)L=bins[li];}
      if(R===0){var ri=(k+s)%360; if(bins[ri]>0)R=bins[ri];}
      if(L>0&&R>0)break;
    }
    if(L>0&&R>0) bins[k]=(L+R)/2;
    else if(L>0) bins[k]=L;
    else if(R>0) bins[k]=R;
  }
  var pp=[];
  var now=performance.now();
  for(var b=0;b<360;b++)speedBins[b]=0;
  for(var bin=0;bin<360;bin++){
    var dmm=bins[bin];
    if(dmm<=0) continue;
    var ang=bin*Math.PI/180;
    var wx=-Math.sin(ang)*dmm/1000;
    var wy= Math.cos(ang)*dmm/1000;
    if(mirror)wx=-wx;
    var xmm=wx*1000,ymm=wy*1000;
    if(prevSeen[bin]){
      var dxw=xmm-prevX[bin], dyw=ymm-prevY[bin];
      var dt=(now-prevTime[bin])/1000;
      if(dt>0.01&&dt<2){speedBins[bin]=Math.max(speedBins[bin],Math.sqrt(dxw*dxw+dyw*dyw)/dt);}
    }
    prevX[bin]=xmm; prevY[bin]=ymm; prevTime[bin]=now; prevSeen[bin]=1;
    if(wx<WX0)wx=WX0;else if(wx>WX1)wx=WX1;
    if(wy<WY0)wy=WY0;else if(wy>WY1)wy=WY1;
    pp.push([wx,wy,bin*100,bin]);
  }
  pp.sort(function(a,b){return a[2]-b[2];});
  lastPersons=(personsOn&&d.persons)?d.persons:[];
  document.getElementById('ppl').innerText=personsOn?lastPersons.length:'-';
  var pl=document.getElementById('ppllist');
  if(!personsOn){pl.innerText='People detection off';}
  else if(lastPersons.length===0){pl.innerText='People: 0';}
  else{var lines=['People: '+lastPersons.length];
    for(var k=0;k<lastPersons.length;k++){var pn=lastPersons[k];
      lines.push((k+1)+': x='+(pn.x>=0?'+':'')+pn.x.toFixed(2)+' y='+(pn.y>=0?'+':'')+pn.y.toFixed(2)+' m  a='+Math.round(pn.a)+' d='+pn.d.toFixed(2)+'m');}
    pl.innerText=lines.join('\n');}
  if(rawOn){
    for(var i=0;i<pp.length;i++){
      var p=pp[i];
      var spd=speedBins[p[3]];
      var px=w2x(p[0]), py=w2y(p[1]);
      var t=Math.min(1,spd/1000);
      var hue=60-60*t;
      ctx.fillStyle='hsla('+hue+',90%,55%,0.95)';
      ctx.beginPath();ctx.arc(px,py,(2+2*t)*dpr,0,Math.PI*2);ctx.fill();
    }
    if(personsOn)for(var pi=0;pi<lastPersons.length;pi++){var p=lastPersons[pi];drawSmiley(w2x(mirror?-p.x:p.x),w2y(p.y),14*dpr);}
    document.getElementById('np').innerText=pp.length;
    document.getElementById('rpm').innerText=d.rpm;
    return;
  }
  if(pp.length>=2){
    // floor polygon: sensor (0,0) → sorted points → close
    var gp=[];
    gp.push([w2x(0),w2y(0)]);
    for(var i=0;i<pp.length;i++){gp.push([w2x(pp[i][0]),w2y(pp[i][1])]);}
    ctx.beginPath();
    for(var i=0;i<gp.length;i++){if(i===0)ctx.moveTo(gp[i][0],gp[i][1]);else ctx.lineTo(gp[i][0],gp[i][1]);}
    ctx.closePath();
    ctx.fillStyle='rgba(18,38,64,0.55)';ctx.fill();
    // extruded walls along the LiDAR perimeter
    var segs=[];
    for(var i=1;i<gp.length;i++){segs.push({a:gp[i],b:gp[i===gp.length-1?1:i+1]});}
    segs.sort(function(s1,s2){return (s1.a[1]+s1.b[1])-(s2.a[1]+s2.b[1]);});
    for(var i=0;i<segs.length;i++){
      var s=segs[i];
      var ax=s.a[0],ay=s.a[1],bx=s.b[0],by=s.b[1];
      var tay=ay-WALL_PX,tby=by-WALL_PX;
      var topY=Math.min(tay,tby),botY=Math.max(ay,by);
      var grad=ctx.createLinearGradient(0,topY,0,botY);
      grad.addColorStop(0,   'rgba(200,235,250,0.95)');
      grad.addColorStop(0.35,'rgba(95,165,210,0.9)');
      grad.addColorStop(1,   'rgba(15,45,85,0.85)');
      ctx.beginPath();
      ctx.moveTo(ax,ay);ctx.lineTo(bx,by);ctx.lineTo(bx,tby);ctx.lineTo(ax,tay);
      ctx.closePath();
      ctx.fillStyle=grad;ctx.fill();
      ctx.strokeStyle='rgba(230,250,255,0.95)';
      ctx.lineWidth=1.4*dpr;
      ctx.beginPath();ctx.moveTo(ax,tay);ctx.lineTo(bx,tby);ctx.stroke();
    }
  }
  // Sensor dot
  ctx.fillStyle='#7cf0ff';
  ctx.beginPath();ctx.arc(w2x(0),w2y(0),5*dpr,0,Math.PI*2);ctx.fill();

  if(personsOn)for(var pi=0;pi<lastPersons.length;pi++){var p=lastPersons[pi];var prad=Math.max(12,28-22*p.d/4)*dpr;drawSmiley(w2x(mirror?-p.x:p.x),w2y(p.y)-WALL_PX*1.3,prad);}
  document.getElementById('np').innerText=pp.length;
  document.getElementById('rpm').innerText=d.rpm;
}
async function poll(){
  if(!paused){
    try{
      var r=await fetch('/lidar/scan',{cache:'no-store'});
      var d=await r.json();
      if(d&&d.pose){
        lastPose=d.pose;
        if(d.pose.valid){
          var t=poseTrail[poseTrail.length-1];
          if(!t || (t[0]-d.pose.x)*(t[0]-d.pose.x)+(t[1]-d.pose.y)*(t[1]-d.pose.y)>0.0025){
            poseTrail.push([d.pose.x,d.pose.y]);
            if(poseTrail.length>400) poseTrail.shift();
          }
        }
      }
      // Map grew? Refetch — but only while the user is looking at it, to
      // avoid hammering the device with full-map JSON every poll cycle.
      if(mapOn && typeof d.map_n === 'number' && d.map_n !== refMapN) fetchMap();
      if(mapOn) drawMap(d); else draw(d);
      frames++;
      var now=performance.now();
      if(now-lastFps>1000){document.getElementById('fps').innerText=(frames*1000/(now-lastFps)).toFixed(1);frames=0;lastFps=now;}
    }catch(e){}
  }
  setTimeout(poll,80);
}
poll();
</script></body></html>)rawhtml";
