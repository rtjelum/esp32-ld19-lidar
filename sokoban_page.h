#pragma once

// Sokoban game web page. Served at http://<device>/sokoban.
// Isometric canvas renderer with 50 built-in levels (defined inline in the JS
// `L` array — separate from sokoban_levels.h, which is currently unused).
// Two input modes: on-screen D-pad / arrow keys, and LiDAR motion control —
// the page polls /lidar/scan, maps the nearest detected person onto a 3x3
// floor grid in front of the sensor, and fires a move when they step into a
// new cell. The grid bounds (GRID_HX, GRID_Y0..Y1) are tunable in the JS.

static const char SOKOBAN_PAGE_HTML[] = R"rawhtml(<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Sokoban</title><style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',sans-serif;background:linear-gradient(160deg,#3a1c5a 0%,#1a0d2e 60%,#050510 100%);color:#fff3e0;min-height:100vh;padding:14px;display:flex;flex-direction:column;align-items:center;gap:10px}
h1{font-size:1.4rem;letter-spacing:4px;background:linear-gradient(90deg,#ffd54f,#ff8a65,#f06292);-webkit-background-clip:text;background-clip:text;color:transparent;margin:8px 0;font-weight:800}
#st{font-size:.9rem;color:#ffd54f;text-align:center}
canvas{background:#1b0d2e;border:2px solid #ffd54f;border-radius:12px;box-shadow:0 4px 24px rgba(255,167,38,.25);max-width:100%;height:auto;touch-action:none}
.btn{width:100%;max-width:310px;padding:12px;background:linear-gradient(135deg,#ff7043,#ffca28);border:none;border-radius:10px;color:#3e2723;font-size:.9rem;font-weight:800;cursor:pointer;margin-top:5px}
.btn.s{background:rgba(255,255,255,.08);color:#ffe082;font-weight:500;border:1px solid rgba(255,213,79,.2)}
.pad{display:grid;grid-template-columns:repeat(3,60px);gap:10px;margin-top:10px}
.pb{width:60px;height:60px;background:rgba(255,213,79,.15);border:1px solid rgba(255,213,79,.4);border-radius:10px;color:#ffe082;display:flex;align-items:center;justify-content:center;font-size:24px;user-select:none;-webkit-tap-highlight-color:transparent}
.pb:active{background:rgba(255,167,38,.5);color:#fff}
@media (hover:hover) and (pointer:fine){.pad,#btnRst,#btnReset{display:none!important}}
body.trk-on .pad{display:none!important}
body:not(.trk-on) #trkGrid,body:not(.trk-on) #trkStatus{display:none}
#trkBar{display:flex;flex-direction:column;align-items:center;gap:4px;margin-top:6px}
#trkStatus{font-size:.8rem;color:#cfe1ff;font-family:monospace}
#trkGrid{display:grid;grid-template-columns:repeat(3,70px);grid-template-rows:repeat(3,70px);gap:6px;position:relative}
.tc{width:70px;height:70px;background:rgba(255,213,79,.08);border:1px solid rgba(255,213,79,.2);border-radius:8px;display:flex;align-items:center;justify-content:center;font-size:30px;color:#ffe082}
.tc.b{visibility:hidden}
.tc.active{background:rgba(255,213,79,.78);color:#3e2723;border-color:#ffd54f}
#trkDot{position:absolute;width:18px;height:18px;border-radius:50%;background:#7cf0ff;border:2px solid #fff;box-shadow:0 0 10px #7cf0ff;pointer-events:none;display:none;transform:translate(-50%,-50%);z-index:2}
</style></head><body class='trk-on'>
<h1>SOKOBAN</h1><div id='st'>Moves: 0</div><canvas id='g' width='320' height='240'></canvas>
<div class='pad'>
<div></div><div class='pb' onclick='mvK(0,-1)'>&uarr;</div><div></div>
<div class='pb' onclick='mvK(-1,0)'>&larr;</div><div class='pb' onclick='mvK(0,1)'>&darr;</div><div class='pb' onclick='mvK(1,0)'>&rarr;</div>
</div>
<div id='trkBar'>
  <button class='btn s' id='trkBtn' style='max-width:220px;padding:6px 10px;margin-top:0' onclick='trkToggle()'>Tracker: ON</button>
  <div id='trkGrid'>
    <div class='tc b'></div><div class='tc' data-cell='up'>&uarr;</div><div class='tc b'></div>
    <div class='tc' data-cell='left'>&larr;</div><div class='tc' data-cell='idle'>&middot;</div><div class='tc' data-cell='right'>&rarr;</div>
    <div class='tc b'></div><div class='tc' data-cell='down'>&darr;</div><div class='tc b'></div>
    <div id='trkDot'></div>
  </div>
  <div id='trkStatus'>none</div>
</div>
<button class='btn' id='btnRst' onclick='reset()'>Restart Level</button>
<button class='btn s' onclick='location.href="/"'>Back to Menu</button>
<script>
var L=[["####","# .#","#  ###","#*@  #","#  $ #","#  ###","####"],
["######","#    #","# #@ #","# $* #","# .* #","#    #","######"],
["  ####","###  ####","#     $ #","# #  #$ #","# . .#@ #","#########"],
["########","#      #","# .**$@#","#      #","#####  #","    ####"],
[" #######"," #     #"," # .$. #","## $@$ #"," #  .$. #","#      #","########"],
["###### #####","#    ###   #","# $$     #@#","# $ #...   #","#   ########","#####"],
["#######","#     #","# .$. #","# $.$ #","# .$. #","# $.$ #","#  @  #","#######"],
["  ######","  # ..@#","  # $$ #","  ## ###","   # #","   # #","#### #","#    ##","# #   #","#   # #","###   #","  #####"],
["#####","#.  ##","#@$$ #","##   #"," ##  #","  ##.#","   ###"],
["      #####","      #.  #","      #.# #","#######.# #","# @ $ $ $ #","# # # # ###","#       #","#########"],
["  ######","  #    #","  # ##@##","### # $ #","# ..# $ #","#       #","#  ######","####"],
["#####","#   ##","# $  #","## $ ####"," ###@.  #","  #  .# #","  #     #","  #######"],
["####","#. ##","#.@ #","#. $#","##$ ###"," # $  #"," #    #"," #  ###"," ####"],
["#######","#     #","# # # #","#. $*@#","#   ###","#####"],
["     ###","######@##","#    .* #","#   #   #","#####$# #","    #   #","    #####"],
[" ####"," #  ####"," #     ##","## ##   #","#. .# @$##","#   # $$ #","#  .#    #","##########"],
["#####","# @ #","#...#","#$$$##","#    #","#    #","######"],
["#######","#     #","#. .  #","# ## ##","#  $ #","###$ #","  #@ #","  #  #","  ####"],
["########","#   .. #","#  @$$ #","##### ##","   #  #","   #  #","   #  #","   ####"],
["#######","#     ###","#  @$$..#","#### ## #","  #     #","  #  ####","  #  #","  ####"],
["####","#  ####","# . . #","# $$#@#","##    #"," ######"],
["#####","#   ###","#. .  #","#   # #","## #  #"," #@$$ #"," #    #"," #  ###"," ####"],
["#######","#  *  #","#     #","## # ##"," #$@.#"," #   #"," #####"],
["# #####","  #   #","###$$@#","#   ###","#     #","# . . #","#######"],
[" ####"," #  ###"," # $$ #","##... #","#  @$ #","#   ###","#####"],
[" #####"," # @ #"," #   #","###$ #","# ...#","# $$ #","###  #","  ####"],
["######","#   .#","# ## ##","#  $$@#","# #   #","#.  ###","#####"],
["#####","#   #","# @ #","# $$###","##. . #"," #    #"," ######"],
["     #####","     #   ##","     #    #"," ######   #","##     #. #","# $ $ @  ##","# ######.#","#        #","##########"],
["####","#  ###","# $$ #","#... #","# @$ #","#   ##","#####"],
["  ####"," ##  #","##@$.##","# $$  #","# . . #","###   #","  #####"],
[" ####","##  ###","#     #","#.**$@#","#   ###","##  #"," ####"],
["#######","#. #  #","#  $  #","#. $#@#","#  $  #","#. #  #","#######"],
["  ####","###  ####","#       #","#@$***. #","#       #","#########"],
["  ####"," ##  #"," #. $#"," #.$ #"," #.$ #"," #.$ #"," #. $##"," #   @#"," ##   #","  #####"],
["####","#  ############","# $ $ $ $ $ @ #","# .....       #","###############"],
["      ###","##### #.#","#   ###.#","#   $ #.#","# $  $  #","#####@# #","    #   #","    #####"],
["##########","#        #","# ##.### #","# # $$ . #","# . @$## #","#####    #","    ######"],
["#####","#   ####","# # # .#","#    $ ###","### #$.  #","#   #@   #","# # ######","#   #","#####"],
[" #####"," #   #","##   ##","# $$$ #","# .+. #","#######"],
["#######","#     #","#@$$$ ##","#  #...#","##    ##"," ######"],
["   ####","   #  #","   #@ #","####$.#","#   $.#","# # $.#","#    ##","######"],
["     ####","     # @#","     #  #","###### .#","#   $  .#","#  $$# .#","#    ####","###  #","  ####"],
["#####","#@$.#","#####"],
["######","#... #","#  $ #","# #$##","#  $ #","#  @ #","######"],
[" ######","##    #","#  ## #","# # $ #","#  * .#","## #@##"," #   #"," #####"],
["  #######","###     #","# $ $   #","# ### #####","# @ . .   #","#   ###   #","##### #####"],
["######","#  @ #","#  # ##","# .#  ##","# .$$$ #","# .#   #","####   #","   #####"],
["######","# @  #","# $# #","# $  #","# $ ##","### ####"," #  #  #"," #...  #"," #     #"," #######"],
["  ####","###  #####","#  $  @..#","# $    # #","### #### #","  #      #","  ########"]];
var cur=0,map=[],px,py,mv=0;
var c=document.getElementById('g'),ctx=c.getContext('2d');
// Isometric constants
var TW=20,TH=12,WH=11,BH=14,PH=36,offX=0,offY=0;
function ix(x,y){return (x-y)*TW+offX;}
function iy(x,y){return (x+y)*TH+offY;}
function diamond(cx,cy,fill,stroke){
  ctx.beginPath();
  ctx.moveTo(cx,cy);ctx.lineTo(cx+TW,cy+TH);ctx.lineTo(cx,cy+2*TH);ctx.lineTo(cx-TW,cy+TH);ctx.closePath();
  if(fill){ctx.fillStyle=fill;ctx.fill();}
  if(stroke){ctx.strokeStyle=stroke;ctx.stroke();}
}
function cube(x,y,ht,topC,leftC,rightC,edge){
  var cx=ix(x,y),cy=iy(x,y);
  ctx.fillStyle=rightC;
  ctx.beginPath();
  ctx.moveTo(cx+TW,cy+TH);ctx.lineTo(cx,cy+2*TH);ctx.lineTo(cx,cy+2*TH-ht);ctx.lineTo(cx+TW,cy+TH-ht);
  ctx.closePath();ctx.fill();
  if(edge){ctx.strokeStyle=edge;ctx.lineWidth=1;ctx.stroke();}
  ctx.fillStyle=leftC;
  ctx.beginPath();
  ctx.moveTo(cx-TW,cy+TH);ctx.lineTo(cx,cy+2*TH);ctx.lineTo(cx,cy+2*TH-ht);ctx.lineTo(cx-TW,cy+TH-ht);
  ctx.closePath();ctx.fill();
  if(edge)ctx.stroke();
  ctx.fillStyle=topC;
  ctx.beginPath();
  ctx.moveTo(cx,cy-ht);ctx.lineTo(cx+TW,cy+TH-ht);ctx.lineTo(cx,cy+2*TH-ht);ctx.lineTo(cx-TW,cy+TH-ht);
  ctx.closePath();ctx.fill();
  if(edge)ctx.stroke();
}
function drawPawn(x,y,colA,colB){
  var cx=ix(x,y),cy=iy(x,y)+TH;
  ctx.fillStyle='rgba(0,0,0,0.35)';
  ctx.beginPath();ctx.ellipse(cx,cy+TH*0.6,TW*0.45,TH*0.45,0,0,Math.PI*2);ctx.fill();
  var bodyH=Math.max(TW*0.95,18);
  ctx.fillStyle=colA;
  ctx.beginPath();ctx.ellipse(cx,cy-bodyH*0.35,TW*0.38,bodyH*0.55,0,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle=colB;ctx.lineWidth=1.5;ctx.stroke();
  var hr=Math.max(TW*0.28,5);
  ctx.fillStyle='#ffe0b2';
  ctx.beginPath();ctx.arc(cx,cy-bodyH*0.95,hr,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle='#5d4037';ctx.lineWidth=1;ctx.stroke();
  ctx.fillStyle='#3e2723';
  ctx.beginPath();ctx.arc(cx-hr*0.4,cy-bodyH*0.95,Math.max(hr*0.18,1),0,Math.PI*2);ctx.fill();
  ctx.beginPath();ctx.arc(cx+hr*0.4,cy-bodyH*0.95,Math.max(hr*0.18,1),0,Math.PI*2);ctx.fill();
}
function drawGoal(x,y){
  var cx=ix(x,y),cy=iy(x,y)+TH;
  ctx.fillStyle='#f8bbd0';
  ctx.beginPath();ctx.ellipse(cx,cy,TW*0.42,TH*0.62,0,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle='#e91e63';ctx.lineWidth=2;ctx.stroke();
  ctx.fillStyle='#e91e63';
  ctx.beginPath();ctx.ellipse(cx,cy,TW*0.16,TH*0.24,0,0,Math.PI*2);ctx.fill();
  ctx.lineWidth=1;
}
function drawSparkle(x,y){
  var cx=ix(x,y),cy=iy(x,y)+TH-BH;
  ctx.strokeStyle='#fff59d';ctx.lineWidth=2;
  ctx.beginPath();
  ctx.moveTo(cx-TW*0.25,cy);ctx.lineTo(cx+TW*0.25,cy);
  ctx.moveTo(cx,cy-TH*0.4);ctx.lineTo(cx,cy+TH*0.4);
  ctx.stroke();ctx.lineWidth=1;
}
function load(i){
  mv=0;map=L[i].map(function(r){return r.split('');});
  for(var y=0;y<map.length;y++)for(var x=0;x<map[y].length;x++){
    if(map[y][x]=='@'){px=x;py=y;map[y][x]=' ';}
    if(map[y][x]=='+'){px=x;py=y;map[y][x]='.';}
  }
  draw();
}
function cliffR(x,y){
  var cx=ix(x,y),cy=iy(x,y);
  ctx.fillStyle='#5d4037';
  ctx.beginPath();
  ctx.moveTo(cx+TW,cy+TH);ctx.lineTo(cx,cy+2*TH);ctx.lineTo(cx,cy+2*TH+PH);ctx.lineTo(cx+TW,cy+TH+PH);
  ctx.closePath();ctx.fill();
}
function cliffL(x,y){
  var cx=ix(x,y),cy=iy(x,y);
  ctx.fillStyle='#8d6e63';
  ctx.beginPath();
  ctx.moveTo(cx-TW,cy+TH);ctx.lineTo(cx,cy+2*TH);ctx.lineTo(cx,cy+2*TH+PH);ctx.lineTo(cx-TW,cy+TH+PH);
  ctx.closePath();ctx.fill();
}
function draw(){
  var h=map.length,w=0;
  for(var i=0;i<h;i++)if(map[i].length>w)w=map[i].length;
  if(w<1||h<1)return;
  TW=Math.min(22,Math.max(10,Math.floor(440/(w+h))));
  TH=Math.round(TW*0.58);
  WH=Math.round(TW*0.55);
  BH=Math.round(TW*0.75);
  PH=Math.round(TW*1.8);
  var topPad=Math.round(TW*1.2);
  c.width=(w+h)*TW+4;
  c.height=(w+h)*TH+topPad+PH+8;
  offX=h*TW+2;
  offY=topPad+4;
  // Sunset gradient backdrop
  var grad=ctx.createLinearGradient(0,0,0,c.height);
  grad.addColorStop(0,'#7e57c2');
  grad.addColorStop(0.55,'#f06292');
  grad.addColorStop(1,'#ffb74d');
  ctx.fillStyle=grad;ctx.fillRect(0,0,c.width,c.height);
  // Flood-fill reachable cells from player
  var reach={};
  var st=[[px,py]];
  while(st.length){
    var p=st.pop(),qx=p[0],qy=p[1],k=qx+','+qy;
    if(reach[k])continue;
    if(qy<0||qy>=h)continue;
    var row=map[qy]||[];
    if(qx<0||qx>=row.length)continue;
    if(row[qx]=='#')continue;
    reach[k]=true;
    st.push([qx+1,qy]);st.push([qx-1,qy]);st.push([qx,qy+1]);st.push([qx,qy-1]);
  }
  // Mesa = reachable cells, plus '#' cells whose orthogonal neighbours are all on-grid
  // and are either walls or reachable (i.e. interior walls). Perimeter '#' cells are excluded.
  function isMesa(x,y){
    if(y<0||y>=h)return false;
    var row=map[y];if(!row||x<0||x>=row.length)return false;
    if(reach[x+','+y])return true;
    if(row[x]!='#')return false;
    var dirs=[[1,0],[-1,0],[0,1],[0,-1]];
    for(var d=0;d<4;d++){
      var nx=x+dirs[d][0],ny=y+dirs[d][1];
      if(ny<0||ny>=h)return false;
      var nr=map[ny];if(!nr||nx<0||nx>=nr.length)return false;
      var v=nr[nx];
      if(v!='#'&&!reach[nx+','+ny])return false;
    }
    return true;
  }
  // Render row-major: cliffs first (extend down), then top diamond, then objects
  for(var y=0;y<h;y++){
    var row=map[y];
    for(var x=0;x<row.length;x++){
      if(!isMesa(x,y))continue;
      if(!isMesa(x+1,y))cliffR(x,y);
      if(!isMesa(x,y+1))cliffL(x,y);
      var even=((x+y)%2)===0;
      diamond(ix(x,y),iy(x,y),even?'#e1f5fe':'#b3e5fc','#4fc3f7');
      var v=row[x];
      if(v=='#'){
        cube(x,y,WH,'#ffe082','#ffb74d','#fb8c00','#5d4037');
      } else {
        if(v=='.')drawGoal(x,y);
        if(v=='$'||v=='*'){
          if(v=='*'){cube(x,y,BH,'#ffd700','#ffc107','#ff8f00','#4e342e');drawSparkle(x,y);}
          else      {cube(x,y,BH,'#a5d6a7','#66bb6a','#388e3c','#1b5e20');}
        }
      }
      if(x===px&&y===py)drawPawn(x,y,'#26c6da','#006064');
    }
  }
  document.getElementById('st').innerText="Level "+(cur+1)+" — Moves: "+mv;
}
function mvK(dx,dy){
    var nx=px+dx,ny=py+dy,t=map[ny]&&map[ny][nx];
    if(t==' '||t=='.'){px=nx;py=ny;mv++;}
    else if(t=='$'||t=='*'){
      var bx=nx+dx,by=ny+dy,bt=map[by]&&map[by][bx];
      if(bt==' '||bt=='.'){
        map[ny][nx]=(t=='$')?' ':'.';
        map[by][bx]=(bt==' ')?'$':'*';
        px=nx;py=ny;mv++;
      }
    }
    draw();
    if(!map.some(r=>r.includes('.'))){
      setTimeout(()=>{if(cur<L.length-1){cur++;load(cur);}else{alert('All levels cleared!');cur=0;load(0);}},100);
    }
}
window.onkeydown=e=>{
  var dx=0,dy=0;
  if(e.key=='ArrowUp')dy=-1;if(e.key=='ArrowDown')dy=1;
  if(e.key=='ArrowLeft')dx=-1;if(e.key=='ArrowRight')dx=1;
  if(dx||dy)mvK(dx,dy);
};
function reset(){load(cur);}

// ── People-tracker controls ─────────────────────────────────────────────────
// 3×3 floor grid in front of the sensor (world coords from /lidar/scan).
// Cells:  (mid,top)=↑  (left,mid)=←  (mid,mid)=idle  (right,mid)=→  (mid,bot)=↓
// Move fires once on cell-entry; same-cell re-entry needs leaving first.
// All sizes in metres. Shrink GRID_HX / extend GRID_Y0..1 to taste.
var GRID_HX  = 0.75;          // half-width of grid (x ∈ [-HX, +HX])
var GRID_Y0  = 1.0;           // near edge of grid (y minimum)
var GRID_Y1  = 2.5;           // far edge of grid (y maximum)
var CELL_HX  = 0.25;          // half-width of the centre column (idle / up / down lane)
var CELL_Y_NEAR = 1.5;        // boundary between top (near, ↑) and middle rows
var CELL_Y_FAR  = 2.0;        // boundary between middle and bottom (far, ↓) rows
var trackerOn = true;
var lastCell = 'none';
function cellFor(p){
  if(!p) return 'none';
  var x = p.x, y = p.y;
  if (y < GRID_Y0 || y > GRID_Y1 || x < -GRID_HX || x > GRID_HX) return 'outside';
  var col = x < -CELL_HX ? 0 : (x > CELL_HX ? 2 : 1);
  // row 0 = near sensor (top of grid display, "up"), 2 = far from sensor (bottom, "down").
  var row = y < CELL_Y_NEAR ? 0 : (y > CELL_Y_FAR ? 2 : 1);
  if (col === 1 && row === 0) return 'up';
  if (col === 0 && row === 1) return 'left';
  if (col === 1 && row === 1) return 'idle';
  if (col === 2 && row === 1) return 'right';
  if (col === 1 && row === 2) return 'down';
  return 'corner';
}
function updateGrid(cell, p){
  var cells = document.querySelectorAll('#trkGrid .tc');
  for (var i = 0; i < cells.length; i++) cells[i].classList.remove('active');
  var el = document.querySelector('#trkGrid .tc[data-cell="'+cell+'"]');
  if (el) el.classList.add('active');
  var s = cell;
  if (p) s += ' (x=' + (p.x>=0?'+':'') + p.x.toFixed(2) + ' y=' + p.y.toFixed(2) + ')';
  document.getElementById('trkStatus').innerText = s;
  // Absolute-position dot inside the mini-grid (top-down floor view).
  // Grid spans world x ∈ [-GRID_HX, +GRID_HX] m, y ∈ [GRID_Y0, GRID_Y1] m.
  // Grid is 222×222 px (3 cells × 70 px + 2 gaps × 6 px); y=GRID_Y0 maps to the top.
  var dot = document.getElementById('trkDot');
  if (!p) { dot.style.display = 'none'; return; }
  var GW = 222, GH = 222;
  var lx = (p.x + GRID_HX) / (2 * GRID_HX) * GW;
  var ly = (p.y - GRID_Y0) / (GRID_Y1 - GRID_Y0) * GH;
  if (lx < 0) lx = 0; else if (lx > GW) lx = GW;
  if (ly < 0) ly = 0; else if (ly > GH) ly = GH;
  dot.style.left = lx + 'px';
  dot.style.top  = ly + 'px';
  dot.style.display = 'block';
}
function trkToggle(){
  trackerOn = !trackerOn;
  document.getElementById('trkBtn').innerText = 'Tracker: ' + (trackerOn ? 'ON' : 'OFF');
  document.body.classList.toggle('trk-on', trackerOn);
  if (!trackerOn) { lastCell = 'none'; updateGrid('off', null); }
}
async function trkPoll(){
  if (trackerOn) {
    try{
      var r = await fetch('/lidar/scan', {cache:'no-store'});
      var d = await r.json();
      var p = (d && d.persons && d.persons.length) ? d.persons[0] : null;
      var cell = cellFor(p);
      updateGrid(cell, p);
      if (cell !== lastCell) {
        if (cell === 'up')         mvK(0, -1);
        else if (cell === 'down')  mvK(0,  1);
        else if (cell === 'left')  mvK(-1, 0);
        else if (cell === 'right') mvK( 1, 0);
        lastCell = cell;
      }
    } catch (e) {}
  }
  setTimeout(trkPoll, 150);
}
trkPoll();

setInterval(()=>fetch('/ping'),1000);
load(0);
</script></body></html>)rawhtml";
