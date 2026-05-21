#pragma once

// AP-mode WiFi setup page. Served at http://192.168.4.1/ when the ESP32 has
// no stored credentials (or fails to connect) and falls back to the
// "battle-setup" SoftAP. Scans networks, posts to /wifi/connect, reboots.

static const char SETUP_PAGE_HTML[] = R"rawhtml(<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Setup</title><style>
body{font-family:sans-serif;background:#050510;color:#fff;padding:20px;text-align:center}
input{width:100%;max-width:300px;padding:10px;margin:10px 0;border-radius:5px;border:none}
button{background:#7c6fff;color:#fff;padding:10px 20px;border:none;border-radius:5px;cursor:pointer}
.net{cursor:pointer;padding:8px;border-bottom:1px solid #222}
</style></head><body><h1>WiFi Setup</h1>
<div id='f'>
<input id='s' placeholder='SSID'><br><input id='p' type='password' placeholder='Password'><br>
<button onclick='c()'>Connect</button></div>
<div id='l' style='margin-top:20px'>Scanning for networks...</div>
<script>
function c(){
  fetch('/wifi/connect?ssid='+encodeURIComponent(document.getElementById('s').value)+'&pass='+encodeURIComponent(document.getElementById('p').value))
  .then(()=>alert('Rebooting...'));
}
fetch('/wifi/scan').then(r=>r.json()).then(d=>{
  if(!d.length){document.getElementById('l').innerHTML='No networks found.';return;}
  let h='<h3>Available Networks:</h3>';
  d.forEach(n=>h+='<div class="net" onclick="document.getElementById(\'s\').value=\''+n.ssid+'\'">'+n.ssid+' ('+n.rssi+'dBm)</div>');
  document.getElementById('l').innerHTML=h;
}).catch(e=>{
  document.getElementById('l').innerHTML='Scan failed. Please enter details manually.';
});
</script></body></html>)rawhtml";
