/* ==========================================================================
 * ĐỒ ÁN CHUYÊN NGÀNH CƠ ĐIỆN TỬ (ME4071) - HCMUT
 * Đề tài: Robot dò line phát hiện và tránh vật cản (Topic 3)
 * Phiên bản: Arduino IDE (ESP32) - Tích hợp đầy đủ:
 *   - 8 mắt IR TCRT5000 qua MUX HEF4051B
 *   - Cảm biến khoảng cách ToF VL53L0X (I2C)
 *   - Servo lái MG90S (LEDC 50Hz 14-bit)
 *   - Động cơ JGB37 + Driver cầu H TB67H450/DRV8871 (LEDC 20kHz 8-bit)
 *   - Encoder cầu phương JGB37 (Interrupt)
 *   - Bộ điều khiển bám quỹ đạo phi tuyến Lyapunov + lọc Adaptive EMA
 *   - Bộ điều khiển vận tốc PI (hoặc Manual PWM)
 *   - FreeRTOS Đa nhiệm 2 nhân (Core 0 & Core 1)
 *   - Wi-Fi SoftAP + Web Dashboard + WebSocket Telemetry + Cập nhật OTA
 *
 * HƯỚNG DẪN BIÊN DỊCH TRÊN ARDUINO IDE:
 *   1. Mở file ROBOT_DACN_Arduino.ino bằng Arduino IDE
 *   2. Chọn Board: "ESP32 Dev Module" (không cần cài thêm thư viện ngoài)
 *   3. Chọn Flash Size: "16MB (128Mb)" (hoặc 4MB tùy mạch)
 *   4. Chọn Partition Scheme: "Minimal SPIFFS (1.9MB APP with OTA)" hoặc "Default 4MB with spiffs"
 *   5. Bấm Upload
 * ========================================================================== */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <Wire.h>
#include <driver/ledc.h>
#include <math.h>
#include <string.h>

/* ==========================================================================
 * 0. GIAO DIỆN WEB DASHBOARD NHÚNG TRỰC TIẾP TRONG FLASH (PROGMEM)
 * ========================================================================== */
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Robot Dashboard - HCMUT ME4071</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:#0f172a;color:#e2e8f0;overflow-x:hidden}
.header{background:linear-gradient(135deg,#1e3a5f,#2563eb);padding:12px 20px;display:flex;justify-content:space-between;align-items:center;box-shadow:0 2px 10px rgba(0,0,0,.3)}
.header h1{font-size:18px;font-weight:600}
.header .status{display:flex;align-items:center;gap:8px;font-size:13px}
.header .dot{width:10px;height:10px;border-radius:50%;background:#ef4444;animation:pulse 1.5s infinite}
.header .dot.connected{background:#22c55e;animation:none}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;padding:12px;max-width:1400px;margin:0 auto}
@media(max-width:900px){.grid{grid-template-columns:1fr}}
.card{background:#1e293b;border-radius:12px;padding:16px;border:1px solid #334155}
.card h2{font-size:14px;color:#94a3b8;margin-bottom:12px;text-transform:uppercase;letter-spacing:1px}
.metric{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:8px}
.metric .label{color:#94a3b8;font-size:13px}
.metric .value{font-size:24px;font-weight:700;font-variant-numeric:tabular-nums}
.metric .unit{font-size:12px;color:#64748b;margin-left:4px}
.metric .value.speed{color:#3b82f6}
.metric .value.rpm{color:#f59e0b}
.metric .value.steer{color:#10b981}
.metric .value.error{color:#ef4444}
.metric .value.pwm{color:#8b5cf6}
.sensor-bar-container{display:flex;gap:6px;align-items:flex-end;height:120px;justify-content:center;margin-top:8px}
.sensor-bar{width:36px;background:#334155;border-radius:4px 4px 0 0;position:relative;transition:height .15s ease}
.sensor-bar .fill{position:absolute;bottom:0;width:100%;border-radius:4px 4px 0 0;transition:height .15s ease,background .15s ease}
.sensor-bar .lbl{text-align:center;font-size:11px;color:#94a3b8;position:absolute;bottom:-18px;width:100%}
.sensor-bar .val{text-align:center;font-size:10px;color:#cbd5e1;position:absolute;top:-16px;width:100%}
.state-badge{display:inline-block;padding:4px 12px;border-radius:20px;font-size:12px;font-weight:600}
.state-0{background:#334155;color:#94a3b8}
.state-1{background:#1e3a5f;color:#60a5fa}
.state-2{background:#7c2d12;color:#fb923c}
.state-3{background:#14532d;color:#4ade80}
.state-4{background:#4c1d95;color:#c084fc}
.state-5{background:#166534;color:#86efac}
.state-6{background:#7f1d1d;color:#fca5a5}
#map-canvas{width:100%;height:350px;background:#0f172a;border-radius:8px;border:1px solid #334155}
.ota-section{margin-top:12px}
.ota-section input[type=file]{display:none}
.btn{padding:8px 20px;border:none;border-radius:8px;font-size:13px;font-weight:600;cursor:pointer;transition:all .15s}
.btn-primary{background:#2563eb;color:#fff}.btn-primary:hover{background:#1d4ed8}
.btn-danger{background:#dc2626;color:#fff}.btn-danger:hover{background:#b91c1c}
.btn-success{background:#16a34a;color:#fff}.btn-success:hover{background:#15803d}
.btn:disabled{opacity:.5;cursor:not-allowed}
.progress-bar{width:100%;height:6px;background:#334155;border-radius:3px;margin-top:8px;overflow:hidden}
.progress-bar .fill{height:100%;background:#2563eb;border-radius:3px;transition:width .3s}
.controls{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}
.e2-indicator{position:relative;height:30px;background:#334155;border-radius:4px;margin-top:8px;overflow:hidden}
.e2-indicator .center{position:absolute;left:50%;top:0;bottom:0;width:2px;background:#475569}
.e2-indicator .marker{position:absolute;top:2px;bottom:2px;width:12px;border-radius:3px;background:#ef4444;transition:left .15s ease;transform:translateX(-50%)}
.footer{text-align:center;padding:12px;color:#475569;font-size:11px}
</style>
</head>
<body>
<div class="header">
  <h1>🤖 Robot Dashboard — HCMUT ME4071</h1>
  <div class="status">
    <div class="dot" id="ws-dot"></div>
    <span id="ws-status">Đang kết nối...</span>
  </div>
</div>

<div class="grid">
  <div class="card">
    <h2>📊 Trạng Thái & Tốc Độ</h2>
    <div style="margin-bottom:8px">Trạng thái: <span class="state-badge state-0" id="state-badge">INIT</span></div>
    <div class="metric"><span class="label">Tốc độ xe</span><span><span class="value speed" id="v-speed">0.00</span><span class="unit">m/s</span></span></div>
    <div class="metric"><span class="label">RPM Động cơ</span><span><span class="value rpm" id="v-rpm">0</span><span class="unit">rpm</span></span></div>
    <div class="metric"><span class="label">Góc lái / Servo</span><span><span class="value steer" id="v-steer">0.0</span><span class="unit">° / </span><span class="value steer" id="v-servo" style="color:#38bdf8">90.0</span><span class="unit">°</span></span></div>
    <div class="metric"><span class="label">Lệch tâm e₂</span><span><span class="value error" id="v-e2">0.0</span><span class="unit">mm</span></span></div>
    <div class="metric"><span class="label">PWM Xuất</span><span><span class="value pwm" id="v-pwm">0</span><span class="unit">/255</span></span></div>
    <div class="e2-indicator"><div class="center"></div><div class="marker" id="e2-marker"></div></div>
  </div>

  <div class="card">
    <h2>📡 Mảng Cảm Biến Dò Line (8 Mắt IR - Bước 8mm)</h2>
    <div class="sensor-bar-container" id="sensor-bars">
      <div class="sensor-bar" style="height:120px"><div class="fill" id="sb0"></div><div class="val" id="sv0">0</div><div class="lbl">S1</div></div>
      <div class="sensor-bar" style="height:120px"><div class="fill" id="sb1"></div><div class="val" id="sv1">0</div><div class="lbl">S2</div></div>
      <div class="sensor-bar" style="height:120px"><div class="fill" id="sb2"></div><div class="val" id="sv2">0</div><div class="lbl">S3</div></div>
      <div class="sensor-bar" style="height:120px"><div class="fill" id="sb3"></div><div class="val" id="sv3">0</div><div class="lbl">S4</div></div>
      <div class="sensor-bar" style="height:120px"><div class="fill" id="sb4"></div><div class="val" id="sv4">0</div><div class="lbl">S5</div></div>
      <div class="sensor-bar" style="height:120px"><div class="fill" id="sb5"></div><div class="val" id="sv5">0</div><div class="lbl">S6</div></div>
      <div class="sensor-bar" style="height:120px"><div class="fill" id="sb6"></div><div class="val" id="sv6">0</div><div class="lbl">S7</div></div>
      <div class="sensor-bar" style="height:120px"><div class="fill" id="sb7"></div><div class="val" id="sv7">0</div><div class="lbl">S8</div></div>
    </div>
    <div style="display:flex;justify-content:space-between;margin-top:24px;font-size:12px;color:#64748b">
      <span>← Trái (-28mm)</span><span>Tâm (0mm)</span><span>Phải (+28mm) →</span>
    </div>
  </div>

  <div class="card" style="grid-column:1/-1">
    <h2>🗺️ Bản Đồ Quỹ Đạo Robot (Dead Reckoning)</h2>
    <canvas id="map-canvas"></canvas>
    <div class="controls">
      <button class="btn btn-primary" onclick="clearMap()">🗑️ Xóa Bản Đồ</button>
      <button class="btn btn-success" onclick="toggleRecord();" id="btn-record">⏺ Bắt Đầu Ghi</button>
    </div>
  </div>

  <div class="card">
    <h2>⬆️ Cập Nhật Firmware (OTA)</h2>
    <p style="font-size:13px;color:#94a3b8;margin-bottom:12px">Chọn file .bin firmware mới để nạp không dây qua WiFi.</p>
    <input type="file" id="ota-file" accept=".bin">
    <div class="controls">
      <button class="btn btn-primary" onclick="document.getElementById('ota-file').click()">📂 Chọn File .bin</button>
      <button class="btn btn-danger" onclick="startOTA()" id="btn-ota" disabled>🚀 Nạp Firmware</button>
    </div>
    <div id="ota-info" style="margin-top:8px;font-size:12px;color:#94a3b8"></div>
    <div class="progress-bar" id="ota-progress" style="display:none"><div class="fill" id="ota-fill"></div></div>
  </div>

  <div class="card">
    <h2>🎮 Điều Khiển Động Cơ & Chế Độ</h2>
    <div style="display:flex;gap:8px;margin-bottom:12px">
      <button class="btn btn-primary" id="btn-mode-manual" onclick="setMode('manual')" style="flex:1">⚙️ PWM Cố Định</button>
      <button class="btn" id="btn-mode-auto" onclick="setMode('auto')" style="flex:1;background:#334155;color:#94a3b8">🤖 Tự Động PI</button>
    </div>
    <div id="panel-manual-pwm" style="background:#0f172a;padding:10px;border-radius:8px;margin-bottom:10px">
      <div class="metric"><span class="label">Mức PWM cố định</span><span><span class="value pwm" id="v-pwm-target">45</span><span class="unit">/255</span></span></div>
      <input type="range" min="0" max="255" value="45" id="pwm-slider" style="width:100%;accent-color:#8b5cf6" oninput="setPWM(this.value)">
      <div style="display:flex;justify-content:space-between;font-size:11px;color:#64748b;margin-top:4px">
        <span>0 (Dừng)</span><span>45 (Êm)</span><span>100 (Nhanh)</span><span>255 (Max)</span>
      </div>
    </div>
    <div id="panel-auto-speed" style="display:none;background:#0f172a;padding:10px;border-radius:8px;margin-bottom:10px">
      <div class="metric"><span class="label">Tốc độ PI mục tiêu</span><span><span class="value speed" id="v-target">0.40</span><span class="unit">m/s</span></span></div>
      <input type="range" min="10" max="200" value="40" id="speed-slider" style="width:100%;accent-color:#2563eb" oninput="setSpeed(this.value)">
    </div>
    <div class="controls">
      <button class="btn btn-success" onclick="sendCmd('start')">▶ START</button>
      <button class="btn btn-danger" onclick="sendCmd('stop')">⏹ STOP</button>
      <button class="btn btn-primary" onclick="sendCmd('reset')">🔄 RESET</button>
    </div>
  </div>
</div>

<div class="footer">Đồ Án Chuyên Ngành Cơ Điện Tử ME4071 — HCMUT © 2026</div>

<script>
const STATE_NAMES = ['INIT','WAIT_START','DETECT_OBSTACLE','LINE_TRACKING','INTERSECTION','FINISH','STOPPED'];
let ws = null, recording = false, mapPoints = [], robotX = 0, robotY = 0, robotTheta = Math.PI/2;
const canvas = document.getElementById('map-canvas');
const ctx = canvas.getContext('2d');

function resizeCanvas() {
  canvas.width = canvas.clientWidth * (window.devicePixelRatio || 1);
  canvas.height = canvas.clientHeight * (window.devicePixelRatio || 1);
  ctx.scale(window.devicePixelRatio || 1, window.devicePixelRatio || 1);
  drawMap();
}
window.addEventListener('resize', resizeCanvas);

function connect() {
  const host = location.hostname || '192.168.4.1';
  ws = new WebSocket('ws://' + host + ':81/ws');
  ws.onopen = () => {
    document.getElementById('ws-dot').classList.add('connected');
    document.getElementById('ws-status').textContent = 'Đã kết nối';
    ws.send(JSON.stringify({cmd: 'ping'}));
  };
  ws.onclose = () => {
    document.getElementById('ws-dot').classList.remove('connected');
    document.getElementById('ws-status').textContent = 'Mất kết nối...';
    setTimeout(connect, 2000);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    try { const d = JSON.parse(ev.data); updateUI(d); } catch(e) {}
  };
}

function updateUI(d) {
  document.getElementById('v-speed').textContent = (d.v || 0).toFixed(2);
  document.getElementById('v-rpm').textContent = Math.round(d.rpm || 0);
  document.getElementById('v-steer').textContent = (d.steer || 0).toFixed(1);
  const servoDeg = d.servo !== undefined ? d.servo : (90.0 + (d.steer || 0));
  const elServo = document.getElementById('v-servo');
  if (elServo) elServo.textContent = servoDeg.toFixed(1);
  document.getElementById('v-e2').textContent = (d.e2 || 0).toFixed(1);
  document.getElementById('v-pwm').textContent = Math.round(d.pwm || 0);

  const st = d.state || 0;
  const badge = document.getElementById('state-badge');
  badge.textContent = STATE_NAMES[st] || 'UNKNOWN';
  badge.className = 'state-badge state-' + st;

  if (d.mode && !window.modeInitialized) {
    window.modeInitialized = true;
    setMode(d.mode);
    if (d.mpwm !== undefined) {
      document.getElementById('pwm-slider').value = Math.round(d.mpwm);
      document.getElementById('v-pwm-target').textContent = Math.round(d.mpwm);
    }
  }

  const e2Pct = Math.max(0, Math.min(100, ((d.e2 || 0) + 28) / 56 * 100));
  document.getElementById('e2-marker').style.left = e2Pct + '%';

  const adc = d.adc || [0,0,0,0,0,0,0,0];
  for (let i = 0; i < 8; i++) {
    const pct = Math.min(100, adc[i] / 4095 * 100);
    const bar = document.getElementById('sb' + i);
    bar.style.height = Math.max(2, pct) + '%';
    const g = pct > 40 ? `hsl(${Math.max(0, 120 - pct * 1.2)}, 80%, 50%)` : '#3b82f6';
    bar.style.background = g;
    document.getElementById('sv' + i).textContent = adc[i];
  }

  if (recording && d.v !== undefined && d.steer !== undefined) {
    const dt = 0.05;
    const v = d.v || 0;
    const gamma = (d.steer || 0) * Math.PI / 180;
    const L = 0.225;
    if (Math.abs(gamma) > 0.001) {
      const R = L / Math.tan(gamma);
      const dtheta = (v * dt) / R;
      robotTheta += dtheta;
    }
    robotX += v * dt * Math.cos(robotTheta);
    robotY += v * dt * Math.sin(robotTheta);
    mapPoints.push({x: robotX, y: robotY, lost: d.lost || false, inter: d.inter || false});
    drawMap();
  }
}

function drawMap() {
  const W = canvas.clientWidth, H = canvas.clientHeight;
  ctx.clearRect(0, 0, W, H);

  if (mapPoints.length < 2) {
    ctx.fillStyle = '#475569';
    ctx.font = '14px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('Nhấn "Bắt Đầu Ghi" rồi cho xe chạy để vẽ quỹ đạo', W/2, H/2);
    return;
  }

  let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity;
  mapPoints.forEach(p=>{if(p.x<minX)minX=p.x;if(p.x>maxX)maxX=p.x;if(p.y<minY)minY=p.y;if(p.y>maxY)maxY=p.y});
  const rangeX=Math.max(maxX-minX,0.1),rangeY=Math.max(maxY-minY,0.1);
  const margin=40;
  const scaleX=(W-2*margin)/rangeX,scaleY=(H-2*margin)/rangeY;
  const scale=Math.min(scaleX,scaleY);
  const cx=W/2,cy=H/2;
  const midX=(minX+maxX)/2,midY=(minY+maxY)/2;
  const tx=p=>cx+(p.x-midX)*scale;
  const ty=p=>cy-(p.y-midY)*scale;

  ctx.strokeStyle='#1e293b';ctx.lineWidth=1;
  for(let gx=Math.floor(minX*10)/10;gx<=maxX;gx+=0.1){const sx=tx({x:gx});ctx.beginPath();ctx.moveTo(sx,0);ctx.lineTo(sx,H);ctx.stroke()}
  for(let gy=Math.floor(minY*10)/10;gy<=maxY;gy+=0.1){const sy=ty({y:gy});ctx.beginPath();ctx.moveTo(0,sy);ctx.lineTo(W,sy);ctx.stroke()}

  ctx.lineWidth=2.5;ctx.lineJoin='round';ctx.lineCap='round';
  for(let i=1;i<mapPoints.length;i++){
    ctx.beginPath();ctx.moveTo(tx(mapPoints[i-1]),ty(mapPoints[i-1]));ctx.lineTo(tx(mapPoints[i]),ty(mapPoints[i]));
    ctx.strokeStyle=mapPoints[i].inter?'#c084fc':mapPoints[i].lost?'#ef4444':'#22c55e';
    ctx.stroke();
  }

  ctx.beginPath();ctx.arc(tx(mapPoints[0]),ty(mapPoints[0]),6,0,Math.PI*2);ctx.fillStyle='#3b82f6';ctx.fill();
  ctx.fillStyle='#fff';ctx.font='bold 10px sans-serif';ctx.textAlign='center';ctx.fillText('S',tx(mapPoints[0]),ty(mapPoints[0])+3);

  const last=mapPoints[mapPoints.length-1];
  ctx.beginPath();ctx.arc(tx(last),ty(last),5,0,Math.PI*2);ctx.fillStyle='#f59e0b';ctx.fill();

  const arrowLen=15;
  ctx.beginPath();
  ctx.moveTo(tx(last),ty(last));
  ctx.lineTo(tx(last)+arrowLen*Math.cos(-robotTheta+Math.PI/2),ty(last)+arrowLen*Math.sin(-robotTheta+Math.PI/2));
  ctx.strokeStyle='#f59e0b';ctx.lineWidth=2;ctx.stroke();

  ctx.fillStyle='#64748b';ctx.font='11px sans-serif';ctx.textAlign='left';
  ctx.fillText(`Scale: 1 div = 100mm`, 8, H-8);

  ctx.textAlign='right';
  ctx.fillStyle='#22c55e';ctx.fillRect(W-130,8,10,10);ctx.fillStyle='#94a3b8';ctx.fillText('Bám line',W-8,17);
  ctx.fillStyle='#c084fc';ctx.fillRect(W-130,24,10,10);ctx.fillStyle='#94a3b8';ctx.fillText('Ngã rẽ',W-8,33);
  ctx.fillStyle='#ef4444';ctx.fillRect(W-130,40,10,10);ctx.fillStyle='#94a3b8';ctx.fillText('Mất line',W-8,49);
}

function clearMap(){mapPoints=[];robotX=0;robotY=0;robotTheta=Math.PI/2;drawMap()}
function toggleRecord(){
  recording=!recording;
  const btn=document.getElementById('btn-record');
  btn.textContent=recording?'⏹ Dừng Ghi':'⏺ Bắt Đầu Ghi';
  btn.className=recording?'btn btn-danger':'btn btn-success';
  if(recording){clearMap()}
}

function setPWM(val){
  document.getElementById('v-pwm-target').textContent = val;
  if(ws&&ws.readyState===1)ws.send(JSON.stringify({cmd:'pwm',val:Number(val)}));
}

function setMode(mode){
  const isManual = (mode === 'manual');
  document.getElementById('panel-manual-pwm').style.display = isManual ? 'block' : 'none';
  document.getElementById('panel-auto-speed').style.display = isManual ? 'none' : 'block';
  document.getElementById('btn-mode-manual').className = isManual ? 'btn btn-primary' : 'btn';
  document.getElementById('btn-mode-manual').style.background = isManual ? '#2563eb' : '#334155';
  document.getElementById('btn-mode-manual').style.color = isManual ? '#fff' : '#94a3b8';
  document.getElementById('btn-mode-auto').className = !isManual ? 'btn btn-primary' : 'btn';
  document.getElementById('btn-mode-auto').style.background = !isManual ? '#2563eb' : '#334155';
  document.getElementById('btn-mode-auto').style.color = !isManual ? '#fff' : '#94a3b8';
  if(ws&&ws.readyState===1)ws.send(JSON.stringify({cmd:'mode',val:mode}));
}

function setSpeed(val){
  document.getElementById('v-target').textContent=(val/100).toFixed(2);
  if(ws&&ws.readyState===1)ws.send(JSON.stringify({cmd:'speed',val:val/100}));
}
function sendCmd(cmd){if(ws&&ws.readyState===1)ws.send(JSON.stringify({cmd:cmd}))}

document.getElementById('ota-file').addEventListener('change',function(e){
  const f=e.target.files[0];
  if(f){
    document.getElementById('ota-info').textContent='File: '+f.name+' ('+Math.round(f.size/1024)+' KB)';
    document.getElementById('btn-ota').disabled=false;
  }
});

function startOTA(){
  const f=document.getElementById('ota-file').files[0];
  if(!f)return;
  const btn=document.getElementById('btn-ota');
  btn.disabled=true;btn.textContent='Đang nạp...';
  document.getElementById('ota-progress').style.display='block';

  const xhr=new XMLHttpRequest();
  xhr.open('POST','/ota',true);
  xhr.upload.onprogress=(e)=>{
    if(e.lengthComputable){
      const pct=Math.round(e.loaded/e.total*100);
      document.getElementById('ota-fill').style.width=pct+'%';
      document.getElementById('ota-info').textContent='Đang nạp: '+pct+'%';
    }
  };
  xhr.onload=()=>{
    if(xhr.status===200){
      document.getElementById('ota-info').textContent='✅ Nạp thành công! ESP32 đang khởi động lại...';
      setTimeout(()=>location.reload(),5000);
    } else {
      document.getElementById('ota-info').textContent='❌ Lỗi: '+xhr.responseText;
      btn.disabled=false;btn.textContent='🚀 Nạp Firmware';
    }
  };
  xhr.onerror=()=>{
    document.getElementById('ota-info').textContent='❌ Lỗi kết nối!';
    btn.disabled=false;btn.textContent='🚀 Nạp Firmware';
  };
  const formData = new FormData();
  formData.append('update', f);
  xhr.send(formData);
}

resizeCanvas();
connect();
</script>
</body>
</html>
)rawliteral";

/* ==========================================================================
 * 1. CẤU HÌNH CHÂN CẮM (PIN_CONFIG)
 * ========================================================================== */
/* ================= ENCODER ================= */
#define JGB37_ENC_A_GPIO 35 // Đã đảo chân để bánh quay tới ra RPM dương
#define JGB37_ENC_B_GPIO 34

/* ================= TB67H450FNG / DRV8871 ================= */
#define TB67_IN1_GPIO 33 // PWM1
#define TB67_IN2_GPIO 32 // PWM2

/* ================= MG90S SERVO ================= */
#define MG90S_PWM_GPIO 25

/* ================= TCRT5000 ARRAY (MUX HEF4051B) ================= */
#define TCRT_SEL1_GPIO 19
#define TCRT_SEL2_GPIO 18
#define TCRT_SEL3_GPIO 17
#define TCRT_ADC_GPIO  36

/* ================= VL53L0X ToF ================= */
#define VL53L0X_SDA_GPIO 16
#define VL53L0X_SCL_GPIO 4

/* ================= STATUS LED & BUTTON ================= */
#define LED_RED_GPIO   27
#define LED_GREEN_GPIO 26
#define BOOT_GPIO      0

/* ==========================================================================
 * 2. THÔNG SỐ CƠ KHÍ & ĐIỀU KHIỂN (ROBOT_CONFIG)
 * ========================================================================== */
#define ROBOT_MASS_KG        1.5f    // Khối lượng xe M (kg)
#define ROBOT_WIDTH_M        0.15f   // Vệt bánh sau w (m) = 150 mm
#define ROBOT_LENGTH_M       0.225f  // Chiều dài cơ sở l (m) = 225 mm
#define ROBOT_L1_M           0.05f   // Khoảng cách từ trục sau đến trục lái / CG l1 (m) = 50 mm
#define ROBOT_L2_M           0.175f  // Khoảng cách từ trọng tâm G / trục sau đến cảm biến l2 (m)
#define ROBOT_CG_HEIGHT_M    0.05f   // Chiều cao trọng tâm h (m) = 50 mm

#define WHEEL_DIAMETER_M     0.08f   // Đường kính bánh dẫn động sau D (m) = 80 mm
#define WHEEL_RADIUS_M       0.04f   // Bán kính bánh xe Rw (m) = 40 mm
#define DIFF_GEAR_RATIO      2.5f    // Tỉ số truyền cụm vi sai u_dg = 2.5

#define LINE_SENSOR_COUNT    8       // 8 cảm biến IR qua HEF4051B (kênh Y0 - Y7)
#define LINE_SENSOR_PITCH_MM 8.0f    // Bước cảm biến p = 8 mm
#define LINE_WIDTH_MM        26.0f   // Bề rộng vạch line đen = 26 mm
#define LINE_SENSOR_SPAN_MM  56.0f   // Chiều dài phủ mảng cảm biến (8-1)*8 = 56 mm
#define LINE_SENSOR_HEIGHT_MM 7.0f   // Chiều cao tối ưu h = 7 mm

static const float LINE_SENSOR_POS_MM[LINE_SENSOR_COUNT] = {
    -28.0f, -20.0f, -12.0f, -4.0f, 4.0f, 12.0f, 20.0f, 28.0f
};

#define OBSTACLE_NEAR_DIST_THEORY_MM 475.0f   // Khoảng cách lý thuyết phản xạ nhánh gần
#define OBSTACLE_NEAR_DETECT_MAX_MM  700.0f   // Ngưỡng phân biệt nhánh gần vs nhánh xa
#define OBSTACLE_CALIB_OFFSET_MM     (-41.4f) // Bù sai số tĩnh

#define LYAPUNOV_K1          1.5f
#define LYAPUNOV_K2          0.00018f
#define LYAPUNOV_K3          2.5f

#define MOTOR_CONTROL_PERIOD_MS 10   // Chu kỳ điều khiển Ts = 10 ms (100 Hz)
#define MOTOR_DEADBAND_PWM      15   // Vùng chết PWM (0 - 15)
#define MOTOR_PI_KP             0.3975f
#define MOTOR_PI_KI             2.3751f

#define ROBOT_SPEED_MIN_MPS     0.1f
#define ROBOT_SPEED_MAX_MPS     2.0f
#define ROBOT_MAX_ACCEL_MPS2    2.5f
#define ROBOT_MAX_STEER_ANGLE_RAD 1.1345f // ~ 65 độ

#define WIFI_AP_SSID         "ROBOT_DACN"
#define WIFI_AP_PASS         "12345678"
#define WIFI_AP_CHANNEL      6
#define WIFI_AP_MAX_CONN     4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef RAD_TO_DEG
#undef RAD_TO_DEG
#endif
#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)

/* ==========================================================================
 * 3. KIỂU DỮ LIỆU DÙNG CHUNG (TYPES & STRUCTS)
 * ========================================================================== */
typedef enum {
    ROUTE_UNKNOWN = 0,
    ROUTE_UPPER,    // Case 1: Nhánh gần có vật cản -> Rẽ nhánh trên
    ROUTE_LOWER     // Case 2: Nhánh gần thông thoáng -> Rẽ nhánh dưới
} route_selection_t;

typedef enum {
    ROBOT_STATE_INIT = 0,
    ROBOT_STATE_WAIT_START,
    ROBOT_STATE_DETECT_OBSTACLE,
    ROBOT_STATE_LINE_TRACKING,
    ROBOT_STATE_INTERSECTION,
    ROBOT_STATE_FINISH,
    ROBOT_STATE_STOPPED
} robot_state_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float ts;
    float out_min;
    float out_max;
    float deadband;
    float integral;
    float prev_error;
} pid_controller_t;

typedef struct {
    pid_controller_t pi;
    float target_velocity_mps;
    float target_rpm;
    float actual_rpm;
    float actual_velocity_mps;
    float output_pwm;
} speed_controller_t;

typedef struct {
    uint16_t raw_adc[LINE_SENSOR_COUNT];
    uint16_t calibrated[LINE_SENSOR_COUNT];
    uint16_t min_white[LINE_SENSOR_COUNT];
    uint16_t max_black[LINE_SENSOR_COUNT];
    float position_mm;
    float position_m;
    bool line_lost;
    bool intersection_detected;
    uint8_t active_sensor_count;
} line_sensor_data_t;

typedef struct {
    float alpha_min;
    float alpha_max;
    float sensitivity;
    float value;
} adaptive_ema_t;

/* Khai báo trước các hàm liên kết giữa các module */
robot_state_t state_get_current(void);
void state_set(robot_state_t new_state);
route_selection_t state_get_selected_route(void);
void state_set_selected_route(route_selection_t route, float distance_mm);

void control_set_target_speed(float speed_mps);
void control_set_manual_pwm(float pwm);
void control_set_mode(bool manual_mode);
bool control_is_manual_mode(void);
float control_get_manual_pwm(void);
float control_get_current_speed(void);
float control_get_lateral_error_mm(void);
float control_get_steer_angle_deg(void);
float control_get_servo_angle_deg(void);
float control_get_current_pwm(void);
float control_get_current_rpm(void);

void web_server_send_telemetry(const line_sensor_data_t *line_data);

/* ==========================================================================
 * 4. BỘ ĐIỀU KHIỂN PID & SPEED CONTROL (pid.c & speed_control.c)
 * ========================================================================== */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float ts,
              float out_min, float out_max, float deadband) {
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->ts = (ts > 0.0f) ? ts : 0.01f;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->deadband = deadband;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_compute(pid_controller_t *pid, float setpoint, float feedback) {
    if (!pid) return 0.0f;

    float error = setpoint - feedback;
    float p_term = pid->kp * error;
    float i_term = pid->integral + pid->ki * error * pid->ts;
    float d_term = 0.0f;
    if (pid->ts > 0.0f) {
        d_term = pid->kd * (error - pid->prev_error) / pid->ts;
    }

    float output = p_term + i_term + d_term;

    // Anti-windup
    if (output > pid->out_max) {
        output = pid->out_max;
        if (error < 0.0f) pid->integral = i_term;
    } else if (output < pid->out_min) {
        output = pid->out_min;
        if (error > 0.0f) pid->integral = i_term;
    } else {
        pid->integral = i_term;
    }

    // Bù deadband
    if (output > 0.0f && pid->deadband > 0.0f) {
        output += pid->deadband;
        if (output > pid->out_max) output = pid->out_max;
    } else if (output < 0.0f && pid->deadband > 0.0f) {
        output -= pid->deadband;
        if (output < pid->out_min) output = pid->out_min;
    }

    pid->prev_error = error;
    return output;
}

void pid_reset(pid_controller_t *pid) {
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float speed_control_linear_to_rpm(float velocity_mps) {
    if (velocity_mps <= 0.001f) return 0.0f;
    return (velocity_mps / ((float)M_PI * WHEEL_DIAMETER_M)) * 60.0f * DIFF_GEAR_RATIO;
}

float speed_control_rpm_to_linear(float rpm) {
    if (rpm <= 0.001f) return 0.0f;
    return (rpm / (60.0f * DIFF_GEAR_RATIO)) * ((float)M_PI * WHEEL_DIAMETER_M);
}

void speed_control_init(speed_controller_t *ctrl) {
    if (!ctrl) return;
    pid_init(&ctrl->pi,
             MOTOR_PI_KP,
             MOTOR_PI_KI,
             0.0f,
             (float)MOTOR_CONTROL_PERIOD_MS / 1000.0f,
             0.0f,
             255.0f,
             (float)MOTOR_DEADBAND_PWM);
    ctrl->target_velocity_mps = 0.0f;
    ctrl->target_rpm = 0.0f;
    ctrl->actual_rpm = 0.0f;
    ctrl->actual_velocity_mps = 0.0f;
    ctrl->output_pwm = 0.0f;
}

void speed_control_reset(speed_controller_t *ctrl) {
    if (!ctrl) return;
    pid_reset(&ctrl->pi);
    ctrl->target_velocity_mps = 0.0f;
    ctrl->target_rpm = 0.0f;
    ctrl->output_pwm = 0.0f;
}

float speed_control_update(speed_controller_t *ctrl, float target_velocity_mps, float current_rpm) {
    if (!ctrl) return 0.0f;

    ctrl->target_velocity_mps = target_velocity_mps;
    ctrl->target_rpm = speed_control_linear_to_rpm(target_velocity_mps);
    ctrl->actual_rpm = current_rpm;
    ctrl->actual_velocity_mps = speed_control_rpm_to_linear(current_rpm);

    if (ctrl->target_rpm <= 1.0f) {
        speed_control_reset(ctrl);
        ctrl->output_pwm = 0.0f;
        return 0.0f;
    }

    ctrl->output_pwm = pid_compute(&ctrl->pi, ctrl->target_rpm, ctrl->actual_rpm);
    return ctrl->output_pwm;
}

/* ==========================================================================
 * 5. DRIVER ENCODER JGB37 (jgb37_encoder.c)
 * ========================================================================== */
#define ENCODER_PPR 400.0f

static volatile int64_t s_pulse_count = 0;
static int64_t s_last_pulse_count = 0;
static float s_current_rpm = 0.0f;

void IRAM_ATTR encoder_isr_handler(void) {
    int level_a = digitalRead(JGB37_ENC_A_GPIO);
    int level_b = digitalRead(JGB37_ENC_B_GPIO);
    if (level_a == level_b) {
        s_pulse_count++;
    } else {
        s_pulse_count--;
    }
}

void jgb37_encoder_init(void) {
    pinMode(JGB37_ENC_A_GPIO, INPUT);
    pinMode(JGB37_ENC_B_GPIO, INPUT);
    attachInterrupt(digitalPinToInterrupt(JGB37_ENC_A_GPIO), encoder_isr_handler, CHANGE);
    s_pulse_count = 0;
    s_last_pulse_count = 0;
    s_current_rpm = 0.0f;
}

float jgb37_encoder_update_rpm(float delta_time_s) {
    if (delta_time_s <= 0.0001f) return s_current_rpm;

    int64_t current_count = s_pulse_count;
    int64_t delta_pulses = current_count - s_last_pulse_count;
    s_last_pulse_count = current_count;

    float revs = (float)delta_pulses / ENCODER_PPR;
    s_current_rpm = (revs / delta_time_s) * 60.0f;
    return s_current_rpm;
}

/* ==========================================================================
 * 6. DRIVER ĐỘNG CƠ JGB37 (jgb37_motor.c)
 * ========================================================================== */
#define MOTOR_LEDC_TIMER    LEDC_TIMER_1
#define MOTOR_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_CH_IN1   LEDC_CHANNEL_1
#define MOTOR_LEDC_CH_IN2   LEDC_CHANNEL_2
#define MOTOR_LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define MOTOR_LEDC_FREQ_HZ  20000

void jgb37_motor_stop(void) {
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1, 0);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1);
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2, 0);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2);
}

void jgb37_motor_brake(void) {
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1, 255);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1);
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2, 255);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2);
}

void jgb37_motor_set_pwm(float pwm) {
    if (pwm > 255.0f) pwm = 255.0f;
    if (pwm < -255.0f) pwm = -255.0f;

    uint32_t duty = (uint32_t)fabsf(pwm);
    if (pwm > 0.0f) {
        ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1, duty);
        ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1);
        ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2, 0);
        ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2);
    } else if (pwm < 0.0f) {
        ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1, 0);
        ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1);
        ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2, duty);
        ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2);
    } else {
        jgb37_motor_stop();
    }
}

void jgb37_motor_init(void) {
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode      = MOTOR_LEDC_MODE;
    timer_conf.duty_resolution = MOTOR_LEDC_DUTY_RES;
    timer_conf.timer_num       = MOTOR_LEDC_TIMER;
    timer_conf.freq_hz         = MOTOR_LEDC_FREQ_HZ;
    timer_conf.clk_cfg         = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch_in1 = {};
    ch_in1.gpio_num   = TB67_IN1_GPIO;
    ch_in1.speed_mode = MOTOR_LEDC_MODE;
    ch_in1.channel    = MOTOR_LEDC_CH_IN1;
    ch_in1.intr_type  = LEDC_INTR_DISABLE;
    ch_in1.timer_sel  = MOTOR_LEDC_TIMER;
    ch_in1.duty       = 0;
    ch_in1.hpoint     = 0;
    ledc_channel_config(&ch_in1);

    ledc_channel_config_t ch_in2 = {};
    ch_in2.gpio_num   = TB67_IN2_GPIO;
    ch_in2.speed_mode = MOTOR_LEDC_MODE;
    ch_in2.channel    = MOTOR_LEDC_CH_IN2;
    ch_in2.intr_type  = LEDC_INTR_DISABLE;
    ch_in2.timer_sel  = MOTOR_LEDC_TIMER;
    ch_in2.duty       = 0;
    ch_in2.hpoint     = 0;
    ledc_channel_config(&ch_in2);

    jgb37_motor_stop();
}

/* ==========================================================================
 * 7. DRIVER SERVO LÁI MG90S (mg90s.c)
 * ========================================================================== */
#define SERVO_CENTER_DEG    90.0f
#define SERVO_LEDC_TIMER    LEDC_TIMER_0
#define SERVO_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL  LEDC_CHANNEL_0
#define SERVO_LEDC_DUTY_RES LEDC_TIMER_14_BIT
#define SERVO_LEDC_FREQ_HZ  50

#define SERVO_MIN_PULSE_US  500
#define SERVO_MID_PULSE_US  1500
#define SERVO_MAX_PULSE_US  2500
#define SERVO_PERIOD_US     20000

static float s_current_servo_angle = SERVO_CENTER_DEG;
static float s_current_steer_deg   = 0.0f;
static float s_trim_offset_deg     = 0.0f;
static bool  s_invert_direction    = true;

static uint32_t pulse_to_duty(uint32_t pulse_us) {
    return (uint32_t)(((uint64_t)pulse_us * ((1 << 14) - 1)) / SERVO_PERIOD_US);
}

void mg90s_set_servo_angle(float servo_angle_deg) {
    if (servo_angle_deg < 0.0f)   servo_angle_deg = 0.0f;
    if (servo_angle_deg > 180.0f) servo_angle_deg = 180.0f;

    s_current_servo_angle = servo_angle_deg;
    float pulse_us = SERVO_MIN_PULSE_US + (servo_angle_deg / 180.0f) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    uint32_t duty = pulse_to_duty((uint32_t)pulse_us);
    ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
}

void mg90s_set_angle_deg(float steer_angle_deg) {
    float max_steer = ROBOT_MAX_STEER_ANGLE_RAD * RAD_TO_DEG;
    if (steer_angle_deg > max_steer)  steer_angle_deg = max_steer;
    if (steer_angle_deg < -max_steer) steer_angle_deg = -max_steer;

    s_current_steer_deg = steer_angle_deg;
    float dir = s_invert_direction ? -1.0f : 1.0f;
    float target_servo_angle = SERVO_CENTER_DEG + s_trim_offset_deg + (dir * steer_angle_deg);
    mg90s_set_servo_angle(target_servo_angle);
}

void mg90s_set_angle_rad(float steer_angle_rad) {
    mg90s_set_angle_deg(steer_angle_rad * RAD_TO_DEG);
}

float mg90s_get_servo_angle(void) {
    return s_current_servo_angle;
}

void mg90s_init(void) {
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode      = SERVO_LEDC_MODE;
    timer_conf.duty_resolution = SERVO_LEDC_DUTY_RES;
    timer_conf.timer_num       = SERVO_LEDC_TIMER;
    timer_conf.freq_hz         = SERVO_LEDC_FREQ_HZ;
    timer_conf.clk_cfg         = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch_conf = {};
    ch_conf.gpio_num   = MG90S_PWM_GPIO;
    ch_conf.speed_mode = SERVO_LEDC_MODE;
    ch_conf.channel    = SERVO_LEDC_CHANNEL;
    ch_conf.intr_type  = LEDC_INTR_DISABLE;
    ch_conf.timer_sel  = SERVO_LEDC_TIMER;
    ch_conf.duty       = pulse_to_duty(SERVO_MID_PULSE_US);
    ch_conf.hpoint     = 0;
    ledc_channel_config(&ch_conf);

    mg90s_set_servo_angle(SERVO_CENTER_DEG);
    Serial.printf("[MG90S] Khoi tao Servo thanh cong: GPIO %d, 90 do (1500 us)\n", MG90S_PWM_GPIO);
}

/* ==========================================================================
 * 8. DRIVER MẢNG 8 CẢM BIẾN DÒ LINE TCRT5000 + MUX HEF4051B (tctr5000.c)
 * ========================================================================== */
#define SETTLE_DELAY_US 60

static uint16_t s_default_white[LINE_SENSOR_COUNT] = {60, 50, 60, 60, 60, 60, 70, 70};
static uint16_t s_default_black[LINE_SENSOR_COUNT] = {2800, 2800, 2800, 2850, 2850, 2800, 2800, 2800};

void line_sensor_init(void) {
    pinMode(TCRT_SEL1_GPIO, OUTPUT);
    pinMode(TCRT_SEL2_GPIO, OUTPUT);
    pinMode(TCRT_SEL3_GPIO, OUTPUT);
    digitalWrite(TCRT_SEL1_GPIO, LOW);
    digitalWrite(TCRT_SEL2_GPIO, LOW);
    digitalWrite(TCRT_SEL3_GPIO, LOW);

    analogReadResolution(12);
    analogSetPinAttenuation(TCRT_ADC_GPIO, ADC_11db);
}

static void mux_select_channel(uint8_t ch) {
    digitalWrite(TCRT_SEL1_GPIO, (ch & 0x01) ? HIGH : LOW);
    digitalWrite(TCRT_SEL2_GPIO, (ch & 0x02) ? HIGH : LOW);
    digitalWrite(TCRT_SEL3_GPIO, (ch & 0x04) ? HIGH : LOW);
    delayMicroseconds(SETTLE_DELAY_US);
}

static float calibrate_piecewise_position_mm(float raw_pos_mm) {
    float sign = (raw_pos_mm >= 0.0f) ? 1.0f : -1.0f;
    float abs_pos = fabsf(raw_pos_mm);
    if (abs_pos > 28.0f) abs_pos = 28.0f;
    return sign * abs_pos;
}

void line_sensor_read(line_sensor_data_t *data) {
    if (!data) return;

    float weighted_sum = 0.0f;
    float sum_weights = 0.0f;
    uint8_t active_count = 0;

    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        mux_select_channel(i);

        // Đọc bỏ lần đầu để ổn định điện áp sau chuyển MUX
        (void)analogRead(TCRT_ADC_GPIO);

        int sum_adc = 0;
        const int samples = 8;
        for (int s = 0; s < samples; s++) {
            sum_adc += analogRead(TCRT_ADC_GPIO);
        }
        int raw_val = sum_adc / samples;
        data->raw_adc[i] = (uint16_t)raw_val;

        uint16_t w_min = s_default_white[i];
        uint16_t b_max = s_default_black[i];
        int32_t calib = 0;

        if (b_max > w_min) {
            calib = (int32_t)(raw_val - w_min) * 1000 / (int32_t)(b_max - w_min);
        }
        if (calib < 0) calib = 0;
        if (calib > 1000) calib = 1000;

        data->calibrated[i] = (uint16_t)calib;

        if (calib > 300) {
            active_count++;
        }

        uint16_t weight = (calib >= 50) ? (uint16_t)calib : 0;
        weighted_sum += (float)weight * LINE_SENSOR_POS_MM[i];
        sum_weights += (float)weight;
    }

    data->active_sensor_count = active_count;

    if (sum_weights < 150.0f) {
        data->line_lost = true;
        data->intersection_detected = false;
        if (data->position_mm > 6.0f) {
            data->position_mm = 28.0f;
            data->position_m = 0.028f;
        } else if (data->position_mm < -6.0f) {
            data->position_mm = -28.0f;
            data->position_m = -0.028f;
        }
    } else {
        data->line_lost = false;
        float raw_pos_mm = weighted_sum / sum_weights;
        data->position_mm = calibrate_piecewise_position_mm(raw_pos_mm);
        data->position_m = data->position_mm / 1000.0f;
        data->intersection_detected = (active_count >= 7);
    }
}

/* ==========================================================================
 * 9. DRIVER CẢM BIẾN VẬT CẢN VL53L0X ToF (vl53l0x.c)
 * ========================================================================== */
#define VL53L0X_DEFAULT_ADDR                     0x29
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID      0xC0
#define VL53L0X_REG_SYSRANGE_START               0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS_GPIO 0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS          0x14
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR       0x0B

static bool vl53_write_byte(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(VL53L0X_DEFAULT_ADDR);
    Wire.write(reg);
    Wire.write(data);
    return (Wire.endTransmission() == 0);
}

static bool vl53_read_bytes(uint8_t reg, uint8_t *data, size_t len) {
    Wire.beginTransmission(VL53L0X_DEFAULT_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)VL53L0X_DEFAULT_ADDR, (uint8_t)len) != len) return false;
    for (size_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
    return true;
}

static bool vl53_read_byte(uint8_t reg, uint8_t *data) {
    return vl53_read_bytes(reg, data, 1);
}

bool vl53l0x_init(void) {
    Wire.begin(VL53L0X_SDA_GPIO, VL53L0X_SCL_GPIO, 100000);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t model_id = 0;
    if (vl53_read_byte(VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model_id) && model_id == 0xEE) {
        Serial.printf("[VL53L0X] Detected successfully (ID: 0x%02X)\n", model_id);
    } else {
        Serial.printf("[VL53L0X] Warning: ID check returned 0x%02X\n", model_id);
    }

    vl53_write_byte(0x88, 0x00);
    vl53_write_byte(0x80, 0x01);
    vl53_write_byte(0xFF, 0x01);
    vl53_write_byte(0x00, 0x00);
    vl53_write_byte(0x91, 0x3C);
    vl53_write_byte(0x00, 0x01);
    vl53_write_byte(0xFF, 0x00);
    vl53_write_byte(0x80, 0x00);
    return true;
}

bool vl53l0x_read_distance_mm(uint16_t *raw_dist_mm, float *calib_dist_mm) {
    if (!raw_dist_mm) return false;
    if (!vl53_write_byte(VL53L0X_REG_SYSRANGE_START, 0x01)) return false;

    uint8_t status = 0;
    int timeout = 20;
    while (timeout-- > 0) {
        if (vl53_read_byte(VL53L0X_REG_RESULT_RANGE_STATUS, &status) && (status & 0x01)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (timeout <= 0) return false;

    uint8_t dist_buf[2];
    if (!vl53_read_bytes(0x1E, dist_buf, 2)) return false;
    vl53_write_byte(VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    uint16_t raw = ((uint16_t)dist_buf[0] << 8) | dist_buf[1];
    *raw_dist_mm = raw;

    if (calib_dist_mm) {
        float calib = (float)raw + OBSTACLE_CALIB_OFFSET_MM;
        if (calib < 0.0f) calib = 0.0f;
        *calib_dist_mm = calib;
    }
    return true;
}

route_selection_t vl53l0x_decide_route(float *measured_distance) {
    const int sample_count = 20;
    float sum_dist = 0.0f;
    int valid_samples = 0;

    for (int i = 0; i < sample_count; i++) {
        uint16_t raw = 0;
        float calib = 0.0f;
        if (vl53l0x_read_distance_mm(&raw, &calib)) {
            if (raw > 20 && raw < 2000) {
                sum_dist += calib;
                valid_samples++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(15));
    }

    float avg_dist = 9999.0f;
    if (valid_samples > 0) {
        avg_dist = sum_dist / (float)valid_samples;
    }
    if (measured_distance) {
        *measured_distance = avg_dist;
    }

    if (avg_dist <= OBSTACLE_NEAR_DETECT_MAX_MM) {
        Serial.printf("[VL53L0X] Vat can o nhanh gan (d = %.1f mm <= %.0f mm) -> Chon nhanh TREN\n",
                      avg_dist, (float)OBSTACLE_NEAR_DETECT_MAX_MM);
        return ROUTE_UPPER;
    } else {
        Serial.printf("[VL53L0X] Nhanh gan thong thoang (d = %.1f mm > %.0f mm) -> Chon nhanh DUOI\n",
                      avg_dist, (float)OBSTACLE_NEAR_DETECT_MAX_MM);
        return ROUTE_LOWER;
    }
}

/* ==========================================================================
 * 10. QUẢN LÝ TRẠNG THÁI & NÚT NHẤN / LED (state_task.c)
 * ========================================================================== */
static robot_state_t s_current_state = ROBOT_STATE_INIT;
static route_selection_t s_selected_route = ROUTE_UNKNOWN;
static float s_obstacle_distance_mm = 0.0f;
static SemaphoreHandle_t s_state_mutex = NULL;

void state_manager_init(void) {
    if (!s_state_mutex) {
        s_state_mutex = xSemaphoreCreateMutex();
    }
    pinMode(BOOT_GPIO, INPUT_PULLUP);
    pinMode(LED_RED_GPIO, OUTPUT);
    pinMode(LED_GREEN_GPIO, OUTPUT);
    digitalWrite(LED_RED_GPIO, LOW);
    digitalWrite(LED_GREEN_GPIO, LOW);
}

robot_state_t state_get_current(void) {
    robot_state_t st = s_current_state;
    if (s_state_mutex && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        st = s_current_state;
        xSemaphoreGive(s_state_mutex);
    }
    return st;
}

void state_set(robot_state_t new_state) {
    if (s_state_mutex && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_current_state != new_state) {
            Serial.printf("[STATE_TASK] Chuyen trang thai: %d -> %d\n", s_current_state, new_state);
            s_current_state = new_state;
        }
        xSemaphoreGive(s_state_mutex);
    }
}

route_selection_t state_get_selected_route(void) {
    route_selection_t r = ROUTE_UNKNOWN;
    if (s_state_mutex && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        r = s_selected_route;
        xSemaphoreGive(s_state_mutex);
    }
    return r;
}

void state_set_selected_route(route_selection_t route, float distance_mm) {
    if (s_state_mutex && xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_selected_route = route;
        s_obstacle_distance_mm = distance_mm;
        Serial.printf("[STATE_TASK] Da xac dinh tuyen duong: %s (Khoang cach vat can: %.1f mm)\n",
                      (route == ROUTE_UPPER) ? "NHANH TREN" : "NHANH DUOI", distance_mm);
        xSemaphoreGive(s_state_mutex);
    }
}

bool state_is_start_button_pressed(void) {
    return (digitalRead(BOOT_GPIO) == LOW);
}

void State_Task(void *pvParameters) {
    Serial.printf("[STATE_TASK] Khoi dong tren Core %d\n", xPortGetCoreID());
    state_set(ROBOT_STATE_WAIT_START);

    while (1) {
        robot_state_t state = state_get_current();

        switch (state) {
            case ROBOT_STATE_WAIT_START:
                digitalWrite(LED_GREEN_GPIO, HIGH);
                vTaskDelay(pdMS_TO_TICKS(200));
                digitalWrite(LED_GREEN_GPIO, LOW);
                vTaskDelay(pdMS_TO_TICKS(200));

                if (state_is_start_button_pressed()) {
                    Serial.println("[STATE_TASK] Nut START duoc nhan! Bat dau do vat can...");
                    state_set(ROBOT_STATE_DETECT_OBSTACLE);
                }
                break;

            case ROBOT_STATE_DETECT_OBSTACLE:
                digitalWrite(LED_RED_GPIO, HIGH);
                digitalWrite(LED_GREEN_GPIO, HIGH);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case ROBOT_STATE_LINE_TRACKING:
                digitalWrite(LED_RED_GPIO, LOW);
                digitalWrite(LED_GREEN_GPIO, HIGH);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case ROBOT_STATE_INTERSECTION:
                digitalWrite(LED_GREEN_GPIO, HIGH);
                vTaskDelay(pdMS_TO_TICKS(50));
                digitalWrite(LED_GREEN_GPIO, LOW);
                vTaskDelay(pdMS_TO_TICKS(50));
                break;

            case ROBOT_STATE_FINISH:
                digitalWrite(LED_GREEN_GPIO, HIGH);
                digitalWrite(LED_RED_GPIO, LOW);
                vTaskDelay(pdMS_TO_TICKS(500));
                break;

            case ROBOT_STATE_STOPPED:
            default:
                digitalWrite(LED_GREEN_GPIO, LOW);
                digitalWrite(LED_RED_GPIO, HIGH);
                vTaskDelay(pdMS_TO_TICKS(500));
                break;
        }
    }
}

/* ==========================================================================
 * 11. TASK QUÉT VẬT CẢN ToF (obstacle_task.c)
 * ========================================================================== */
void obstacle_task_init(void) {
    vl53l0x_init();
}

void Obstacle_Task(void *pvParameters) {
    Serial.printf("[OBSTACLE_TASK] Khoi dong tren Core %d\n", xPortGetCoreID());

    while (1) {
        robot_state_t state = state_get_current();

        if (state == ROBOT_STATE_DETECT_OBSTACLE) {
            Serial.println("[OBSTACLE_TASK] Bat dau tien trinh do tim vat can tai vung phan nhanh...");
            float measured_dist = 0.0f;
            route_selection_t route = vl53l0x_decide_route(&measured_dist);
            state_set_selected_route(route, measured_dist);

            vTaskDelay(pdMS_TO_TICKS(500));
            state_set(ROBOT_STATE_LINE_TRACKING);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ==========================================================================
 * 12. TASK ĐIỀU KHIỂN THỜI GIAN THỰC 100Hz (control_task.c)
 * ========================================================================== */
static inline void adaptive_ema_init(adaptive_ema_t *f, float alpha_min,
                                     float alpha_max, float sensitivity,
                                     float initial_value) {
    f->alpha_min = alpha_min;
    f->alpha_max = alpha_max;
    f->sensitivity = sensitivity;
    f->value = initial_value;
}

static inline float adaptive_ema_update(adaptive_ema_t *f, float new_value) {
    float delta = fabsf(new_value - f->value);
    float alpha = f->alpha_min + (f->alpha_max - f->alpha_min) * delta /
                                     (delta + f->sensitivity);
    f->value = (1.0f - alpha) * f->value + alpha * new_value;
    return f->value;
}

static speed_controller_t s_speed_ctrl;
static line_sensor_data_t s_line_data;

static float s_target_linear_speed = 0.4f;
static bool  s_use_manual_pwm      = true;
static float s_manual_pwm          = 45.0f;

static float s_current_steer_rad        = 0.0f;
static float s_prev_e2_m                = 0.0f;
static uint32_t s_lost_line_counter     = 0;
static uint32_t s_intersection_counter  = 0;
static bool s_has_passed_intersection   = false;

void control_system_init(void) {
    line_sensor_init();
    mg90s_init();
    jgb37_motor_init();
    jgb37_encoder_init();
    speed_control_init(&s_speed_ctrl);
    Serial.println("[CONTROL_TASK] Khoi tao he thong dieu khien thanh cong!");
}

void control_set_target_speed(float speed_mps) {
    if (speed_mps < ROBOT_SPEED_MIN_MPS) speed_mps = ROBOT_SPEED_MIN_MPS;
    if (speed_mps > ROBOT_SPEED_MAX_MPS) speed_mps = ROBOT_SPEED_MAX_MPS;
    s_target_linear_speed = speed_mps;
}

float control_get_current_speed(void)      { return s_speed_ctrl.actual_velocity_mps; }
float control_get_lateral_error_mm(void)   { return s_line_data.position_mm; }
float control_get_steer_angle_deg(void)    { return s_current_steer_rad * RAD_TO_DEG; }
float control_get_servo_angle_deg(void)    { return mg90s_get_servo_angle(); }
float control_get_current_pwm(void)        { return s_speed_ctrl.output_pwm; }
float control_get_current_rpm(void)        { return s_speed_ctrl.actual_rpm; }

void control_set_manual_pwm(float pwm) {
    if (pwm < 0.0f) pwm = 0.0f;
    if (pwm > 255.0f) pwm = 255.0f;
    s_manual_pwm = pwm;
    s_use_manual_pwm = true;
    Serial.printf("[CONTROL_TASK] Da dat PWM thu cong: %.0f/255 (Manual PWM)\n", s_manual_pwm);
}

void control_set_mode(bool manual_mode) {
    s_use_manual_pwm = manual_mode;
    Serial.printf("[CONTROL_TASK] Chuyen che do toc do: %s\n", manual_mode ? "MANUAL PWM" : "AUTO PI");
}

bool control_is_manual_mode(void)  { return s_use_manual_pwm; }
float control_get_manual_pwm(void) { return s_manual_pwm; }

void Control_Task(void *pvParameters) {
    Serial.printf("[CONTROL_TASK] Khoi dong tren Core %d voi chu ky %d ms (100Hz)\n",
                  xPortGetCoreID(), MOTOR_CONTROL_PERIOD_MS);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS);
    const float dt = (float)MOTOR_CONTROL_PERIOD_MS / 1000.0f;

    adaptive_ema_t ema_e2, ema_de2, ema_gamma;
    adaptive_ema_init(&ema_e2, 0.25f, 0.85f, 5.0f, 0.0f);
    adaptive_ema_init(&ema_de2, 0.35f, 0.75f, 0.15f, 0.0f);
    adaptive_ema_init(&ema_gamma, 0.55f, 0.90f, 0.10f, 0.0f);

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        robot_state_t state = state_get_current();

        line_sensor_read(&s_line_data);
        float current_motor_rpm = jgb37_encoder_update_rpm(dt);

        static int s_log_counter = 0;
        if (++s_log_counter >= 20) {
            s_log_counter = 0;
            Serial.printf("[CONTROL] ADC:[S1:%4d S2:%4d S3:%4d S4:%4d S5:%4d S6:%4d S7:%4d S8:%4d] | e2:%5.1f mm | RPM:%5.0f | PWM:%3.0f | St:%d%s\n",
                          s_line_data.raw_adc[0], s_line_data.raw_adc[1],
                          s_line_data.raw_adc[2], s_line_data.raw_adc[3],
                          s_line_data.raw_adc[4], s_line_data.raw_adc[5],
                          s_line_data.raw_adc[6], s_line_data.raw_adc[7],
                          s_line_data.position_mm, current_motor_rpm, s_speed_ctrl.output_pwm,
                          (int)state,
                          s_line_data.line_lost ? " (LOST)" : (s_line_data.intersection_detected ? " (INTERSECTION)" : ""));
        }

        static int s_telemetry_counter = 0;
        if (++s_telemetry_counter >= 5) {
            s_telemetry_counter = 0;
            web_server_send_telemetry(&s_line_data);
        }

        if (state == ROBOT_STATE_INIT || state == ROBOT_STATE_WAIT_START ||
            state == ROBOT_STATE_DETECT_OBSTACLE) {
            jgb37_motor_stop();
            mg90s_set_angle_deg(0.0f);
            speed_control_reset(&s_speed_ctrl);
            ema_e2.value = s_line_data.position_mm;
            ema_de2.value = 0.0f;
            ema_gamma.value = 0.0f;
            s_prev_e2_m = s_line_data.position_m;
            s_lost_line_counter = 0;
            s_intersection_counter = 0;
            s_has_passed_intersection = false;
            continue;
        }

        if (state == ROBOT_STATE_FINISH || state == ROBOT_STATE_STOPPED) {
            jgb37_motor_brake();
            mg90s_set_angle_deg(0.0f);
            speed_control_reset(&s_speed_ctrl);
            ema_e2.value = s_line_data.position_mm;
            ema_de2.value = 0.0f;
            ema_gamma.value = 0.0f;
            s_prev_e2_m = s_line_data.position_m;
            continue;
        }

        if (state == ROBOT_STATE_LINE_TRACKING) {
            if (s_line_data.intersection_detected && !s_has_passed_intersection) {
                s_intersection_counter++;
                if (s_intersection_counter >= 3) {
                    Serial.println("[CONTROL_TASK] Phat hien nga re! Kich hoat re nhanh theo lo trinh da chon");
                    state_set(ROBOT_STATE_INTERSECTION);
                    s_intersection_counter = 0;
                }
            } else {
                s_intersection_counter = 0;
            }

            if (s_line_data.line_lost) {
                s_lost_line_counter++;
                if (s_has_passed_intersection && s_lost_line_counter >= 35) {
                    Serial.println("[CONTROL_TASK] Da ve dich! Tien hanh dung xe");
                    state_set(ROBOT_STATE_FINISH);
                    jgb37_motor_brake();
                    continue;
                }
            } else {
                s_lost_line_counter = 0;
            }
        }

        float v_cmd = s_target_linear_speed;
        float omega_cmd = 0.0f;
        float gamma = 0.0f;

        if (state == ROBOT_STATE_INTERSECTION) {
            route_selection_t route = state_get_selected_route();
            if (route == ROUTE_UPPER) {
                gamma = atan2f(0.15f, 0.8f);
            } else {
                gamma = -atan2f(0.15f, 0.8f);
            }
            v_cmd = s_target_linear_speed * 0.75f;

            ema_e2.value = s_line_data.position_mm;
            ema_de2.value = 0.0f;
            s_prev_e2_m = s_line_data.position_m;

            s_intersection_counter++;
            if (s_intersection_counter > 50 ||
                (s_intersection_counter > 20 && s_line_data.active_sensor_count <= 3)) {
                Serial.println("[CONTROL_TASK] Da qua nga re thanh cong, tiep tuc bam line don");
                s_has_passed_intersection = true;
                s_intersection_counter = 0;
                state_set(ROBOT_STATE_LINE_TRACKING);
            }
        } else if (state == ROBOT_STATE_LINE_TRACKING) {
            float e2_mm = s_line_data.position_mm;
            float filtered_e2_mm = adaptive_ema_update(&ema_e2, e2_mm);
            float filtered_e2_m = filtered_e2_mm / 1000.0f;

            float de2_dt = (filtered_e2_m - s_prev_e2_m) / dt;
            s_prev_e2_m = filtered_e2_m;

            float filtered_de2 = adaptive_ema_update(&ema_de2, de2_dt);

            float sin_e3 = 0.0f;
            if (v_cmd > 0.01f) {
                sin_e3 = filtered_de2 / v_cmd;
                if (sin_e3 > 1.0f) sin_e3 = 1.0f;
                if (sin_e3 < -1.0f) sin_e3 = -1.0f;
            }
            float e3 = asinf(sin_e3);
            float e1 = 0.0f;

            float v_d_mms = s_target_linear_speed * 1000.0f;
            omega_cmd = (LYAPUNOV_K2 * v_d_mms * filtered_e2_mm) + (LYAPUNOV_K3 * sin_e3);

            v_cmd = s_target_linear_speed * cosf(e3) + (LYAPUNOV_K1 * e1);
            if (v_cmd < ROBOT_SPEED_MIN_MPS) v_cmd = ROBOT_SPEED_MIN_MPS;

            const float ROBOT_L_STEER_EFF_M = 0.12f;
            gamma = atan2f(ROBOT_L_STEER_EFF_M * omega_cmd, v_cmd);

            if (s_line_data.line_lost) {
                gamma = (filtered_e2_mm > 0.0f) ? ROBOT_MAX_STEER_ANGLE_RAD : -ROBOT_MAX_STEER_ANGLE_RAD;
            }
        }

        float filtered_gamma = adaptive_ema_update(&ema_gamma, gamma);
        s_current_steer_rad = filtered_gamma;
        mg90s_set_angle_rad(filtered_gamma);

        float abs_gamma_deg = fabsf(gamma * RAD_TO_DEG);
        float cornering_factor = 1.0f;
        if (abs_gamma_deg > 10.0f) {
            cornering_factor = 1.0f - 0.35f * ((abs_gamma_deg - 10.0f) / 45.0f);
            if (cornering_factor < 0.65f) cornering_factor = 0.65f;
        }

        float pwm_output = 0.0f;
        if (s_use_manual_pwm) {
            pwm_output = s_manual_pwm * cornering_factor;
            s_speed_ctrl.output_pwm = pwm_output;
            s_speed_ctrl.actual_rpm = current_motor_rpm;
            s_speed_ctrl.actual_velocity_mps = speed_control_rpm_to_linear(current_motor_rpm);
        } else {
            pwm_output = speed_control_update(&s_speed_ctrl, v_cmd * cornering_factor, current_motor_rpm);
        }

        jgb37_motor_set_pwm(pwm_output);
    }
}

/* ==========================================================================
 * 13. WI-FI SOFTAP, HTTP DASHBOARD, WEBSOCKET (/ws) & OTA (/ota)
 *     (Tự thân hoàn toàn trên WebServer chuẩn của ESP32 - Không cần thư viện ngoài)
 * ========================================================================== */
static WebServer s_server(80);
static WiFiServer s_ws_server(81);
static WiFiClient s_ws_client;
static SemaphoreHandle_t s_ws_mutex = NULL;

/* --- SHA-1 & Base64 gọn nhẹ phục vụ bắt tay RFC6455 WebSocket --- */
static uint32_t sha1_rol(uint32_t v, int bits) { return (v << bits) | (v >> (32 - bits)); }

static void sha1_compute(const uint8_t *data, size_t len, uint8_t out[20]) {
    uint32_t h0 = 0x67452301, h1 = 0xEFCDAB89, h2 = 0x98BADCFE, h3 = 0x10325476, h4 = 0xC3D2E1F0;
    uint8_t msg[128];
    memset(msg, 0, sizeof(msg));
    if (len > 119) len = 119;
    memcpy(msg, data, len);
    msg[len] = 0x80;
    size_t total_len = (len + 9 <= 64) ? 64 : 128;
    uint64_t bit_len = (uint64_t)len * 8;
    for (int i = 0; i < 8; i++) msg[total_len - 1 - i] = (uint8_t)(bit_len >> (i * 8));

    for (size_t offset = 0; offset < total_len; offset += 64) {
        uint32_t w[80];
        for (int i = 0; i < 16; i++) {
            w[i] = ((uint32_t)msg[offset + i * 4] << 24) |
                   ((uint32_t)msg[offset + i * 4 + 1] << 16) |
                   ((uint32_t)msg[offset + i * 4 + 2] << 8) |
                   ((uint32_t)msg[offset + i * 4 + 3]);
        }
        for (int i = 16; i < 80; i++) w[i] = sha1_rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
        for (int i = 0; i < 80; i++) {
            uint32_t f, k;
            if (i < 20)      { f = (b & c) | ((~b) & d); k = 0x5A827999; }
            else if (i < 40) { f = b ^ c ^ d;            k = 0x6ED9EBA1; }
            else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
            else             { f = b ^ c ^ d;            k = 0xCA62C1D6; }
            uint32_t temp = sha1_rol(a, 5) + f + e + k + w[i];
            e = d; d = c; c = sha1_rol(b, 30); b = a; a = temp;
        }
        h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
    }
    uint32_t hs[5] = {h0, h1, h2, h3, h4};
    for (int i = 0; i < 5; i++) {
        out[i * 4]     = (hs[i] >> 24) & 0xFF;
        out[i * 4 + 1] = (hs[i] >> 16) & 0xFF;
        out[i * 4 + 2] = (hs[i] >> 8) & 0xFF;
        out[i * 4 + 3] = hs[i] & 0xFF;
    }
}

static String base64_encode_20(const uint8_t in[20]) {
    static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    char out[29];
    int j = 0;
    for (int i = 0; i < 20; i += 3) {
        uint32_t v = ((uint32_t)in[i] << 16) |
                     ((i + 1 < 20 ? (uint32_t)in[i + 1] : 0) << 8) |
                     (i + 2 < 20 ? (uint32_t)in[i + 2] : 0);
        out[j++] = b64[(v >> 18) & 0x3F];
        out[j++] = b64[(v >> 12) & 0x3F];
        out[j++] = (i + 1 < 20) ? b64[(v >> 6) & 0x3F] : '=';
        out[j++] = (i + 2 < 20) ? b64[v & 0x3F] : '=';
    }
    out[j] = '\0';
    return String(out);
}

/* --- Xử lý lệnh JSON nhận từ WebSocket --- */
static void handle_ws_command(const char *json) {
    Serial.printf("[WEB_SERVER] Nhan lenh WebSocket: %s\n", json);
    String s(json);

    if (s.indexOf("\"cmd\":\"start\"") >= 0 || s.indexOf("\"cmd\": \"start\"") >= 0) {
        Serial.println("[WEB_SERVER] Lenh START tu Dashboard!");
        state_set(ROBOT_STATE_DETECT_OBSTACLE);
    } else if (s.indexOf("\"cmd\":\"stop\"") >= 0 || s.indexOf("\"cmd\": \"stop\"") >= 0) {
        Serial.println("[WEB_SERVER] Lenh STOP tu Dashboard!");
        state_set(ROBOT_STATE_STOPPED);
    } else if (s.indexOf("\"cmd\":\"reset\"") >= 0 || s.indexOf("\"cmd\": \"reset\"") >= 0) {
        Serial.println("[WEB_SERVER] Lenh RESET tu Dashboard!");
        state_set(ROBOT_STATE_WAIT_START);
    } else if (s.indexOf("\"cmd\":\"mode\"") >= 0) {
        bool is_manual = (s.indexOf("\"manual\"") >= 0);
        control_set_mode(is_manual);
    } else if (s.indexOf("\"cmd\":\"pwm\"") >= 0) {
        int idx = s.indexOf("\"val\":");
        if (idx >= 0) {
            float val = s.substring(idx + 6).toFloat();
            control_set_manual_pwm(val);
        }
    } else if (s.indexOf("\"cmd\":\"speed\"") >= 0) {
        int idx = s.indexOf("\"val\":");
        if (idx >= 0) {
            float val = s.substring(idx + 6).toFloat();
            control_set_target_speed(val);
            Serial.printf("[WEB_SERVER] Dat toc do muc tieu moi: %.2f m/s\n", val);
        }
    }
}

/* --- Chấp nhận kết nối mới và đọc frame WebSocket từ trình duyệt gửi xuống --- */
static void ws_poll_client(void) {
    if (!s_ws_mutex) return;

    // 1. Kiểm tra có client WebSocket mới kết nối vào cổng 81 không
    WiFiClient new_client = s_ws_server.available();
    if (new_client && new_client.connected()) {
        String key = "";
        unsigned long t0 = millis();
        while (new_client.connected() && (millis() - t0 < 500)) {
            if (new_client.available()) {
                String line = new_client.readStringUntil('\n');
                line.trim();
                if (line.length() == 0) {
                    break; // Kết thúc HTTP Header
                }
                if (line.startsWith("Sec-WebSocket-Key:") || line.startsWith("sec-websocket-key:")) {
                    key = line.substring(18);
                    key.trim();
                }
            }
        }
        if (key.length() > 0) {
            String concat = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
            uint8_t sha1_out[20];
            sha1_compute((const uint8_t *)concat.c_str(), concat.length(), sha1_out);
            String accept = base64_encode_20(sha1_out);

            new_client.print("HTTP/1.1 101 Switching Protocols\r\n"
                             "Upgrade: websocket\r\n"
                             "Connection: Upgrade\r\n"
                             "Sec-WebSocket-Accept: " + accept + "\r\n\r\n");

            if (xSemaphoreTake(s_ws_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (s_ws_client && s_ws_client.connected()) {
                    s_ws_client.stop();
                }
                s_ws_client = new_client;
                s_ws_client.setNoDelay(true);
                xSemaphoreGive(s_ws_mutex);
            }
            Serial.println("[WEB_SERVER] WebSocket client ket noi (Port 81)!");
        } else {
            new_client.stop();
        }
    }

    // 2. Đọc gói tin điều khiển từ client WebSocket hiện tại
    if (xSemaphoreTake(s_ws_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return;

    if (s_ws_client && s_ws_client.connected() && s_ws_client.available() >= 2) {
        uint8_t hdr[2];
        s_ws_client.read(hdr, 2);
        uint8_t opcode = hdr[0] & 0x0F;
        bool masked = (hdr[1] & 0x80) != 0;
        uint64_t payload_len = hdr[1] & 0x7F;

        if (payload_len == 126) {
            uint8_t ext[2];
            if (s_ws_client.readBytes(ext, 2) == 2) {
                payload_len = ((uint16_t)ext[0] << 8) | ext[1];
            }
        }

        uint8_t mask[4] = {0, 0, 0, 0};
        if (masked) {
            s_ws_client.readBytes(mask, 4);
        }

        if (opcode == 0x08) {
            s_ws_client.stop();
        } else if (payload_len > 0 && payload_len < 256) {
            char buf[256];
            size_t r = s_ws_client.readBytes((uint8_t *)buf, (size_t)payload_len);
            for (size_t i = 0; i < r && masked; i++) {
                buf[i] ^= mask[i % 4];
            }
            buf[r] = '\0';
            if (opcode == 0x01) {
                xSemaphoreGive(s_ws_mutex);
                handle_ws_command(buf);
                return;
            }
        }
    }

    xSemaphoreGive(s_ws_mutex);
}

/* --- Gửi Telemetry định kỳ qua WebSocket lên Dashboard --- */
void web_server_send_telemetry(const line_sensor_data_t *line_data) {
    if (!s_ws_mutex) return;
    if (xSemaphoreTake(s_ws_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return;

    if (!s_ws_client || !s_ws_client.connected()) {
        xSemaphoreGive(s_ws_mutex);
        return;
    }

    char json_str[320];
    robot_state_t st = state_get_current();
    float v = control_get_current_speed();
    float steer = control_get_steer_angle_deg();
    float servo = control_get_servo_angle_deg();
    float e2 = control_get_lateral_error_mm();
    float rpm = control_get_current_rpm();
    float pwm = control_get_current_pwm();

    int len = snprintf(json_str, sizeof(json_str),
             "{\"v\":%.2f,\"rpm\":%.0f,\"steer\":%.1f,\"servo\":%.1f,\"e2\":%.1f,\"pwm\":%.0f,\"state\":%d,"
             "\"mode\":\"%s\",\"mpwm\":%.0f,"
             "\"adc\":[%d,%d,%d,%d,%d,%d,%d,%d],\"lost\":%s,\"inter\":%s}",
             v, rpm, steer, servo, e2, pwm, (int)st,
             control_is_manual_mode() ? "manual" : "auto",
             control_get_manual_pwm(),
             line_data ? line_data->raw_adc[0] : 0,
             line_data ? line_data->raw_adc[1] : 0,
             line_data ? line_data->raw_adc[2] : 0,
             line_data ? line_data->raw_adc[3] : 0,
             line_data ? line_data->raw_adc[4] : 0,
             line_data ? line_data->raw_adc[5] : 0,
             line_data ? line_data->raw_adc[6] : 0,
             line_data ? line_data->raw_adc[7] : 0,
             (line_data && line_data->line_lost) ? "true" : "false",
             (line_data && line_data->intersection_detected) ? "true" : "false");

    if (len > 0) {
        uint8_t frame_hdr[4];
        size_t hdr_len = 0;
        frame_hdr[0] = 0x81; // FIN + Text Frame
        if (len <= 125) {
            frame_hdr[1] = (uint8_t)len;
            hdr_len = 2;
        } else {
            frame_hdr[1] = 126;
            frame_hdr[2] = (uint8_t)((len >> 8) & 0xFF);
            frame_hdr[3] = (uint8_t)(len & 0xFF);
            hdr_len = 4;
        }
        s_ws_client.write(frame_hdr, hdr_len);
        s_ws_client.write((const uint8_t *)json_str, len);
    }

    xSemaphoreGive(s_ws_mutex);
}

void web_server_init(void) {
    if (!s_ws_mutex) {
        s_ws_mutex = xSemaphoreCreateMutex();
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL, 0, WIFI_AP_MAX_CONN);

    Serial.println("=== WI-FI ACCESS POINT DA PHAT ===");
    Serial.printf("Ten Wi-Fi (SSID): %s\n", WIFI_AP_SSID);
    Serial.printf("Mat khau (PASS) : %s\n", WIFI_AP_PASS);
    Serial.print("Dia chi IP      : http://");
    Serial.println(WiFi.softAPIP());

    // 1. Endpoint GET / : Trả về trang Dashboard HTML
    s_server.on("/", HTTP_GET, []() {
        s_server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        s_server.sendHeader("Pragma", "no-cache");
        s_server.sendHeader("Expires", "0");
        s_server.send_P(200, "text/html; charset=utf-8", DASHBOARD_HTML);
    });

    // 2. Endpoint POST /ota : Cập nhật Firmware không dây
    s_server.on("/ota", HTTP_POST,
        []() {
            bool ok = !Update.hasError();
            s_server.send(ok ? 200 : 500, "text/plain", ok ? "OTA Success. Rebooting..." : "OTA Failed");
            if (ok) {
                Serial.println("[OTA] === NAP FIRMWARE OTA THANH CONG! Khoi dong lai... ===");
                delay(1000);
                ESP.restart();
            }
        },
        []() {
            HTTPUpload &upload = s_server.upload();
            if (upload.status == UPLOAD_FILE_START) {
                Serial.printf("[OTA] Bat dau nhan file: %s\n", upload.filename.c_str());
                if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                    Update.printError(Serial);
                }
            } else if (upload.status == UPLOAD_FILE_WRITE) {
                if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                    Update.printError(Serial);
                }
            } else if (upload.status == UPLOAD_FILE_END) {
                if (!Update.end(true)) {
                    Update.printError(Serial);
                }
            }
        }
    );

    s_server.begin();
    s_ws_server.begin();
    Serial.println("[WEB_SERVER] HTTP Server (Port 80) & WebSocket (Port 81) & OTA da san sang tai http://192.168.4.1");
}

/* ==========================================================================
 * 14. HÀM SETUP() & LOOP() CHUẨN ARDUINO IDE
 * ========================================================================== */
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== ROBOT DO LINE TRANH VAT CAN (MECHATRONICS PROJECT - HCMUT) ===");

    // 1. Khởi tạo hệ thống quản lý trạng thái & LED/Nút nhấn
    state_manager_init();

    // 2. Khởi tạo driver ngoại vi điều khiển (TCRT5000, MG90S, DRV8871, Encoder)
    control_system_init();

    // 3. Khởi tạo cảm biến khoảng cách VL53L0X
    obstacle_task_init();

    // 4. Khởi tạo Wi-Fi SoftAP, Web Dashboard, WebSocket & OTA Server
    web_server_init();

    Serial.println("Khoi tao toan bo phan cung hoan tat. Khoi chay cac FreeRTOS Tasks...");

    // 5. Tạo các FreeRTOS Tasks:
    // - Control_Task: Vòng lặp điều khiển thời gian thực (10ms / 100Hz), ghim Core 1
    xTaskCreatePinnedToCore(
        Control_Task,
        "Control_Task",
        4096,
        NULL,
        configMAX_PRIORITIES - 2,
        NULL,
        1 // Core 1
    );

    // - Obstacle_Task: Quét ToF xác định nhánh rẽ trước khi chạy, ghim Core 0
    xTaskCreatePinnedToCore(
        Obstacle_Task,
        "Obstacle_Task",
        4096,
        NULL,
        3,
        NULL,
        0 // Core 0
    );

    // - State_Task: Máy trạng thái, xử lý nút bấm, đèn LED, ghim Core 0
    xTaskCreatePinnedToCore(
        State_Task,
        "State_Task",
        4096,
        NULL,
        2,
        NULL,
        0 // Core 0
    );

    Serial.println("San sang! Nhan nut BOOT (GPIO 0) tren mach hoac START tren Web Dashboard de bat dau.");
}

void loop() {
    // Xử lý HTTP requests và các gói tin điều khiển WebSocket
    s_server.handleClient();
    ws_poll_client();
    vTaskDelay(pdMS_TO_TICKS(5));
}
