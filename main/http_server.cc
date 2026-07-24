#include "http_server.h"
#include "config.h"
#include "power.h"
#include "servo.h"
#if ENABLE_AUTO_RUN
#include "auto_run.h"
#endif

#include <esp_http_server.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

static const char *TAG = "action_cat";

// ==================== Web Page ====================
static const char kHtml[] = R"raw(
<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Breathe He</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:monospace;background:#1a1a2e;color:#eee;padding:8px;max-width:520px;margin:auto}
h1{text-align:center;font-size:16px;color:#e94560;margin:6px 0}
.card{background:#16213e;border-radius:8px;padding:10px;margin-bottom:8px}
h2{font-size:13px;margin-bottom:5px;color:#4ecca3}
.row{display:flex;gap:6px;align-items:center;flex-wrap:wrap}
.col{flex:1;min-width:60px}
.btn{padding:8px 14px;border:none;border-radius:4px;font-size:13px;cursor:pointer;color:#fff;margin:3px}
.btn-rec{background:#e94560}.btn-stop{background:#555}.btn-copy{background:#4ecca3;color:#000}
.btn-play{background:#ff6b35}.btn-reset{background:#333}
.btn-on{background:#2ecc71;color:#000}
input,select{width:100%;padding:6px;border:1px solid #333;border-radius:4px;background:#0f3460;color:#eee;font-size:13px;margin:2px 0}
input[type=range]{-webkit-appearance:none;width:100%;height:28px;background:linear-gradient(90deg,#0f3460,#e94560);border-radius:4px;margin:4px 0}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:28px;height:28px;background:#e94560;border-radius:50%}
.grp-head{background:#1a2e3a;border-left:3px solid #4ea3cc}
.grp-lh{background:#1a3a2e;border-left:3px solid #4ecca3}
.grp-rh{background:#2e1a3a;border-left:3px solid #a34ecc}
.grp-crawl{background:#2e2a1a;border-left:3px solid #e67e22}
.grp-hands{background:#1a2e1a;border-left:3px solid #f39c12}
.lbl{display:flex;justify-content:space-between;font-size:11px;color:#aaa}
.val{font-size:20px;font-weight:bold;color:#4ecca3;text-align:center;min-width:50px}
.grp-head .val{color:#4ea3cc}
.grp-lh .val{color:#4ecca3}
.grp-rh .val{color:#a34ecc}
pre{background:#0a0a1a;color:#4ecca3;padding:8px;border-radius:4px;font-size:10px;overflow-x:auto;max-height:300px;overflow-y:auto;white-space:pre-wrap;word-break:break-all}
.status{padding:4px;text-align:center;font-size:12px;color:#888;min-height:20px}
.hint{font-size:10px;color:#666;margin-top:2px}
.joy-row{display:flex;gap:12px;align-items:center;justify-content:center}
.joy{background:#0a0f1e;border-radius:50%;cursor:pointer;touch-action:none;display:block}
.joy-info{display:flex;flex-direction:column;gap:6px;min-width:70px}
.joy-lbl{font-size:11px;color:#aaa;text-align:center}
.joy-lbl .val{font-size:22px;font-weight:bold;display:block}
.grp-hands .joy{border:3px solid #f39c12;box-shadow:0 0 12px rgba(243,156,18,0.25)}
.mode-row{display:flex;gap:4px;align-items:center}
.mode-tag{font-size:9px;padding:2px 6px;border-radius:4px;background:#333;color:#888}
.mode-tag.active{background:#2ecc71;color:#000}
</style>
</head>
<body>
<h1>Breathe He</h1>
<div class="status" id="status">就绪</div>

<!-- HEAD -->
<div class="card grp-head">
<h2>Head 头部 <span style="font-size:10px;color:#888">IO15 | 0=左 90=中 180=右</span></h2>
<div class="lbl"><span>头部角度</span><span class="val" id="v0">90°</span></div>
<input type="range" id="s0" min="0" max="180" value="90" oninput="onSlider()">
<div class="row" style="margin-top:4px">
<button class="btn btn-reset" onclick="setHead(0)">0°左</button>
<button class="btn btn-reset" onclick="setHead(90)">90°中</button>
<button class="btn btn-reset" onclick="setHead(180)">180°右</button>
<button class="btn btn-play" onclick="runHeadNod()" id="btnNod">点头</button>
</div>
</div>

<!-- LEFT HAND -->
<div class="card grp-lh">
<h2>Left Hand 左手 <span style="font-size:10px;color:#888">IO16 | 0=上 90=休息 180=爬下</span></h2>
<div class="lbl"><span>左手角度</span><span class="val" id="v1">90°</span></div>
<input type="range" id="s1" min="0" max="180" value="90" oninput="onSlider()">
<div class="row" style="margin-top:4px">
<button class="btn btn-reset" onclick="setLH(0)">0°上</button>
<button class="btn btn-reset" onclick="setLH(90)">休息</button>
<button class="btn btn-reset" onclick="setLH(180)">爬下</button>
</div>
</div>

<!-- RIGHT HAND -->
<div class="card grp-rh">
<h2>Right Hand 右手 <span style="font-size:10px;color:#888">IO17 | 0=爬下 90=休息 180=上</span></h2>
<div class="lbl"><span>右手角度</span><span class="val" id="v2">90°</span></div>
<input type="range" id="s2" min="0" max="180" value="90" oninput="onSlider()">
<div class="row" style="margin-top:4px">
<button class="btn btn-reset" onclick="setRH(0)">爬下</button>
<button class="btn btn-reset" onclick="setRH(90)">休息</button>
<button class="btn btn-reset" onclick="setRH(180)">180°上</button>
</div>
</div>
<!-- HANDS JOYSTICK (crawl control) -->
<div class="card grp-hands">
<h2>双手摇杆 (0-180) <span style="font-size:10px;color:#888">↑=0°上 | ↓=180°爬行 | ←→=侧重</span></h2>
<div class="joy-row">
<canvas class="joy" id="joyHands" width="180" height="180"></canvas>
<div class="joy-info">
<div class="joy-lbl"><span>左手</span><span class="val" id="v1j">90°</span></div>
<div class="joy-lbl"><span>右手</span><span class="val" id="v2j">90°</span></div>
</div>
</div>
<div class="row" style="margin-top:4px">
	<button class="btn btn-reset" onclick="handsJoy.setAngles(0,180)">上</button>
	<button class="btn btn-on" onclick="handsJoy.setAngles(90,90)">休息</button>
	<button class="btn btn-reset" onclick="handsJoy.setAngles(180,0)">爬行</button>
<span style="font-size:9px;color:#888">拖动摇杆: ↑0° ↓180° ←→转弯</span>
</div>
</div>

	<!-- CRAWLING -->
	<div class="card grp-crawl">
	<h2>爬行控制</h2>
	<div class="hint">交替=直行 | 不同起止=转弯 | 摇杆←→控转弯</div>
	<div class="row" style="margin-top:6px">
	<div class="col">
	<label style="font-size:11px">模式</label>
	<select id="crawlMode" onchange="onCrawlParamChange()">
	<option value="alt">交替爬行</option>
	<option value="both">同时爬行</option>
	<option value="lh">左单臂转弯</option>
	<option value="rh">右单臂转弯</option>
	</select>
	</div>
	</div>
	<div class="row" style="margin-top:4px">
	<div class="col">
	<div class="lbl"><span>左起</span><span id="crawlLhStartVal">90</span></div>
	<input type="range" id="crawlLhStart" min="0" max="180" value="90" oninput="onCrawlParamChange()">
	</div>
	<div class="col">
	<div class="lbl"><span>左止</span><span id="crawlLhEndVal">180</span></div>
	<input type="range" id="crawlLhEnd" min="0" max="180" value="180" oninput="onCrawlParamChange()">
	</div>
	</div>
	<div class="row" style="margin-top:4px">
	<div class="col">
	<div class="lbl"><span>右起</span><span id="crawlRhStartVal">90</span></div>
	<input type="range" id="crawlRhStart" min="0" max="180" value="90" oninput="onCrawlParamChange()">
	</div>
	<div class="col">
	<div class="lbl"><span>右止</span><span id="crawlRhEndVal">0</span></div>
	<input type="range" id="crawlRhEnd" min="0" max="180" value="0" oninput="onCrawlParamChange()">
	</div>
	</div>
	<div class="row" style="margin-top:4px">
	<div class="col" style="flex:2">
	<div class="lbl"><span>频率</span><span id="crawlFreqVal">0.5 Hz</span></div>
	<input type="range" id="crawlFreq" min="5" max="300" value="100" oninput="onCrawlParamChange()">
	</div>
	<div class="col" style="flex:0 0 70px">
	<input id="crawlFreqNum" type="number" min="0.05" max="3.0" step="0.05" value="1.0" style="text-align:center" oninput="onCrawlFreqNum()">
	<label style="font-size:9px;color:#888">Hz</label>
	</div>
	</div>
	<div class="hint"><span style="color:#e67e22" id="crawlInfo">周期 2.0s</span></div>
<div class="row" style="margin-top:6px">
<button class="btn btn-on" id="btnCrawl" onpointerdown="toggleCrawl()">▶ 开始爬行</button>
<button class="btn btn-play" onpointerdown="crawlStep()">单步爬行</button>
<button class="btn btn-stop" onpointerdown="stopCrawl()">⏹ 停止</button>
	<button class="btn" id="btnAutoPlay" style="background:#2ecc71;color:#000" onpointerdown="toggleAutoPlay()">自运行动画</button>
	<button class="btn" id="btnHardSwing" style="background:#333;color:#fff" onpointerdown="toggleHardSwing()">曲线: sin²</button>
</div>
<div id="crawlStatus" style="font-size:11px;color:#e67e22;margin-top:4px;min-height:16px"></div>
	<!-- Battery -->
	<div style="margin-top:6px;display:flex;align-items:center;gap:8px">
	<span style="font-size:11px;color:#aaa">🔋 电量</span>
	<span id="batteryLevel" style="font-size:16px;font-weight:bold;color:#4ecca3">--%</span>
	<span id="batteryMv" style="font-size:10px;color:#888">--mV</span>
	</div>
</div>

<!-- PRESETS -->
	<div class="card">
	<h2>快捷预设</h2>
	<div class="row">
	<button class="btn btn-reset" onclick="preset(90,90,90)">全部休息</button>
	<button class="btn btn-reset" onclick="preset(90,180,0)">爬行姿势</button>
	</div>
	<div class="row" style="margin-top:3px">
	<button class="btn btn-reset" onpointerdown="onCrawlPreset('alt',90,180,90,0)">前进(交替)</button>
	<button class="btn btn-reset" onpointerdown="onCrawlPreset('both',90,180,90,0)">前进(同时)</button>
	</div>
	<div class="row" style="margin-top:3px">
	<button class="btn btn-reset" onpointerdown="onCrawlPreset('alt',90,30,90,180)">左转</button>
	<button class="btn btn-reset" onpointerdown="onCrawlPreset('alt',90,180,90,30)">右转</button>
	</div>
	<div class="row" style="margin-top:3px">
	<button class="btn btn-reset" onpointerdown="onCrawlPreset('lh',90,180,90,90)">左单臂</button>
	<button class="btn btn-reset" onpointerdown="onCrawlPreset('rh',90,90,90,0)">右单臂</button>
	</div>
	</div>

<!-- RECORDING -->
<div class="card">
<h2>录制设置</h2>
<div class="row">
<div class="col"><label style="font-size:11px">时长(秒)</label>
<input id="duration" type="number" value="3" min="1" max="30" onchange="onDurChange()"></div>
<div class="col"><label style="font-size:11px">步数</label>
<input id="steps" type="number" value="30" min="1" max="300" readonly></div>
</div>
<div class="row" style="margin-top:6px">
<input id="name" placeholder="动作名称(英文)" style="flex:1" value="my_action">
<button class="btn btn-rec" id="btnRec" onpointerdown="toggleRecord()">开始录制</button>
</div>
<div class="hint">录制时每100ms记录一帧 (3通道: 头/左手/右手)</div>
</div>

<div class="card" id="outputCard" style="display:none">
<h2>生成代码 <button class="btn btn-copy" onpointerdown="copyCode()">复制</button>
<button class="btn btn-play" onpointerdown="playRecord()">播放</button></h2>
<pre id="code"></pre>
<div class="row"><span style="font-size:10px;color:#888" id="info"></span></div>
</div>

<script>
let recording = false, frames = [], timer = null;
let crawlRunning = false;
let crawlTimer = null;

const REST_H = 90, REST_LH = 90, REST_RH = 90;

function $(id){return document.getElementById(id)}

function onDurChange(){
  let sec = parseInt($('duration').value) || 3;
  $('steps').value = sec * 10;
}
onDurChange();

async function api(url, body){
  try{
    let o = body ? {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)} : {method:'GET'};
    let r = await fetch(url, o);
    return await r.text();
  }catch(e){ setStatus('连接失败','#e94560'); return null; }
}

function getAngles(){
  return [
    parseInt($('s0').value),
    parseInt($('s1').value),
    parseInt($('s2').value)
  ];
}

async function sendAngles(){
  let a = getAngles();
  $('v0').textContent = a[0]+'°';
  $('v1').textContent = a[1]+'°';
  $('v2').textContent = a[2]+'°';
  $('v1j').textContent = a[1]+'°';
  $('v2j').textContent = a[2]+'°';
  await api('/api/servo', {angles:a});
  if(recording) frames.push([...a]);
}

class HandsJoystick {
  constructor(canvasId){
    this.canvas = $(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.cx = 90; this.cy = 90; this.r = 78;
    this.kx = 90; this.ky = 90; this.kr = 22;
    this.lh = REST_LH; this.rh = REST_RH;
    this.active = false;
    this.animId = 0;
    this.canvas.addEventListener('pointerdown', e => { cancelAnimationFrame(this.animId); this.active = true; this.move(e); });
    this.canvas.addEventListener('pointermove', e => { if(this.active) this.move(e); });
    this.canvas.addEventListener('pointerup', () => { this.active = false; this.springBack(); });
    this.canvas.addEventListener('pointerleave', () => { this.active = false; this.springBack(); });
    this.canvas.addEventListener('pointercancel', () => { this.active = false; this.springBack(); });
    this.draw();
  }
  move(e){
    e.preventDefault();
    let rect = this.canvas.getBoundingClientRect();
    let sx = this.canvas.width / rect.width;
    let sy = this.canvas.height / rect.height;
    let mx = (e.clientX - rect.left) * sx;
    let my = (e.clientY - rect.top) * sy;
    let dx = mx - this.cx, dy = my - this.cy;
    let dist = Math.sqrt(dx*dx + dy*dy);
    if(dist > this.r) { dx *= this.r / dist; dy *= this.r / dist; }
    this.kx = this.cx + dx;
    this.ky = this.cy + dy;
    this.updateAngles();
    this.draw();
  }
  springBack(){
    let ease = t => t < 0.5 ? 4*t*t*t : 1 - Math.pow(-2*t + 2, 3) / 2;
    let sx = this.kx, sy = this.ky;
    let dx = this.cx - sx, dy = this.cy - sy;
    if(Math.abs(dx) < 0.3 && Math.abs(dy) < 0.3){
      this.kx = this.cx; this.ky = this.cy;
      this.lh = REST_LH; this.rh = REST_RH;
      this.updateAngles();
      this.draw();
      this.animId = 0;
      return;
    }
    let t0 = performance.now();
    let duration = 180;
    let step = (now) => {
      let t = Math.min(1, (now - t0) / duration);
      this.kx = sx + dx * ease(t);
      this.ky = sy + dy * ease(t);
      this.updateAngles();
      this.draw();
      if(t < 1) this.animId = requestAnimationFrame(step);
      else {
        this.kx = this.cx; this.ky = this.cy;
        this.lh = REST_LH; this.rh = REST_RH;
        this.updateAngles();
        this.draw();
        this.animId = 0;
      }
    };
    this.animId = requestAnimationFrame(step);
  }
  updateAngles(){
    let dy = -(this.ky - this.cy) / this.r;
    let dx = (this.kx - this.cx) / this.r;

    let lhBase = REST_LH - Math.round(dy * 90);
    let rhBase = REST_RH + Math.round(dy * 90);

    let lhDisp = lhBase - REST_LH;
    let rhDisp = rhBase - REST_RH;
    if(dx < 0) { lhDisp *= (1 + dx); }
    if(dx > 0) { rhDisp *= (1 - dx); }

    this.lh = REST_LH + Math.round(lhDisp);
    this.rh = REST_RH + Math.round(rhDisp);
    this.lh = Math.max(0, Math.min(180, this.lh));
    this.rh = Math.max(0, Math.min(180, this.rh));

    $('s1').value = this.lh; $('s2').value = this.rh;
    $('v1').textContent = this.lh + '°'; $('v2').textContent = this.rh + '°';
    $('v1j').textContent = this.lh + '°'; $('v2j').textContent = this.rh + '°';
    sendAngles();
  }
  draw() {
    let c = this.ctx, w = this.canvas.width, h = this.canvas.height;
    c.clearRect(0, 0, w, h);
    c.beginPath(); c.arc(this.cx, this.cy, this.r, 0, Math.PI*2);
    c.strokeStyle = 'rgba(255,255,255,0.2)'; c.lineWidth = 2; c.stroke();
    c.beginPath(); c.moveTo(this.cx-this.r, this.cy); c.lineTo(this.cx+this.r, this.cy);
    c.moveTo(this.cx, this.cy-this.r); c.lineTo(this.cx, this.cy+this.r);
    c.strokeStyle = 'rgba(255,255,255,0.08)'; c.lineWidth = 1; c.stroke();
    c.font = '10px monospace';
    c.fillStyle = '#4ecca3'; c.fillText('0°上', this.cx-10, this.cy-this.r+12);
    c.fillStyle = '#e67e22'; c.fillText('180°爬', this.cx-10, this.cy+this.r-4);
    c.fillStyle = '#888'; c.fillText('LH', this.cx-this.r+2, this.cy+4);
    c.fillText('RH', this.cx+this.r-16, this.cy+4);
    c.beginPath(); c.arc(this.cx, this.cy, 4, 0, Math.PI*2);
    c.fillStyle = 'rgba(255,255,255,0.1)'; c.fill();
    c.beginPath(); c.arc(this.kx, this.ky, this.kr, 0, Math.PI*2);
    let grad = c.createRadialGradient(this.kx-5, this.ky-5, 3, this.kx, this.ky, this.kr);
    grad.addColorStop(0, '#f39c12'); grad.addColorStop(1, '#e67e22');
    c.fillStyle = grad; c.fill();
    c.strokeStyle = 'rgba(255,255,255,0.4)'; c.lineWidth = 2; c.stroke();
  }
  setAngles(lh, rh){
    cancelAnimationFrame(this.animId);
    let lhOff = (lh - REST_LH) / 90;
    let rhOff = (rh - REST_RH) / 90;
    let dy = (Math.abs(lhOff) > Math.abs(rhOff)) ? -lhOff : rhOff;
    let absL = Math.abs(lhOff), absR = Math.abs(rhOff);
    let dx = (absL < absR) ? -(1 - absL / Math.max(0.01, absR))
           : (absR < absL) ? (1 - absR / Math.max(0.01, absL))
           : 0;
    dy = Math.max(-1, Math.min(1, dy));
    dx = Math.max(-1, Math.min(1, dx));
    this.kx = this.cx + dx * this.r;
    this.ky = this.cy + dy * this.r;
    this.lh = lh; this.rh = rh;
    this.draw();
  }
}
let handsJoy = new HandsJoystick('joyHands');

function onSlider(){ sendAngles(); }

function setHead(a){ $('s0').value = a; sendAngles(); }
function setLH(a){ $('s1').value = a; handsJoy.setAngles(a, parseInt($('s2').value)); sendAngles(); }
function setRH(a){ $('s2').value = a; handsJoy.setAngles(parseInt($('s1').value), a); sendAngles(); }

function preset(h, lh, rh){
  $('s0').value = h; $('s1').value = lh; $('s2').value = rh;
  handsJoy.setAngles(lh, rh);
  sendAngles();
}

function calcCrawlAngles(phase){
  let mode = window._crawlMode;
  let lhS = window._crawlLhS, lhE = window._crawlLhE;
  let rhS = window._crawlRhS, rhE = window._crawlRhE;

  let val_lh = Math.sin(phase * Math.PI);
  val_lh = val_lh * val_lh;

  let phase_rh = (mode === 'alt') ? (phase + 0.5) % 1 : phase;
  let val_rh = Math.sin(phase_rh * Math.PI);
  val_rh = val_rh * val_rh;

  let lh, rh;
  if (mode === 'lh') {
    lh = lhS + (lhE - lhS) * val_lh;
    rh = REST_RH;
  } else if (mode === 'rh') {
    lh = REST_LH;
    rh = rhS + (rhE - rhS) * val_rh;
  } else {
    lh = lhS + (lhE - lhS) * val_lh;
    rh = rhS + (rhE - rhS) * val_rh;
  }
  return [Math.round(lh), Math.round(rh)];
}

function onCrawlParamChange(){
  let hzX100 = parseInt($('crawlFreq').value);
  let hz = hzX100 / 100;
  let lhS = parseInt($('crawlLhStart').value);
  let lhE = parseInt($('crawlLhEnd').value);
  let rhS = parseInt($('crawlRhStart').value);
  let rhE = parseInt($('crawlRhEnd').value);
  let cycleTime = (1 / hz).toFixed(2);

  $('crawlFreqVal').textContent = hz.toFixed(2) + ' Hz';
  $('crawlFreqNum').value = hz.toFixed(1);
  $('crawlLhStartVal').textContent = lhS;
  $('crawlLhEndVal').textContent = lhE;
  $('crawlRhStartVal').textContent = rhS;
  $('crawlRhEndVal').textContent = rhE;
  $('crawlInfo').textContent = '周期 ' + cycleTime + 's | L:'+lhS+'->'+lhE+' R:'+rhS+'->'+rhE;

  window._crawlMode = $('crawlMode').value;
  window._crawlLhS = lhS; window._crawlLhE = lhE;
  window._crawlRhS = rhS; window._crawlRhE = rhE;
  window._crawlPeriodS = 1 / hz;
}

function onCrawlPreset(mode, lhS, lhE, rhS, rhE){
  $('crawlMode').value = mode;
  $('crawlLhStart').value = lhS; $('crawlLhEnd').value = lhE;
  $('crawlRhStart').value = rhS; $('crawlRhEnd').value = rhE;
  onCrawlParamChange();
  if(crawlRunning) stopCrawl();
  toggleCrawl();
}

function onCrawlFreqNum(){
  let hz = parseFloat($('crawlFreqNum').value);
  if(isNaN(hz) || hz < 0.05) hz = 0.05;
  if(hz > 3.0) hz = 3.0;
  $('crawlFreq').value = Math.round(hz * 100);
  onCrawlParamChange();
}

window._crawlMode = 'alt';
window._crawlLhS = 90; window._crawlLhE = 180;
window._crawlRhS = 90; window._crawlRhE = 0;
window._crawlPeriodS = 1.0;
onCrawlParamChange();

async function toggleCrawl(){
  if(crawlRunning){ stopCrawl(); return; }
  crawlRunning = true;
  $('btnCrawl').textContent = '⏸ 停止爬行';
  $('btnCrawl').className = 'btn btn-stop';
  let mode = $('crawlMode').value;
  let modeNames = {alt:'交替', both:'同时', lh:'左单臂转弯', rh:'右单臂转弯'};
  $('crawlStatus').textContent = '爬行中(本地): ' + (modeNames[mode]||mode);

  let hzX100 = parseInt($('crawlFreq').value);
  let hz = hzX100 / 100;
  let lhS = parseInt($('crawlLhStart').value);
  let lhE = parseInt($('crawlLhEnd').value);
  let rhS = parseInt($('crawlRhStart').value);
  let rhE = parseInt($('crawlRhEnd').value);
  await api('/api/crawl', {
    mode: mode,
    lh_start: lhS, lh_end: lhE,
    rh_start: rhS, rh_end: rhE,
    period: 1 / hz
  });
}

function stopCrawl(){
  crawlRunning = false;
  $('btnCrawl').textContent = '▶ 开始爬行';
  $('btnCrawl').className = 'btn btn-on';
  $('crawlStatus').textContent = '已停止';
  api('/api/crawl', {action:'stop'});
}

async function crawlStep(){
  if (crawlRunning) stopCrawl();
  await new Promise(r => setTimeout(r, 100));
  toggleCrawl();
  let hzX100 = parseInt($('crawlFreq').value);
  let hz = hzX100 / 100;
  let periodMs = (1 / hz) * 1000;
  await new Promise(r => setTimeout(r, periodMs));
  stopCrawl();
  preset(REST_H, REST_LH, REST_RH);
}

let nodding = false;

async function runHeadNod(){
  if(nodding){ nodding = false; return; }
  nodding = true;
  while(nodding){
    let t = ((new Date()).getTime() / 1000) % 2 / 2;
    let a = Math.round(REST_H + Math.sin(t * Math.PI * 2) * 20);
    $('s0').value = a;
    sendAngles();
    await new Promise(r => setTimeout(r, 50));
  }
  $('s0').value = REST_H;
  sendAngles();
}

async function toggleRecord(){
  if(!recording){
    let name = $('name').value.trim();
    if(!name){ setStatus('请输入动作名称','#e94560'); return; }
    frames = [];
    recording = true;
    $('btnRec').style.background = '#555';
    $('outputCard').style.display = 'none';
    setStatus('录制中... 摇动摇杆做动作','#4ecca3');
    let step = 0;
    let total = parseInt($('steps').value) || 30;
    frames.push(getAngles());
    step++;
    setStatus('录制中... '+step+'/'+total,'#4ecca3');
    timer = setInterval(async () => {
      let a = getAngles();
      frames.push([...a]);
      step++;
      setStatus('录制中... '+step+'/'+total,'#4ecca3');
      if(step >= total) toggleRecord();
    }, 100);
  } else {
    recording = false;
    if(timer) clearInterval(timer);
    timer = null;
    $('btnRec').style.background = '#e94560';
    let total = parseInt($('steps').value) || 30;
    if(frames.length > total) frames = frames.slice(0, total);
    while(frames.length < total) frames.push([...frames[frames.length-1]]);
    generateCode();
    setStatus('录制完成: '+frames.length+'帧','#4ecca3');
  }
}

function capitalize(s){
  return s.charAt(0).toUpperCase() + s.slice(1).replace(/[^a-zA-Z0-9_]/g,'_');
}

function generateCode(){
  let name = $('name').value.trim() || 'my_action';
  let totalSteps = frames.length;
  let duration = (totalSteps * 0.1).toFixed(1);
  let code = '// action_' + name + ': ' + totalSteps + ' steps = ' + duration + 's\n';
  code += '// [Head, LeftHand, RightHand]\n';
  code += 'static const ServoActionStep<3> kAction' + capitalize(name) + 'Steps[] = {\n';
  let lines = [];
  for(let i=0; i<frames.length; i++){
    let a = frames[i];
    lines.push('    {{' + a[0] + ',' + a[1] + ',' + a[2] + '}}');
  }
  code += lines.join(',\n') + ',\n';
  code += '};\n';
  code += 'static const ServoActionSeries kAction' + capitalize(name) + 'Series[] = {\n';
  code += '    {kAction' + capitalize(name) + 'Steps[0].angles, ' + totalSteps + '},\n';
  code += '};\n';
  $('outputCard').style.display = 'block';
}

function copyCode(){
  navigator.clipboard.writeText(code).then(()=>setStatus('已复制到剪贴板!','#4ecca3'));
}

async function playRecord(){
  if(frames.length==0) return;
  setStatus('播放中...','#ff6b35');
  for(let i=0; i<frames.length; i++){
    let a = frames[i];
    $('s0').value=a[0]; $('s1').value=a[1]; $('s2').value=a[2];
    await api('/api/servo', {angles:a});
    handsJoy.setAngles(a[1], a[2]);
    await new Promise(r=>setTimeout(r,100));
  }
  setStatus('播放完毕','#4ecca3');
}

async function updateBattery(){
  let r = await api('/api/battery');
  if(r){
    try{
      let b = JSON.parse(r);
      $('batteryLevel').textContent = b.level + '%';
      $('batteryMv').textContent = (b.voltage_mv/1000).toFixed(2) + 'V';
      let c = b.level > 20 ? '#4ecca3' : '#e94560';
      $('batteryLevel').style.color = c;
    }catch(e){}
  }
}
setInterval(updateBattery, 5000);
updateBattery();

async function toggleAutoPlay(){
  let r = await api('/api/autoplay', {});
  if(r){ try{ let s = JSON.parse(r); refreshAutoPlayUI(s); }catch(e){} }
}
async function toggleHardSwing(){
  let r0 = await api('/api/autoplay');
  let cur = false;
  if(r0){ try{ cur = JSON.parse(r0).hard_swing; }catch(e){} }
  let r = await api('/api/autoplay', {hard_swing: !cur});
  if(r){ try{ let s = JSON.parse(r); refreshAutoPlayUI(s); }catch(e){} }
}
function refreshAutoPlayUI(s){
  let btn = $('btnAutoPlay');
  let on = s.autoplay;
  btn.textContent = '自运行动画: ' + (on ? 'ON' : 'OFF');
  btn.style.background = on ? '#2ecc71' : '#555';
  btn.style.color = on ? '#000' : '#fff';

  let hbtn = $('btnHardSwing');
  let hard = s.hard_swing;
  hbtn.textContent = '曲线: ' + (hard ? '硬摆' : 'sin²');
  hbtn.style.background = hard ? '#e67e22' : '#333';
}
(async function(){
  let r = await api('/api/autoplay');
  if(r){ try{ let s = JSON.parse(r); refreshAutoPlayUI(s); }catch(e){} }
})();
</script>
</body>
</html>
)raw";

// ==================== HTTP Handlers ====================

static esp_err_t HandleRoot(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_send(req, kHtml, strlen(kHtml));
  return ESP_OK;
}

static esp_err_t HandleServo(httpd_req_t *req)
{
  char buf[256] = {};
  int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (ret <= 0) { httpd_resp_send_500(req); return ESP_FAIL; }
  buf[ret] = 0;

  int angles[3] = {90, 90, 90};
  const char *p = strstr(buf, "\"angles\":");
  if (p)
  {
    p += 9;
    for (int i = 0; i < kServoCount; i++)
    {
      while (*p == ' ' || *p == '[' || *p == ',') p++;
      angles[i] = atoi(p);
      while (*p && *p != ',' && *p != ']') p++;
    }
  }
  for (int i = 0; i < kServoCount; i++)
    SetServoAngle(i, angles[i]);

#if ENABLE_AUTO_RUN
  if (IsAutoRunRunning())
  {
    SetAutoRunRunning(false);
    ESP_LOGI(TAG, "Auto-play paused by manual servo command");
  }
#endif

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_sendstr(req, "OK");
  return ESP_OK;
}

static esp_err_t HandleBattery(httpd_req_t *req)
{
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"voltage_mv\":%d,\"level\":%d}",
           GetBatteryVoltageMv(), GetBatteryLevel());
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, buf);
  return ESP_OK;
}

// ==================== Web-triggered crawl task ====================

static volatile bool web_crawl_running_ = false;
static float web_crawl_period_s_ = 2.0f;
static int web_crawl_lh_start_ = 90, web_crawl_lh_end_ = 30;
static int web_crawl_rh_start_ = 90, web_crawl_rh_end_ = 180;
static char web_crawl_mode_[8] = "alt";

static void WebCrawlTask(void *arg)
{
  constexpr int kTickMs = 20;
  uint32_t tick = 0;
  ESP_LOGI(TAG, "Web crawl start: mode=%s period=%.1fs L:%d->%d R:%d->%d",
           web_crawl_mode_, (double)web_crawl_period_s_,
           web_crawl_lh_start_, web_crawl_lh_end_,
           web_crawl_rh_start_, web_crawl_rh_end_);

  while (web_crawl_running_)
  {
#if ENABLE_AUTO_RUN
    SetAutoRunRunning(false);  // Pause auto-run to avoid servo conflict
#endif
    float t_s = tick * kTickMs / 1000.0f;
    float eff_period = IsAutoRunHardSwing() ? web_crawl_period_s_ / HARD_SWING_SPEED_X : web_crawl_period_s_;
    float phase = t_s / eff_period;
    phase = phase - floorf(phase);

    float val_lh, val_rh;
    if (IsAutoRunHardSwing()) {
      bool at_end = (phase >= 0.5f);
      val_lh = at_end ? 1.0f : 0.0f;
      bool is_alt = (strcmp(web_crawl_mode_, "alt") == 0);
      bool rh_at_end = is_alt ? !at_end : at_end;
      val_rh = rh_at_end ? 1.0f : 0.0f;
    } else {
      val_lh = sinf(phase * M_PI);
      val_lh = val_lh * val_lh;
      float phase_rh = (strcmp(web_crawl_mode_, "alt") == 0) ? (phase + 0.5f) : phase;
      phase_rh = phase_rh - floorf(phase_rh);
      val_rh = sinf(phase_rh * M_PI);
      val_rh = val_rh * val_rh;
    }

    int lh, rh;
    if (strcmp(web_crawl_mode_, "lh") == 0) {
      lh = web_crawl_lh_start_ + (int)((web_crawl_lh_end_ - web_crawl_lh_start_) * val_lh);
      rh = 90;
    } else if (strcmp(web_crawl_mode_, "rh") == 0) {
      lh = 90;
      rh = web_crawl_rh_start_ + (int)((web_crawl_rh_end_ - web_crawl_rh_start_) * val_rh);
    } else {
      lh = web_crawl_lh_start_ + (int)((web_crawl_lh_end_ - web_crawl_lh_start_) * val_lh);
      rh = web_crawl_rh_start_ + (int)((web_crawl_rh_end_ - web_crawl_rh_start_) * val_rh);
    }

    SetServoAngle(1, lh);
    SetServoAngle(2, rh);

    tick++;
    vTaskDelay(pdMS_TO_TICKS(kTickMs));
  }

  SetServoAngle(1, 90);
  SetServoAngle(2, 90);
  ESP_LOGI(TAG, "Web crawl stopped");
  vTaskDelete(nullptr);
}

static esp_err_t HandleCrawl(httpd_req_t *req);

// ==================== Server startup ====================

void StartHttpServer()
{
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.max_uri_handlers = 10;
  httpd_handle_t server = nullptr;
  httpd_start(&server, &cfg);

  httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = HandleRoot, .user_ctx = nullptr};
  httpd_register_uri_handler(server, &root);

  httpd_uri_t servo = {.uri = "/api/servo", .method = HTTP_POST, .handler = HandleServo, .user_ctx = nullptr};
  httpd_register_uri_handler(server, &servo);

  httpd_uri_t battery = {.uri = "/api/battery", .method = HTTP_GET, .handler = HandleBattery, .user_ctx = nullptr};
  httpd_register_uri_handler(server, &battery);

#if ENABLE_AUTO_RUN
  httpd_uri_t autoplay_get = {.uri = "/api/autoplay", .method = HTTP_GET, .handler = HandleAutoPlay, .user_ctx = nullptr};
  httpd_uri_t autoplay_post = {.uri = "/api/autoplay", .method = HTTP_POST, .handler = HandleAutoPlay, .user_ctx = nullptr};
  httpd_register_uri_handler(server, &autoplay_get);
  httpd_register_uri_handler(server, &autoplay_post);
#endif

  httpd_uri_t crawl = {.uri = "/api/crawl", .method = HTTP_POST, .handler = HandleCrawl, .user_ctx = nullptr};
  httpd_register_uri_handler(server, &crawl);

  ESP_LOGI(TAG, "HTTP server started on http://192.168.4.1");
}

// ==================== HandleCrawl (needs WebCrawlTask defined) ====================

static esp_err_t HandleCrawl(httpd_req_t *req)
{
  char buf[256] = {};
  int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (ret <= 0) { httpd_resp_send_500(req); return ESP_FAIL; }
  buf[ret] = 0;

  if (strstr(buf, "\"action\":\"stop\""))
  {
    web_crawl_running_ = false;
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
  }

  auto parse_int = [&](const char *key, int def) -> int {
    const char *p = strstr(buf, key);
    if (p) { p += strlen(key) + 2; return atoi(p); }
    return def;
  };
  auto parse_float = [&](const char *key, float def) -> float {
    const char *p = strstr(buf, key);
    if (p) { p += strlen(key) + 2; return atof(p); }
    return def;
  };

  const char *mode_p = strstr(buf, "\"mode\":\"");
  if (mode_p)
  {
    mode_p += 8;
    strncpy(web_crawl_mode_, mode_p, sizeof(web_crawl_mode_) - 1);
    char *q = strchr(web_crawl_mode_, '"');
    if (q) *q = 0;
  }

  web_crawl_lh_start_ = parse_int("\"lh_start\"", 90);
  web_crawl_lh_end_   = parse_int("\"lh_end\"", 30);
  web_crawl_rh_start_ = parse_int("\"rh_start\"", 90);
  web_crawl_rh_end_   = parse_int("\"rh_end\"", 180);
  web_crawl_period_s_  = parse_float("\"period\"", 2.0f);
  if (web_crawl_period_s_ < 0.3f) web_crawl_period_s_ = 0.3f;
  if (web_crawl_period_s_ > 20.0f) web_crawl_period_s_ = 20.0f;

  web_crawl_running_ = false;
  vTaskDelay(pdMS_TO_TICKS(50));

  web_crawl_running_ = true;
  xTaskCreate(WebCrawlTask, "web_crawl", 4096, nullptr, 3, nullptr);

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_sendstr(req, "OK");
  return ESP_OK;
}
