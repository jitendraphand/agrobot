// Single-page control panel served by the robot (works offline, no CDN).
#pragma once
#include <Arduino.h>

static const char WEB_UI[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>AgroBot</title>
<style>
:root{--bg:#10160f;--card:#1b2519;--ink:#e9f2e4;--mute:#9db097;--acc:#f28c28;--ok:#4fbf5f;--bad:#e5484d;--line:#2c3a29}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.4 system-ui,sans-serif}
header{display:flex;align-items:center;gap:12px;padding:10px 16px;border-bottom:1px solid var(--line)}
header h1{font-size:18px;margin:0;flex:1}#state{padding:3px 10px;border-radius:12px;background:var(--card);font-weight:600}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:12px;padding:12px;max-width:1100px;margin:auto}
section{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px}
h2{font-size:13px;letter-spacing:.06em;text-transform:uppercase;color:var(--mute);margin:0 0 8px}
button{background:#2a3827;color:var(--ink);border:1px solid var(--line);border-radius:8px;padding:10px 12px;font:inherit;cursor:pointer}
button:active{background:var(--acc);color:#000}.row{display:flex;gap:8px;flex-wrap:wrap;margin:6px 0}
.stop{background:var(--bad);border-color:var(--bad);color:#fff;font-weight:700;flex:1}
#pad{width:220px;height:220px;border-radius:50%;background:#0c110b;border:2px solid var(--line);margin:6px auto;position:relative;touch-action:none}
#knob{width:70px;height:70px;border-radius:50%;background:var(--acc);position:absolute;left:75px;top:75px;pointer-events:none}
label{display:grid;grid-template-columns:78px 1fr 46px;gap:8px;align-items:center;margin:4px 0}
input[type=range]{width:100%}input[type=number]{width:74px;background:#0c110b;color:var(--ink);border:1px solid var(--line);border-radius:6px;padding:6px}
table{width:100%;border-collapse:collapse}td{padding:3px 0;border-bottom:1px solid var(--line)}td:last-child{text-align:right;font-variant-numeric:tabular-nums}
#err{color:var(--bad);min-height:1.2em}#cam{width:100%;border-radius:8px;background:#000;min-height:120px}
small{color:var(--mute)}
</style></head><body>
<header><h1>AgroBot Mk1</h1><span id="state">...</span><button class="stop" style="flex:0 0 auto" onclick="cmd('STOP')">STOP</button></header>
<main>
<section><h2>Drive</h2><div id="pad"><div id="knob"></div></div>
<div class="row"><button onclick="cmd('MOVE 250')">+25 cm</button><button onclick="cmd('MOVE -250')">-25 cm</button>
<button onclick="cmd('TURN 90')">Left 90</button><button onclick="cmd('TURN -90')">Right 90</button></div>
<small>Drag the pad to drive. Releasing it, or losing WiFi, stops the robot.</small></section>

<section><h2>Arm</h2>
<label>Yaw<input type="range" id="j0" min="-135" max="135" value="0"><span id="v0"></span></label>
<label>Shoulder<input type="range" id="j1" min="0" max="180" value="115"><span id="v1"></span></label>
<label>Elbow<input type="range" id="j2" min="-150" max="120" value="-105"><span id="v2"></span></label>
<label>Wrist<input type="range" id="j3" min="-90" max="90" value="-10"><span id="v3"></span></label>
<label>Gripper<input type="range" id="j4" min="0" max="50" value="10"><span id="v4"></span></label>
<div class="row"><button onclick="cmd('ARM HOME')">Home</button><button onclick="cmd('ARM STOW')">Stow</button>
<button onclick="cmd('GRIP OPEN')">Open</button><button onclick="cmd('GRIP CLOSE')">Close</button></div></section>

<section><h2>Field tasks</h2>
<div class="row">Row offset <input type="number" id="rowy" value="335"> mm <small>(+left / -right)</small></div>
<div class="row"><button onclick="cmd('PROBE '+val('rowy'))">Probe + spray if dry</button><button onclick="cmd('PUMP 2')">Spray 2 s</button></div>
<div class="row">Plants <input type="number" id="n" value="3" min="1" max="8"> first <input type="number" id="first" value="600"> spacing <input type="number" id="sp" value="600"> mm</div>
<div class="row"><button onclick="cmd('ROW '+val('n')+' '+val('first')+' '+val('sp')+' '+val('rowy'))">Run row mission</button></div>
<div class="row">Fruit x <input type="number" id="fx" value="190"> y <input type="number" id="fy" value="305"> z <input type="number" id="fz" value="290"> dia <input type="number" id="fd" value="42"></div>
<div class="row"><button onclick="cmd('PICK '+val('fx')+' '+val('fy')+' '+val('fz')+' '+val('fd'))">Pick to basket</button></div>
<div class="row"><button class="stop" onclick="cmd('ESTOP')">E-STOP</button><button onclick="cmd('RESET')">Reset</button></div>
<div id="err"></div></section>

<section><h2>Telemetry</h2><table>
<tr><td>Battery</td><td id="t_batt"></td></tr><tr><td>Soil moisture (live)</td><td id="t_soil"></td></tr>
<tr><td>Soil log</td><td id="t_log"></td></tr><tr><td>Obstacle range</td><td id="t_range"></td></tr>
<tr><td>Air</td><td id="t_air"></td></tr><tr><td>Odometry x / y / heading</td><td id="t_odo"></td></tr>
<tr><td>Gripper position</td><td id="t_tcp"></td></tr><tr><td>Mission step</td><td id="t_step"></td></tr><tr><td>Pump</td><td id="t_pump"></td></tr>
</table></section>

<section><h2>Camera</h2><img id="cam" alt="ESP32-CAM stream">
<div class="row">Stream URL <input id="camurl" style="flex:1;background:#0c110b;color:var(--ink);border:1px solid var(--line);border-radius:6px;padding:6px" value="http://192.168.4.2/stream">
<button onclick="cam()">Show</button></div></section>
</main>
<script>
const $=id=>document.getElementById(id),val=id=>$(id).value;
function cmd(c){return fetch('/cmd?c='+encodeURIComponent(c)).then(r=>r.text()).then(t=>{$('err').textContent=t.startsWith('ERR')?t:'';return t}).catch(()=>{$('err').textContent='link lost'})}
function cam(){$('cam').src=val('camurl')}
// --- joystick ---
let vx=0,wz=0,held=false;const pad=$('pad'),knob=$('knob');
function setKnob(x,y){knob.style.left=(75+x*75)+'px';knob.style.top=(75+y*75)+'px'}
function move(e){const r=pad.getBoundingClientRect();let x=(e.clientX-r.left-110)/75,y=(e.clientY-r.top-110)/75;
const m=Math.hypot(x,y);if(m>1){x/=m;y/=m}setKnob(x,y);vx=Math.round(-y*350);wz=Math.round(-x*70)}
pad.addEventListener('pointerdown',e=>{held=true;pad.setPointerCapture(e.pointerId);move(e)});
pad.addEventListener('pointermove',e=>{if(held)move(e)});
function release(){if(held){held=false;vx=wz=0;setKnob(0,0);cmd('VEL 0 0')}}
pad.addEventListener('pointerup',release);pad.addEventListener('pointercancel',release);
setInterval(()=>{if(held)cmd('VEL '+vx+' '+wz)},150);
// --- arm sliders ---
let armTimer=null,touching=false;
for(let i=0;i<5;i++){const s=$('j'+i);s.addEventListener('input',()=>{touching=true;$('v'+i).textContent=s.value;clearTimeout(armTimer);
armTimer=setTimeout(()=>{(i<4?cmd('ARM J '+[0,1,2,3].map(k=>val('j'+k)).join(' ')):cmd('GRIP '+val('j4'))).then(()=>setTimeout(()=>touching=false,1500))},250)})}
// --- telemetry ---
function poll(){fetch('/status').then(r=>r.json()).then(s=>{
$('state').textContent=s.state;$('state').style.background=s.state=='FAULT'||s.state=='ESTOP'?'var(--bad)':s.state=='IDLE'?'var(--card)':'var(--ok)';
if(s.err&&s.state!='IDLE')$('err').textContent=s.err;
$('t_batt').textContent=s.batt.toFixed(1)+' V';$('t_soil').textContent=s.soil+' %';$('t_log').textContent=s.log.length?s.log.join(' / ')+' %':'-';
$('t_range').textContent=s.range>4?'clear':(s.range*100).toFixed(0)+' cm';$('t_air').textContent=s.temp.toFixed(1)+' C  '+s.hum+' %RH';
$('t_odo').textContent=(s.x*100).toFixed(0)+' cm / '+(s.y*100).toFixed(0)+' cm / '+s.th.toFixed(0)+' deg';
$('t_tcp').textContent=s.tcp.join(', ')+' mm';$('t_step').textContent=s.steps?s.step+' / '+s.steps:'-';$('t_pump').textContent=s.pump?'ON':'off';
if(!touching){for(let i=0;i<4;i++){$('j'+i).value=s.arm[i];$('v'+i).textContent=Math.round(s.arm[i])}$('j4').value=s.grip;$('v4').textContent=Math.round(s.grip)}
}).catch(()=>{$('state').textContent='offline'})}
setInterval(poll,500);poll();
</script></body></html>
)HTML";
