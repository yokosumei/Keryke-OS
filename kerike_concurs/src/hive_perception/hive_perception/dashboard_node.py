#!/usr/bin/env python3
"""
HIVE Dashboard v4 — distance bar pe dreapta, listening banner pentru wake word.
"""
import json
import queue
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from flask import Flask, Response, jsonify, render_template_string
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

# Feed la 2Hz, nu imagine bruta continua (cost de banda/CPU) -- vezi ARHITECTURA_SISTEM.md.
# Detectia/reactia reala (YOLO, spatial_risk_node) ruleaza la rata proprie pe
# topicurile ROS, independent de cat de des redeseneaza dashboard-ul JPEG-uri
# pentru afisare -- incetinirea asta NU incetineste sistemul real.
DASHBOARD_FEED_PERIOD_S = 0.5

app = Flask(__name__)

# Modurile in care poate fi feed-ul YOLO, ciclate de /toggle (segmentarea
# ruleaza mereu separat, in panelul propriu -- vezi seg_jpeg -- nu mai e
# nevoie sa alegi intre yolo si seg, le vezi pe amandoua simultan):
#   yolo -> bbox-uri din yolov8n.pt (modelul din models/, /perception/detections_yolo)
#   raw  -> camera fara overlay
_MODES = ["yolo", "raw"]

# Interval metric adancime pt colorizare -- acelasi clamp ca in
# depth_node.py (DEPTH_CLAMP_MIN_M / DEPTH_CLAMP_MAX_M).
_DEPTH_MIN_M = 0.15
_DEPTH_MAX_M = 10.0

_state = {
    "frame_jpeg": None,
    "seg_jpeg": None,
    "depth_jpeg": None,
    "lock": threading.Lock(),
    "mode": "yolo",
    "frames": 0,
    "last_classes": [],
    "event_queue": queue.Queue(maxsize=200),
}


def colorize_depth(depth_m: np.ndarray) -> np.ndarray:
    """Normalizeaza adancimea metrica [_DEPTH_MIN_M, _DEPTH_MAX_M] -> colormap JET."""
    clipped = np.clip(depth_m, _DEPTH_MIN_M, _DEPTH_MAX_M)
    norm = ((clipped - _DEPTH_MIN_M) / (_DEPTH_MAX_M - _DEPTH_MIN_M) * 255.0).astype(np.uint8)
    return cv2.applyColorMap(norm, cv2.COLORMAP_JET)


PAGE = r"""<!doctype html>
<html lang="ro"><head><meta charset="utf-8">
<title>Keryke-OS Dashboard</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<style>
  :root {
    --accent:#ffd700; --bg:#0a0a0a; --panel:#15151a; --border:#2a2a30;
    --text:#e1e1e6; --muted:#888; --danger:#e24b4a; --ok:#04d361;
    --warn:#ff8800;
  }
  * { box-sizing: border-box; }
  html, body { height:100%; }
  body { background:var(--bg); color:var(--text);
         font-family:'Segoe UI',sans-serif; margin:0; padding:10px;
         display:flex; flex-direction:column; overflow:hidden; }
  h1 { color:var(--accent); text-align:center; margin:0 0 8px 0;
       font-size:1.2rem; flex:0 0 auto; }
  .controls { text-align:center; margin:0 0 8px 0; flex:0 0 auto; }
  button { padding:8px 14px; background:var(--accent); color:#000;
           border:none; border-radius:4px; cursor:pointer;
           margin-right:6px; font-weight:600; }
  button.off { background:#444; color:#aaa; }

  .page { width:100%; flex:1 1 auto; min-height:0;
          display:flex; flex-direction:column; gap:10px; }

  .row-video, .row-gauges, .row-logs {
    display:flex; gap:10px; min-height:0;
  }
  .row-video  { flex:5 1 0; }
  .row-gauges { flex:2 1 0; }
  .row-logs   { flex:2 1 0; }

  .row-video  .panel { flex:1; min-width:0; display:flex; flex-direction:column; }
  .row-gauges .panel { flex:1; min-width:150px; display:flex; flex-direction:column; }
  .row-logs   .panel { flex:1; min-width:0; display:flex; flex-direction:column; }

  .panel { background:var(--panel); border:1px solid var(--border);
           border-radius:8px; padding:10px; min-height:0; }
  .panel h2 { margin:0 0 6px 0; font-size:0.85rem; color:var(--ok);
              text-transform:uppercase; letter-spacing:1px; font-weight:600;
              flex:0 0 auto; }
  .panel img { width:100%; flex:1 1 auto; min-height:0; border-radius:4px;
               display:block; object-fit:contain; background:#050507; }
  canvas { background:#050507; border-radius:4px; display:block;
           width:100%; flex:1 1 auto; min-height:0; }

  #imu-container { width:100%; flex:1 1 auto; min-height:0; background:#050507;
                    border-radius:4px; overflow:hidden; }

  .stats { font-family:monospace; color:var(--muted); font-size:0.75rem;
           padding-top:4px; line-height:1.3; flex:0 0 auto; }
  .speech-log { background:#050507; padding:8px; border-radius:4px;
                flex:1 1 auto; min-height:0; overflow-y:auto;
                font-family:monospace; font-size:0.8rem; text-align:left; }
  .speech-log .entry { padding:2px 0; border-bottom:1px solid #1a1a20; }

  .listening-banner {
    background: linear-gradient(90deg, #04d361, #ffd700);
    color: #000; padding: 8px; text-align: center;
    font-weight: bold; border-radius: 6px; display: none;
    margin-bottom: 6px; font-size: 1rem; flex:0 0 auto;
  }

  .alert { background:var(--danger); color:white; padding:8px; text-align:center;
           font-weight:bold; border-radius:6px; display:none; margin-bottom:6px;
           flex:0 0 auto; }

  .dist-big {
    font-family:monospace; font-size:1.5rem; font-weight:bold;
    text-align:center; padding:2px 0; color:var(--ok); flex:0 0 auto;
  }
  .dist-big.danger { color:var(--danger); }
  .dist-big.warn { color:var(--warn); }

  @media (max-width: 1100px) {
    html, body { height:auto; }
    body { overflow:auto; }
    .row-video, .row-gauges, .row-logs { flex-direction: column; }
    .row-gauges .panel { min-width:0; }
    .row-video .panel img, canvas, .speech-log { height:280px; }
  }
</style></head>
<body>
  <h1>Keryke-OS — Perception Dashboard</h1>
  <div class="controls">
    <button onclick="fetch('/toggle')">Toggle YOLO (yolo -&gt; raw)</button>
    <button id="speech-btn" onclick="toggleSpeech()">Speech: ON</button>
  </div>

  <div class="listening-banner" id="listen-banner">ASCULT... vorbește acum</div>
  <div class="alert" id="alert-banner">ATENȚIE — OBSTACOL FOARTE APROAPE!</div>
  <div class="alert" id="walkable-banner">Atenție! Nu ești pe zona sigură.</div>

  <div class="page">
    <!-- RAND 1: VIDEO, latime completa -->
    <div class="row-video">
      <div class="panel">
        <h2>Live Feed (yolo / raw)</h2>
        <img src="/video">
        <div class="stats" id="stats">Loading...</div>
      </div>
      <div class="panel">
        <h2>Segmentare (zona sigura)</h2>
        <img src="/seg_video">
      </div>
      <div class="panel">
        <h2>Adancime</h2>
        <img src="/depth_video">
      </div>
    </div>

    <!-- RAND 2: GAUGE-URI, compact -->
    <div class="row-gauges">
      <div class="panel">
        <h2>Decizie / Servo</h2>
        <canvas id="servo-gauge-canvas" width="500" height="90"></canvas>
        <div class="stats" id="servo-reason">Fara date inca.</div>
      </div>
      <div class="panel">
        <h2>Distance (2s)</h2>
        <div class="dist-big" id="dist-big">— mm</div>
        <canvas id="dist-bar-canvas" width="500" height="90"></canvas>
      </div>
      <div class="panel">
        <h2>TOF Radar</h2>
        <canvas id="tof-canvas" width="500" height="160"></canvas>
      </div>
      <div class="panel">
        <h2>IMU 3D</h2>
        <div id="imu-container"></div>
      </div>
      <div class="panel">
        <h2>Yaw Pendulum</h2>
        <canvas id="pendulum-canvas" width="500" height="140"></canvas>
      </div>
      <div class="panel">
        <h2>Vesta haptica</h2>
        <canvas id="vest-canvas" width="500" height="90"></canvas>
        <div class="stats" id="vest-status">Fara date inca.</div>
      </div>
    </div>

    <!-- RAND 3: LOG-URI, jos de tot -->
    <div class="row-logs">
      <div class="panel">
        <h2>Speech Log</h2>
        <div class="speech-log" id="speech-log"></div>
      </div>
      <div class="panel">
        <h2>Alerte audio (YAMNet)</h2>
        <div class="speech-log" id="audio-log"></div>
      </div>
    </div>
  </div>

<script>
  // ===== CANVAS RESIZE =====
  // Layout-ul e acum flex/vh (umple ecranul), nu px fix -- canvas-urile
  // au atribute width/height initiale mici (500x90 etc.) folosite ca
  // sistem de coordonate de desen; le sincronizam cu marimea reala
  // afisata (clientWidth/clientHeight) ca sa nu iasa neclare/intinse.
  function resizeCanvases() {
    document.querySelectorAll('canvas').forEach(function (c) {
      const w = c.clientWidth, h = c.clientHeight;
      if (w > 0 && h > 0 && (c.width !== w || c.height !== h)) {
        c.width = w; c.height = h;
      }
    });
  }
  window.addEventListener('resize', resizeCanvases);
  resizeCanvases(); // script-ul ruleaza dupa parsarea DOM-ului de mai sus,
                     // layout-ul flex e deja calculat -- nu trebuie sa
                     // astept 'load' (nesigur cu feed-urile MJPEG).

  // ===== SPEECH =====
  let speechEnabled = true;
  let preferredVoice = null;
  function pickVoice() {
    const voices = speechSynthesis.getVoices();
    preferredVoice = voices.find(v => v.lang.startsWith('ro')) || voices[0];
  }
  speechSynthesis.onvoiceschanged = pickVoice; pickVoice();
  function toggleSpeech() {
    speechEnabled = !speechEnabled;
    const b = document.getElementById('speech-btn');
    b.innerText = 'Speech: ' + (speechEnabled ? 'ON' : 'OFF');
    b.className = speechEnabled ? '' : 'off';
    if (!speechEnabled) speechSynthesis.cancel();
  }
  function speak(text) {
    if (!speechEnabled || !('speechSynthesis' in window)) return;
    speechSynthesis.cancel();
    const u = new SpeechSynthesisUtterance(text);
    if (preferredVoice) u.voice = preferredVoice;
    u.lang = preferredVoice ? preferredVoice.lang : 'ro-RO';
    speechSynthesis.speak(u);
  }
  function logSpeech(text) {
    const log = document.getElementById('speech-log');
    const e = document.createElement('div'); e.className = 'entry';
    e.innerText = `[${new Date().toLocaleTimeString()}] ${text}`;
    log.appendChild(e); log.scrollTop = log.scrollHeight;
    while (log.querySelectorAll('.entry').length > 50) log.querySelector('.entry').remove();
  }

  // ===== ALERTE AUDIO (YAMNet) =====
  function logAudioAlert(raw) {
    let a;
    try { a = JSON.parse(raw); } catch (err) { return; }
    const log = document.getElementById('audio-log');
    const e = document.createElement('div'); e.className = 'entry';
    const color = a.nivel === 'pericol' ? 'var(--danger)'
                : a.nivel === 'atentie' ? 'var(--warn)' : 'var(--muted)';
    e.style.color = color;
    e.innerText = `[${new Date().toLocaleTimeString()}] [${a.nivel.toUpperCase()}] `
                + `${a.tip} (${(a.scor * 100).toFixed(0)}%)`;
    log.appendChild(e); log.scrollTop = log.scrollHeight;
    while (log.querySelectorAll('.entry').length > 50) log.querySelector('.entry').remove();
  }

  // ===== ZONA SIGURA (segmentare) =====
  function updateWalkableBanner(raw) {
    let s;
    try { s = JSON.parse(raw); } catch (err) { return; }
    const banner = document.getElementById('walkable-banner');
    if (s.on_path) {
      banner.style.display = 'none';
    } else {
      banner.style.display = 'block';
      banner.innerText = s.message || 'Atenție! Nu ești pe zona sigură.';
    }
  }

  // ===== DECIZIE / SERVO (spatial_risk_node) =====
  const servoCanvas = document.getElementById('servo-gauge-canvas');
  const servoCtx = servoCanvas.getContext('2d');
  const servoReasonEl = document.getElementById('servo-reason');

  function actionToPosition(action) {
    if (action === 'servo_left') return -1;
    if (action === 'servo_right') return 1;
    return 0;
  }
  function riskColor(level) {
    if (level === 'critical') return '#e24b4a';
    if (level === 'high') return '#ff8800';
    if (level === 'medium') return '#ffd700';
    if (level === 'low') return '#04d361';
    return '#5a5a66';
  }
  function drawServoGauge(pos, color, label) {
    const W = servoCanvas.width, H = servoCanvas.height;
    servoCtx.fillStyle = '#050507';
    servoCtx.fillRect(0, 0, W, H);

    servoCtx.strokeStyle = '#2a2a30';
    servoCtx.lineWidth = 2;
    servoCtx.beginPath();
    servoCtx.moveTo(20, H/2 + 8); servoCtx.lineTo(W-20, H/2 + 8);
    servoCtx.stroke();

    servoCtx.fillStyle = '#5a5a66';
    servoCtx.font = 'bold 11px monospace';
    servoCtx.textAlign = 'center';
    servoCtx.fillText('STANGA', 45, H - 6);
    servoCtx.fillText('CENTRU', W/2, H - 6);
    servoCtx.fillText('DREAPTA', W-45, H - 6);

    const x = W/2 + pos * (W/2 - 40);
    servoCtx.fillStyle = color;
    servoCtx.shadowBlur = 12;
    servoCtx.shadowColor = color;
    servoCtx.beginPath();
    servoCtx.arc(x, H/2 + 8, 12, 0, 2*Math.PI);
    servoCtx.fill();
    servoCtx.shadowBlur = 0;

    servoCtx.fillStyle = '#e1e1e6';
    servoCtx.font = 'bold 13px monospace';
    servoCtx.fillText(label, W/2, 18);
  }
  drawServoGauge(0, '#5a5a66', 'ASTEPTARE');

  function updateServoGauge(raw) {
    let d;
    try { d = JSON.parse(raw); } catch (err) { return; }
    const pos = actionToPosition(d.action);
    const color = riskColor(d.risk_level);
    drawServoGauge(pos, color, (d.action || 'none').toUpperCase());
    if (servoReasonEl) servoReasonEl.innerText = d.reason || '';
  }

  // ===== TOF RADAR (yaw-controlled) =====
  const tofCanvas = document.getElementById('tof-canvas');
  const tofCtx = tofCanvas.getContext('2d');
  const TOF_CX = tofCanvas.width / 2;
  const TOF_CY = tofCanvas.height - 16;
  const TOF_MAX_MM = 2500;
  const TOF_DANGER_MM = 500;
  const TOF_WARN_MM = 1000;
  let tofHistory = [];

  function drawTofBackground() {
    tofCtx.fillStyle = '#050507';
    tofCtx.fillRect(0, 0, tofCanvas.width, tofCanvas.height);
    tofCtx.strokeStyle = '#2a2a30'; tofCtx.lineWidth = 1;
    for (let r = 500; r <= TOF_MAX_MM; r += 500) {
      const sR = (r / TOF_MAX_MM) * (tofCanvas.height - 30);
      tofCtx.beginPath(); tofCtx.arc(TOF_CX, TOF_CY, sR, Math.PI, 0); tofCtx.stroke();
      tofCtx.fillStyle = '#5a5a66'; tofCtx.font = '10px monospace';
      tofCtx.fillText(`${r/1000}m`, TOF_CX + sR - 18, TOF_CY - 4);
    }
    tofCtx.beginPath(); tofCtx.moveTo(TOF_CX, TOF_CY); tofCtx.lineTo(TOF_CX, 6); tofCtx.stroke();
  }

  function updateTofRadar(point) {
    const now = Date.now();
    if (point.distance > 0 && point.distance < TOF_MAX_MM) {
      tofHistory.push({
        angle: (point.angle * Math.PI) / 180,
        distance: point.distance, ts: now
      });
    }
    tofHistory = tofHistory.filter(p => (now - p.ts) <= 2000);
    drawTofBackground();
    let danger = false;
    tofHistory.forEach(p => {
      const rScreen = (p.distance / TOF_MAX_MM) * (tofCanvas.height - 30);
      const a = p.angle - Math.PI/2;
      const x = TOF_CX + rScreen * Math.cos(a);
      const y = TOF_CY + rScreen * Math.sin(a);
      const alpha = 1 - ((now - p.ts) / 2000);
      if (p.distance < TOF_DANGER_MM) {
        danger = true;
        tofCtx.fillStyle = `rgba(226, 75, 74, ${alpha})`;
        tofCtx.shadowBlur = 8; tofCtx.shadowColor = 'red';
      } else if (p.distance < TOF_WARN_MM) {
        tofCtx.fillStyle = `rgba(255, 136, 0, ${alpha})`;
        tofCtx.shadowBlur = 4; tofCtx.shadowColor = 'orange';
      } else {
        tofCtx.fillStyle = `rgba(4, 211, 97, ${alpha})`;
        tofCtx.shadowBlur = 0;
      }
      tofCtx.beginPath(); tofCtx.arc(x, y, 4, 0, 2*Math.PI); tofCtx.fill();
    });
    document.getElementById('alert-banner').style.display = danger ? 'block' : 'none';
  }
  drawTofBackground();

  // ===== BARA DISTANȚĂ =====
  const distCanvas = document.getElementById('dist-bar-canvas');
  const distCtx = distCanvas.getContext('2d');
  const distBig = document.getElementById('dist-big');
  let distHistory = [];
  const DIST_HISTORY_MS = 2000;
  const DIST_MAX_MM = 2500;

  function drawDistBar(currentDist) {
    const W = distCanvas.width;
    const H = distCanvas.height;

    distCtx.fillStyle = '#050507';
    distCtx.fillRect(0, 0, W, H);

    distCtx.strokeStyle = '#1f1f24';
    distCtx.lineWidth = 1;
    distCtx.fillStyle = '#5a5a66';
    distCtx.font = '9px monospace';
    for (let d = 500; d <= DIST_MAX_MM; d += 500) {
      const x = (d / DIST_MAX_MM) * W;
      distCtx.beginPath();
      distCtx.moveTo(x, 0); distCtx.lineTo(x, H);
      distCtx.stroke();
      distCtx.fillText(`${d/1000}m`, x + 3, H - 4);
    }

    distCtx.fillStyle = 'rgba(226, 75, 74, 0.1)';
    distCtx.fillRect(0, 0, (500/DIST_MAX_MM)*W, H);
    distCtx.fillStyle = 'rgba(255, 136, 0, 0.08)';
    distCtx.fillRect((500/DIST_MAX_MM)*W, 0, ((1000-500)/DIST_MAX_MM)*W, H);

    const now = Date.now();
    distHistory = distHistory.filter(d => (now - d.ts) <= DIST_HISTORY_MS);

    distHistory.forEach(d => {
      const age = now - d.ts;
      const t = age / DIST_HISTORY_MS;
      const xPos = (d.dist / DIST_MAX_MM) * W;
      const alpha = 1 - t;
      if (d.dist < 500) distCtx.fillStyle = `rgba(226,75,74,${alpha})`;
      else if (d.dist < 1000) distCtx.fillStyle = `rgba(255,136,0,${alpha})`;
      else distCtx.fillStyle = `rgba(4,211,97,${alpha})`;
      distCtx.beginPath();
      distCtx.arc(xPos, H/2, 3, 0, 2*Math.PI);
      distCtx.fill();
    });

    if (currentDist > 0) {
      const xPos = Math.min((currentDist / DIST_MAX_MM) * W, W - 2);
      let color = '#04d361';
      if (currentDist < 500) color = '#e24b4a';
      else if (currentDist < 1000) color = '#ff8800';
      distCtx.fillStyle = color;
      distCtx.fillRect(xPos - 3, 8, 6, H - 24);

      distCtx.shadowBlur = 10;
      distCtx.shadowColor = color;
      distCtx.beginPath();
      distCtx.arc(xPos, H/2, 7, 0, 2*Math.PI);
      distCtx.fill();
      distCtx.shadowBlur = 0;
    }
  }

  function updateDistBar(distMm) {
    if (distMm > 0 && distMm < DIST_MAX_MM) {
      distHistory.push({ dist: distMm, ts: Date.now() });
    }
    drawDistBar(distMm);

    let cls = '';
    if (distMm < 500) cls = 'danger';
    else if (distMm < 1000) cls = 'warn';
    distBig.className = 'dist-big ' + cls;
    if (distMm <= 0) distBig.innerText = '— mm';
    else if (distMm < 1000) distBig.innerText = `${distMm} mm`;
    else distBig.innerText = `${(distMm/1000).toFixed(2)} m`;
  }
  drawDistBar(0);

  // ===== IMU 3D =====
  const imuContainer = document.getElementById('imu-container');
  const scene = new THREE.Scene();
  const cam = new THREE.PerspectiveCamera(45,
      imuContainer.clientWidth / imuContainer.clientHeight, 0.1, 100);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(imuContainer.clientWidth, imuContainer.clientHeight);
  renderer.setClearColor(0x050507, 1);
  imuContainer.appendChild(renderer.domElement);

  const caneGeom = new THREE.CylinderGeometry(0.3, 0.3, 6, 24);
  const caneMat = new THREE.MeshPhongMaterial({ color: 0x04d361 });
  const cane = new THREE.Mesh(caneGeom, caneMat);
  scene.add(cane);

  const headGeom = new THREE.SphereGeometry(0.55, 16, 16);
  const headMat = new THREE.MeshPhongMaterial({ color: 0xffd700 });
  const head = new THREE.Mesh(headGeom, headMat);
  head.position.y = 3.2; cane.add(head);

  const grid = new THREE.GridHelper(10, 10, 0x04d361, 0x333333);
  grid.position.y = -3; scene.add(grid);
  scene.add(new THREE.DirectionalLight(0xffffff, 0.9).position.set(5,10,7));
  scene.add(new THREE.AmbientLight(0x404040));
  cam.position.set(0, 2, 11); cam.lookAt(0, 0, 0);

  function animate3D() { requestAnimationFrame(animate3D); renderer.render(scene, cam); }
  animate3D();
  window.addEventListener('resize', () => {
    cam.aspect = imuContainer.clientWidth / imuContainer.clientHeight;
    cam.updateProjectionMatrix();
    renderer.setSize(imuContainer.clientWidth, imuContainer.clientHeight);
  });

  // ===== PENDUL YAW =====
  const pendCanvas = document.getElementById('pendulum-canvas');
  const pendCtx = pendCanvas.getContext('2d');
  let currentYaw = 0;
  let yawSmoothed = 0;

  function drawPendulum() {
    requestAnimationFrame(drawPendulum);
    yawSmoothed += (currentYaw - yawSmoothed) * 0.15;

    const W = pendCanvas.width;
    const H = pendCanvas.height;
    const cx = W / 2;
    const cy = H / 2;
    const radius = Math.min(W, H) / 2 - 18;

    pendCtx.fillStyle = '#050507';
    pendCtx.fillRect(0, 0, W, H);

    pendCtx.strokeStyle = '#2a2a30';
    pendCtx.lineWidth = 1.5;
    pendCtx.beginPath();
    pendCtx.arc(cx, cy, radius, 0, 2 * Math.PI);
    pendCtx.stroke();

    pendCtx.fillStyle = '#5a5a66';
    pendCtx.font = 'bold 11px monospace';
    pendCtx.textAlign = 'center';
    pendCtx.textBaseline = 'middle';
    pendCtx.fillText('FAȚĂ',  cx, cy - radius - 6);
    pendCtx.fillText('SPATE', cx, cy + radius + 8);
    pendCtx.fillText('STG',   cx - radius - 12, cy);
    pendCtx.fillText('DRP',   cx + radius + 12, cy);

    pendCtx.strokeStyle = '#1f1f24';
    pendCtx.lineWidth = 1;
    pendCtx.beginPath();
    pendCtx.moveTo(cx - radius, cy); pendCtx.lineTo(cx + radius, cy);
    pendCtx.moveTo(cx, cy - radius); pendCtx.lineTo(cx, cy + radius);
    pendCtx.stroke();

    const tipX = cx + radius * Math.sin(yawSmoothed);
    const tipY = cy - radius * Math.cos(yawSmoothed);

    pendCtx.strokeStyle = '#ffd700';
    pendCtx.lineWidth = 4;
    pendCtx.lineCap = 'round';
    pendCtx.beginPath();
    pendCtx.moveTo(cx, cy);
    pendCtx.lineTo(tipX, tipY);
    pendCtx.stroke();

    pendCtx.fillStyle = '#ffd700';
    pendCtx.beginPath();
    pendCtx.arc(cx, cy, 5, 0, 2 * Math.PI);
    pendCtx.fill();

    pendCtx.fillStyle = '#04d361';
    pendCtx.beginPath();
    pendCtx.arc(tipX, tipY, 7, 0, 2 * Math.PI);
    pendCtx.fill();

    const deg = (yawSmoothed * 180 / Math.PI).toFixed(0);
    pendCtx.fillStyle = '#e1e1e6';
    pendCtx.font = 'bold 14px monospace';
    pendCtx.fillText(`${deg}°`, cx, cy + radius * 0.55);
  }
  drawPendulum();

  // ===== VESTA HAPTICA (haptic_vest_node, /vest/haptic_state) =====
  // Nodul publica TRANZITIILE de decizie + heartbeat 1 Hz (deci >3 s de
  // tacere inseamna chiar nod oprit); anvelopa de puls (on/off in timp) o
  // redam aici cu ACEEASI logica precum _pattern_on() din
  // haptic_vest_node.py -- ce clipeste pe ecran e ce simte pielea.
  const vestCanvas = document.getElementById('vest-canvas');
  const vestCtx = vestCanvas.getContext('2d');
  const vestStatusEl = document.getElementById('vest-status');
  const VEST_LABELS = ['STG-EXT', 'STG', 'CTR', 'DRP', 'DRP-EXT'];
  const VEST_STALE_MS = 3000;   // fara evenimente SSE -> nodul e oprit/cazut
  let vestState = null;
  let vestLastMs = 0;

  function vestPatternOn(pattern, nowS) {
    // Copie 1:1 a _pattern_on() (haptic_vest_node.py) -- pastreaza in lockstep.
    if (pattern === 'alert')  return (nowS * 4.0) % 1.0 < 0.5;
    if (pattern === 'double') { const p = nowS % 1.0; return p < 0.12 || (p >= 0.24 && p < 0.36); }
    if (pattern === 'pulse')  return (nowS * 2.5) % 1.0 < 0.5;
    if (pattern === 'blip')   return nowS % 2.0 < 0.12;
    if (pattern === 'short')  return nowS % 1.2 < 0.15;
    return false;
  }
  function vestSourceColor(source) {
    if (source === 'critical') return '#e24b4a';
    if (source === 'audio_pericol') return '#ff8800';
    if (source === 'obstacol_high') return '#ff8800';
    if (source === 'off_path') return '#ffd700';
    if (source === 'ghidare' || source === 'ghidare_centru') return '#04d361';
    return '#5a5a66';
  }
  function drawVest() {
    requestAnimationFrame(drawVest);
    const W = vestCanvas.width, H = vestCanvas.height;
    vestCtx.fillStyle = '#050507';
    vestCtx.fillRect(0, 0, W, H);

    const nodeDead = (Date.now() - vestLastMs) > VEST_STALE_MS;
    const s = (!nodeDead && vestState) ? vestState : null;
    const active = s ? (s.active_motors || []) : [];
    const on = s ? vestPatternOn(s.pattern, Date.now() / 1000) : false;
    const color = s ? vestSourceColor(s.source) : '#5a5a66';

    for (let i = 0; i < 5; i++) {
      const x = W * (i + 0.5) / 5;
      const y = H / 2 - 4;
      const isActive = active.includes(i);
      vestCtx.strokeStyle = '#2a2a30';
      vestCtx.lineWidth = 2;
      vestCtx.beginPath(); vestCtx.arc(x, y, 13, 0, 2 * Math.PI); vestCtx.stroke();
      if (isActive && on) {
        vestCtx.fillStyle = color;
        vestCtx.shadowBlur = 6 + 14 * (s.intensity || 0);
        vestCtx.shadowColor = color;
        vestCtx.beginPath();
        vestCtx.arc(x, y, 4 + 9 * (s.intensity || 0), 0, 2 * Math.PI);
        vestCtx.fill();
        vestCtx.shadowBlur = 0;
      }
      vestCtx.fillStyle = '#5a5a66';
      vestCtx.font = 'bold 10px monospace';
      vestCtx.textAlign = 'center';
      vestCtx.fillText(VEST_LABELS[i], x, H - 6);
    }

    if (!vestState) vestStatusEl.innerText = 'Fara date inca.';
    else if (nodeDead) vestStatusEl.innerText = 'FARA DATE (nod oprit?)';
    else vestStatusEl.innerText =
      `${vestState.source} · ${vestState.pattern} · ` +
      `intensitate ${((vestState.intensity || 0) * 100).toFixed(0)}%`;
  }
  drawVest();
  function updateVestState(raw) {
    try { vestState = JSON.parse(raw); } catch (err) { return; }
    vestLastMs = Date.now();
  }

  // ===== SSE =====
  const evt = new EventSource('/events');
  evt.addEventListener('speak', e => { logSpeech(e.data); speak(e.data); });
  evt.addEventListener('audio_alert', e => { logAudioAlert(e.data); });
  evt.addEventListener('walkable_status', e => { updateWalkableBanner(e.data); });
  evt.addEventListener('risk', e => { updateServoGauge(e.data); });
  evt.addEventListener('vest_state', e => { updateVestState(e.data); });
  evt.addEventListener('status', e => {
    const banner = document.getElementById('listen-banner');
    if (e.data === 'listening') {
      banner.style.display = 'block';
      banner.innerText = 'ASCULT... vorbește acum';
      banner.style.background = 'linear-gradient(90deg, #04d361, #ffd700)';
    } else if (e.data === 'thinking') {
      banner.style.display = 'block';
      banner.innerText = 'Mă gândesc...';
      banner.style.background = 'linear-gradient(90deg, #ffd700, #ff8800)';
    } else {
      banner.style.display = 'none';
    }
  });
  evt.addEventListener('sensor_data', e => {
    const d = JSON.parse(e.data);
    updateTofRadar({ angle: d.angle || 0, distance: d.distance || 0 });
    updateDistBar(d.distance || 0);
    cane.rotation.x = d.pitch || 0;
    cane.rotation.z = -(d.roll || 0);
    cane.rotation.y = d.yaw || 0;
    currentYaw = d.yaw || 0;
  });
  evt.onerror = () => console.warn('SSE error');

  setInterval(() => fetch('/stats').then(r=>r.json()).then(s => {
    document.getElementById('stats').innerText =
      `Frames: ${s.frames} · Mode: ${s.mode} · Last: [${s.last_classes.join(', ')}]`;
  }), 500);
</script>
</body></html>
"""


@app.route("/")
def index(): return render_template_string(PAGE)


@app.route("/video")
def video():
    def gen():
        while True:
            with _state["lock"]: jpg = _state["frame_jpeg"]
            if jpg is not None:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
            time.sleep(DASHBOARD_FEED_PERIOD_S)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/seg_video")
def seg_video():
    def gen():
        while True:
            with _state["lock"]: jpg = _state["seg_jpeg"]
            if jpg is not None:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
            time.sleep(DASHBOARD_FEED_PERIOD_S)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/depth_video")
def depth_video():
    def gen():
        while True:
            with _state["lock"]: jpg = _state["depth_jpeg"]
            if jpg is not None:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
            time.sleep(DASHBOARD_FEED_PERIOD_S)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/events")
def events():
    def stream():
        yield "event: ping\ndata: connected\n\n"
        while True:
            try:
                event = _state["event_queue"].get(timeout=10.0)
                safe = event["data"].replace("\n", " ").replace("\r", " ")
                yield f"event: {event['type']}\ndata: {safe}\n\n"
            except queue.Empty:
                yield "event: ping\ndata: keepalive\n\n"
    return Response(stream(), mimetype="text/event-stream")


@app.route("/toggle")
def toggle():
    with _state["lock"]:
        i = _MODES.index(_state["mode"])
        _state["mode"] = _MODES[(i + 1) % len(_MODES)]
    return _state["mode"]


@app.route("/stats")
def stats():
    with _state["lock"]:
        return jsonify({"frames": _state["frames"], "mode": _state["mode"],
                        "last_classes": _state["last_classes"]})


def color_for(label):
    h = hash(label); return (h*73 % 256, h*131 % 256, h*197 % 256)


class DashboardNode(Node):
    def __init__(self):
        super().__init__("hive_dashboard")
        self.cv_bridge = CvBridge()
        self.last_image = None
        self.last_detections_yolo = []
        self.last_depth = None
        self.last_risk = None
        self.last_sensor_emit = 0.0
        self.sensor_interval = 0.033

        self.create_subscription(Image, "/perception/image_raw", self.image_cb, 10)
        self.create_subscription(Detection2DArray, "/perception/detections_yolo",
                                 self.det_yolo_cb, 10)
        self.create_subscription(Image, "/keryke/depth/metric", self.depth_cb, 5)
        # Overlay-ul de segmentare vine deja colorat + comprimat JPEG din
        # SegmentationRosNode (yolo_segmentation_node.py) -- dashboard-ul
        # doar il forwardeaza, nu mai recalculeaza HSV separat (elimina o
        # dublare reala de CPU intre cele doua procese).
        self.create_subscription(CompressedImage, "/perception/segmentation_overlay",
                                 self.seg_overlay_cb, 1)
        self.create_subscription(String, "/audio/speak", self.speak_cb, 10)
        self.create_subscription(String, "/keryke/sensors", self.sensor_cb, 30)
        self.create_subscription(String, "/audio/status", self.status_cb, 10)
        self.create_subscription(String, "/audio/alerts", self.audio_alert_cb, 10)
        self.create_subscription(String, "/perception/walkable_status",
                                 self.walkable_status_cb, 10)
        self.create_subscription(String, "/keryke/risk", self.risk_cb, 10)
        # Starea vestei vine la tranzitii de decizie + heartbeat 1 Hz (nu
        # 20 Hz) -- anvelopa de puls o anima browserul, in lockstep cu
        # _pattern_on(); heartbeat-ul face fiabila detectia "nod oprit".
        self.create_subscription(String, "/vest/haptic_state",
                                 self.vest_state_cb, 10)
        self.create_timer(DASHBOARD_FEED_PERIOD_S, self.compose_frame)
        self.create_timer(DASHBOARD_FEED_PERIOD_S, self.compose_depth_frame)
        self.get_logger().info("Dashboard v4 — http://0.0.0.0:5000")

    def image_cb(self, msg):
        self.last_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

    def det_yolo_cb(self, msg):
        self.last_detections_yolo = msg.detections

    def depth_cb(self, msg):
        self.last_depth = self.cv_bridge.imgmsg_to_cv2(msg, "32FC1")

    def seg_overlay_cb(self, msg):
        with _state["lock"]:
            _state["seg_jpeg"] = bytes(msg.data)

    def audio_alert_cb(self, msg):
        try: _state["event_queue"].put_nowait({"type": "audio_alert", "data": msg.data})
        except queue.Full: pass

    def walkable_status_cb(self, msg):
        try: _state["event_queue"].put_nowait({"type": "walkable_status", "data": msg.data})
        except queue.Full: pass

    def risk_cb(self, msg):
        try:
            self.last_risk = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError):
            pass
        try: _state["event_queue"].put_nowait({"type": "risk", "data": msg.data})
        except queue.Full: pass

    def vest_state_cb(self, msg):
        try: _state["event_queue"].put_nowait({"type": "vest_state", "data": msg.data})
        except queue.Full: pass

    def speak_cb(self, msg):
        try: _state["event_queue"].put_nowait({"type": "speak", "data": msg.data})
        except queue.Full: pass

    def sensor_cb(self, msg):
        now = time.time()
        if now - self.last_sensor_emit < self.sensor_interval: return
        self.last_sensor_emit = now
        try: _state["event_queue"].put_nowait({"type": "sensor_data", "data": msg.data})
        except queue.Full: pass

    def status_cb(self, msg):
        try: _state["event_queue"].put_nowait({"type": "status", "data": msg.data})
        except queue.Full: pass

    def compose_frame(self):
        if self.last_image is None: return
        frame = self.last_image.copy()
        with _state["lock"]: mode = _state["mode"]
        classes_seen = []
        if mode == "yolo":
            frame_w = frame.shape[1]
            for d in self.last_detections_yolo:
                if not d.results: continue
                cls = d.results[0].hypothesis.class_id
                score = d.results[0].hypothesis.score
                cx, cy = d.bbox.center.position.x, d.bbox.center.position.y
                w, h = d.bbox.size_x, d.bbox.size_y
                x1, y1 = int(cx-w/2), int(cy-h/2)
                x2, y2 = int(cx+w/2), int(cy+h/2)
                color = color_for(cls)
                cv2.rectangle(frame, (x1,y1), (x2,y2), color, 2)
                label = f"{cls} {score:.2f}"
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(frame, (x1, y1-th-8), (x1+tw+4, y1), color, -1)
                cv2.putText(frame, label, (x1+2, y1-4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)
                classes_seen.append(cls)

            # bulina de directie -- reflecta DECIZIA REALA de evitare obstacole
            # din spatial_risk_node (/keryke/risk, ToF+YOLO+audio), nu "obiectul
            # cel mai increzator din cadru" -- aia putea fi chiar userul insusi
            # daca era singurul lucru vizibil in cadru.
            if self.last_risk is not None:
                action = self.last_risk.get("action", "none")
                reason = self.last_risk.get("reason", "")
                pos = {"servo_left": -1, "servo_right": 1}.get(action, 0)
                frame_h = frame.shape[0]
                dot_x = int(frame_w / 2 + pos * (frame_w / 2 - 40))
                dot_y = int(frame_h * 0.9)
                if action == "servo_center_stop":
                    dot_color = (0, 0, 255)
                elif action == "none":
                    dot_color = (0, 200, 0)
                else:
                    dot_color = (0, 140, 255)
                cv2.circle(frame, (dot_x, dot_y), 10, dot_color, -1)
                cv2.putText(frame, reason[:60], (10, 56),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, dot_color, 2)

            cv2.putText(frame, "HIVE AI (yolov8n)", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        else:
            cv2.putText(frame, "RAW", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200,200,200), 2)

        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with _state["lock"]:
                _state["frame_jpeg"] = jpg.tobytes()
                _state["frames"] += 1
                _state["last_classes"] = sorted(set(classes_seen))

    def compose_depth_frame(self):
        if self.last_depth is None: return
        colored = colorize_depth(self.last_depth)
        ok, jpg = cv2.imencode(".jpg", colored, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with _state["lock"]:
                _state["depth_jpeg"] = jpg.tobytes()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000,
                               debug=False, use_reloader=False, threaded=True),
        daemon=True,
    ).start()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__": main()
