import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { SparkRenderer, SplatMesh } from '@sparkjsdev/spark';
import { rigidCamera, updateRigidCamera, committedPose } from './camera_rig.js';

const $ = (id) => document.getElementById(id);
const viewResponse = await fetch('/assets/view.json', { cache: 'no-store' });
if (!viewResponse.ok) throw new Error('prepared view configuration unavailable');
const view = await viewResponse.json();
$('scene-scale').textContent = view.synthetic_mirrored ? '合成镜像环道 · ×1' : `×${view.scene_scale}`;
const renderer = new THREE.WebGLRenderer({ antialias: false });
renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.NoToneMapping;
$('world').append(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color('#aac6d8');
const camera = new THREE.PerspectiveCamera(65, 1, 0.1, 2000);
camera.up.set(0, 0, 1);
camera.position.fromArray(view.overview_eye);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.enabled = false;
const mainSpark = new SparkRenderer({ renderer, maxStdDev: 2.5, minAlpha: 0.03 });
scene.add(mainSpark);
scene.add(new THREE.AmbientLight(0xffffff, 2.5));
const sun = new THREE.DirectionalLight(0xffffff, 2);
sun.position.set(60, -90, 180); scene.add(sun);
const splats = new SplatMesh({ url: view.scene_asset ?? '/assets/city.ply' });
splats.scale.setScalar(view.scene_scale);
scene.add(splats);
await splats.initialized;

const plane = new THREE.Group();
const airframe = new THREE.Group();
airframe.rotation.y = -Math.PI / 2; // Longitudinal axis = simulator +Z_FLU.
plane.add(airframe);
const material = new THREE.MeshStandardMaterial({ color: '#ffffff', roughness: 0.35 });
function box(size, position, color = material) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), color);
  mesh.position.set(...position); airframe.add(mesh);
}
box([1.05, 0.15, 0.14], [0, 0, 0]);
box([0.23, 1.65, 0.055], [-0.08, 0, 0]);
box([0.18, 0.52, 0.04], [-0.42, 0, 0.04]);
box([0.22, 0.035, 0.21], [-0.42, 0, 0.13], new THREE.MeshBasicMaterial({ color: '#ff744c' }));
scene.add(plane);
const reference = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: '#4fbaff' }));
const trail = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: '#ff8d4c' }));
const markers = new THREE.Group(); scene.add(reference, trail, markers);

const { width, height } = view.camera_rig;
const sensors = view.camera_rig.cameras.map((spec) => {
  const rig = rigidCamera(spec, width, height);
  // Independent Spark sort/accumulator state for each optical viewpoint.
  const spark = new SparkRenderer({ renderer, maxStdDev: 2.5, minAlpha: 0.03, target: { width, height } });
  const surface = document.createElement('canvas'); surface.width = width; surface.height = height;
  const context = surface.getContext('2d');
  const image = context.createImageData(width, height);
  const stream = surface.captureStream(0);
  const track = stream.getVideoTracks()[0];
  const video = $(`${spec.id}-video`);
  video.srcObject = stream; video.muted = true; video.play().catch(() => {});
  return { spec, rig, spark, surface, context, image, stream, track };
});

let live = null, verification = null, queuedCommand = null, markerKey = '', lastReception = 0;
let samples = [[0, ...view.initial_position, 0, 0, 0, 1]], receivedPoseAt = performance.now(), latestPoseTime = -1;
let showReference = true, showMarkers = true;
const heading = new THREE.Vector3(1, 0, 0); // External observer ONLY; never used by a sensor.
function updateLine(line, points) {
  const old = line.geometry;
  line.geometry = new THREE.BufferGeometry().setFromPoints(points.map((p) => new THREE.Vector3(...p)));
  old.dispose();
}
async function command(operation) {
  $('play').disabled = $('restart').disabled = true;
  try {
    const result = await fetch('/api/control', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ operation }) });
    if (!result.ok) throw new Error('command rejected');
    queuedCommand = (await result.json()).id;
  } catch {
    $('status').textContent = '操作未确认，请检查仿真服务';
    $('play').disabled = $('restart').disabled = false;
  }
}
$('play').onclick = () => command(live?.phase === 'paused' ? 'resume' : 'pause');
$('restart').onclick = () => command('restart');
const labels = { paused: '已暂停 · 可开始或继续', 'initial-hover': '初始悬停 · 在线规划加速段', flying: '闭环试验中 · 实际速度见仪表', 'return-hover': '已回到起点 · 末端悬停', complete: '轨迹执行结束 · 待独立验收', failed: '仿真停止 · 查看运行日志' };
async function poll() {
  try {
    const response = await fetch('/assets/live.json', { cache: 'no-store' });
    if (!response.ok) throw new Error('waiting');
    live = await response.json(); lastReception = performance.now();
    const velocity = new THREE.Vector3(...live.v);
    if (velocity.length() > 0.3) heading.lerp(velocity.clone().setZ(0).normalize(), 0.2).normalize();
    updateLine(reference, live.reference); updateLine(trail, live.trail ?? []);
    const key = JSON.stringify(live.waypoints);
    if (markerKey !== key) {
      for (const child of [...markers.children]) {
        child.geometry?.dispose(); child.material?.dispose(); markers.remove(child);
      }
      live.waypoints.slice(0, -1).forEach((p, i) => {
        const ring = new THREE.Mesh(new THREE.TorusGeometry(2, 0.09, 8, 32), new THREE.MeshBasicMaterial({ color: i ? '#4fbaff' : '#64f4bd' }));
        ring.position.set(...p); markers.add(ring);
      });
      markerKey = key;
    }
    if (queuedCommand === live.command_id) queuedCommand = null;
    const ended = ['failed', 'complete'].includes(live.phase);
    $('play').disabled = Boolean(queuedCommand) || ended;
    $('restart').disabled = Boolean(queuedCommand) || (!ended && live.phase !== 'paused') || live.pending;
    $('play').textContent = live.phase === 'paused' ? (live.t ? '继续试验' : '启动试验') : '暂停';
    const verified = live.phase === 'complete' && verification?.status === 'passed'
      && verification.stream === live.stream && verification.recording_name === live.recording.split('/').at(-1);
    $('acceptance').textContent = verified ? '本次 10 m/s 连续环航已独立验收。仅仿真，非实机飞行。' : '当前运行尚未独立验收。仅仿真，非实机飞行。';
    $('acceptance').style.color = verified ? '#63ecbf' : '#ffbf86';
    $('status').textContent = verified ? '一圈完成 · 已核验 CSV、速度、地图和返航' : (labels[live.phase] ?? live.phase);
    $('time').textContent = `${live.t.toFixed(2)} s`;
    $('speed').textContent = `${velocity.length().toFixed(2)} m/s`;
    $('legs').textContent = `${live.leg} / ${live.total}`;
    $('position').textContent = `实际位置 [m]：${live.p.map((v) => v.toFixed(1)).join(', ')}`;
    $('plan').textContent = `${live.pending ? '在线规划中 · ' : ''}最近规划 ${live.plan_ms.toFixed(1)} ms · 蓝参考 / 橙实际`;
  } catch { $('status').textContent = '等待 ROS2 实时状态 · 不播放预录轨迹'; }
  finally { setTimeout(poll, 100); }
}
async function pollPoses() {
  try {
    const response = await fetch('/assets/poses.json', { cache: 'no-store' });
    if (!response.ok) throw new Error('waiting');
    const data = await response.json();
    const t = data.samples.at(-1)[0];
    if (t !== latestPoseTime) {
      samples = data.samples; latestPoseTime = t; receivedPoseAt = performance.now();
    }
  } catch { /* Keep the last committed pose, never invent motion. */ }
  finally { setTimeout(pollPoses, 40); }
}
async function pollVerification() {
  try {
    const response = await fetch('/assets/verification.json', { cache: 'no-store' });
    verification = response.ok ? await response.json() : null;
  } catch { verification = null; }
  finally { setTimeout(pollVerification, 1000); }
}
poll(); pollPoses(); pollVerification();
$('reference').onclick = () => { showReference = !showReference; $('reference').textContent = showReference ? '隐藏规划线' : '显示规划线'; };
$('markers').onclick = () => { showMarkers = !showMarkers; $('markers').textContent = showMarkers ? '隐藏航点' : '显示航点'; };
$('view').onchange = () => { controls.enabled = $('view').value === 'orbit'; controls.target.copy(plane.position); };
function resize() {
  const { clientWidth: w, clientHeight: h } = $('world');
  camera.aspect = w / h; camera.updateProjectionMatrix(); renderer.setSize(w, h);
}
resize(); addEventListener('resize', resize);
$('loading').style.display = 'none';
let previous = performance.now(), frames = 0, frameTime = previous;
async function animate(now) {
  try {
    const dt = Math.min((now - previous) / 1000, 0.2); previous = now;
    const pose = committedPose(samples, samples.at(-1)[0] - 0.12 + (now - receivedPoseAt) / 1000);
    plane.position.copy(pose.p); plane.quaternion.copy(pose.q);
    const mode = $('view').value;
    if (mode === 'orbit') controls.update();
    else {
      const eye = mode === 'overview' ? new THREE.Vector3(...view.overview_eye) : pose.p.clone().addScaledVector(heading, -30).add(new THREE.Vector3(0, 0, 45));
      const target = mode === 'overview' ? new THREE.Vector3(...view.overview_target) : pose.p;
      camera.position.lerp(eye, 1 - Math.exp(-5 * dt)); camera.lookAt(target);
    }
    // Same exact interpolated committed pose drives BOTH cameras for this frame.
    // No mission graphics / observer aircraft glyph in sensor imagery.
    plane.visible = reference.visible = trail.visible = markers.visible = false;
    for (const sensor of sensors) updateRigidCamera(sensor.rig, pose.p, pose.q);
    const pixels = await Promise.all(sensors.map(async (sensor) => {
      await sensor.spark.update({ scene, camera: sensor.rig.camera });
      sensor.spark.renderTarget({ scene, camera: sensor.rig.camera });
      return sensor.spark.readTarget();
    }));
    sensors.forEach((sensor, index) => {
      const row = width * 4;
      for (let y = 0; y < height; ++y) sensor.image.data.set(pixels[index].subarray((height - 1 - y) * row, (height - y) * row), y * row);
      sensor.context.putImageData(sensor.image, 0, 0);
      sensor.track.requestFrame();
      $(`${sensor.spec.id}-time`).textContent = `${pose.t.toFixed(2)} s`;
    });
    plane.visible = trail.visible = true; reference.visible = showReference; markers.visible = showMarkers;
    mainSpark.render(scene, camera);
    // Read-only QA evidence: sensor transforms are reproducible from this body pose.
    window.cityCameraDebug = { t: pose.t, body: { p: pose.p.toArray(), q: pose.q.toArray() },
      cameras: sensors.map((s) => ({ id: s.spec.id, p: s.rig.camera.position.toArray(), q: s.rig.camera.quaternion.toArray(),
        forward: s.rig.camera.getWorldDirection(new THREE.Vector3()).toArray(), live: s.track.readyState,
        width, height, mount: s.spec })), sceneScale: view.scene_scale };
    if (live && (performance.now() - lastReception > 2500 || Date.now() - live.wall_unix_ms > 2500)) $('status').textContent = '状态连接中断 · 相机保持最后姿态';
    frames++;
    if (now - frameTime > 1000) {
      $('fps').textContent = `双相机 ${Math.round(frames * 1000 / (now - frameTime))} FPS · 每路 ${width}×${height}`;
      frames = 0; frameTime = now;
    }
    requestAnimationFrame(animate);
  } catch (error) { $('status').textContent = `相机渲染停止：${error.message}`; console.error(error); }
}
requestAnimationFrame(animate);
