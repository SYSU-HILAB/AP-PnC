import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import path from 'node:path';
import * as THREE from 'three';
import { rigidCamera, updateRigidCamera, committedPose } from './camera_rig.js';

const root = process.env.AP_PNC_DIR;
assert.ok(root && path.isAbsolute(root));
const config = JSON.parse(readFileSync(path.join(root, 'infra/sim_infra/lidar_sim/web/camera_rig.json')));
function close(actual, expected) { assert.ok(actual.distanceTo(new THREE.Vector3(...expected)) < 1e-10); }

test('hover: head looks skyward, belly forward; physical lever arms preserved', () => {
  const nose = rigidCamera(config.cameras[0], 640, 360);
  const belly = rigidCamera(config.cameras[1], 640, 360);
  const p = new THREE.Vector3(74, -12, 18), q = new THREE.Quaternion();
  updateRigidCamera(nose, p, q); updateRigidCamera(belly, p, q);
  close(nose.camera.getWorldDirection(new THREE.Vector3()), [0, 0, 1]);
  close(belly.camera.getWorldDirection(new THREE.Vector3()), [1, 0, 0]);
  close(nose.camera.position, [74, -12, 18.6]);
  close(belly.camera.position, [74.18, -12, 18.05]);
});

test('90-degree tail-sitter transition: head forward, belly downward; no horizon stabilization', () => {
  const p = new THREE.Vector3(74, -12, 18);
  const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI / 2);
  const nose = rigidCamera(config.cameras[0], 640, 360);
  const belly = rigidCamera(config.cameras[1], 640, 360);
  updateRigidCamera(nose, p, q); updateRigidCamera(belly, p, q);
  close(nose.camera.getWorldDirection(new THREE.Vector3()), [1, 0, 0]);
  close(belly.camera.getWorldDirection(new THREE.Vector3()), [0, 0, -1]);
  close(nose.camera.position, [74.6, -12, 18]);
  close(belly.camera.position, [74.05, -12, 17.82]);
});

test('arbitrary body attitude rotates optical axes and image up together', () => {
  const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(0.6, -0.3, 1.2));
  for (const spec of config.cameras) {
    const rig = rigidCamera(spec, 640, 360);
    updateRigidCamera(rig, new THREE.Vector3(), q);
    const forward = new THREE.Vector3(...spec.forward_flu).applyQuaternion(q);
    assert.ok(forward.distanceTo(rig.camera.getWorldDirection(new THREE.Vector3())) < 1e-10);
    const expectedUp = new THREE.Vector3(...spec.up_flu).applyQuaternion(q);
    assert.ok(expectedUp.distanceTo(new THREE.Vector3(0, 1, 0).applyQuaternion(rig.camera.quaternion)) < 1e-10);
    const centre = rig.camera.position.clone().addScaledVector(forward, 10).project(rig.camera);
    assert.ok(Math.abs(centre.x) < 1e-10 && Math.abs(centre.y) < 1e-10);
  }
});

test('render interpolation uses committed poses only and never extrapolates', () => {
  const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2);
  const poses = [[0, 0, 0, 0, 0, 0, 0, 1], [1, 10, 0, 0, ...q.toArray()]];
  const half = committedPose(poses, 0.5);
  close(half.p, [5, 0, 0]);
  assert.ok(Math.abs(half.q.angleTo(new THREE.Quaternion()) - Math.PI / 4) < 1e-10);
  close(committedPose(poses, 5).p, [10, 0, 0]);
  assert.equal(committedPose(poses, 5).t, 1);
  assert.equal(committedPose(poses, -1).t, 0);
});
