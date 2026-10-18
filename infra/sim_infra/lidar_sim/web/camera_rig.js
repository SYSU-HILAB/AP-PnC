import * as THREE from 'three';

// Body/world quaternions are xyzw. Camera looks down its local -Z (Three.js).
export function rigidCamera(spec, width, height) {
  const forward = new THREE.Vector3(...spec.forward_flu);
  const up = new THREE.Vector3(...spec.up_flu);
  if (Math.abs(forward.length() - 1) > 1e-6 || Math.abs(up.length() - 1) > 1e-6 || Math.abs(forward.dot(up)) > 1e-6) {
    throw new Error('camera axes must be orthonormal');
  }
  const right = forward.clone().cross(up);
  const rotation = new THREE.Matrix4().makeBasis(right, up, forward.clone().negate());
  return {
    camera: new THREE.PerspectiveCamera(spec.vertical_fov_deg, width / height, 0.05, 2000),
    offset: new THREE.Vector3(...spec.position_flu_m),
    bodyFromCamera: new THREE.Quaternion().setFromRotationMatrix(rotation),
  };
}

export function updateRigidCamera(rig, position, bodyQuaternion) {
  // T_WC = T_WB * T_BC, including lever arm. No world-up lock / heading / lookAt.
  rig.camera.position.copy(rig.offset).applyQuaternion(bodyQuaternion).add(position);
  rig.camera.quaternion.copy(bodyQuaternion).multiply(rig.bodyFromCamera);
  rig.camera.updateMatrixWorld(true);
}

export function committedPose(samples, time) {
  if (!samples.length) throw new Error('no committed pose samples');
  const t = Math.max(samples[0][0], Math.min(time, samples.at(-1)[0]));
  let index = 0;
  while (index + 1 < samples.length && samples[index + 1][0] < t) index++;
  const a = samples[index], b = samples[Math.min(index + 1, samples.length - 1)];
  const alpha = b[0] === a[0] ? 0 : (t - a[0]) / (b[0] - a[0]);
  return {
    t,
    p: new THREE.Vector3(...a.slice(1, 4)).lerp(new THREE.Vector3(...b.slice(1, 4)), alpha),
    q: new THREE.Quaternion(...a.slice(4, 8)).normalize().slerp(new THREE.Quaternion(...b.slice(4, 8)).normalize(), alpha),
  };
}
