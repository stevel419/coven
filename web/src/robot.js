// VAMP's Panda, loaded once from the service and cloned for every arm on screen
import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import URDFLoader from 'urdf-loader';

export const JOINT_NAMES = Array.from({ length: 7 }, (_, i) => `panda_joint${i + 1}`);

let loading = null;

// The robot with its visual meshes and its collision spheres, once every file has arrived
export function loadPanda() {
  loading ??= new Promise((resolve, reject) => {
    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    loader.parseCollision = true;
    loader.loadMeshCb = (path, meshManager, done) =>
      new OBJLoader(meshManager).load(path, (object) => done(object), undefined, (error) => done(null, error));
    let robot = null;
    manager.onLoad = () => (robot ? resolve(robot) : reject(new Error('no robot')));
    manager.onError = (url) => reject(new Error(`could not load ${url}`));
    loader.load('panda/panda_spherized.urdf', (r) => { robot = r; });
  });
  return loading;
}

const colliderMaterial = new THREE.MeshBasicMaterial({ color: 0x46c2a5, transparent: true, opacity: 0.35, depthWrite: false });

// A copy of the robot drawn with `material`; its collision spheres start hidden
export function clonePanda(robot, material) {
  const copy = robot.clone();
  copy.traverse((o) => {
    if (o.isURDFCollider) o.visible = false;
    if (!o.isMesh) return;
    let collider = false;
    for (let p = o.parent; p; p = p.parent) collider ||= p.isURDFCollider;
    o.material = collider ? colliderMaterial : material;
  });
  return copy;
}

export function setPose(robot, q) {
  JOINT_NAMES.forEach((name, i) => robot.joints[name].setJointValue(q[i]));
}

export function showColliders(robot, visible) {
  robot.traverse((o) => { if (o.isURDFCollider) o.visible = visible; });
}

export const lerp = (a, b, s) => a.map((v, i) => v + (b[i] - v) * s);

// Seconds to spend on each path segment, at `speed` radians per second on the joint that moves most
export function durations(path, speed = 1.2) {
  return path.slice(1).map((q, i) => Math.max(...q.map((v, j) => Math.abs(v - path[i][j]))) / speed);
}

// The configuration at time t along the path
export function poseAt(path, times, t) {
  for (let i = 0; i < times.length; ++i) {
    if (t <= times[i]) return lerp(path[i], path[i + 1], times[i] ? t / times[i] : 1);
    t -= times[i];
  }
  return path[path.length - 1];
}
