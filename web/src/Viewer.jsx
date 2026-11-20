import { useEffect, useMemo, useState } from 'react';
import * as THREE from 'three';
import { Canvas, useFrame } from '@react-three/fiber';
import { Line, OrbitControls, TransformControls } from '@react-three/drei';
import { useStore } from './store.js';
import { clonePanda, durations, lerp, loadPanda, poseAt, setPose, showColliders } from './robot.js';

// Playback time along the current path, shared with the panel's frame slider
export const clock = { t: 0, total: 0 };

const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

export default function Viewer() {
  const [robot, setRobot] = useState(null);
  const [error, setError] = useState(null);
  const select = (id) => useStore.getState().set({ selected: id });

  useEffect(() => {
    loadPanda().then(setRobot, (e) => setError(e.message));
  }, []);

  return (
    <div className="view">
      <Canvas camera={{ position: [1.5, 1.25, 1.65], fov: 45, near: 0.05, far: 50 }} onPointerMissed={() => select(null)}>
        <color attach="background" args={[css('--bg')]} />
        <hemisphereLight args={[0xffffff, 0x8a8a80, 2.2]} />
        <directionalLight position={[2, 4, 3]} intensity={2} />
        <Floor />
        <OrbitControls makeDefault target={[0, 0.45, 0]} />
        {/* The service's z-up frame inside three.js's y-up world */}
        <group rotation-x={-Math.PI / 2}>
          <Obstacles />
          {robot && <Arms robot={robot} />}
        </group>
        {/* Outside the turned group: the gizmo places itself from the object's world transform */}
        <Gizmo />
      </Canvas>
      {!robot && <div className="overlay">{error ? `Could not load the robot model: ${error}` : 'Loading the Panda model…'}</div>}
      <div className="hint">Drag to orbit · scroll to zoom · click an obstacle to move it</div>
    </div>
  );
}

// A floor grid of 10 cm squares around the robot, wide enough to hold every obstacle
function Floor() {
  const obstacles = useStore((s) => s.obstacles);
  const size = useMemo(() => {
    let reach = 1.0;  // the Panda reaches about 0.85 m
    for (const o of obstacles) {
      const bound = o.kind === 'sphere' ? o.radius : o.kind === 'cuboid' ? Math.hypot(...o.halfExtents) : o.length / 2 + o.radius;
      reach = Math.max(reach, Math.hypot(o.position[0], o.position[1]) + bound);
    }
    return 2 * Math.ceil(reach * 2) / 2 + 1;  // half-metre steps, with a margin
  }, [obstacles]);
  const color = css('--grid');
  return <gridHelper key={size} args={[size, Math.round(size * 10), color, color]} />;
}

// The three.js object of each obstacle, by id
const objects = {};

function Obstacles() {
  const obstacles = useStore((s) => s.obstacles);
  const selected = useStore((s) => s.selected);
  const opacity = useStore((s) => s.display.obstacleOpacity);
  return obstacles.map((o) => (
    <Obstacle key={o.id} o={o} selected={o.id === selected} opacity={opacity}
      register={(object) => { if (object) objects[o.id] = object; else delete objects[o.id]; }} />
  ));
}

function Gizmo() {
  const selected = useStore((s) => s.selected);
  const obstacles = useStore((s) => s.obstacles);
  const mode = useStore((s) => s.mode);
  const [target, setTarget] = useState(null);

  // TransformControls needs the selected object itself, which exists only after it renders
  useEffect(() => setTarget(selected ? objects[selected] ?? null : null), [selected, obstacles]);
  if (!target) return null;

  const changed = () => useStore.getState().setObstacle(selected, {
    position: target.position.toArray(),
    euler: [target.rotation.x, target.rotation.y, target.rotation.z],
  });
  return <TransformControls object={target} mode={mode} size={0.8} space="local" onObjectChange={changed} />;
}

function Obstacle({ o, selected, opacity, register }) {
  const color = css('--obstacle');
  const select = (e) => {
    e.stopPropagation();
    useStore.getState().set({ selected: o.id });
  };
  return (
    <group ref={register} position={o.position} rotation={[...o.euler, 'ZYX']} onClick={select}>
      <mesh scale={o.kind === 'sphere' ? o.radius : o.kind === 'cuboid' ? o.halfExtents : 1}
        rotation-x={o.kind === 'capsule' ? Math.PI / 2 : 0}>
        {o.kind === 'sphere' && <sphereGeometry args={[1, 40, 20]} />}
        {o.kind === 'cuboid' && <boxGeometry args={[2, 2, 2]} />}
        {o.kind === 'capsule' && <capsuleGeometry args={[o.radius, o.length, 8, 24]} />}
        <meshStandardMaterial color={color} roughness={0.55} transparent depthWrite={false}
          opacity={selected ? Math.min(1, opacity + 0.3) : opacity}
          emissive={color} emissiveIntensity={selected ? 0.35 : 0} />
      </mesh>
    </group>
  );
}

function Arms({ robot }) {
  const path = useStore((s) => s.path);
  const start = useStore((s) => s.start);
  const goal = useStore((s) => s.goal);
  const display = useStore((s) => s.display);
  const accent = css('--accent');

  const materials = useMemo(() => ({
    solid: new THREE.MeshStandardMaterial({ color: 0xd8d7d2, roughness: 0.5, metalness: 0.1 }),
    ghost: new THREE.MeshStandardMaterial({ color: accent, transparent: true, depthWrite: false }),
  }), [accent]);
  materials.ghost.opacity = display.ghostOpacity;

  const arms = useMemo(() => ({
    moving: clonePanda(robot, materials.solid),
    start: clonePanda(robot, materials.ghost),
    goal: clonePanda(robot, materials.ghost),
    probe: clonePanda(robot, materials.solid),
  }), [robot, materials]);

  useEffect(() => {
    for (const arm of [arms.moving, arms.start, arms.goal]) showColliders(arm, display.colliders);
  }, [arms, display.colliders]);
  useEffect(() => setPose(arms.start, start), [arms, start]);
  useEffect(() => setPose(arms.goal, goal), [arms, goal]);

  const times = useMemo(() => durations(path), [path]);
  useEffect(() => {
    clock.t = 0;
    clock.total = times.reduce((a, b) => a + b, 0);
  }, [times]);

  // The hand's trace along the path, for the path line
  const trace = useMemo(() => {
    if (!path.length) return null;
    const hand = arms.probe.links.panda_hand;
    const points = [];
    for (let i = 0; i + 1 < path.length; ++i) {
      const steps = Math.max(2, Math.ceil(times[i] * 20));
      for (let s = 0; s <= steps; ++s) {
        setPose(arms.probe, lerp(path[i], path[i + 1], s / steps));
        arms.probe.updateMatrixWorld(true);
        points.push(hand.getWorldPosition(new THREE.Vector3()).toArray());
      }
    }
    return points;
  }, [arms, path, times]);

  useFrame((_, dt) => {
    const { playing, loop, speed } = useStore.getState();
    if (!path.length) {
      setPose(arms.moving, start);
      return;
    }
    if (playing) {
      clock.t += dt * speed;
      if (clock.t > clock.total + 1) clock.t = loop ? 0 : clock.total;  // hold the goal for a second
    }
    setPose(arms.moving, poseAt(path, times, Math.min(clock.t, clock.total)));
  });

  return (
    <>
      <primitive object={arms.moving} />
      {display.ghosts && <primitive object={arms.start} />}
      {display.ghosts && <primitive object={arms.goal} />}
      {display.states && <States robot={robot} path={path} times={times} opacity={display.ghostOpacity} />}
      {/* The probe arm is not in the scene, so the trace is in the service's frame like everything here */}
      {trace && <Line points={trace} color={css('--path')} lineWidth={2.5} depthTest={false} renderOrder={2} />}
    </>
  );
}

// Arms at evenly spaced times along the path, colored from the start color to the goal color
function States({ robot, path, times, opacity }) {
  const arms = useMemo(() => {
    if (path.length < 2) return [];
    const total = times.reduce((a, b) => a + b, 0);
    const count = 12;
    const from = new THREE.Color(css('--state-from'));
    const to = new THREE.Color(css('--state-to'));
    return Array.from({ length: count }, (_, k) => {
      const material = new THREE.MeshStandardMaterial({ color: from.clone().lerp(to, k / (count - 1)), transparent: true, depthWrite: false });
      const arm = clonePanda(robot, material);
      setPose(arm, poseAt(path, times, (total * k) / (count - 1)));
      return { arm, material };
    });
  }, [robot, path, times]);
  for (const { material } of arms) material.opacity = opacity;
  return arms.map(({ arm }, k) => <primitive key={k} object={arm} />);
}
