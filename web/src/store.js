import { create } from 'zustand';

export const JOINTS = 7;
export const PRESETS = {
  Ready: [0, -0.785, 0, -2.356, 0, 1.571, 0.785],
  'Reach left': [2.35, 1, 0, -0.8, 0, 2.5, 0.785],
  'Reach right': [-2.35, 1, 0, -0.8, 0, 2.5, 0.785],
  'Low front': [0, 0.6, 0, -2.0, 0, 2.6, 0.785],
};

// VAMP's sphere cage example: 14 spheres of radius 0.2 around the robot
const CAGE = [[0.55, 0, 0.25], [0.35, 0.35, 0.25], [0, 0.55, 0.25], [-0.55, 0, 0.25], [-0.35, -0.35, 0.25],
  [0, -0.55, 0.25], [0.35, -0.35, 0.25], [0.35, 0.35, 0.8], [0, 0.55, 0.8], [-0.35, 0.35, 0.8], [-0.55, 0, 0.8],
  [-0.35, -0.35, 0.8], [0, -0.55, 0.8], [0.35, -0.35, 0.8]];

export const BUILT_IN = {
  custom: { problems: () => [{ obstacles: [sphere('sphere 1', [0.45, 0.2, 0.55], 0.12)],
    start: PRESETS.Ready, goal: PRESETS['Reach left'] }] },
  sphere_cage: { problems: () => [{
    obstacles: CAGE.map((c, i) => sphere(`sphere ${i + 1}`, c, 0.2)), start: PRESETS.Ready, goal: PRESETS['Reach left'] }] },
};

// "table_under_pick" -> "Table under pick"
export const scenarioLabel = (name) => name.charAt(0).toUpperCase() + name.slice(1).replaceAll('_', ' ');

let nextId = 1;
const base = (kind, name, position) => ({ id: nextId++, kind, name, position, euler: [0, 0, 0] });
export const sphere = (name, position, radius) => ({ ...base('sphere', name, position), radius });
export const cuboid = (name, position, halfExtents, euler = [0, 0, 0]) => ({ ...base('cuboid', name, position), euler, halfExtents });
export const capsule = (name, position, radius, length, euler = [0, 0, 0]) =>
  ({ ...base('capsule', name, position), euler, radius, length });

// A MotionBenchMaker problem from the service, in the page's form
export function fromProblem({ scene, start, goal }) {
  return {
    start, goal,
    obstacles: [
      ...scene.spheres.map((s) => sphere(s.name, s.center, s.radius)),
      ...scene.cuboids.map((c) => cuboid(c.name, c.center, c.half_extents, c.euler)),
      ...scene.capsules.map((c) => capsule(c.name, c.center, c.radius, c.length, c.euler)),
    ],
  };
}

// The scene body the service takes
export function sceneBody(obstacles) {
  const of = (kind) => obstacles.filter((o) => o.kind === kind);
  return {
    spheres: of('sphere').map((o) => ({ center: o.position, radius: o.radius })),
    cuboids: of('cuboid').map((o) => ({ center: o.position, euler: o.euler, half_extents: o.halfExtents })),
    capsules: of('capsule').map((o) => ({ center: o.position, euler: o.euler, radius: o.radius, length: o.length })),
  };
}

export const useStore = create((set) => ({
  // Scene
  scenarios: Object.keys(BUILT_IN),
  scenario: 'custom',
  problem: 0,
  problemCount: 1,
  obstacles: [],
  selected: null,
  mode: 'translate',
  version: null,

  // Query
  start: PRESETS.Ready,
  goal: PRESETS['Reach left'],
  editing: 'start',
  auto: true,

  // Result
  path: [],
  reply: null,
  rtt: null,
  status: { text: 'No plan yet.', bad: false },

  // Playback and display
  playing: true,
  loop: true,
  speed: 1,
  display: { colliders: false, ghosts: true, states: false, ghostOpacity: 0.3, obstacleOpacity: 0.45 },

  // Service
  server: null,
  clients: 0,
  load: null,
  help: false,

  set,
  setObstacle: (id, change) => set((s) => ({ obstacles: s.obstacles.map((o) => (o.id === id ? { ...o, ...change } : o)) })),
  addObstacle: (o) => set((s) => ({ obstacles: [...s.obstacles, o], selected: o.id })),
  removeObstacle: (id) => set((s) => ({ obstacles: s.obstacles.filter((o) => o.id !== id), selected: null })),
  setJoint: (which, i, value) => set((s) => ({ [which]: s[which].map((v, j) => (j === i ? value : v)) })),
  setDisplay: (change) => set((s) => ({ display: { ...s.display, ...change } })),
}));
