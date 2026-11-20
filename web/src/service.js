// Everything the page sends to the planning service: the scene, plan requests, problem sets, and the
// simulated clients of the load panel.
import { BUILT_IN, fromProblem, sceneBody, useStore } from './store.js';

// One scene per browser tab, so two tabs do not overwrite each other
export const SCENE = `web-${Math.random().toString(36).slice(2, 7)}`;

export async function api(method, path, body) {
  const response = await fetch(path, {
    method,
    headers: { 'Content-Type': 'application/json' },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const data = response.status === 204 ? null : await response.json();
  if (!response.ok) throw new Error(data?.error || `HTTP ${response.status}`);
  return data;
}

// Scene updates and plans, one request at a time; changes made meanwhile go out when it returns
let busy = false;
let sceneDirty = false;
let planDirty = false;
let resent = false;  // the scene was just sent again after the server had dropped it

export function planNow() {
  planDirty = true;
  pump();
}

async function pump() {
  if (busy || !(sceneDirty || planDirty)) return;
  busy = true;
  const { set } = useStore.getState();
  try {
    if (sceneDirty) {
      sceneDirty = false;
      const { version } = await api('PUT', `/scenes/${SCENE}`, sceneBody(useStore.getState().obstacles));
      set({ version });
    }
    if (planDirty) {
      planDirty = false;
      const { start, goal } = useStore.getState();
      const begin = performance.now();
      const reply = await api('POST', `/scenes/${SCENE}/plan`, { start, goal });
      const rtt = performance.now() - begin;
      resent = false;
      const n = reply.path.length;
      set({
        reply, rtt, path: reply.path,
        status: n ? { text: n === 2 ? 'Path found: the straight line is free.' : `Path found through ${n - 2} roadmap vertices.`, bad: false }
          : { text: 'No path: the start or goal is in collision, or the roadmap has no free route.', bad: true },
      });
    }
  } catch (e) {
    if (e.message === 'unknown_scene' && !resent) {
      // The server dropped this tab's scene (restart, or unused for a while): send it again, once
      resent = sceneDirty = planDirty = true;
    } else {
      set({ status: { text: e.message, bad: true } });
    }
  } finally {
    busy = false;
    pump();
  }
}

// Watch the store: obstacle changes update the scene, and any change replans when that is on
useStore.subscribe((state, previous) => {
  const sceneChanged = state.obstacles !== previous.obstacles;
  const queryChanged = sceneChanged || state.start !== previous.start || state.goal !== previous.goal;
  if (sceneChanged) sceneDirty = true;
  if (queryChanged && state.auto) planDirty = true;
  if (queryChanged && !state.auto) state.set({ path: [] });
  pump();
});

// Problem sets: the built-in ones, and the MotionBenchMaker scenarios the service provides
const problemSets = {};

export async function loadProblem(scenario, index) {
  const { set } = useStore.getState();
  if (!problemSets[scenario]) {
    problemSets[scenario] = BUILT_IN[scenario]
      ? BUILT_IN[scenario].problems()
      : (await api('GET', `/problems/${scenario}`)).map(fromProblem);
  }
  const problems = problemSets[scenario];
  const problem = problems[Math.min(index, problems.length - 1)];
  set({
    scenario, problem: index, problemCount: problems.length, selected: null, path: [],
    obstacles: problem.obstacles.map((o) => ({ ...o })), start: problem.start, goal: problem.goal,
  });
}

export async function start() {
  const { set } = useStore.getState();
  try {
    const scenarios = await api('GET', '/problems');
    set({ scenarios: [...Object.keys(BUILT_IN), ...scenarios] });
    await loadProblem(scenarios.includes('table_pick') ? 'table_pick' : 'custom', 0);
  } catch {
    await loadProblem('custom', 0);  // no MotionBenchMaker problems without internet access
  }
  pollStats();
  setInterval(pollStats, 1000);
}

async function pollStats() {
  try {
    useStore.getState().set({ server: await api('GET', '/stats') });
  } catch {
    useStore.getState().set({ server: null });
  }
}

// Load panel: simulated clients replanning the current query as fast as they can. Browsers open at
// most six connections per server, and the page itself needs one.
export const MAX_CLIENTS = 5;
let clients = [];
let samples = [];  // [time, latency ms]

export function setClients(n) {
  useStore.getState().set({ clients: n });
  while (clients.length < n) {
    const client = { running: true };
    clients.push(client);
    (async () => {
      while (client.running) {
        const { start: from, goal: to } = useStore.getState();
        const begin = performance.now();
        try {
          await api('POST', `/scenes/${SCENE}/plan`, { start: from, goal: to });
          samples.push([performance.now(), performance.now() - begin]);
        } catch {
          await new Promise((r) => setTimeout(r, 200));
        }
      }
    })();
  }
  while (clients.length > n) clients.pop().running = false;
}

setInterval(() => {
  const now = performance.now();
  samples = samples.filter(([t]) => now - t < 2000);
  const { clients: n, set } = useStore.getState();
  if (!n) {
    set({ load: null });
    return;
  }
  const latencies = samples.map(([, l]) => l).sort((a, b) => a - b);
  const at = (p) => latencies[Math.min(latencies.length - 1, Math.floor(p * latencies.length))] ?? 0;
  set({ load: { rate: samples.length / 2, p50: at(0.5), p99: at(0.99) } });
}, 500);
