import { useEffect, useState } from 'react';
import { capsule, cuboid, PRESETS, scenarioLabel, sphere, useStore } from './store.js';
import { loadProblem, MAX_CLIENTS, planNow, SCENE, setClients } from './service.js';
import { clock } from './Viewer.jsx';

// The Panda's joint limits (radians)
const LOWER = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973];
const UPPER = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973];

const fmt = (v, digits = 2) => (v === undefined || v === null ? '–' : Number(v).toFixed(digits));

function Section({ title, children, open = true }) {
  return (
    <details className="section" open={open}>
      <summary>{title}</summary>
      <div className="body">{children}</div>
    </details>
  );
}

function Number3({ label, value, step, onChange, scale = 1 }) {
  return (
    <div className="triple">
      <span>{label}</span>
      {value.map((v, i) => (
        <input key={i} type="number" step={step} value={Number((v * scale).toFixed(3))} aria-label={`${label} ${'xyz'[i]}`}
          onChange={(e) => e.target.value !== '' && onChange(value.map((w, j) => (j === i ? Number(e.target.value) / scale : w)))} />
      ))}
    </div>
  );
}

function Field({ label, value, step, min, onChange }) {
  return (
    <label className="field">
      <span>{label}</span>
      <input type="number" step={step} min={min} value={Number(value.toFixed(3))}
        onChange={(e) => e.target.value !== '' && Number(e.target.value) >= min && onChange(Number(e.target.value))} />
    </label>
  );
}

export default function Panel() {
  const s = useStore();
  const selected = s.obstacles.find((o) => o.id === s.selected);
  const q = s[s.editing];
  const DEG = 180 / Math.PI;

  return (
    <aside className="panel">
      <header>
        <h1>Coven</h1>
        {!s.server && <p className="status bad">Service not reachable</p>}
      </header>

      <Section title="Scene">
        <select value={s.scenario} aria-label="Scenario" onChange={(e) => loadProblem(e.target.value, 0)}>
          {s.scenarios.map((name) => <option key={name} value={name}>{scenarioLabel(name)}</option>)}
        </select>
        {s.problemCount > 1 && (
          <label className="slider">
            <span>Problem {s.problem + 1} / {s.problemCount}</span>
            <input type="range" min="0" max={s.problemCount - 1} value={s.problem}
              onChange={(e) => loadProblem(s.scenario, Number(e.target.value))} />
          </label>
        )}
        <div className="sub">Scene <code>{SCENE}</code>{s.version ? `, version ${s.version}` : ''}</div>
      </Section>

      <Section title={`Obstacles (${s.obstacles.length})`}>
        <div className="row">
          <button onClick={() => s.addObstacle(sphere(`sphere ${s.obstacles.length + 1}`, [0.45, 0.2, 0.55], 0.1))}>+ Sphere</button>
          <button onClick={() => s.addObstacle(cuboid(`box ${s.obstacles.length + 1}`, [0.45, 0.2, 0.4], [0.1, 0.1, 0.1]))}>+ Box</button>
          <button onClick={() => s.addObstacle(capsule(`capsule ${s.obstacles.length + 1}`, [0.45, 0.2, 0.5], 0.05, 0.2))}>+ Capsule</button>
        </div>
        <select value={s.selected ?? ''} aria-label="Selected obstacle" onChange={(e) => s.set({ selected: e.target.value ? Number(e.target.value) : null })}>
          <option value="">Select an obstacle…</option>
          {s.obstacles.map((o) => <option key={o.id} value={o.id}>{o.name} ({o.kind})</option>)}
        </select>
        {selected && (
          <>
            <div className="segmented" role="group" aria-label="Drag mode">
              {['translate', 'rotate'].map((m) => (
                <button key={m} aria-pressed={s.mode === m} onClick={() => s.set({ mode: m })}>{m === 'translate' ? 'Move' : 'Rotate'}</button>
              ))}
            </div>
            <Number3 label="Position" step={0.01} value={selected.position} onChange={(position) => s.setObstacle(selected.id, { position })} />
            {selected.kind !== 'sphere' && (
              <Number3 label="Rotation °" step={5} scale={DEG} value={selected.euler} onChange={(euler) => s.setObstacle(selected.id, { euler })} />
            )}
            {selected.kind === 'cuboid' && (
              <Number3 label="Half size" step={0.01} value={selected.halfExtents}
                onChange={(halfExtents) => halfExtents.every((h) => h > 0) && s.setObstacle(selected.id, { halfExtents })} />
            )}
            <div className="row">
              {selected.kind !== 'cuboid' && <Field label="Radius" step={0.01} min={0.005} value={selected.radius} onChange={(radius) => s.setObstacle(selected.id, { radius })} />}
              {selected.kind === 'capsule' && <Field label="Length" step={0.01} min={0} value={selected.length} onChange={(length) => s.setObstacle(selected.id, { length })} />}
            </div>
            <div className="row">
              <button onClick={() => s.removeObstacle(selected.id)}>Delete</button>
              <button onClick={() => s.set({ selected: null })}>Done</button>
            </div>
          </>
        )}
      </Section>

      <Section title="Start and goal">
        <div className="row">
          <div className="segmented" role="group" aria-label="Pose to edit">
            {['start', 'goal'].map((w) => (
              <button key={w} aria-pressed={s.editing === w} onClick={() => s.set({ editing: w })}>{w === 'start' ? 'Start' : 'Goal'}</button>
            ))}
          </div>
          <select value="" aria-label="Preset pose" onChange={(e) => e.target.value && s.set({ [s.editing]: PRESETS[e.target.value] })}>
            <option value="">Preset…</option>
            {Object.keys(PRESETS).map((name) => <option key={name}>{name}</option>)}
          </select>
        </div>
        {q.map((v, i) => (
          <label className="joint" key={i}>
            <span>q{i + 1}</span>
            <input type="range" min={LOWER[i]} max={UPPER[i]} step="0.01" value={v} aria-label={`Joint ${i + 1}`}
              onChange={(e) => s.setJoint(s.editing, i, Number(e.target.value))} />
            <span className="value">{fmt(v)}</span>
          </label>
        ))}
      </Section>

      <Section title="Plan">
        <div className="row">
          <button className="primary" onClick={planNow}>Plan</button>
          <label className="check"><input type="checkbox" checked={s.auto} onChange={(e) => s.set({ auto: e.target.checked })} /> Replan on every change</label>
        </div>
        <p className={s.status.bad ? 'status bad' : 'status'}>{s.status.text}</p>
        <dl>
          <dt>Waypoints</dt><dd>{s.path.length || '–'}</dd>
          <dt>Scene version</dt><dd>{s.reply?.version ?? '–'}</dd>
          <dt>Rounds</dt><dd>{s.reply?.rounds ?? '–'}</dd>
          <dt>Collision checks</dt><dd>{s.reply?.checks ?? '–'}</dd>
          <dt>Searches</dt><dd>{s.reply?.searches ?? '–'}</dd>
          <dt>Service time</dt><dd>{s.reply ? `${fmt(s.reply.total_ms)} ms` : '–'}</dd>
          <dt>Round trip</dt><dd>{s.rtt ? `${fmt(s.rtt, 1)} ms` : '–'}</dd>
        </dl>
      </Section>

      <Section title="Playback">
        <div className="row">
          <button onClick={() => s.set({ playing: !s.playing })} disabled={!s.path.length}>{s.playing ? 'Pause' : 'Play'}</button>
          <button onClick={() => { clock.t = 0; }} disabled={!s.path.length}>Restart</button>
          <label className="check"><input type="checkbox" checked={s.loop} onChange={(e) => s.set({ loop: e.target.checked })} /> Loop</label>
        </div>
        <Scrubber enabled={s.path.length > 0} />
        <label className="slider">
          <span>Speed {fmt(s.speed, 2)}×</span>
          <input type="range" min="0.25" max="3" step="0.25" value={s.speed} onChange={(e) => s.set({ speed: Number(e.target.value) })} />
        </label>
      </Section>

      <Section title="Display" open={false}>
        <label className="check"><input type="checkbox" checked={s.display.colliders} onChange={(e) => s.setDisplay({ colliders: e.target.checked })} /> Collision spheres</label>
        <label className="check"><input type="checkbox" checked={s.display.ghosts} onChange={(e) => s.setDisplay({ ghosts: e.target.checked })} /> Start and goal ghosts</label>
        <label className="check"><input type="checkbox" checked={s.display.states} onChange={(e) => s.setDisplay({ states: e.target.checked })} /> States along the path</label>
        <label className="slider">
          <span>Ghost opacity {fmt(s.display.ghostOpacity)}</span>
          <input type="range" min="0.05" max="1" step="0.05" value={s.display.ghostOpacity} onChange={(e) => s.setDisplay({ ghostOpacity: Number(e.target.value) })} />
        </label>
        <label className="slider">
          <span>Obstacle opacity {fmt(s.display.obstacleOpacity)}</span>
          <input type="range" min="0.1" max="1" step="0.05" value={s.display.obstacleOpacity} onChange={(e) => s.setDisplay({ obstacleOpacity: Number(e.target.value) })} />
        </label>
      </Section>

      <Section title="Load">
        <p className="sub">Simulated clients replan this query as fast as they can, sharing the scene and its checks.</p>
        <label className="slider">
          <span>Clients {s.clients}</span>
          <input type="range" min="0" max={MAX_CLIENTS} step="1" value={s.clients} onChange={(e) => setClients(Number(e.target.value))} />
        </label>
        <dl>
          <dt>Plans per second</dt><dd>{s.load ? fmt(s.load.rate, 0) : '–'}</dd>
          <dt>Latency p50</dt><dd>{s.load ? `${fmt(s.load.p50, 1)} ms` : '–'}</dd>
          <dt>Latency p99</dt><dd>{s.load ? `${fmt(s.load.p99, 1)} ms` : '–'}</dd>
        </dl>
        <h3>Service</h3>
        <dl>
          <dt>Roadmap vertices</dt><dd>{s.server?.vertices.toLocaleString() ?? '–'}</dd>
          <dt>Threads</dt><dd>{s.server?.threads ?? '–'}</dd>
          <dt>Edge checking</dt><dd>{s.server ? (s.server.eager ? 'eager' : 'lazy') : '–'}</dd>
          <dt>Requests</dt><dd>{s.server?.requests.toLocaleString() ?? '–'}</dd>
          <dt>Scene updates</dt><dd>{s.server?.updates.toLocaleString() ?? '–'}</dd>
          <dt>Rounds</dt><dd>{s.server?.rounds.toLocaleString() ?? '–'}</dd>
          <dt>Checks per round</dt><dd>{s.server?.rounds ? fmt(s.server.checks / s.server.rounds, 1) : '–'}</dd>
        </dl>
      </Section>
    </aside>
  );
}

// The frame slider follows playback, and dragging it pauses
function Scrubber({ enabled }) {
  const [, redraw] = useState(0);
  useEffect(() => {
    const timer = setInterval(() => redraw((n) => n + 1), 100);
    return () => clearInterval(timer);
  }, []);
  const t = Math.min(clock.t, clock.total);
  return (
    <label className="slider">
      <span>Time {enabled ? `${fmt(t, 1)} / ${fmt(clock.total, 1)} s` : '–'}</span>
      <input type="range" min="0" max={clock.total || 1} step="0.01" value={t} disabled={!enabled}
        onChange={(e) => { clock.t = Number(e.target.value); useStore.getState().set({ playing: false }); }} />
    </label>
  );
}

