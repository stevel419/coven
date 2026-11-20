import { useEffect } from 'react';
import Help from './Help.jsx';
import Panel from './Panel.jsx';
import Viewer from './Viewer.jsx';
import { start } from './service.js';
import { useStore } from './store.js';

let started = false;

export default function App() {
  const help = useStore((s) => s.help);
  const set = useStore((s) => s.set);

  useEffect(() => {
    // StrictMode runs effects twice in development; start the service connection once
    if (!started) {
      started = true;
      start();
    }
  }, []);

  return (
    <div className="app">
      <Viewer />
      <button className="help-button" aria-label="About Coven" onClick={() => set({ help: true })}>?</button>
      <Panel />
      {help && <Help close={() => set({ help: false })} />}
    </div>
  );
}
