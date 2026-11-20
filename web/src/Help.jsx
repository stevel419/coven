import { useEffect, useRef } from 'react';

const Link = ({ href, children }) => <a href={href} target="_blank" rel="noreferrer">{children}</a>;

export default function Help({ close }) {
  const ref = useRef(null);
  useEffect(() => {
    ref.current?.focus();
    const key = (e) => e.key === 'Escape' && close();
    window.addEventListener('keydown', key);
    return () => window.removeEventListener('keydown', key);
  }, [close]);

  return (
    <div className="backdrop" onClick={close}>
      <div className="dialog" role="dialog" aria-modal="true" aria-labelledby="help-title" tabIndex={-1} ref={ref}
        onClick={(e) => e.stopPropagation()}>
        <h2 id="help-title">Coven</h2>
        <p className="tagline">Multi-core roadmap planning and replanning on VAMP</p>
        <p>
          Coven builds VAMP's probabilistic roadmap (PRM) on every CPU core, about 11 times faster than VAMP's serial
          PRM on 16 cores, and gets the same roadmap. This page is a client of <code>coven-server</code>, which keeps
          one roadmap for the Panda and plans for any number of clients in any number of scenes. Every change you make
          is a scene update and every plan a request, over the same HTTP API any program uses. Try the Load panel to
          see several clients share the service.
        </p>

        <h3>Planner: Lazy PRM with batched checks</h3>
        <p>
          A plan request searches the roadmap for its shortest route, assuming every edge not yet checked is free,
          then checks only the edges on that route. Requests that arrive together are planned in rounds: all their
          unchecked edges go into one parallel loop, and an edge two requests need is checked once. A request is
          answered when its whole route is valid. Start the server with <code>--eager</code> to recheck the whole
          roadmap whenever a scene changes instead, which pays off when obstacles move every few milliseconds.
        </p>
        <p>
          Every scene change makes a new scene version. A request plans in the newest version when it is taken in,
          and its reply names that version, so a path is always collision-free in a scene that existed.
        </p>

        <h3>Dataset: MotionBenchMaker</h3>
        <p>
          The benchmark scenes come from MotionBenchMaker, a toolkit for generating and benchmarking motion planning
          problems. Here are the 700 problems for the 7-DoF Franka Emika Panda that VAMP ships: table pick, table under
          pick, box, three bookshelves and cage, 100 problems each.
        </p>

        <h3>References</h3>
        <ul>
          <li>VAMP: Motions in Microseconds via Vectorized Sampling-Based Planning (ICRA 2024){' '}
            <Link href="https://doi.org/10.1109/ICRA57147.2024.10611190">(Publisher)</Link>{' '}
            <Link href="https://arxiv.org/abs/2309.14545">(arXiv)</Link></li>
          <li>Path Planning Using Lazy PRM (ICRA 2000){' '}
            <Link href="https://doi.org/10.1109/ROBOT.2000.844107">(Publisher)</Link></li>
          <li>MotionBenchMaker: A Tool to Generate and Benchmark Motion Planning Datasets (RA-L 2022){' '}
            <Link href="https://doi.org/10.1109/LRA.2021.3133603">(Publisher)</Link>{' '}
            <Link href="https://arxiv.org/abs/2108.01584">(arXiv)</Link></li>
        </ul>

        <h3>Source code</h3>
        <ul className="links">
          <li><Link href="https://github.com/stevel419/coven">Coven on GitHub</Link></li>
          <li><Link href="https://github.com/KavrakiLab/vamp">VAMP on GitHub</Link></li>
          <li><Link href="https://github.com/KavrakiLab/motion_bench_maker">MotionBenchMaker on GitHub</Link></li>
        </ul>

        <button className="close" onClick={close}>Close</button>
      </div>
    </div>
  );
}
