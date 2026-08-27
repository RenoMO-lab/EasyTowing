# EasyTowing

Engineering foundation for multi-axle trailer steering design, simulation, and optimization.

Current status:

- Repository audit, architecture notes, and the steering model specification are documented.
- Dependency-free Python engineering core is implemented.
- Arbitrary-axle ideal steering, editable primary/companion rigid-link linkage, clearance analysis, deterministic optimization, and export bundle generation are implemented.
- SVG browser demo includes live articulation, project creation, revision history, and restore.
- The browser linkage editor feeds live kinematics, optimization, project snapshots, and engineering exports.
- Steering-curve sweep preview shows ideal and actual linkage response across the full articulation range.
- Swept-path preview and PDF engineering reports are available from the browser export links.
- Manual DXF import assignment and parametric reconstruction are available from the browser.
- Body-chain primitives are in place for future multi-trailer coordination; the current rigid-link solver is not yet a generalized multi-axle tie-rod network.
- Persistent project state is stored in `.easytowing-state/projects.json`.
- Analytical tests cover steering, linkage, collision, optimization, reporting, and project storage.

Remaining planned extensions:

- full multi-trailer coordination
- generalized multi-axle mechanical linkage networks

## Local run

```bash
python -m pip install -e .
python -m unittest discover -s tests -v
python -m easytowing --port 8000
```

Open `http://127.0.0.1:8000` in a browser after starting the demo server.
