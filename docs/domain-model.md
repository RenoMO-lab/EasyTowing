# Domain Model

## Implemented core objects

### `ProjectRevision`

- `id`
- `created_at`
- `note`
- `beta_deg`
- `optimization_mode`
- `wheelbase_mm`
- `track_mm`
- optional normalized `vehicle_config` containing arbitrary axle geometry and metadata
- optional normalized `linkage_config`
- optional `combination_config`, `mechanism_graph_config`, driver arcs, and wheel assignments
- optional full-range sweep validation and optimization evidence in `snapshot`
- optional Monroc acceptance criteria, evaluator identity, and PASS/FAIL evidence

### `ProjectRecord`

- `id`
- `name`
- `created_at`
- `updated_at`
- `active_revision_id`
- `revisions`

### `ProjectStore`

- persistent JSON-backed project collection for local development
- PostgreSQL-backed project/revision collection when configured for the server
- create, append revision, restore revision, and organization scoping operations

### `Point2D`

- `x_mm`
- `y_mm`

### `Pose2D`

- `x_mm`
- `y_mm`
- `yaw_rad`

### `Wheel`

- `id`
- `axle_id`
- `side`
- `center`
- optional tire dimensions for later clearance modeling

### `Axle`

- `id`
- `center`
- `track_mm`
- `wheel_count` plus explicit `wheel_lateral_offsets_mm` for multi-wheel axles;
  the two-wheel case keeps the conventional left/right defaults
- `steerable`
- `steering_mode`
- optional load and user-defined steering angle
- optional steering limits

### `RigidBody`

- `id`
- `name`
- `pose`
- optional rectangular dimensions or a local polygon safety envelope
- `parent_joint_id`
- `child_joint_ids`

### `ArticulationJoint`

- `id`
- `parent_body_id`
- `child_body_id`
- `parent_anchor`
- `child_anchor`
- `articulation_rad`
- `maximum_articulation_deg`, the physical drawbar stop for this joint; the
  default model limit is +/-45 degrees and it is enforced in every pose solve
- optional `sweep_min_deg`, `sweep_max_deg`, and `sweep_step_deg` metadata
  defining the signed range for Cartesian multi-joint validation

### `MountedAxle`

- `axle`
- `body_id`
- `local_center`

### `VehicleCombination`

- `id`
- `name`
- `bodies`
- `joints`
- `mounted_axles`
- `root_body_id`

The combination solver resolves the connected body chain and mounted-axle poses
before ideal steering is calculated. Body envelopes are included in collision
and clearance analysis. Body-to-component contact is excluded only when the
component is mounted on that same body; connected articulation joints do not
hide real body overlap at non-neutral poses. Connected mechanism members are
exempt only at their shared centerline joint; additional overlap remains a
hard collision.

### `VehicleLayout`

- `id`
- `name`
- `axles`
- `body_length_mm`
- `body_width_mm`

The ideal steering solver accepts arbitrary axle counts and signed axle
coordinates. Axle metadata is preserved through project revisions, JSON/CSV
exports, and browser reconstruction. The generalized mechanism graph supports
multiple body-mounted components and named wheel mappings. The browser builder
creates a repeatable reference module per steerable axle and exposes editable
points, rigid members, angle outputs, driver arcs, and wheel mappings,
including shared point connections. CAD-derived topology import, automatic CAD
feature recognition, and production-grade topology authoring remain outside the
current scope.

### `OptimizationVariable`

- `id`
- `current`
- `minimum`
- `maximum`
- `enabled`
- `preferred`

### `OptimizationResult`

- baseline and optimized metrics
- optimized variable set
- improvement summary

## Planned higher-level objects

- `Trailer`
- `Constraint`
- `SimulationResult`
- CAD source and assignment metadata

## Design rules

- Stable unique IDs on all entities
- No array-position identity
- Geometry stored separately from solver outputs
- Solver outputs are immutable snapshots
