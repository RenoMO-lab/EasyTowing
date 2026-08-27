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
- `snapshot`

### `ProjectRecord`

- `id`
- `name`
- `created_at`
- `updated_at`
- `active_revision_id`
- `revisions`

### `ProjectStore`

- persistent JSON-backed project collection
- create, append revision, and restore revision operations

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
- `wheel_count`
- `steerable`
- `steering_mode`
- optional load and user-defined steering angle
- optional steering limits

### `RigidBody`

- `id`
- `name`
- `pose`
- optional body dimensions for summary/output
- `parent_joint_id`
- `child_joint_ids`

### `ArticulationJoint`

- `id`
- `parent_body_id`
- `child_body_id`
- `parent_anchor`
- `child_anchor`
- `articulation_rad`

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

### `VehicleLayout`

- `id`
- `name`
- `axles`
- `body_length_mm`
- `body_width_mm`

The ideal steering solver accepts arbitrary axle counts and signed axle
coordinates. Axle metadata is preserved through project revisions, JSON/CSV
exports, and browser reconstruction. The current mechanical solver remains
limited to its primary and optional companion linkage paths.

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

## Planned objects

- `Trailer`
- `SteeringPivot`
- `SteeringArm`
- `TieRod`
- `BellCrank`
- `Constraint`
- `SimulationResult`

## Design rules

- Stable unique IDs on all entities
- No array-position identity
- Geometry stored separately from solver outputs
- Solver outputs are immutable snapshots
