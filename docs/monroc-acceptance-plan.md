# Monroc Acceptance Plan

Status: draft for Monroc review. No criterion in this document is approved
until a Monroc design owner signs off the value and the representative cases.

## Release rule

A design can be presented as an engineering candidate only when the saved
revision contains:

1. A complete body, axle, articulation, mechanism, and wheel-mapping definition.
2. A passing current-pose hard-check result.
3. A passing full Cartesian articulation-range result for every configured
   design case, with every joint range and grid step recorded.
4. A comparison against an approved reference design or an independent hand
   calculation.
5. Independent reviewer approval by a different user from the submitter.

The application must continue to label a failed or incomplete result as
diagnostic evidence. A passing calculation is not a manufacturing release by
itself.

## Criteria to approve

| Criterion | Proposed gate | Monroc value / method | Evidence |
| --- | --- | --- | --- |
| Mechanism closure | Every rigid-member residual is within the configured solver tolerance | Confirm tolerance and worst-case method | Per-pose residuals |
| Collision | No non-connected body, tire, point, or member collision | Confirm envelope definitions and contact policy | Minimum pair and failing pose |
| Clearance | Minimum clearance is at or above the configured target | `TBD` mm by component class and load case | Minimum value, pair, and pose |
| Steering accuracy | Maximum actual-versus-ideal wheel error is within the approved limit | `TBD` deg by axle and maneuver case | Wheel error curves and maximum |
| Synchronization | Linked axle phase/ratio error is within the approved limit | `TBD` deg or ratio tolerance | Synchronization channels |
| Articulation | Every requested pose is within the approved drawbar and steering stops | Confirm positive and negative limits | Sweep bounds and stop checks |
| Maneuver definition | The radius/ICR and sign convention are traceable to the test case | Confirm maneuver source and convention | Saved maneuver inputs |
| Manual calculation agreement | Tool output agrees with an independent calculation within the approved tolerance | `TBD` tolerance and calculation owner | Signed comparison sheet |
| CAD traceability | Imported or entered geometry can be identified by source, revision, and mapping review | Confirm CAD formats and metadata | Source file hash and assignment review |
| Export control | Only a passing, independently approved revision can be called a release artifact | Confirm document and CAD release process | Approval event and export manifest |

## Representative case matrix

Populate one row per existing approved design before enabling Monroc release
use. The current repository deliberately contains no customer geometry or
approved values.

| Case ID | Combination | Bodies / axles | Articulation range | CAD source and revision | Hand calculation | Approved reference | Owner | Status |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `MONROC-01` | `TBD` | `TBD` | `TBD` | `TBD` | `TBD` | `TBD` | `TBD` | Not started |
| `MONROC-02` | `TBD` | `TBD` | `TBD` | `TBD` | `TBD` | `TBD` | `TBD` | Not started |
| `MONROC-03` | `TBD` | `TBD` | `TBD` | `TBD` | `TBD` | `TBD` | `TBD` | Not started |

At minimum, the pilot should cover one single-body design, one two-body
combination, and one chained multi-body combination if those designs exist in
the approved Monroc catalogue.

## Validation procedure

1. Freeze the CAD revision, geometry units, coordinate convention, tire/load
   assumptions, and maneuver definition.
2. Enter or import the model and record every CAD-to-model assignment.
3. Run the current-pose and full-range checks without optimization.
4. Compare ideal and actual steering against the approved design and hand
   calculation.
5. Run optimization only inside Monroc-approved variable bounds; retain the
   baseline and optimized evidence separately.
6. Repeat the checks in the browser as a designer and as an independent
   reviewer.
7. Record discrepancies, corrective actions, and final disposition in the
   project revision and audit history.

## Approval record

Before this plan becomes a release policy, record:

- Monroc design owner and reviewer roles.
- Approved values for every `TBD` criterion.
- The CAD source location, revision, and checksum for each case.
- The independent calculation owner and signed comparison.
- The decision date and the first software version accepted for pilot use.
