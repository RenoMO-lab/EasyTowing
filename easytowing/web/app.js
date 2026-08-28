const state = {
  activeRequest: 0,
  optimizationRequest: 0,
  projectRequest: 0,
  viewRequest: 0,
  dxfImportRequest: 0,
  dxfImportText: "",
  dxfImportSourceName: "",
  dxfImportPayload: null,
  cadSourceArtifact: null,
  interactiveEdit: false,
  draggingAxleId: null,
  currentProjectId: null,
  activeProjectRevisionId: null,
  authToken: sessionStorage.getItem("easytowing_auth_token") || null,
  authPrincipal: null,
  authRequired: false,
  artifactStorageBackend: "response-only",
  reviewerUsers: [],
  approvalStatus: null,
  approvalHistory: [],
  currentPayload: null,
  maneuverResolved: false,
  optimizationPayload: null,
  currentValidationPass: false,
  sweepValidationPayload: null,
  acceptanceResult: null,
  acceptanceCriteriaDirty: false,
  workspaceDirty: false,
  activeRevisionHasFullRangeEvidence: false,
  activeWorkflowStep: "project",
  // New work starts in the explicit multi-body path. Loading an existing
  // legacy revision still switches this back to false in renderProjectFromDetail.
  combinationActive: true,
  combinationId: "workspace_combination",
  combinationName: "Workspace vehicle combination",
  combinationSynchronizations: [],
  combinationBodies: [
    {
      id: "body_1",
      name: "Rear body",
      lengthMm: 1800,
      widthMm: 3200,
      bodyPolygon: [],
      bodyPolygonText: "",
      bodyPolygonError: null,
      parentBodyId: null,
      parentJointId: null,
      articulationDeg: 0,
      articulationMinDeg: -45,
      articulationMaxDeg: 45,
      articulationStepDeg: 5,
      articulationLimitDeg: 45,
      parentAnchorXmm: 0,
      parentAnchorYmm: 0,
      childAnchorXmm: 0,
      childAnchorYmm: 0,
      axles: [{ id: "body_1_axle_1", xMm: 0, yMm: 0, trackMm: 2500, mode: "FORCED_STEER", wheelCount: 2, maximumSteeringAngleDeg: 45, steeringStopDeg: null, tireWidthMm: 400, outsideDiameterMm: 1000 }],
    },
    {
      id: "body_2",
      name: "Front body",
      lengthMm: 1800,
      widthMm: 3200,
      bodyPolygon: [],
      bodyPolygonText: "",
      bodyPolygonError: null,
      parentBodyId: "body_1",
      parentJointId: "joint_2",
      articulationDeg: 0,
      articulationMinDeg: -45,
      articulationMaxDeg: 45,
      articulationStepDeg: 5,
      articulationLimitDeg: 45,
      parentAnchorXmm: 2180,
      parentAnchorYmm: 0,
      childAnchorXmm: -2180,
      childAnchorYmm: 0,
      axles: [{ id: "body_2_axle_1", xMm: 0, yMm: 0, trackMm: 2500, mode: "FORCED_STEER", wheelCount: 2, maximumSteeringAngleDeg: 45, steeringStopDeg: null, tireWidthMm: 400, outsideDiameterMm: 1000 }],
    },
  ],
  mechanismGraph: null,
  mechanismGraphEditorDraft: null,
  mechanismDrivers: [],
  steeringAssignments: [],
  customAxles: [],
  vehicleConfig: null,
  designCases: [],
  linkageConfig: null,
  optimizationEnabledIds: null,
  geometry: {
    wheelbaseMm: 4360,
    trackMm: 2500,
    bodyLengthMm: 6160,
    bodyWidthMm: 3200,
    origin: { x_mm: 0, y_mm: 0 },
    bodyPolygon: [],
    frontArticulationPoint: null,
    rearArticulationPoint: null,
    kingpinPoint: null,
    maximumArticulationDeg: 45,
  },
  betaRange: {
    minDeg: -45,
    maxDeg: 45,
  },
  optimizationSettings: {
    clearanceTargetMm: 20,
    steeringErrorWeight: 1,
    synchronizationErrorWeight: 0.5,
    clearanceWeight: 12,
    clearanceViolationWeight: 250,
    failureWeight: 100000,
    preferredWeight: 0.05,
    complexityWeight: 0.02,
  },
  displayMode: localStorage.getItem("easytowing_display_mode") || "simulation",
  dimensionedSketchUrl: null,
};

const diagram = document.getElementById("diagram");
const dimensionedSketch = document.getElementById("dimensioned-sketch");
const betaSlider = document.getElementById("beta-slider");
const betaValue = document.getElementById("beta-value");
const radiusValue = document.getElementById("radius-value");
const maxAngleValue = document.getElementById("max-angle-value");
const phaseValue = document.getElementById("phase-value");
const actualErrorValue = document.getElementById("actual-error-value");
const synchronizationErrorValue = document.getElementById("synchronization-error-value");
const betaLabel = document.getElementById("beta-label");
const statusModeValue = document.getElementById("status-mode-value");
const diagramTitle = document.getElementById("diagram-title");
const diagramDescription = document.getElementById("diagram-description");
const viewModeSelect = document.getElementById("view-mode-select");
const radiusChip = document.getElementById("radius-chip");
const wheelTable = document.getElementById("wheel-table");
const synchronizationTable = document.getElementById("synchronization-table");
const linkageSteerValue = document.getElementById("linkage-steer-value");
const linkageErrorValue = document.getElementById("linkage-error-value");
const linkageResidualValue = document.getElementById("linkage-residual-value");
const linkageBranchValue = document.getElementById("linkage-branch-value");
const clearanceValue = document.getElementById("clearance-value");
const clearancePairValue = document.getElementById("clearance-pair-value");
const clearanceStatusValue = document.getElementById("clearance-status-value");
const optimizeMode = document.getElementById("optimize-mode");
const optimizeButton = document.getElementById("optimize-button");
const optimizeCompareButton = document.getElementById("optimize-compare-button");
const optimizeApplyButton = document.getElementById("optimize-apply-button");
const optimizeRejectButton = document.getElementById("optimize-reject-button");
const optimizeBaselineScore = document.getElementById("opt-baseline-score");
const optimizeOptimizedScore = document.getElementById("opt-optimized-score");
const optimizeBaselineRms = document.getElementById("opt-baseline-rms");
const optimizeOptimizedRms = document.getElementById("opt-optimized-rms");
const optimizeBaselineClearance = document.getElementById("opt-baseline-clearance");
const optimizeOptimizedClearance = document.getElementById("opt-optimized-clearance");
const optimizeRunStats = document.getElementById("opt-run-stats");
const optimizeFeasibilityCard = document.getElementById("opt-feasibility-card");
const optimizeFeasibilityStatus = document.getElementById("opt-feasibility-status");
const optimizeFeasibilityReasons = document.getElementById("opt-feasibility-reasons");
const optimizeVariableTable = document.getElementById("opt-variable-table");
const optimizeVariableConfig = document.getElementById("opt-variable-config");
const optimizeClearanceTarget = document.getElementById("opt-clearance-target");
const optimizeSteeringWeight = document.getElementById("opt-steering-weight");
const optimizeSynchronizationWeight = document.getElementById("opt-synchronization-weight");
const optimizeClearanceWeight = document.getElementById("opt-clearance-weight");
const optimizeClearanceViolationWeight = document.getElementById("opt-clearance-violation-weight");
const optimizeFailureWeight = document.getElementById("opt-failure-weight");
const optimizePreferredWeight = document.getElementById("opt-preferred-weight");
const optimizeComplexityWeight = document.getElementById("opt-complexity-weight");
const designCaseAddButton = document.getElementById("design-case-add-button");
const designCaseConfig = document.getElementById("design-case-config");
const designCaseStatus = document.getElementById("design-case-status");
const exportJsonLink = document.getElementById("export-json");
const exportCsvLink = document.getElementById("export-csv");
const exportPdfLink = document.getElementById("export-pdf");
const exportPngLink = document.getElementById("export-png");
const exportSvgLink = document.getElementById("export-svg");
const exportDxfLink = document.getElementById("export-dxf");
const exportReleaseLink = document.getElementById("export-release");
const exportNote = document.getElementById("export-note");
const dxfFileInput = document.getElementById("dxf-file-input");
const dxfImportButton = document.getElementById("dxf-import-button");
const dxfApplyButton = document.getElementById("dxf-apply-button");
const dxfRetainSourceButton = document.getElementById("dxf-retain-source-button");
const dxfImportStatus = document.getElementById("dxf-import-status");
const dxfMetadataStatus = document.getElementById("dxf-metadata-status");
const dxfSourceRetentionStatus = document.getElementById("dxf-source-retention-status");
const dxfSourceUnits = document.getElementById("dxf-source-units");
const dxfCoordinateSystem = document.getElementById("dxf-coordinate-system");
const dxfEntityCount = document.getElementById("dxf-entity-count");
const dxfSupportedCount = document.getElementById("dxf-supported-count");
const dxfBoundsValue = document.getElementById("dxf-bounds-value");
const dxfLayoutValue = document.getElementById("dxf-layout-value");
const dxfParametricValue = document.getElementById("dxf-parametric-value");
const dxfEntityTable = document.getElementById("dxf-entity-table");
const steeringCurvesImage = document.getElementById("steering-curves-image");
const sweptPathImage = document.getElementById("swept-path-image");
const steeringCurvesStatus = document.getElementById("steering-curves-status");
const sweptPathStatus = document.getElementById("swept-path-status");
const projectSelector = document.getElementById("project-selector");
const projectNameInput = document.getElementById("project-name-input");
const projectCreateButton = document.getElementById("project-create-button");
const projectSaveButton = document.getElementById("project-save-button");
const projectNoteInput = document.getElementById("project-note-input");
const projectIdValue = document.getElementById("project-id-value");
const projectActiveRevisionValue = document.getElementById("project-active-revision-value");
const projectRevisionCountValue = document.getElementById("project-revision-count-value");
const projectEngineeringCard = document.getElementById("project-engineering-card");
const projectEngineeringState = document.getElementById("project-engineering-state");
const projectEngineeringDetail = document.getElementById("project-engineering-detail");
const projectReviewCard = document.getElementById("project-review-card");
const projectReviewState = document.getElementById("project-review-state");
const projectReviewDetail = document.getElementById("project-review-detail");
const projectScopeCard = document.getElementById("project-scope-card");
const projectModelScope = document.getElementById("project-model-scope");
const projectModelDetail = document.getElementById("project-model-detail");
const projectRevisionList = document.getElementById("project-revision-list");
const authStatus = document.getElementById("auth-status");
const authForm = document.getElementById("auth-form");
const authOrganizationInput = document.getElementById("auth-organization");
const authEmailInput = document.getElementById("auth-email");
const authPasswordInput = document.getElementById("auth-password");
const authLoginButton = document.getElementById("auth-login-button");
const authLogoutButton = document.getElementById("auth-logout-button");
const userProvisioning = document.getElementById("user-provisioning");
const userCreateForm = document.getElementById("user-create-form");
const userCreateName = document.getElementById("user-create-name");
const userCreateEmail = document.getElementById("user-create-email");
const userCreateRole = document.getElementById("user-create-role");
const userCreatePassword = document.getElementById("user-create-password");
const userCreateButton = document.getElementById("user-create-button");
const userCreateStatus = document.getElementById("user-create-status");
const workspaceAccessCard = document.getElementById("workspace-access-card");
const reviewState = document.getElementById("review-state");
const reviewStatusNote = document.getElementById("review-status-note");
const reviewSubmitButton = document.getElementById("review-submit-button");
const reviewApproveButton = document.getElementById("review-approve-button");
const reviewRejectButton = document.getElementById("review-reject-button");
const reviewerSelector = document.getElementById("reviewer-selector");
const reviewerAssignButton = document.getElementById("reviewer-assign-button");
const reviewerAssignmentStatus = document.getElementById("reviewer-assignment-status");
const currentValidationGuidance = document.getElementById("current-validation-guidance");
const sweepValidationGuidance = document.getElementById("sweep-validation-guidance");
const workflowNextTitle = document.getElementById("workflow-next-title");
const workflowNextDetail = document.getElementById("workflow-next-detail");
const workflowNextButton = document.getElementById("workflow-next-button");
const workflowGuide = document.getElementById("workflow-guide");
const projectStartCard = document.getElementById("project-start-card");
const projectStartTitle = document.getElementById("project-start-title");
const projectStartDetail = document.getElementById("project-start-detail");
const projectStartButton = document.getElementById("project-start-button");
const acceptanceCaseIdInput = document.getElementById("acceptance-case-id");
const acceptanceMinClearanceInput = document.getElementById("acceptance-min-clearance");
const acceptanceMaxWheelErrorInput = document.getElementById("acceptance-max-wheel-error");
const acceptanceMaxSyncErrorInput = document.getElementById("acceptance-max-sync-error");
const acceptanceMaxResidualInput = document.getElementById("acceptance-max-residual");
const acceptanceRequireFullRangeInput = document.getElementById("acceptance-require-full-range");
const acceptanceEvaluateButton = document.getElementById("acceptance-evaluate-button");
const acceptanceStatusNote = document.getElementById("acceptance-status-note");
const acceptanceChecks = document.getElementById("acceptance-checks");
const releaseChecklistState = document.getElementById("release-checklist-state");
const releaseChecklist = document.getElementById("release-checklist");
const releaseChecklistNote = document.getElementById("release-checklist-note");
const resultsDecisionCard = document.getElementById("results-decision-card");
const resultsDecisionStatus = document.getElementById("results-decision-status");
const resultsDecisionSummary = document.getElementById("results-decision-summary");
const resultsDecisionChecks = document.getElementById("results-decision-checks");
const resultsDecisionNote = document.getElementById("results-decision-note");
const approvalHistory = document.getElementById("approval-history");
const bodyChainBodyCountValue = document.getElementById("body-chain-body-count");
const bodyChainJointCountValue = document.getElementById("body-chain-joint-count");
const bodyChainRootValue = document.getElementById("body-chain-root");
const bodyChainTable = document.getElementById("body-chain-table");
const wheelbaseInput = document.getElementById("wheelbase-input");
const trackInput = document.getElementById("track-input");
const curveStepInput = document.getElementById("curve-step-input");
const betaMinInput = document.getElementById("beta-min-input");
const betaMaxInput = document.getElementById("beta-max-input");
const betaMinLabel = document.getElementById("beta-min-label");
const betaMaxLabel = document.getElementById("beta-max-label");
const geometryApplyButton = document.getElementById("geometry-apply-button");
const geometryStatus = document.getElementById("geometry-status");
const frontArticulationInput = document.getElementById("front-articulation-input");
const rearArticulationInput = document.getElementById("rear-articulation-input");
const kingpinInput = document.getElementById("kingpin-input");
const vehicleMaxArticulationInput = document.getElementById("vehicle-max-articulation-input");
const interactiveEditToggle = document.getElementById("interactive-edit-toggle");
const interactiveEditStatus = document.getElementById("interactive-edit-status");
const customAxleCountInput = document.getElementById("custom-axle-count");
const customTurnRadiusInput = document.getElementById("custom-turn-radius");
const customAxleConfig = document.getElementById("custom-axle-config");
const customAxleApplyButton = document.getElementById("custom-axle-apply-button");
const customAxleStatus = document.getElementById("custom-axle-status");
const linkageConfig = document.getElementById("linkage-config");
const linkageCompanionConfig = document.getElementById("linkage-companion-config");
const linkageCompanionEnabled = document.getElementById("linkage-companion-enabled");
const linkageApplyButton = document.getElementById("linkage-apply-button");
const linkageResetButton = document.getElementById("linkage-reset-button");
const linkageConfigStatus = document.getElementById("linkage-config-status");
const workflowSteps = [...document.querySelectorAll("[data-workflow-step]")];
const workflowStepNumber = document.getElementById("workflow-step-number");
const workflowStepTitle = document.getElementById("workflow-step-title");
const workflowStepDescription = document.getElementById("workflow-step-description");
const workflowGuideQuestion = document.getElementById("workflow-guide-question");
const workflowGuideSteps = document.getElementById("workflow-guide-steps");
const workflowGuideResultTitle = document.getElementById("workflow-guide-result-title");
const workflowGuideResult = document.getElementById("workflow-guide-result");
const workflowGuideRule = document.getElementById("workflow-guide-rule");
const combinationBodyCountInput = document.getElementById("combination-body-count");
const combinationTurnRadiusInput = document.getElementById("combination-turn-radius");
const combinationConfig = document.getElementById("combination-config");
const combinationFields = document.getElementById("combination-fields");
const combinationModeState = document.getElementById("combination-mode-state");
const combinationModeNote = document.getElementById("combination-mode-note");
const combinationActivateButton = document.getElementById("combination-activate-button");
const combinationCalculateButton = document.getElementById("combination-calculate-button");
const combinationStatus = document.getElementById("combination-status");
const legacyGeometryCard = document.querySelector(".legacy-geometry-card");
const legacyLinkageCard = document.querySelector(".legacy-linkage-card");
const currentValidationCard = document.getElementById("current-validation-card");
const currentValidationStatus = document.getElementById("current-validation-status");
const currentValidationSummary = document.getElementById("current-validation-summary");
const currentValidationChecks = document.getElementById("current-validation-checks");
const currentSteeringInterpretation = document.getElementById("current-steering-interpretation");
const currentSteeringStatus = document.getElementById("current-steering-status");
const currentSteeringDetail = document.getElementById("current-steering-detail");
const sweepValidationStepInput = document.getElementById("sweep-validation-step");
const sweepValidationButton = document.getElementById("sweep-validation-button");
const sweepValidationStatus = document.getElementById("sweep-validation-status");
const sweepValidationVerdict = document.getElementById("sweep-validation-verdict");
const sweepValidationSolved = document.getElementById("sweep-validation-solved");
const sweepValidationClearance = document.getElementById("sweep-validation-clearance");
const sweepValidationFailure = document.getElementById("sweep-validation-failure");
const mechanismGraphBuildButton = document.getElementById("mechanism-graph-build-button");
const mechanismGraphSolveButton = document.getElementById("mechanism-graph-solve-button");
const mechanismGraphPointCount = document.getElementById("mechanism-graph-point-count");
const mechanismGraphMemberCount = document.getElementById("mechanism-graph-member-count");
const mechanismGraphAssignmentCount = document.getElementById("mechanism-graph-assignment-count");
const mechanismGraphMapping = document.getElementById("mechanism-graph-mapping");
const mechanismGraphStatus = document.getElementById("mechanism-graph-status");
const mechanismGraphEditor = document.getElementById("mechanism-graph-editor");
const mechanismGraphEditorStatus = document.getElementById("mechanism-graph-editor-status");
const mechanismGraphApplyButton = document.getElementById("mechanism-graph-apply-button");
const mechanismGraphAddPointButton = document.getElementById("mechanism-graph-add-point-button");
const mechanismGraphAddMemberButton = document.getElementById("mechanism-graph-add-member-button");
const mechanismGraphAddOutputButton = document.getElementById("mechanism-graph-add-output-button");
const mechanismGraphAddDriverButton = document.getElementById("mechanism-graph-add-driver-button");
const mechanismGraphAddAssignmentButton = document.getElementById("mechanism-graph-add-assignment-button");
const mechanismPointEditor = document.getElementById("mechanism-point-editor");
const mechanismMemberEditor = document.getElementById("mechanism-member-editor");
const mechanismOutputEditor = document.getElementById("mechanism-output-editor");
const mechanismDriverEditor = document.getElementById("mechanism-driver-editor");
const mechanismAssignmentEditor = document.getElementById("mechanism-assignment-editor");

const FAILURE_GUIDANCE = {
  KINEMATICS: {
    title: "Check body and joint geometry",
    action: "Verify body dimensions, joint anchors, articulation bounds, and the explicit maneuver radius.",
  },
  MECHANISM: {
    title: "Make the mechanism solvable",
    action: "Check rigid member lengths, fixed and driven point positions, branch continuity, and wheel-output mappings.",
  },
  COLLISION: {
    title: "Remove component overlap",
    action: "Open Clearance focus, inspect the highlighted pair, then move the components or correct their envelopes. Connected joints are excluded; other overlaps are hard failures.",
  },
  CLEARANCE: {
    title: "Increase minimum clearance",
    action: "Move the conflicting pivot or link, or revise the envelope until the configured clearance target is met.",
  },
  STEERING_LIMIT_EXCEEDED: {
    title: "Respect the steering stop",
    action: "Change the linkage ratio or geometry, or confirm a larger physical steering stop. Do not treat the rod as an implicit stop.",
  },
  DRAWBAR_LIMIT_EXCEEDED: {
    title: "Respect the articulation stop",
    action: "Reduce the requested articulation range or update the approved drawbar limit.",
  },
  MULTIBODY_KINEMATIC_INCONSISTENT: {
    title: "Resolve multi-body closure",
    action: "Check joint anchors, body-local coordinates, and the common maneuver radius for the failing body.",
  },
  LINKAGE_NO_SOLUTION: {
    title: "Check linkage reach",
    action: "Adjust link lengths or pivot locations so the fixed-length circles intersect throughout the requested range.",
  },
  LINKAGE_BRANCH_CHANGE: {
    title: "Prevent branch switching",
    action: "Check the neutral assembly branch and incremental motion, then redesign near toggle positions.",
  },
  ACTUAL_STEERING_UNSOLVED: {
    title: "Complete wheel mapping",
    action: "Map every required wheel to a valid mechanism output and verify steering direction and ratio.",
  },
  OPTIMIZATION_NO_FEASIBLE_SOLUTION: {
    title: "No feasible candidate",
    action: "Relax only approved design bounds or targets, or change the mechanism. Do not apply an infeasible proposal.",
  },
};

const DISPLAY_MODES = {
  simulation: {
    label: "Simulation",
    status: "Simulation",
    title: "Top-view kinematics",
    description: "Live articulation, linkage, and clearance over the ideal steering core.",
    chip: "Live view",
  },
  rays: {
    label: "Steering rays",
    status: "Steering rays",
    title: "ICR steering rays",
    description: "Wheel normals converge on the selected instantaneous center of rotation.",
    chip: "ICR rays",
  },
  clearance: {
    label: "Clearance focus",
    status: "Clearance focus",
    title: "Clearance focus",
    description: "Minimum-clearance envelopes and the active linkage state.",
    chip: "Clearance",
  },
  error: {
    label: "Error focus",
    status: "Error focus",
    title: "Ideal versus actual error",
    description: "Linkage output error against the ideal front-axle steering target.",
    chip: "Error focus",
  },
  dimensions: {
    label: "Dimensioned sketch",
    status: "Sketch preview",
    title: "Dimensioned engineering sketch",
    description: "Before/after linkage comparison with dimension callouts and optimized overlays.",
    chip: "Sketch preview",
  },
  optimized: {
    label: "Optimized design",
    status: "Optimized design",
    title: "Optimized dimensioned design",
    description: "Dimension callouts and linkage comparisons for the selected optimization mode.",
    chip: "Optimized",
  },
};

const REFERENCE_LINKAGE_CONFIG = {
  id: "reference_demo_linkage",
  bell_crank_pivot_x_mm: 0,
  bell_crank_pivot_y_mm: 0,
  bell_crank_input_arm_length_mm: 200,
  bell_crank_input_neutral_angle_deg: 0,
  bell_crank_output_arm_length_mm: 180,
  bell_crank_output_neutral_angle_deg: 90,
  input_rod_length_mm: 120,
  steering_pivot_x_mm: 560,
  steering_pivot_y_mm: 180,
  steering_arm_length_mm: 180,
  steering_arm_neutral_angle_deg: 99.24835833789221,
  tie_rod_length_mm: 560,
  steering_stop_deg: null,
  companion_enabled: true,
  companion_steering_pivot_x_mm: 560,
  companion_steering_pivot_y_mm: -180,
  companion_steering_arm_length_mm: 180,
  companion_steering_arm_neutral_angle_deg: -8.13066584450665,
  companion_tie_rod_length_mm: 600,
  driver_arc_center_x_mm: 180,
  driver_arc_center_y_mm: 120,
  driver_arc_radius_mm: 20,
};

const LINKAGE_FIELDS = [
  { key: "bell_crank_pivot_x_mm", label: "Bell-crank pivot X", step: "1" },
  { key: "bell_crank_pivot_y_mm", label: "Bell-crank pivot Y", step: "1" },
  { key: "bell_crank_input_arm_length_mm", label: "Input arm length", step: "1" },
  { key: "bell_crank_input_neutral_angle_deg", label: "Input neutral angle", step: "0.1" },
  { key: "bell_crank_output_arm_length_mm", label: "Output arm length", step: "1" },
  { key: "bell_crank_output_neutral_angle_deg", label: "Output neutral angle", step: "0.1" },
  { key: "input_rod_length_mm", label: "Input rod length", step: "1" },
  { key: "steering_pivot_x_mm", label: "Steering pivot X", step: "1" },
  { key: "steering_pivot_y_mm", label: "Steering pivot Y", step: "1" },
  { key: "steering_arm_length_mm", label: "Steering arm length", step: "1" },
  { key: "steering_arm_neutral_angle_deg", label: "Steering arm neutral angle", step: "0.1" },
  { key: "tie_rod_length_mm", label: "Tie rod length", step: "1" },
  { key: "steering_stop_deg", label: "Steering stop (optional)", step: "0.1", optional: true },
  { key: "driver_arc_center_x_mm", label: "Driver arc center X", step: "1" },
  { key: "driver_arc_center_y_mm", label: "Driver arc center Y", step: "1" },
  { key: "driver_arc_radius_mm", label: "Driver arc radius", step: "1" },
];

const COMPANION_LINKAGE_FIELDS = [
  { key: "companion_steering_pivot_x_mm", label: "Companion pivot X", step: "1" },
  { key: "companion_steering_pivot_y_mm", label: "Companion pivot Y", step: "1" },
  { key: "companion_steering_arm_length_mm", label: "Companion arm length", step: "1" },
  { key: "companion_steering_arm_neutral_angle_deg", label: "Companion neutral angle", step: "0.1" },
  { key: "companion_tie_rod_length_mm", label: "Companion tie rod length", step: "1" },
];

const WORKFLOW_COPY = {
  project: ["01", "Project", "Open a design project or create a controlled starting revision."],
  vehicle: ["02", "Vehicle", "Define the rigid bodies, articulation joints, mounted axles, and packaging geometry."],
  maneuver: ["03", "Maneuver", "Set the articulation state and explicit maneuver radius, then inspect ideal steering."],
  mechanism: ["04", "Mechanism", "Define the physical steering graph, linkage dimensions, and CAD assignments."],
  validate: ["05", "Validate", "Review steering error, branch continuity, collisions, and minimum clearance."],
  optimize: ["06", "Optimize", "Select bounded variables and search only for designs that satisfy every hard constraint."],
  results: ["07", "Results", "Compare the controlled proposal, apply a passing design, and generate engineering outputs."],
};

const WORKFLOW_GUIDANCE = {
  project: {
    question: "What study am I saving, and which revision will be reviewed?",
    steps: [
      "Open an existing project or create a named project.",
      "Save a revision after every meaningful model change.",
      "Use the dashboard to track the active revision, engineering verdict, and review gate.",
    ],
    resultTitle: "You are done with this step when",
    result: "A named revision exists and the dashboard identifies the revision that will be calculated.",
    rule: "Unsaved calculations cannot be submitted for review or treated as release evidence.",
  },
  vehicle: {
    question: "What physical towing combination is being analyzed?",
    steps: [
      "Enter one rigid-body row per vehicle or trailer and choose each body's parent.",
      "Add every axle and wheel position, then set each articulation stop and sweep range.",
      "Use a confirmed CAD outline when available; otherwise the rectangular envelope is an explicit fallback.",
    ],
    resultTitle: "You are done with this step when",
    result: "The body tree, axle count, wheel locations, joint stops, and packaging envelopes match the source layout.",
    rule: "A missing body envelope or articulation stop is an incomplete physical model, not a passing assumption.",
  },
  maneuver: {
    question: "What turning case should the combination be tested against?",
    steps: [
      "Enter the signed root turn radius for the maneuver case.",
      "Resolve the maneuver to calculate every articulated body pose.",
      "Inspect the ideal wheel headings before describing the mechanical solution.",
    ],
    resultTitle: "This calculation produces",
    result: "The maneuver ICR, articulated body poses, and ideal steering targets used by the mechanism comparison.",
    rule: "Positive and negative radius values represent opposite directions; keep the sign convention traceable to the test case.",
  },
  mechanism: {
    question: "How does the physical hardware turn each mapped wheel?",
    steps: [
      "Build the component graph from fixed points, driven points, rigid members, and angle outputs.",
      "Map every steerable wheel to the output that physically controls it, then solve the graph.",
      "Use local body coordinates for installed components so articulation carries the mechanism correctly.",
    ],
    resultTitle: "This calculation produces",
    result: "Actual wheel angles, closure residuals, branch continuity, and the physical mechanism pose for the maneuver.",
    rule: "A visual graph is not evidence: every required wheel needs a named mapping and a successful graph solve.",
  },
  validate: {
    question: "Does the current design remain physically feasible across the requested range?",
    steps: [
      "Review the current-pose closure, collision, clearance, and steering checks.",
      "Run the complete Cartesian sweep for every configured articulation joint and design case.",
      "Use the first failing pose, pair, and measured value to correct the model or mechanism.",
    ],
    resultTitle: "This step decides",
    result: "A hard engineering PASS or FAIL, with the evaluated poses, residuals, clearance, collision, and steering evidence retained.",
    rule: "PASS means every configured hard check passes. FAIL or incomplete evidence is diagnostic only and cannot authorize release.",
  },
  optimize: {
    question: "Can bounded design variables improve the mechanism without violating hard constraints?",
    steps: [
      "Keep the validated baseline and choose only variables with approved bounds.",
      "Run optimization as a proposal search, not as an automatic design change.",
      "Inspect the candidate and rerun the complete validation before considering it for review.",
    ],
    resultTitle: "This step produces",
    result: "A baseline-versus-candidate comparison showing variable changes, steering error, clearance, residuals, and feasibility.",
    rule: "An optimizer candidate is never approval. An invalid or unvalidated candidate cannot be applied or exported as approved.",
  },
  results: {
    question: "Is this saved revision sufficiently evidenced and approved for release?",
    steps: [
      "Compare ideal, actual, and error for every mapped wheel and synchronization channel.",
      "Confirm current-pose PASS, full-range PASS, signed-off Monroc acceptance, and independent approval.",
      "Export evidence for diagnosis; export a controlled release only when the release gate is explicitly PASS.",
    ],
    resultTitle: "The release decision is",
    result: "A single decision card backed by the saved revision, full-range evidence, acceptance result, review history, and audit trail.",
    rule: "Only engineering PASS plus full-range evidence, approved Monroc criteria, and independent reviewer approval authorizes release.",
  },
};

function panelFor(element) {
  return element?.closest(".wheel-table-card, .metric-card, .note-card") || null;
}

function initializeWorkflowPanels() {
  const panelAssignments = new Map();
  const assign = (step, ...elements) => {
    for (const element of elements) {
      const panel = panelFor(element);
      if (panel) {
        panelAssignments.set(panel, step);
      }
    }
  };

  assign("project", projectNameInput);
  assign("vehicle", geometryApplyButton, combinationCalculateButton, bodyChainTable);
  assign("maneuver", betaValue, radiusValue, maxAngleValue, phaseValue, wheelTable, steeringCurvesImage, sweptPathImage);
  assign("mechanism", linkageConfig, linkageSteerValue, linkageErrorValue, linkageResidualValue, linkageBranchValue, dxfFileInput);
  assign("validate", actualErrorValue, synchronizationErrorValue, clearanceValue, clearancePairValue, clearanceStatusValue);
  assign("optimize", optimizeButton);
  assign("results", exportJsonLink);

  const directPanels = document.querySelectorAll(
    ".info-panel > .wheel-table-card, .info-panel > .metric-card, .info-panel > .note-card",
  );
  for (const panel of directPanels) {
    const explicitStep = panel.dataset.workflowPanel;
    panel.dataset.workflowPanel = explicitStep || panelAssignments.get(panel) || "maneuver";
  }
}

function setWorkflowStep(step) {
  if (!Object.prototype.hasOwnProperty.call(WORKFLOW_COPY, step)) {
    return;
  }
  state.activeWorkflowStep = step;
  const [number, title, description] = WORKFLOW_COPY[step];
  workflowStepNumber.textContent = `Step ${number}`;
  workflowStepTitle.textContent = title;
  workflowStepDescription.textContent = description;
  renderWorkflowGuide(step);
  for (const button of workflowSteps) {
    button.classList.toggle("is-active", button.dataset.workflowStep === step);
  }
  for (const panel of document.querySelectorAll("[data-workflow-panel]")) {
    panel.classList.toggle("workflow-panel-hidden", panel.dataset.workflowPanel !== step);
  }
  renderWorkflowProgress();
}

function renderWorkflowGuide(step) {
  const guidance = WORKFLOW_GUIDANCE[step];
  if (!guidance || !workflowGuideQuestion || !workflowGuideSteps || !workflowGuideResultTitle || !workflowGuideResult || !workflowGuideRule) {
    return;
  }
  workflowGuideQuestion.replaceChildren();
  const questionLabel = document.createElement("strong");
  questionLabel.textContent = "The engineering question: ";
  workflowGuideQuestion.append(questionLabel, document.createTextNode(guidance.question));

  workflowGuideSteps.replaceChildren();
  for (const stepText of guidance.steps) {
    const item = document.createElement("li");
    item.textContent = stepText;
    workflowGuideSteps.appendChild(item);
  }
  workflowGuideResultTitle.textContent = guidance.resultTitle;
  workflowGuideResult.textContent = guidance.result;
  workflowGuideRule.replaceChildren();
  const ruleLabel = document.createElement("strong");
  ruleLabel.textContent = "Rule: ";
  workflowGuideRule.append(ruleLabel, document.createTextNode(guidance.rule));
}

function isLegacyRevisionMode() {
  return Boolean(state.currentProjectId && !state.combinationActive && state.vehicleConfig);
}

function workflowStepStates() {
  const hasProject = Boolean(state.currentProjectId);
  const legacyRevision = isLegacyRevisionMode();
  const hasVehicle = Boolean(state.combinationActive || state.vehicleConfig);
  const hasManeuver = Boolean(state.maneuverResolved || state.currentPayload);
  const hasMechanism = state.combinationActive
    ? Boolean(state.mechanismGraph)
    : Boolean(state.linkageConfig && state.currentPayload?.linkage);
  const hasSolvedMechanism = state.combinationActive
    ? Boolean(state.currentPayload?.mechanism_graph)
    : Boolean(state.currentPayload?.linkage);
  const hasFullRange = !state.combinationActive || state.sweepValidationPayload?.status === "PASS";
  const hasValidation = Boolean(state.currentValidationPass && hasFullRange);
  const hasOptimization = Boolean(state.optimizationPayload?.optimized);
  const resultState = !hasProject || !hasManeuver || !hasSolvedMechanism
    ? "WAIT"
    : !state.currentValidationPass
      ? "FAIL"
      : !hasFullRange
        ? "INCOMPLETE"
        : "PASS";
  return {
    project: hasProject ? (state.workspaceDirty ? "EDITING" : "READY") : "START",
    vehicle: legacyRevision ? "TODO" : (hasVehicle ? "READY" : "TODO"),
    maneuver: legacyRevision ? "WAIT" : (hasManeuver ? "READY" : "TODO"),
    mechanism: legacyRevision
      ? "WAIT"
      : (hasSolvedMechanism ? "PASS" : (hasMechanism ? "READY" : "TODO")),
    validate: legacyRevision ? "WAIT" : (hasValidation ? "PASS" : (hasSolvedMechanism ? "TODO" : "WAIT")),
    optimize: legacyRevision
      ? "WAIT"
      : (hasOptimization
      ? (state.optimizationPayload.optimized.feasible === true ? "READY" : "FAIL")
      : "OPTIONAL"),
    results: legacyRevision ? "WAIT" : resultState,
  };
}

function nextWorkflowAction() {
  const hasProject = Boolean(state.currentProjectId);
  const legacyRevision = isLegacyRevisionMode();
  const hasVehicle = Boolean(state.combinationActive || state.vehicleConfig);
  const hasManeuver = Boolean(state.maneuverResolved || state.currentPayload);
  const hasMechanism = state.combinationActive
    ? Boolean(state.mechanismGraph)
    : Boolean(state.linkageConfig && state.currentPayload?.linkage);
  const hasSolvedMechanism = state.combinationActive
    ? Boolean(state.currentPayload?.mechanism_graph)
    : Boolean(state.currentPayload?.linkage);
  const hasFullRange = !state.combinationActive || state.sweepValidationPayload?.status === "PASS";

  if (!hasProject) {
    return {
      step: "project",
      title: "Open or create the project",
      detail: "Start with a named project so every calculation and review decision is saved as a revision.",
      button: "Go to Project",
      action: "create-project",
      activeButton: "Start setup",
    };
  }
  if (legacyRevision) {
    return {
      step: "vehicle",
      title: "Switch this revision to multi-body workflow",
      detail: "This saved revision is the legacy single-layout study. Activate the explicit towing-combination model before continuing.",
      button: "Use multi-body workflow",
      action: "activate-combination",
      activeButton: "Start multi-body setup",
    };
  }
  if (!hasVehicle) {
    return {
      step: "vehicle",
      title: "Define the towing combination",
      detail: "Enter every body, axle, wheel envelope, articulation joint, and physical joint stop.",
      button: "Go to Vehicle",
      action: "open-vehicle",
      activeButton: "Review vehicle inputs",
    };
  }
  if (!hasManeuver) {
    return {
      step: "maneuver",
      title: "Resolve the maneuver",
      detail: "Set the signed root turn radius and calculate the current articulated pose.",
      button: "Go to Maneuver",
      action: "resolve-maneuver",
      activeButton: "Resolve maneuver",
    };
  }
  if (!hasMechanism) {
    return {
      step: "mechanism",
      title: state.combinationActive ? "Build and solve the mechanism graph" : "Apply the steering linkage",
      detail: state.combinationActive
        ? "Create the rigid links, driver arcs, outputs, and named wheel mappings, then solve them."
        : "Define the physical linkage dimensions and apply the configuration before validation.",
      button: "Go to Mechanism",
      action: "build-mechanism",
      activeButton: "Build graph",
    };
  }
  if (!hasSolvedMechanism) {
    return {
      step: "mechanism",
      title: "Solve the mechanism graph",
      detail: "The maneuver is resolved. Solve the configured mechanism to generate current-pose engineering evidence.",
      button: "Go to Mechanism",
      action: "solve-mechanism",
      activeButton: "Solve mechanism",
    };
  }
  if (!state.currentValidationPass) {
    return {
      step: "validate",
      title: "Resolve current-pose failures",
      detail: "Review the failed hard check and its engineering guidance before running the full range.",
      button: "Go to Validate",
      action: "open-validate",
      activeButton: "Review validation",
    };
  }
  if (!hasFullRange) {
    return {
      step: "validate",
      title: "Run the full articulation validation",
      detail: "Check every configured Cartesian combination of joint angles. A partial sweep cannot support approval.",
      button: "Go to Validate",
      action: "run-sweep",
      activeButton: "Run full-range validation",
    };
  }
  return {
    step: "results",
    title: state.workspaceDirty ? "Save the validated revision" : "Review the engineering result",
    detail: state.workspaceDirty
      ? "Save this revision before submitting it for independent review or controlled release."
      : "Compare actual versus ideal steering, inspect clearance, and use Optimize only for a feasible proposal.",
    button: "Go to Results",
    action: state.workspaceDirty ? "save-revision" : "open-results",
    activeButton: state.workspaceDirty ? "Save revision" : "Review results",
  };
}

function runWorkflowNextAction(action) {
  switch (action) {
    case "create-project":
      projectStartButton.click();
      return;
    case "activate-combination":
      combinationActivateButton.click();
      return;
    case "resolve-maneuver":
      combinationCalculateButton.click();
      return;
    case "build-mechanism":
      mechanismGraphBuildButton.click();
      return;
    case "solve-mechanism":
      mechanismGraphSolveButton.click();
      return;
    case "run-sweep":
      sweepValidationButton.click();
      return;
    case "save-revision":
      projectSaveButton.click();
      return;
    default:
      setWorkflowStep(workflowNextButton.dataset.workflowTarget || "project");
  }
}

function renderWorkflowProgress() {
  const states = workflowStepStates();
  for (const button of workflowSteps) {
    const step = button.dataset.workflowStep;
    const status = states[step] || "WAIT";
    button.dataset.workflowState = status;
    button.setAttribute("aria-label", `${button.querySelector("strong")?.textContent || step} ${status}`);
  }
  const next = nextWorkflowAction();
  if (workflowNextTitle && workflowNextDetail && workflowNextButton) {
    workflowNextTitle.textContent = next.title;
    workflowNextDetail.textContent = next.detail;
    const actionIsAvailable = state.activeWorkflowStep === next.step;
    workflowNextButton.textContent = actionIsAvailable ? (next.activeButton || next.button) : next.button;
    workflowNextButton.dataset.workflowTarget = next.step;
    workflowNextButton.dataset.workflowAction = actionIsAvailable ? (next.action || "") : "";
  }
  renderProjectStartCard(next);
  renderProjectDashboardStatus();
}

function renderProjectStartCard(next = nextWorkflowAction()) {
  if (!projectStartCard || !projectStartTitle || !projectStartDetail || !projectStartButton) {
    return;
  }
  const legacyRevision = isLegacyRevisionMode();
  projectStartCard.dataset.status = !state.currentProjectId
    ? "start"
    : (legacyRevision ? "legacy" : "active");
  projectStartTitle.textContent = !state.currentProjectId
    ? "Create or open a project"
    : next.title;
  projectStartDetail.textContent = !state.currentProjectId
    ? "Every calculation is saved as a named revision before review."
    : next.detail;
  projectStartButton.textContent = !state.currentProjectId
    ? "Start setup"
    : (legacyRevision ? "Use multi-body workflow" : `Continue: ${next.step[0].toUpperCase()}${next.step.slice(1)}`);
  projectStartButton.dataset.workflowTarget = next.step;
}

function renderProjectDashboardStatus() {
  if (!projectEngineeringState || !projectReviewState || !projectModelScope) {
    return;
  }

  const hasRevision = Boolean(state.currentProjectId && state.activeProjectRevisionId);
  const fullRangePass = state.combinationActive
    ? state.sweepValidationPayload?.status === "PASS"
    : state.activeRevisionHasFullRangeEvidence;
  let engineeringStatus = "INCOMPLETE";
  let engineeringDetail = "Run the current pose and full-range checks.";
  if (!hasRevision) {
    engineeringStatus = "NO REVISION";
    engineeringDetail = "Create or open a project revision to begin.";
  } else if (isLegacyRevisionMode()) {
    engineeringStatus = "LEGACY";
    engineeringDetail = "Legacy single-layout revision. Activate multi-body workflow before continuing.";
  } else if (state.workspaceDirty) {
    engineeringStatus = "EDITING";
    engineeringDetail = "Unsaved changes invalidate the saved engineering evidence.";
  } else if (!state.currentPayload) {
    engineeringDetail = state.combinationActive && state.maneuverResolved && state.mechanismGraph
      ? "Solve the physical mechanism to generate current-pose evidence."
      : "Resolve the maneuver and solve the physical mechanism.";
  } else if (!state.currentValidationPass) {
    engineeringStatus = "FAIL";
    engineeringDetail = "One or more current-pose hard checks failed.";
  } else if (!fullRangePass) {
    engineeringDetail = "Current pose passes; full-range evidence is still required.";
  } else {
    engineeringStatus = "PASS";
    engineeringDetail = "Current pose and required range checks pass.";
  }
  projectEngineeringState.textContent = engineeringStatus;
  projectEngineeringDetail.textContent = engineeringDetail;
  projectEngineeringCard.dataset.status = engineeringStatus.toLowerCase().replace(" ", "-");

  const reviewStatus = state.approvalStatus?.status || "draft";
  const reviewLabels = {
    draft: ["DRAFT", "Saved revision is not submitted for independent review."],
    submitted: ["SUBMITTED", "Waiting for the assigned reviewer to decide."],
    approved: ["APPROVED", "Independent approval is recorded for this revision."],
    rejected: ["REJECTED", "Update the design, save a new revision, and resubmit."],
  };
  const [reviewLabel, reviewDetail] = reviewLabels[reviewStatus] || [reviewStatus.toUpperCase(), "Review state is recorded for this revision."];
  projectReviewState.textContent = reviewLabel;
  projectReviewDetail.textContent = hasRevision ? reviewDetail : "Save a revision before submitting it for review.";
  projectReviewCard.dataset.status = reviewStatus;

  const combination = state.currentPayload?.vehicle_combination;
  const axles = state.currentPayload?.vehicle?.axle_count;
  const bodies = combination?.body_count ?? combination?.bodies?.length;
  if (Number.isFinite(Number(bodies)) && Number.isFinite(Number(axles))) {
    projectModelScope.textContent = `${Number(bodies)} bodies / ${Number(axles)} axles`;
    projectModelDetail.textContent = "Explicit articulated combination is active.";
    projectScopeCard.dataset.status = "ready";
  } else if (state.vehicleConfig?.axles?.length) {
    projectModelScope.textContent = `${state.vehicleConfig.axles.length} axle study`;
    projectModelDetail.textContent = "Legacy single-layout study. Use the multi-body workflow for towing combinations.";
    projectScopeCard.dataset.status = "legacy";
  } else {
    projectModelScope.textContent = "n/a";
    projectModelDetail.textContent = "Define the towing combination.";
    projectScopeCard.dataset.status = "waiting";
  }
}

function formatAngle(value) {
  if (value === null || value === undefined || !Number.isFinite(Number(value))) {
    return "n/a";
  }
  value = Number(value);
  const sign = value > 0 ? "+" : "";
  return `${sign}${value.toFixed(2)} deg`;
}

function formatDistance(value) {
  if (value === null || Number.isNaN(value)) {
    return "Straight";
  }
  const sign = value > 0 ? "+" : "";
  return `${sign}${value.toFixed(1)} mm`;
}

function formatMillimeters(value) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "n/a";
  }
  return `${value.toFixed(1)} mm`;
}

function transformBodyPoint(localPoint, pose) {
  const yaw = pose?.yaw_rad ?? 0;
  const cosYaw = Math.cos(yaw);
  const sinYaw = Math.sin(yaw);
  return {
    x_mm: (pose?.x_mm ?? 0) + localPoint.x_mm * cosYaw - localPoint.y_mm * sinYaw,
    y_mm: (pose?.y_mm ?? 0) + localPoint.x_mm * sinYaw + localPoint.y_mm * cosYaw,
  };
}

function bodyOutlinePoints(body) {
  const halfLength = (body.body_length_mm ?? 0) / 2;
  const halfWidth = (body.body_width_mm ?? 0) / 2;
  const pose = body.pose || {};
  const corners = Array.isArray(body.body_polygon) && body.body_polygon.length >= 3
    ? body.body_polygon
    : [
      { x_mm: -halfLength, y_mm: -halfWidth },
      { x_mm: halfLength, y_mm: -halfWidth },
      { x_mm: halfLength, y_mm: halfWidth },
      { x_mm: -halfLength, y_mm: halfWidth },
    ];

  return corners.map((corner) => {
    const world = transformBodyPoint(corner, pose);
    const svgPoint = toSvgPoint(world);
    return `${svgPoint.x},${svgPoint.y}`;
  }).join(" ");
}

function formatBodyPose(body) {
  const pose = body.pose || {};
  const x = formatMillimeters(pose.x_mm ?? null);
  const y = formatMillimeters(pose.y_mm ?? null);
  const yaw = formatAngle(pose.yaw_deg ?? (pose.yaw_rad === undefined ? null : pose.yaw_rad * 180 / Math.PI));
  return `x ${x} | y ${y} | yaw ${yaw}`;
}

function linkageFieldInput(field, value) {
  const label = document.createElement("label");
  label.textContent = field.label;
  const input = document.createElement("input");
  input.id = `linkage-${field.key}`;
  input.type = "number";
  input.step = field.step;
  input.inputMode = "decimal";
  input.placeholder = field.optional ? "none" : "0";
  input.value = value === null || value === undefined ? "" : String(value);
  input.addEventListener("input", () => {
    markWorkspaceDirty("Linkage inputs changed. Apply the linkage and save a new revision before review.");
    linkageConfigStatus.textContent = "Linkage edits are unapplied. Apply to solve the current design.";
  });
  label.appendChild(input);
  return label;
}

function renderLinkageConfig() {
  const values = state.linkageConfig || REFERENCE_LINKAGE_CONFIG;
  linkageConfig.replaceChildren();
  for (const field of LINKAGE_FIELDS) {
    linkageConfig.appendChild(linkageFieldInput(field, values[field.key]));
  }

  linkageCompanionEnabled.checked = values.companion_enabled !== false;
  linkageCompanionConfig.replaceChildren();
  for (const field of COMPANION_LINKAGE_FIELDS) {
    const label = linkageFieldInput(field, values[field.key]);
    label.querySelector("input").disabled = !linkageCompanionEnabled.checked;
    linkageCompanionConfig.appendChild(label);
  }
  linkageConfigStatus.textContent = state.linkageConfig
    ? "Custom linkage active. Slider results use the applied component dimensions."
    : "Reference linkage active. Edit the rigid components, then apply a custom design.";
}

function linkageInputNumber(key, optional = false) {
  const input = document.getElementById(`linkage-${key}`);
  const raw = input?.value.trim() || "";
  if (optional && raw === "") {
    return null;
  }
  const value = Number(raw);
  if (!Number.isFinite(value)) {
    throw new Error(`${key} must be a finite number.`);
  }
  return value;
}

function readLinkageConfig() {
  const config = { ...REFERENCE_LINKAGE_CONFIG };
  for (const field of LINKAGE_FIELDS) {
    config[field.key] = linkageInputNumber(field.key, field.optional);
  }
  config.companion_enabled = linkageCompanionEnabled.checked;
  for (const field of COMPANION_LINKAGE_FIELDS) {
    config[field.key] = linkageInputNumber(field.key, !config.companion_enabled);
  }
  return config;
}

function serializedLinkageConfig(sourceValues = state.linkageConfig || REFERENCE_LINKAGE_CONFIG) {
  const values = sourceValues;
  const point = (xKey, yKey) => ({
    x_mm: Number(values[xKey]),
    y_mm: Number(values[yKey]),
  });
  return {
    id: values.id || "custom_linkage",
    bell_crank_pivot: point("bell_crank_pivot_x_mm", "bell_crank_pivot_y_mm"),
    bell_crank_input_arm_length_mm: Number(values.bell_crank_input_arm_length_mm),
    bell_crank_input_neutral_angle_deg: Number(values.bell_crank_input_neutral_angle_deg),
    bell_crank_output_arm_length_mm: Number(values.bell_crank_output_arm_length_mm),
    bell_crank_output_neutral_angle_deg: Number(values.bell_crank_output_neutral_angle_deg),
    input_rod_length_mm: Number(values.input_rod_length_mm),
    steering_pivot: point("steering_pivot_x_mm", "steering_pivot_y_mm"),
    steering_arm_length_mm: Number(values.steering_arm_length_mm),
    steering_arm_neutral_angle_deg: Number(values.steering_arm_neutral_angle_deg),
    tie_rod_length_mm: Number(values.tie_rod_length_mm),
    steering_stop_deg: values.steering_stop_deg === null ? null : Number(values.steering_stop_deg),
    companion_steering_pivot: values.companion_enabled === false
      ? null
      : point("companion_steering_pivot_x_mm", "companion_steering_pivot_y_mm"),
    companion_steering_arm_length_mm: values.companion_enabled === false
      ? null
      : Number(values.companion_steering_arm_length_mm),
    companion_steering_arm_neutral_angle_deg: Number(values.companion_steering_arm_neutral_angle_deg),
    companion_tie_rod_length_mm: values.companion_enabled === false
      ? null
      : Number(values.companion_tie_rod_length_mm),
    driver_arc_center: point("driver_arc_center_x_mm", "driver_arc_center_y_mm"),
    driver_arc_radius_mm: Number(values.driver_arc_radius_mm),
  };
}

function optimizedLinkageConfig() {
  const values = { ...(state.linkageConfig || REFERENCE_LINKAGE_CONFIG) };
  for (const variable of state.optimizationPayload?.variables_after || []) {
    const optimized = Number(variable.optimized);
    if (Object.prototype.hasOwnProperty.call(values, variable.id) && Number.isFinite(optimized)) {
      values[variable.id] = optimized;
    }
  }
  return values;
}

function storedLinkageConfig(raw) {
  if (!raw || typeof raw !== "object") {
    return null;
  }
  const config = { ...REFERENCE_LINKAGE_CONFIG };
  const pointValue = (point, xKey, yKey) => {
    if (point && typeof point === "object") {
      config[xKey] = Number(point.x_mm);
      config[yKey] = Number(point.y_mm);
    }
  };
  const scalarValue = (key, fallback = config[key]) => {
    if (raw[key] !== undefined && raw[key] !== null && Number.isFinite(Number(raw[key]))) {
      config[key] = Number(raw[key]);
    } else if (fallback !== undefined) {
      config[key] = fallback;
    }
  };
  config.id = raw.id || config.id;
  pointValue(raw.bell_crank_pivot, "bell_crank_pivot_x_mm", "bell_crank_pivot_y_mm");
  pointValue(raw.steering_pivot, "steering_pivot_x_mm", "steering_pivot_y_mm");
  pointValue(raw.companion_steering_pivot, "companion_steering_pivot_x_mm", "companion_steering_pivot_y_mm");
  pointValue(raw.driver_arc_center, "driver_arc_center_x_mm", "driver_arc_center_y_mm");
  for (const key of [
    "bell_crank_input_arm_length_mm",
    "bell_crank_output_arm_length_mm",
    "input_rod_length_mm",
    "steering_arm_length_mm",
    "tie_rod_length_mm",
    "companion_steering_arm_length_mm",
    "companion_tie_rod_length_mm",
    "driver_arc_radius_mm",
    "steering_stop_deg",
  ]) {
    scalarValue(key);
  }
  for (const key of [
    "bell_crank_input_neutral_angle_deg",
    "bell_crank_output_neutral_angle_deg",
    "steering_arm_neutral_angle_deg",
    "companion_steering_arm_neutral_angle_deg",
  ]) {
    if (raw[key] !== undefined) {
      config[key] = Number(raw[key]);
    } else {
      const radKey = key.replace("_deg", "_rad");
      if (raw[radKey] !== undefined) {
        config[key] = Number(raw[radKey]) * 180 / Math.PI;
      }
    }
  }
  config.companion_enabled = raw.companion_steering_pivot !== null
    && raw.companion_steering_pivot !== undefined
    && raw.companion_tie_rod_length_mm !== null;
  return config;
}

function getDxfRoleOptions(payload) {
  if (payload && Array.isArray(payload.role_options) && payload.role_options.length > 0) {
    return payload.role_options;
  }
  return [
    { value: "", label: "Unassigned" },
    { value: "body_envelope", label: "Body envelope" },
    { value: "chassis_outline", label: "Chassis outline" },
    { value: "axle_centerline", label: "Axle centerline" },
    { value: "linkage_segment", label: "Linkage segment" },
    { value: "steering_arm", label: "Steering arm" },
    { value: "tie_rod", label: "Tie rod" },
    { value: "bell_crank", label: "Bell crank" },
    { value: "articulation_pivot", label: "Articulation pivot" },
    { value: "pivot", label: "Pivot" },
    { value: "wheel_marker", label: "Wheel marker" },
    { value: "icr_marker", label: "ICR marker" },
    { value: "annotation", label: "Annotation" },
    { value: "reference_point", label: "Reference point" },
    { value: "arc_reference", label: "Arc reference" },
    { value: "block_reference", label: "Block reference" },
    { value: "drawbar_or_frame", label: "Drawbar / frame" },
  ];
}

function roleLabelForValue(value, roleOptions) {
  const option = roleOptions.find((entry) => entry.value === value);
  return option ? option.label : (value || "Unassigned");
}

function renderVehicleCombinationSummary(payload) {
  const combination = payload.vehicle_combination;
  bodyChainTable.replaceChildren();

  if (!combination) {
    bodyChainBodyCountValue.textContent = "n/a";
    bodyChainJointCountValue.textContent = "n/a";
    bodyChainRootValue.textContent = "n/a";
    const row = document.createElement("div");
    row.className = "wheel-row";
    const label = document.createElement("span");
    label.className = "label";
    label.textContent = "No articulated chain";
    const value = document.createElement("span");
    value.className = "value";
    value.textContent = "n/a";
    row.append(label, value);
    bodyChainTable.appendChild(row);
    return;
  }

  bodyChainBodyCountValue.textContent = `${combination.body_count ?? combination.bodies?.length ?? 0}`;
  bodyChainJointCountValue.textContent = `${combination.joint_count ?? combination.joints?.length ?? 0}`;
  bodyChainRootValue.textContent = combination.root_body_id || "n/a";

  for (const body of combination.bodies || []) {
    const row = document.createElement("div");
    row.className = "wheel-row";
    const label = document.createElement("span");
    label.className = "label";
    label.textContent = body.name || body.id;
    const value = document.createElement("span");
    value.className = "value";
    value.textContent = formatBodyPose(body);
    row.append(label, value);
    bodyChainTable.appendChild(row);
  }

  for (const joint of combination.joints || []) {
    const row = document.createElement("div");
    row.className = "wheel-row";
    const label = document.createElement("span");
    label.className = "label";
    label.textContent = `Joint ${joint.id}`;
    const value = document.createElement("span");
    value.className = "value";
    value.textContent = `${joint.parent_body_id} -> ${joint.child_body_id} | ${formatAngle(joint.articulation_deg ?? (joint.articulation_rad === undefined ? null : joint.articulation_rad * 180 / Math.PI))}`;
    row.append(label, value);
    bodyChainTable.appendChild(row);
  }
}

function renderVehicleCombinationOverlay(payload) {
  const combination = payload.vehicle_combination;
  if (!combination) {
    return;
  }

  const bodiesById = new Map((combination.bodies || []).map((body) => [body.id, body]));
  (combination.bodies || []).forEach((body, index) => {
    const points = bodyOutlinePoints(body);
    diagram.appendChild(svgEl("polygon", {
      points,
      class: `body-chain-outline ${index === 0 ? "body-chain-root" : "body-chain-child"}`,
    }));

    const center = toSvgPoint(body.pose || { x_mm: 0, y_mm: 0 });
    diagram.appendChild(svgEl("circle", {
      cx: center.x,
      cy: center.y,
      r: 18,
      class: "body-chain-center",
    }));

    diagram.appendChild(svgEl("text", {
      x: center.x + 36,
      y: center.y - 26,
      class: "body-chain-label",
    })).textContent = body.name || body.id;
  });

  for (const joint of combination.joints || []) {
    const parentBody = bodiesById.get(joint.parent_body_id);
    const childBody = bodiesById.get(joint.child_body_id);
    if (!parentBody || !childBody) {
      continue;
    }

    const parentAnchor = toSvgPoint(transformBodyPoint(joint.parent_anchor, parentBody.pose || {}));
    const childAnchor = toSvgPoint(transformBodyPoint(joint.child_anchor, childBody.pose || {}));
    diagram.appendChild(svgEl("line", {
      x1: parentAnchor.x,
      y1: parentAnchor.y,
      x2: childAnchor.x,
      y2: childAnchor.y,
      class: "body-chain-joint",
    }));
    diagram.appendChild(svgEl("circle", {
      cx: parentAnchor.x,
      cy: parentAnchor.y,
      r: 11,
      class: "body-chain-joint-node",
    }));
    diagram.appendChild(svgEl("circle", {
      cx: childAnchor.x,
      cy: childAnchor.y,
      r: 11,
      class: "body-chain-joint-node",
    }));
  }
}

function svgEl(name, attrs = {}) {
  const el = document.createElementNS("http://www.w3.org/2000/svg", name);
  for (const [key, value] of Object.entries(attrs)) {
    el.setAttribute(key, value);
  }
  return el;
}

function clearSvg(node) {
  while (node.firstChild) {
    node.removeChild(node.firstChild);
  }
}

function toSvgY(yMm) {
  return -yMm;
}

function toSvgPoint(point) {
  return { x: point.x_mm, y: toSvgY(point.y_mm) };
}

function parseOptionalPointInput(input, label) {
  const value = input.value.trim();
  if (!value) {
    return null;
  }
  const parts = value.split(/[\s,]+/).filter(Boolean);
  if (parts.length !== 2) {
    throw new Error(`${label} must use X,Y coordinates in millimetres.`);
  }
  const point = { x_mm: Number(parts[0]), y_mm: Number(parts[1]) };
  if (!Number.isFinite(point.x_mm) || !Number.isFinite(point.y_mm)) {
    throw new Error(`${label} coordinates must be finite numbers.`);
  }
  return point;
}

function syncGeometryMetadataFromInputs() {
  const maximumArticulationDeg = Number(vehicleMaxArticulationInput.value);
  if (!Number.isFinite(maximumArticulationDeg) || maximumArticulationDeg < 0) {
    throw new Error("Vehicle maximum articulation must be a non-negative number.");
  }
  state.geometry.frontArticulationPoint = parseOptionalPointInput(
    frontArticulationInput,
    "Front pivot",
  );
  state.geometry.rearArticulationPoint = parseOptionalPointInput(
    rearArticulationInput,
    "Rear pivot",
  );
  state.geometry.kingpinPoint = parseOptionalPointInput(kingpinInput, "Kingpin");
  state.geometry.maximumArticulationDeg = maximumArticulationDeg;
}

function syncGeometryMetadataInputs() {
  const pointText = (point) => point ? `${point.x_mm}, ${point.y_mm}` : "";
  frontArticulationInput.value = pointText(state.geometry.frontArticulationPoint);
  rearArticulationInput.value = pointText(state.geometry.rearArticulationPoint);
  kingpinInput.value = pointText(state.geometry.kingpinPoint);
  vehicleMaxArticulationInput.value = String(state.geometry.maximumArticulationDeg ?? 45);
}

function diagramWorldPoint(event) {
  const bounds = diagram.getBoundingClientRect();
  const viewBox = diagram.viewBox.baseVal;
  const fractionX = (event.clientX - bounds.left) / bounds.width;
  const fractionY = (event.clientY - bounds.top) / bounds.height;
  const svgX = viewBox.x + fractionX * viewBox.width;
  const svgY = viewBox.y + fractionY * viewBox.height;
  return { x_mm: svgX, y_mm: -svgY };
}

function hydrateEditableVehicleFromPayload(payload) {
  if (state.vehicleConfig || !payload?.vehicle_config) {
    return Boolean(state.vehicleConfig);
  }
  state.vehicleConfig = storedVehicleConfig(payload.vehicle_config);
  if (!state.vehicleConfig) {
    return false;
  }
  customAxleCountInput.value = String(state.customAxles.length);
  renderCustomAxleConfig();
  syncGeometryMetadataInputs();
  return true;
}

function updateDraggedAxlePreview(axleId, nextCenter) {
  const payloadAxle = (state.currentPayload?.axles || []).find((axle) => axle.axle_id === axleId);
  if (!payloadAxle) {
    return;
  }
  const previousCenter = payloadAxle.center;
  const delta = {
    x_mm: nextCenter.x_mm - previousCenter.x_mm,
    y_mm: nextCenter.y_mm - previousCenter.y_mm,
  };
  payloadAxle.center = nextCenter;
  const wheels = Array.isArray(payloadAxle.wheels) && payloadAxle.wheels.length > 0
    ? payloadAxle.wheels
    : [payloadAxle.left_wheel, payloadAxle.right_wheel];
  for (const wheel of wheels.filter(Boolean)) {
    wheel.center = {
      x_mm: wheel.center.x_mm + delta.x_mm,
      y_mm: wheel.center.y_mm + delta.y_mm,
    };
  }
  renderDiagram(state.currentPayload, {
    showIcr: state.displayMode === "simulation" || state.displayMode === "rays",
    showError: state.displayMode === "error",
  });
}

function lineFromHeading(point, headingRad, lengthMm) {
  return {
    x2: point.x_mm + Math.cos(headingRad) * lengthMm,
    y2: toSvgY(point.y_mm + Math.sin(headingRad) * lengthMm),
  };
}

function renderWheelTable(axles, actualSteering = null) {
  wheelTable.replaceChildren();
  const actualWheels = new Map(
    (actualSteering?.axles || []).flatMap((axle) => {
      const wheels = Array.isArray(axle.wheels) && axle.wheels.length > 0
        ? axle.wheels
        : [axle.left_wheel, axle.right_wheel];
      return wheels
        .filter(Boolean)
        .map((wheel) => [wheel.wheel_id, wheel]);
    }),
  );
  for (const axle of axles) {
    const wheels = Array.isArray(axle.wheels) && axle.wheels.length > 0
      ? axle.wheels
      : [axle.left_wheel, axle.right_wheel];
    for (const wheel of wheels.filter(Boolean)) {
      const actualWheel = actualWheels.get(wheel.wheel_id);
      const row = document.createElement("div");
      row.className = "wheel-row";
      const label = document.createElement("span");
      label.className = "label";
      label.textContent = wheel.wheel_id || `${axle.axle_id} ${wheel.side}`;
      const value = document.createElement("span");
      value.className = "value";
      const idealAngle = wheel.steering_angle_deg ?? wheel.heading_deg;
      const actualAngle = actualWheel?.steering_angle_deg ?? actualSteering?.wheel_angles_deg?.[wheel.wheel_id];
      const error = actualSteering?.errors_deg?.[wheel.wheel_id];
      value.textContent = actualAngle === undefined
        ? `Ideal: ${formatAngle(idealAngle)}`
        : `Ideal: ${formatAngle(idealAngle)} / Actual: ${formatAngle(actualAngle)} / Error: ${formatAngle(error)}`;
      if (actualWheel?.source) {
        value.title = `Actual steering source: ${actualWheel.source}`;
      }
      row.append(label, value);
      wheelTable.appendChild(row);
    }
  }
}

function renderSynchronizationTable(payload) {
  synchronizationTable.replaceChildren();
  const synchronizations = payload.vehicle_config?.steering_synchronizations || [];
  if (synchronizations.length === 0) {
    const row = document.createElement("div");
    row.className = "wheel-row";
    row.textContent = "No configured synchronization channels";
    synchronizationTable.appendChild(row);
    return;
  }
  const actual = payload.actual_steering || {};
  for (const sync of synchronizations) {
    const row = document.createElement("div");
    row.className = "wheel-row";
    const label = document.createElement("span");
    label.className = "label";
    label.textContent = `${sync.id} (${sync.mode}) -> ${sync.target_axle_id}`;
    const value = document.createElement("span");
    value.className = "value";
    const idealTarget = actual.synchronization_ideal_target_angles_deg?.[sync.id];
    const actualTarget = actual.axle_center_angles_deg?.[sync.target_axle_id];
    const error = actual.synchronization_errors_deg?.[sync.id];
    value.textContent = `Ideal: ${formatAngle(idealTarget)} / Actual: ${formatAngle(actualTarget)} / Error: ${formatAngle(error)}`;
    row.append(label, value);
    synchronizationTable.appendChild(row);
  }
}

function renderOptimizationVariables(variables) {
  optimizeVariableTable.replaceChildren();
  for (const variable of variables) {
    const row = document.createElement("div");
    row.className = "wheel-row";
    const label = document.createElement("span");
    label.className = "label";
    label.textContent = variable.id;
    const value = document.createElement("span");
    value.className = "value";
    const delta = variable.delta >= 0 ? "+" : "";
    value.textContent = `${variable.current.toFixed(2)} -> ${variable.optimized.toFixed(2)} (${delta}${variable.delta.toFixed(2)})`;
    row.append(label, value);
    optimizeVariableTable.appendChild(row);
  }
}

function renderOptimizationVariableConfig(variables) {
  optimizeVariableConfig.replaceChildren();
  if (!Array.isArray(variables) || variables.length === 0) {
    const empty = document.createElement("div");
    empty.className = "optimization-empty";
    empty.textContent = "No optimization variables available.";
    optimizeVariableConfig.appendChild(empty);
    return;
  }

  if (state.optimizationEnabledIds === null) {
    state.optimizationEnabledIds = new Set(
      variables.filter((variable) => variable.enabled).map((variable) => variable.id),
    );
  }

  for (const variable of variables) {
    const row = document.createElement("label");
    row.className = "optimization-variable-row";

    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.checked = state.optimizationEnabledIds.has(variable.id);
    checkbox.addEventListener("change", () => {
      if (checkbox.checked) {
        state.optimizationEnabledIds.add(variable.id);
      } else {
        state.optimizationEnabledIds.delete(variable.id);
      }
    });

    const name = document.createElement("span");
    name.className = "optimization-variable-name";
    name.textContent = variable.id;

    const bounds = document.createElement("span");
    bounds.className = "optimization-variable-bounds";
    bounds.textContent = `${variable.current.toFixed(2)} | ${variable.minimum.toFixed(2)} to ${variable.maximum.toFixed(2)}`;

    row.append(checkbox, name, bounds);
    optimizeVariableConfig.appendChild(row);
  }
}

function updateDxfApplyButtonState(enabled) {
  const metadataReady = Boolean(
    state.dxfImportPayload
      && dxfSourceUnits.value
      && dxfCoordinateSystem.value,
  );
  const unsupportedCount = Number(state.dxfImportPayload?.unsupported_entity_count || 0);
  const layoutReady = Boolean(state.dxfImportPayload?.reconstructed_vehicle);
  dxfApplyButton.disabled = !enabled || !metadataReady || unsupportedCount > 0 || !layoutReady;
  updateDxfSourceRetentionState();
}

function updateDxfSourceRetentionState() {
  if (!dxfRetainSourceButton || !dxfSourceRetentionStatus) {
    return;
  }
  const revisionCadSource = state.vehicleConfig?.cad_source;
  const importedCadSource = state.dxfImportPayload?.import_ready === true
    ? (state.dxfImportPayload.reconstructed_vehicle?.cad_source || state.dxfImportPayload.cad_source)
    : null;
  const confirmedCadSource = revisionCadSource
    && importedCadSource
    && revisionCadSource.source_name === importedCadSource.source_name
    && revisionCadSource.source_sha256 === importedCadSource.source_sha256
    ? importedCadSource
    : null;
  const hasConfirmedSource = Boolean(
    state.dxfImportText
      && confirmedCadSource?.metadata_confirmed === true
      && confirmedCadSource.source_name
      && state.dxfImportSourceName === confirmedCadSource.source_name,
  );
  const hasSavedRevision = Boolean(state.currentProjectId && state.activeProjectRevisionId);
  if (state.cadSourceArtifact) {
    dxfRetainSourceButton.disabled = true;
    dxfSourceRetentionStatus.textContent = `Source retained as ${state.cadSourceArtifact.filename || "CAD artifact"} (${state.cadSourceArtifact.content_sha256 || "checksum recorded"}).`;
    return;
  }
  dxfRetainSourceButton.disabled = !hasConfirmedSource
    || !hasSavedRevision
    || state.workspaceDirty
    || state.artifactStorageBackend !== "filesystem";
  if (!hasConfirmedSource) {
    dxfSourceRetentionStatus.textContent = "Apply confirmed CAD assignments before retaining the exact source bytes.";
  } else if (!hasSavedRevision) {
    dxfSourceRetentionStatus.textContent = "Save this revision before retaining the exact source bytes.";
  } else if (state.workspaceDirty) {
    dxfSourceRetentionStatus.textContent = "Save the current workspace changes before retaining the source on this revision.";
  } else if (state.artifactStorageBackend !== "filesystem") {
    dxfSourceRetentionStatus.textContent = "Durable artifact storage is not configured; the source hash is retained, but bytes cannot be attached.";
  } else {
    dxfSourceRetentionStatus.textContent = "The exact confirmed source can now be attached to this saved revision.";
  }
}

function renderDxfMetadata(payload, { activated = false } = {}) {
  const detectedUnits = payload?.detected_units;
  const sourceUnits = payload?.source_units;
  const selectionReady = Boolean(
    state.dxfImportText
      && dxfSourceUnits.value
      && dxfCoordinateSystem.value,
  );
  const selectedUnits = dxfSourceUnits.value || sourceUnits;
  const unitText = selectedUnits && selectedUnits !== "unitless"
    ? `Selected units: ${selectedUnits}`
    : "Source units: not confirmed";
  const headerText = detectedUnits
    ? `DXF header reports ${detectedUnits}.`
    : "DXF header does not declare units.";
  const unsupportedCount = Number(payload?.unsupported_entity_count || 0);
  if (unsupportedCount > 0) {
    dxfMetadataStatus.textContent = `${unsupportedCount} unsupported DXF entr${unsupportedCount === 1 ? "y" : "ies"} were omitted. Re-export supported geometry before applying assignments.`;
    dxfMetadataStatus.classList.add("warning");
    return;
  }
  if (!payload?.reconstructed_vehicle) {
    dxfMetadataStatus.textContent = "No valid vehicle layout was reconstructed. Assign a body envelope and axle centerlines, then correct any CAD geometry warnings before applying.";
    dxfMetadataStatus.classList.add("warning");
    return;
  }
  if (payload?.import_ready || selectionReady) {
    const frameText = payload?.import_ready
      ? "Axis frame confirmed for model use."
      : "Axis frame selected for model use.";
    const transformText = activated
      ? "Source scaling and metadata were applied to the active layout."
      : "Apply will rescale and record this source metadata.";
    dxfMetadataStatus.textContent = `${unitText}. ${frameText} ${headerText} ${transformText}`;
    dxfMetadataStatus.classList.remove("warning");
    return;
  }
  dxfMetadataStatus.textContent = `${unitText}. ${headerText} Select both fields before applying assignments.`;
  dxfMetadataStatus.classList.add("warning");
}

function updateDxfRowState(row, select) {
  const assignedRole = row.dataset.assignedRole ?? "";
  const isModified = select.value !== assignedRole;
  row.classList.toggle("modified", isModified);
  updateDxfApplyButtonState(Boolean(state.dxfImportText));
  if (isModified) {
    dxfImportStatus.textContent = "Assignments changed. Apply to rebuild the layout preview.";
  }
}

function renderDxfEntities(payload) {
  dxfEntityTable.replaceChildren();
  const entities = payload?.entities || [];
  if (!entities || entities.length === 0) {
    const empty = document.createElement("div");
    empty.className = "dxf-empty-row";
    const label = document.createElement("span");
    label.className = "label";
    label.textContent = "No entities";
    const value = document.createElement("span");
    value.className = "value";
    value.textContent = "Import a DXF file";
    empty.append(label, value);
    dxfEntityTable.appendChild(empty);
    return;
  }

  const roleOptions = getDxfRoleOptions(payload);
  for (const entity of entities) {
    const row = document.createElement("div");
    row.className = "dxf-entity-row";
    row.title = entity.reason || entity.summary || "";
    row.dataset.entityIndex = String(entity.index);
    row.dataset.assignedRole = entity.assigned_role ?? entity.suggested_role ?? "";

    const main = document.createElement("div");
    main.className = "dxf-entity-main";

    const label = document.createElement("div");
    label.className = "dxf-entity-label";
    const layerPart = entity.layer ? ` / ${entity.layer}` : "";
    label.textContent = `#${String(entity.index).padStart(2, "0")} ${entity.entity_type}${layerPart}`;

    const summary = document.createElement("div");
    summary.className = "dxf-entity-summary";
    summary.textContent = entity.summary || "No summary";

    const detail = document.createElement("div");
    detail.className = "dxf-entity-detail";
    const assignedRole = entity.assigned_role ?? entity.suggested_role ?? "";
    const suggestedRole = entity.suggested_role ?? "";
    const assignedLabel = roleLabelForValue(assignedRole, roleOptions);
    const suggestedLabel = roleLabelForValue(suggestedRole, roleOptions);
    detail.textContent = `Assigned: ${assignedLabel} | Suggested: ${suggestedLabel}`;

    main.append(label, summary, detail);

    const controls = document.createElement("div");
    controls.className = "dxf-entity-controls";
    const select = document.createElement("select");
    select.className = "dxf-entity-select";
    select.dataset.entityIndex = String(entity.index);
    select.setAttribute("aria-label", `Role assignment for ${label.textContent}`);
    for (const option of roleOptions) {
      const optionEl = document.createElement("option");
      optionEl.value = option.value;
      optionEl.textContent = option.label;
      select.appendChild(optionEl);
    }
    select.value = assignedRole;
    select.addEventListener("change", () => {
      updateDxfRowState(row, select);
    });
    controls.append(select);

    row.append(main, controls);
    dxfEntityTable.appendChild(row);
  }

  updateDxfApplyButtonState(entities.length > 0 && Boolean(state.dxfImportText));
}

function renderDxfImportSummary(payload) {
  state.cadSourceArtifact = null;
  state.dxfImportPayload = payload;
  const bounds = payload.bounds_mm;
  const vehicle = payload.reconstructed_vehicle;
  const parametric = payload.parametric_mechanism;
  const selectableUnits = new Set(["mm", "cm", "m", "in"]);
  dxfSourceUnits.value = selectableUnits.has(payload.source_units) ? payload.source_units : "";
  dxfCoordinateSystem.value = payload.coordinate_system || "";
  dxfEntityCount.textContent = String(payload.entity_count ?? 0);
  dxfSupportedCount.textContent = String(payload.supported_entity_count ?? 0);
  dxfBoundsValue.textContent = bounds
    ? `${formatMillimeters(bounds.min_x_mm)} .. ${formatMillimeters(bounds.max_x_mm)} / ${formatMillimeters(bounds.min_y_mm)} .. ${formatMillimeters(bounds.max_y_mm)}`
    : "n/a";
  dxfLayoutValue.textContent = vehicle
    ? `${formatMillimeters(vehicle.body_length_mm)} x ${formatMillimeters(vehicle.body_width_mm)} | ${vehicle.axles?.length ?? 0} axles`
    : "n/a";
  dxfParametricValue.textContent = parametric
    ? `${parametric.components?.length ?? 0} components / ${parametric.reference_points?.length ?? 0} points`
    : "n/a";
  const warningText = Array.isArray(payload.warnings) && payload.warnings.length > 0
    ? payload.warnings.join(" | ")
    : `Imported ${payload.source_name || "DXF"}`;
  dxfImportStatus.textContent = warningText;
  renderDxfMetadata(payload);
  updateDxfApplyButtonState(Boolean(state.dxfImportText && payload.entities && payload.entities.length));
  renderDxfEntities(payload);
}

function formatTimestamp(isoString) {
  const value = new Date(isoString);
  if (Number.isNaN(value.getTime())) {
    return isoString;
  }
  return value.toLocaleString();
}

const ROLE_PERMISSIONS = {
  viewer: ["project:read", "report:read"],
  designer: ["project:read", "project:write", "report:read", "job:submit", "revision:submit"],
  reviewer: ["project:read", "report:read", "revision:approve", "audit:read"],
  admin: ["project:read", "project:write", "report:read", "job:submit", "revision:submit", "revision:approve", "audit:read", "user:manage"],
};

function hasPermission(permission) {
  if (!state.authRequired && !state.authPrincipal) {
    return true;
  }
  return Boolean(state.authPrincipal && ROLE_PERMISSIONS[state.authPrincipal.role]?.includes(permission));
}

function renderAccessControlledControls() {
  const canWrite = hasPermission("project:write");
  projectCreateButton.disabled = !canWrite;
  projectSaveButton.disabled = !canWrite;
  projectNameInput.disabled = !canWrite;
  projectNoteInput.disabled = !canWrite;
  for (const action of projectRevisionList.querySelectorAll(".revision-action")) {
    action.disabled = !canWrite;
    action.title = action.disabled
      ? "Sign in with designer or admin access to load a revision."
      : "Load this revision";
  }
  if (state.authRequired && !state.authPrincipal && workspaceAccessCard) {
    workspaceAccessCard.open = true;
  }
  setOptimizationProposalState(Boolean(state.optimizationPayload?.optimized?.feasible === true));
}

function renderProjectRevisions(project) {
  projectRevisionList.replaceChildren();
  const revisions = project?.revisions || [];
  for (const revision of revisions) {
    const row = document.createElement("div");
    row.className = `revision-row${revision.id === project.active_revision_id ? " active" : ""}`;

    const text = document.createElement("div");
    const title = document.createElement("div");
    title.className = "revision-title";
    title.textContent = revision.note || revision.id;
    const meta = document.createElement("div");
    meta.className = "revision-meta";
    const minBeta = Number(revision.beta_min_deg ?? -45);
    const maxBeta = Number(revision.beta_max_deg ?? 45);
    const acceptance = revision.accepted_optimization ? " | APPLIED" : "";
    meta.textContent = `${revision.id} | ${formatTimestamp(revision.created_at)} | beta ${Number(revision.beta_deg).toFixed(1)} deg | case ${minBeta.toFixed(0)}..${maxBeta.toFixed(0)} deg | ${revision.optimization_mode}${acceptance}`;
    text.append(title, meta);

    const action = document.createElement("button");
    action.type = "button";
    action.className = "revision-action";
    action.textContent = "Load";
    action.disabled = !hasPermission("project:write");
    action.title = action.disabled
      ? "Sign in with designer or admin access to load a revision."
      : "Load this revision";
    action.addEventListener("click", () => restoreProjectRevision(revision.id));

    row.append(text, action);
    projectRevisionList.appendChild(row);
  }
}

function renderProjectSelector(projects, selectedId = null) {
  if (!projectSelector) {
    return;
  }
  projectSelector.replaceChildren();
  if (!Array.isArray(projects) || projects.length === 0) {
    const empty = document.createElement("option");
    empty.value = "";
    empty.textContent = "No projects yet";
    projectSelector.appendChild(empty);
    projectSelector.disabled = true;
    return;
  }
  for (const project of projects) {
    const option = document.createElement("option");
    option.value = project.id;
    option.textContent = `${project.name || project.id} (${project.revision_count ?? 0} revisions)`;
    projectSelector.appendChild(option);
  }
  projectSelector.value = selectedId && projects.some((project) => project.id === selectedId)
    ? selectedId
    : projects[0].id;
  projectSelector.disabled = false;
}

function renderProjectSummary(project) {
  if (!project) {
    state.activeProjectRevisionId = null;
    state.approvalStatus = null;
    state.approvalHistory = [];
    state.acceptanceResult = null;
    state.activeRevisionHasFullRangeEvidence = false;
    projectIdValue.textContent = "n/a";
    projectActiveRevisionValue.textContent = "n/a";
    projectRevisionCountValue.textContent = "0";
    projectRevisionList.replaceChildren();
    renderMonrocAcceptance(null);
    renderApprovalHistory();
    renderReviewControls();
    renderProjectDashboardStatus();
    return;
  }

  state.activeProjectRevisionId = project.active_revision_id || null;
  if (projectSelector) {
    let option = [...projectSelector.options].find((item) => item.value === project.id);
    if (!option) {
      option = document.createElement("option");
      option.value = project.id;
      projectSelector.appendChild(option);
    }
    option.textContent = `${project.name || project.id} (${project.revision_count ?? project.revisions?.length ?? 0} revisions)`;
    projectSelector.value = project.id;
    projectSelector.disabled = false;
  }
  projectIdValue.textContent = project.id;
  projectActiveRevisionValue.textContent = project.active_revision_id || "n/a";
  projectRevisionCountValue.textContent = String(project.revision_count ?? project.revisions?.length ?? 0);
  renderProjectRevisions(project);
  renderReviewControls();
  renderProjectDashboardStatus();
}

function renderFailureGuidance(element, guidanceOrIds) {
  if (!element) {
    return;
  }
  const guidance = (Array.isArray(guidanceOrIds) ? guidanceOrIds : [])
    .map((item) => {
      if (typeof item === "string") {
        const fallback = FAILURE_GUIDANCE[item] || {
          title: `Investigate ${item}`,
          action: "Review the detailed failure and correct the design before saving or submitting this revision.",
        };
        return { check_id: item, ...fallback };
      }
      return item;
    })
    .filter((item) => item && item.title && item.action);
  element.replaceChildren();
  element.hidden = guidance.length === 0;
  if (guidance.length === 0) {
    return;
  }
  const heading = document.createElement("h4");
  heading.textContent = "Next engineering actions";
  const list = document.createElement("div");
  list.className = "validation-guidance-list";
  for (const item of guidance) {
    const row = document.createElement("div");
    row.className = "validation-guidance-item";
    const title = document.createElement("strong");
    title.textContent = item.check_id ? `${item.check_id}: ${item.title}` : item.title;
    const action = document.createElement("p");
    action.textContent = item.action;
    row.append(title, action);
    list.appendChild(row);
  }
  element.append(heading, list);
}

function renderApprovalHistory() {
  if (!approvalHistory) {
    return;
  }
  approvalHistory.replaceChildren();
  const events = Array.isArray(state.approvalHistory) ? state.approvalHistory : [];
  if (events.length === 0) {
    const empty = document.createElement("p");
    empty.className = "curve-note";
    empty.textContent = "No approval events recorded.";
    approvalHistory.appendChild(empty);
    return;
  }
  const eventLabels = {
    REVISION_SUBMITTED: "Submitted for review",
    REVISION_APPROVED: "Approved",
    REVISION_REJECTED: "Rejected",
  };
  for (const event of events) {
    const row = document.createElement("div");
    row.className = "approval-history-row";
    const title = document.createElement("strong");
    title.textContent = eventLabels[event.event_type] || event.event_type || "Approval event";
    const meta = document.createElement("span");
    meta.textContent = `${formatTimestamp(event.created_at)} | actor ${event.actor_user_id || "system"}`;
    row.append(title, meta);
    const note = event.metadata?.note;
    if (note) {
      const noteElement = document.createElement("p");
      noteElement.textContent = note;
      row.appendChild(noteElement);
    }
    approvalHistory.appendChild(row);
  }
}

function acceptanceIsReleaseApproved(result) {
  return result?.status === "PASS" && result?.criteria_approval?.status === "APPROVED";
}

function renderMonrocAcceptance(result) {
  if (!acceptanceStatusNote || !acceptanceChecks || !acceptanceEvaluateButton) {
    return;
  }
  const displayedResult = state.acceptanceCriteriaDirty ? null : result;
  const status = displayedResult?.status || "NOT_CONFIGURED";
  acceptanceStatusNote.dataset.status = status.toLowerCase();
  if (status === "PASS") {
    acceptanceStatusNote.textContent = acceptanceIsReleaseApproved(displayedResult)
      ? `PASS: ${displayedResult.case_id} matches the approved Monroc profile and satisfies its limits.`
      : `PASS: ${displayedResult.case_id} satisfies the entered Monroc limits, but the profile is not release-approved.`;
  } else if (status === "FAIL") {
    acceptanceStatusNote.textContent = `FAIL: ${displayedResult.case_id || "case"} does not satisfy one or more configured Monroc limits.`;
  } else if (status === "UNAPPROVED") {
    acceptanceStatusNote.textContent = `UNAPPROVED: ${displayedResult.message || "The entered limits do not match an approved Monroc profile."}`;
  } else if (state.acceptanceCriteriaDirty) {
    acceptanceStatusNote.textContent = "Acceptance criteria changed. Re-evaluate this saved revision before review.";
  } else {
    acceptanceStatusNote.textContent = "No Monroc criteria evaluated for this revision.";
  }
  acceptanceChecks.replaceChildren();
  for (const check of displayedResult?.checks || []) {
    const row = document.createElement("div");
    row.className = "validation-check";
    row.dataset.status = String(check.status || "FAIL").toLowerCase();
    const checkStatus = document.createElement("strong");
    checkStatus.textContent = check.status || "FAIL";
    const detail = document.createElement("span");
    detail.textContent = `${check.label || check.id}: ${check.detail || "No detail"}`;
    row.append(checkStatus, detail);
    acceptanceChecks.appendChild(row);
  }
  if (displayedResult?.criteria_approval) {
    const profileRow = document.createElement("div");
    profileRow.className = "validation-check";
    profileRow.dataset.status = displayedResult.criteria_approval.status === "APPROVED" ? "pass" : "fail";
    const profileStatus = document.createElement("strong");
    profileStatus.textContent = displayedResult.criteria_approval.status;
    const profileDetail = document.createElement("span");
    profileDetail.textContent = `Acceptance profile: ${displayedResult.criteria_approval.message || "No profile detail"}`;
    profileRow.append(profileStatus, profileDetail);
    acceptanceChecks.appendChild(profileRow);
  }
  acceptanceEvaluateButton.disabled = !state.currentProjectId
    || !state.activeProjectRevisionId
    || !hasPermission("project:write");
}

function markAcceptanceCriteriaDirty() {
  if (state.acceptanceCriteriaDirty) {
    return;
  }
  state.acceptanceCriteriaDirty = true;
  state.acceptanceResult = null;
  renderMonrocAcceptance(null);
  renderCurrentSteeringInterpretation(state.currentPayload);
  renderReleaseChecklist();
  updateExportLinks();
}

async function evaluateMonrocAcceptance() {
  if (!state.currentProjectId || !state.activeProjectRevisionId) {
    throw new Error("Save a project revision before evaluating Monroc acceptance.");
  }
  const caseId = acceptanceCaseIdInput.value.trim();
  const limits = {
    minimum_clearance_mm: Number(acceptanceMinClearanceInput.value),
    maximum_wheel_error_deg: Number(acceptanceMaxWheelErrorInput.value),
    maximum_synchronization_error_deg: Number(acceptanceMaxSyncErrorInput.value),
    maximum_mechanism_residual_mm: Number(acceptanceMaxResidualInput.value),
  };
  if (!caseId || Object.values(limits).some((value) => !Number.isFinite(value) || value < 0)) {
    throw new Error("Enter a case ID and non-negative numeric limits for every criterion.");
  }

  acceptanceEvaluateButton.disabled = true;
  acceptanceStatusNote.dataset.status = "running";
  acceptanceStatusNote.textContent = "Evaluating the saved revision against the configured Monroc limits...";
  try {
    const response = await fetch(
      `/api/projects/${encodeURIComponent(state.currentProjectId)}/revisions/${encodeURIComponent(state.activeProjectRevisionId)}/acceptance`,
      {
        method: "POST",
        headers: authHeaders(),
        body: JSON.stringify({
          criteria: {
            case_id: caseId,
            ...limits,
            require_full_range: acceptanceRequireFullRangeInput.checked,
          },
        }),
      },
    );
    const payload = await response.json();
    if (!response.ok) {
      throw new Error(payload.message || `HTTP ${response.status}`);
    }
    state.acceptanceCriteriaDirty = false;
    state.acceptanceResult = payload.acceptance || null;
    renderMonrocAcceptance(state.acceptanceResult);
    renderCurrentSteeringInterpretation(state.currentPayload);
    renderReleaseChecklist();
    updateExportLinks();
  } finally {
    if (!state.acceptanceResult || state.acceptanceResult.status !== "PASS") {
      acceptanceEvaluateButton.disabled = !state.currentProjectId
        || !state.activeProjectRevisionId
        || !hasPermission("project:write");
    }
  }
}

function renderReleaseChecklist() {
  if (!releaseChecklist || !releaseChecklistState || !releaseChecklistNote) {
    return;
  }
  const hasRevision = Boolean(state.currentProjectId && state.activeProjectRevisionId);
  const currentPass = hasRevision && !state.workspaceDirty && state.currentValidationPass;
  const fullRangePass = hasRevision && !state.workspaceDirty && state.activeRevisionHasFullRangeEvidence;
  const approval = state.approvalStatus;
  const independentlyApproved = approval?.status === "approved"
    && Boolean(approval.submitted_by)
    && Boolean(approval.decided_by)
    && approval.submitted_by !== approval.decided_by;
  const acceptancePass = !state.acceptanceCriteriaDirty && acceptanceIsReleaseApproved(state.acceptanceResult);
  const checklist = [
    {
      label: "Saved revision",
      status: hasRevision && !state.workspaceDirty ? "pass" : "fail",
      detail: !hasRevision
        ? "Create or save a revision first."
        : (state.workspaceDirty
          ? "Unsaved workspace changes are present; save a new revision before review."
          : `Revision ${state.activeProjectRevisionId} is loaded.`),
    },
    {
      label: "Current hard checks",
      status: currentPass ? "pass" : "fail",
      detail: currentPass ? "Kinematics, mechanism, collision, and clearance checks pass." : "Run the current pose and resolve every hard-check failure.",
    },
    {
      label: "Full articulation range",
      status: fullRangePass ? "pass" : "fail",
      detail: fullRangePass
        ? "Saved evidence covers the configured articulation/design-case range."
        : (state.combinationActive
          ? "Run and save a passing full-range validation sweep."
          : "Run a hard-feasible optimization and save the accepted result."),
    },
    {
      label: "Monroc acceptance criteria",
      status: acceptancePass
        ? "pass"
        : (["FAIL", "UNAPPROVED"].includes(state.acceptanceResult?.status) ? "fail" : "pending"),
      detail: acceptancePass
        ? `Case ${state.acceptanceResult.case_id} passes the configured Monroc limits.`
        : (state.acceptanceResult?.status === "FAIL"
          ? "Correct the failing acceptance criteria before approval."
        : (state.acceptanceResult?.status === "UNAPPROVED"
          ? "The entered limits are trial criteria only; use the configured approved Monroc profile before approval."
          : "Enter signed-off Monroc limits and evaluate this revision.")),
    },
    {
      label: "Independent approval",
      status: independentlyApproved ? "pass" : (approval?.status === "rejected" ? "fail" : "pending"),
      detail: independentlyApproved
        ? `Approved by ${approval.decided_by}, separate from submitter ${approval.submitted_by}.`
        : (approval?.status === "submitted"
          ? "Waiting for a reviewer other than the submitting designer."
          : "Submit the saved revision for independent review."),
    },
  ];
  releaseChecklist.replaceChildren();
  for (const item of checklist) {
    const row = document.createElement("div");
    row.className = "release-check";
    row.dataset.status = item.status;
    const status = document.createElement("strong");
    status.textContent = item.status.toUpperCase();
    const detail = document.createElement("span");
    detail.textContent = `${item.label}: ${item.detail}`;
    row.append(status, detail);
    releaseChecklist.appendChild(row);
  }
  const ready = checklist.every((item) => item.status === "pass");
  releaseChecklistState.dataset.status = ready ? "ready" : "blocked";
  releaseChecklistState.textContent = ready ? "READY" : "BLOCKED";
  releaseChecklistNote.textContent = ready
    ? "All configured release gates pass. Manufacturing release still requires the approved Monroc process outside this prototype."
    : (approval?.status === "approved" && !currentPass
      ? "Approval does not override a failed current engineering check."
      : "A passing calculation is necessary but not sufficient for manufacturing release.");
  renderResultsDecision({ ready, currentPass, fullRangePass, acceptancePass, independentlyApproved });
}

function renderResultsDecision({ ready, currentPass, fullRangePass, acceptancePass, independentlyApproved }) {
  if (!resultsDecisionCard || !resultsDecisionStatus || !resultsDecisionSummary || !resultsDecisionChecks || !resultsDecisionNote) {
    return;
  }
  const currentStatus = state.currentPayload
    ? (currentPass ? "PASS" : "FAIL")
    : "NOT RUN";
  const fullRangeStatus = state.combinationActive
    ? (state.sweepValidationPayload?.status || "NOT RUN")
    : (fullRangePass ? "PASS" : "NOT RUN");
  const engineeringStatus = !state.currentPayload
    ? "NOT RUN"
    : !currentPass || fullRangeStatus === "FAIL"
      ? "FAIL"
      : fullRangeStatus !== "PASS"
        ? "INCOMPLETE"
        : "PASS";
  const decisionChecks = [
    { label: "Current pose hard checks", status: currentStatus },
    { label: state.combinationActive ? "Full articulation range" : "Design-range evidence", status: fullRangeStatus },
    { label: "Monroc acceptance", status: acceptancePass ? "PASS" : (state.acceptanceResult?.status === "FAIL" ? "FAIL" : (state.acceptanceResult?.status === "UNAPPROVED" ? "UNAPPROVED" : "PENDING")) },
    { label: "Independent approval", status: independentlyApproved ? "PASS" : "PENDING" },
    { label: "Manufacturing release", status: ready ? "READY" : "BLOCKED" },
  ];
  resultsDecisionCard.dataset.status = engineeringStatus === "PASS" && ready
    ? "pass"
    : engineeringStatus === "FAIL"
      ? "fail"
      : "pending";
  resultsDecisionStatus.textContent = engineeringStatus;
  resultsDecisionSummary.textContent = engineeringStatus === "PASS"
    ? (ready
      ? "Engineering checks pass and every release gate is complete."
      : "The engineering checks pass, but this revision is not released.")
    : engineeringStatus === "FAIL"
      ? "This design is diagnostic only. Resolve the failed hard check before review or release."
      : "The engineering decision is incomplete. Run the current pose and full-range validation before interpreting the result.";
  resultsDecisionChecks.replaceChildren();
  for (const item of decisionChecks) {
    const row = document.createElement("div");
    row.className = "decision-check";
    row.dataset.status = item.status.toLowerCase();
    const status = document.createElement("strong");
    status.textContent = item.status;
    const label = document.createElement("span");
    label.textContent = item.label;
    row.append(status, label);
    resultsDecisionChecks.appendChild(row);
  }
  resultsDecisionNote.textContent = ready
    ? "Controlled release is available only for this saved, accepted, independently approved revision."
    : engineeringStatus === "FAIL"
      ? "Do not apply or export this result as an approved design. Diagnostic exports remain available for troubleshooting."
      : "Diagnostic results are not a manufacturing release. Complete validation, signed-off Monroc criteria, and independent approval.";
}

function renderReviewerOptions() {
  if (!reviewerSelector) {
    return;
  }
  const selectedReviewerId = state.approvalStatus?.assigned_reviewer_id || "";
  reviewerSelector.replaceChildren();
  const unassigned = document.createElement("option");
  unassigned.value = "";
  unassigned.textContent = "Unassigned: any eligible reviewer";
  reviewerSelector.appendChild(unassigned);
  const eligibleUsers = state.reviewerUsers.filter((user) => ["reviewer", "admin"].includes(user.role));
  for (const user of eligibleUsers) {
    const option = document.createElement("option");
    option.value = user.user_id;
    option.textContent = `${user.display_name} (${user.email})`;
    reviewerSelector.appendChild(option);
  }
  if (selectedReviewerId && !eligibleUsers.some((user) => user.user_id === selectedReviewerId)) {
    const unknown = document.createElement("option");
    unknown.value = selectedReviewerId;
    unknown.textContent = `Assigned user ${selectedReviewerId} (not available)`;
    reviewerSelector.appendChild(unknown);
  }
  reviewerSelector.value = selectedReviewerId;
  reviewerSelector.disabled = !hasPermission("user:manage")
    || state.approvalStatus?.status === "approved";
}

function eligibleReviewerCount() {
  return state.reviewerUsers.filter((user) => ["reviewer", "admin"].includes(user.role)).length;
}

function renderReviewControls() {
  const status = state.approvalStatus?.status || "draft";
  const role = state.authPrincipal?.role || (state.authRequired ? null : "admin");
  const hasRevision = Boolean(state.currentProjectId && state.activeProjectRevisionId);
  const assignedReviewerId = state.approvalStatus?.assigned_reviewer_id || null;
  const canSubmit = hasRevision
    && ["designer", "admin"].includes(role)
    && !["submitted", "approved"].includes(status)
    && !state.acceptanceCriteriaDirty
    && !state.workspaceDirty;
  const canDecide = hasRevision
    && ["reviewer", "admin"].includes(role)
    && status === "submitted"
    && (!assignedReviewerId || assignedReviewerId === state.authPrincipal?.user_id);
  reviewState.textContent = status.toUpperCase();
  reviewState.dataset.status = status;
  reviewSubmitButton.disabled = !canSubmit;
  reviewApproveButton.disabled = !canDecide;
  reviewRejectButton.disabled = !canDecide;
  renderReviewerOptions();
  const assignedReviewer = state.reviewerUsers.find(
    (user) => user.user_id === state.approvalStatus?.assigned_reviewer_id,
  );
  const canAssignReviewer = hasRevision
    && hasPermission("user:manage")
    && status !== "approved";
  reviewerAssignButton.disabled = !canAssignReviewer
    || reviewerSelector.value === (state.approvalStatus?.assigned_reviewer_id || "");
  if (!canAssignReviewer) {
    reviewerAssignmentStatus.textContent = state.authRequired && !state.authPrincipal
      ? "Sign in as an administrator to route this revision."
      : "Only an administrator can route a revision to an independent reviewer.";
  } else if (eligibleReviewerCount() === 0) {
    reviewerAssignmentStatus.textContent = "No active reviewer accounts are available in this organization.";
  } else if (assignedReviewer) {
    reviewerAssignmentStatus.textContent = `Currently assigned to ${assignedReviewer.display_name} (${assignedReviewer.email}).`;
  } else if (state.approvalStatus?.assigned_reviewer_id) {
    reviewerAssignmentStatus.textContent = `Currently assigned to user ${state.approvalStatus.assigned_reviewer_id}, which is not available.`;
  } else {
    reviewerAssignmentStatus.textContent = "Assigning a reviewer makes the review owner explicit; the submitter still cannot self-approve.";
  }
  if (state.authRequired && !state.authPrincipal) {
    reviewStatusNote.textContent = "Sign in to submit or independently review this revision.";
  } else if (role === "viewer") {
    reviewStatusNote.textContent = "Viewer access is read-only. A designer must submit the revision for review.";
  } else if (!hasRevision) {
    reviewStatusNote.textContent = "Save a revision before submitting it for independent review.";
  } else if (state.workspaceDirty) {
    reviewStatusNote.textContent = "Unsaved workspace changes are present. Save a new revision before submitting it for review.";
  } else if (state.acceptanceCriteriaDirty) {
    reviewStatusNote.textContent = "Acceptance criteria changed. Re-evaluate the saved revision before submitting it for review.";
  } else if (status === "draft") {
    reviewStatusNote.textContent = "Draft revision. Submit the saved engineering evidence when it is ready for review.";
  } else if (status === "submitted") {
    reviewStatusNote.textContent = assignedReviewerId
      ? "Submitted and routed to the assigned reviewer. Only that reviewer can decide this revision."
      : "Submitted. A reviewer must check the engineering PASS and decide independently.";
  } else if (status === "approved") {
    reviewStatusNote.textContent = "Approved by an independent reviewer. This status does not override a failed engineering check.";
  } else {
    reviewStatusNote.textContent = "Rejected. Update the design, save a new revision, and submit it again.";
  }
  renderReleaseChecklist();
  renderProjectDashboardStatus();
}

async function loadApprovalStatus() {
  if (!state.currentProjectId || !state.activeProjectRevisionId) {
    state.approvalStatus = null;
    state.approvalHistory = [];
    renderApprovalHistory();
    renderReviewControls();
    return;
  }
  try {
    const path = `/api/projects/${encodeURIComponent(state.currentProjectId)}/revisions/${encodeURIComponent(state.activeProjectRevisionId)}/approval`;
    const response = await fetch(path, { headers: authHeaders() });
    const payload = await response.json();
    if (!response.ok) {
      throw new Error(payload.message || `HTTP ` + response.status);
    }
    state.approvalStatus = payload.approval;
  } catch (error) {
    state.approvalStatus = null;
    reviewStatusNote.textContent = `Review state unavailable: ${error.message}`;
  }
  await loadApprovalHistory();
  renderReviewControls();
}

async function loadApprovalHistory() {
  if (!state.currentProjectId || !state.activeProjectRevisionId) {
    state.approvalHistory = [];
    renderApprovalHistory();
    return;
  }
  try {
    const path = `/api/projects/${encodeURIComponent(state.currentProjectId)}/revisions/${encodeURIComponent(state.activeProjectRevisionId)}/approval-history`;
    const response = await fetch(path, { headers: authHeaders() });
    const payload = await response.json();
    if (!response.ok) {
      throw new Error(payload.message || `HTTP ` + response.status);
    }
    state.approvalHistory = Array.isArray(payload.events) ? payload.events : [];
  } catch (error) {
    state.approvalHistory = [];
  }
  renderApprovalHistory();
}

async function assignReviewer() {
  if (!state.currentProjectId || !state.activeProjectRevisionId || !reviewerSelector) {
    return;
  }
  reviewerAssignButton.disabled = true;
  reviewerAssignmentStatus.textContent = "Saving reviewer assignment...";
  try {
    const path = `/api/projects/${encodeURIComponent(state.currentProjectId)}/revisions/${encodeURIComponent(state.activeProjectRevisionId)}/reviewer`;
    const response = await fetch(path, {
      method: "POST",
      headers: authHeaders(),
      body: JSON.stringify({ reviewer_user_id: reviewerSelector.value || null }),
    });
    const payload = await response.json();
    if (!response.ok) {
      throw new Error(payload.message || `HTTP ${response.status}`);
    }
    state.approvalStatus = payload.approval;
    await loadApprovalHistory();
    renderReviewControls();
  } finally {
    renderReviewControls();
  }
}

async function submitRevisionForReview() {
  if (!state.currentProjectId || !state.activeProjectRevisionId) {
    return;
  }
  reviewSubmitButton.disabled = true;
  const path = `/api/projects/${encodeURIComponent(state.currentProjectId)}/revisions/${encodeURIComponent(state.activeProjectRevisionId)}/submit`;
  const response = await fetch(path, {
    method: "POST",
    headers: authHeaders(),
    body: JSON.stringify({ note: projectNoteInput.value || "Submitted for independent review" }),
  });
  const payload = await response.json();
  if (!response.ok) {
    throw new Error(payload.message || `HTTP ` + response.status);
  }
  state.approvalStatus = payload.approval;
  await loadApprovalHistory();
  renderReviewControls();
}

async function decideRevision(approved) {
  if (!state.currentProjectId || !state.activeProjectRevisionId) {
    return;
  }
  const path = `/api/projects/${encodeURIComponent(state.currentProjectId)}/revisions/${encodeURIComponent(state.activeProjectRevisionId)}/approval`;
  const response = await fetch(path, {
    method: "POST",
    headers: authHeaders(),
    body: JSON.stringify({
      approved,
      note: projectNoteInput.value || (approved ? "Independent review complete" : "Revision requires changes"),
    }),
  });
  const payload = await response.json();
  if (!response.ok) {
    throw new Error(payload.message || `HTTP ` + response.status);
  }
  state.approvalStatus = payload.approval;
  await loadApprovalHistory();
  renderReviewControls();
  updateExportLinks();
}

function updateExportLinks() {
  renderWorkflowProgress();
  const exportLinks = [
    exportJsonLink,
    exportCsvLink,
    exportPdfLink,
    exportPngLink,
    exportSvgLink,
    exportDxfLink,
    exportReleaseLink,
  ];
  const releaseEngineeringReady = !state.workspaceDirty
    && state.currentValidationPass
    && state.activeRevisionHasFullRangeEvidence
    && acceptanceIsReleaseApproved(state.acceptanceResult);
  if (state.combinationActive) {
    for (const link of exportLinks) {
      link.href = "#";
      link.setAttribute("aria-disabled", "true");
      link.title = "Manufacturing export requires a validated and controlled multi-body revision.";
    }
    if (state.currentProjectId && state.activeProjectRevisionId) {
      const base = `/api/projects/${encodeURIComponent(state.currentProjectId)}/revisions/${encodeURIComponent(state.activeProjectRevisionId)}`;
      for (const [link, extension] of [
        [exportJsonLink, "json"],
        [exportCsvLink, "csv"],
        [exportPdfLink, "pdf"],
        [exportPngLink, "png"],
        [exportSvgLink, "svg"],
        [exportDxfLink, "dxf"],
      ]) {
        link.href = `${base}/export.${extension}`;
        link.removeAttribute("aria-disabled");
        link.removeAttribute("title");
      }
      if (state.approvalStatus?.status === "approved" && releaseEngineeringReady) {
        exportReleaseLink.href = `${base}/release.json`;
        exportReleaseLink.removeAttribute("aria-disabled");
        exportReleaseLink.removeAttribute("title");
      } else {
        exportReleaseLink.title = "Engineering PASS, Monroc acceptance PASS, and independent approval are required.";
      }
    }
    exportNote.textContent = state.approvalStatus?.status === "approved" && releaseEngineeringReady
      ? "Diagnostic evidence and the controlled release manifest are available. The manifest is tied to this approved revision."
      : "Diagnostic JSON, CSV, PDF, SVG, DXF, and PNG are available. The controlled release manifest stays blocked until engineering PASS, Monroc acceptance PASS, and independent approval.";
    return;
  }
  for (const link of exportLinks) {
    link.removeAttribute("aria-disabled");
    link.removeAttribute("title");
  }
  if (state.currentProjectId && state.activeProjectRevisionId && state.approvalStatus?.status === "approved" && releaseEngineeringReady) {
    exportReleaseLink.href = `/api/projects/${encodeURIComponent(state.currentProjectId)}/revisions/${encodeURIComponent(state.activeProjectRevisionId)}/release.json`;
    exportReleaseLink.removeAttribute("aria-disabled");
    exportReleaseLink.removeAttribute("title");
  } else {
    exportReleaseLink.href = "#";
    exportReleaseLink.setAttribute("aria-disabled", "true");
    exportReleaseLink.title = "Save, validate, evaluate Monroc acceptance, and independently approve a project revision first.";
  }
  exportNote.textContent = state.approvalStatus?.status === "approved" && releaseEngineeringReady
    ? "Diagnostic exports and the controlled release manifest are available for this independently approved revision."
    : "Diagnostic exports use the current beta and optimization mode. The controlled release manifest requires engineering PASS, Monroc acceptance PASS, and independent approval.";
  const betaDeg = Number(betaSlider.value);
  const mode = optimizeMode.value;
  const linkageQuery = state.linkageConfig
    ? `&linkage=${encodeURIComponent(JSON.stringify(serializedLinkageConfig()))}`
    : "";
  const query = `beta_deg=${encodeURIComponent(betaDeg)}&mode=${encodeURIComponent(mode)}&${geometryQuery()}${linkageQuery}${vehicleConfigQuery()}`;
  exportJsonLink.href = `/api/export.json?${query}`;
  exportCsvLink.href = `/api/export.csv?${query}`;
  exportPdfLink.href = `/api/export.pdf?${query}`;
  exportPngLink.href = `/api/export.png?${query}`;
  exportSvgLink.href = `/api/export.svg?${query}`;
  exportDxfLink.href = `/api/export.dxf?${query}`;
}

async function importSelectedDxfFile() {
  const file = dxfFileInput.files && dxfFileInput.files[0];
  if (!file) {
    dxfImportStatus.textContent = "Choose a DXF file first.";
    return;
  }

  const requestId = ++state.dxfImportRequest;
  dxfImportButton.disabled = true;
  dxfImportButton.textContent = "Importing...";
  dxfImportStatus.textContent = `Reading ${file.name}...`;
  try {
    const dxfText = await file.text();
    state.dxfImportText = dxfText;
    state.dxfImportSourceName = file.name;
    const response = await fetch("/api/import.dxf", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        source_name: file.name,
        dxf_text: dxfText,
      }),
    });
    if (!response.ok) {
      const errorText = await response.text();
      throw new Error(errorText || `HTTP ${response.status}`);
    }
    const payload = await response.json();
    if (requestId !== state.dxfImportRequest) {
      return;
    }
    renderDxfImportSummary(payload);
    dxfImportButton.textContent = "Re-import";
  } catch (error) {
    if (requestId === state.dxfImportRequest) {
      dxfImportStatus.textContent = `DXF import failed: ${error.message}`;
    }
  } finally {
    if (requestId === state.dxfImportRequest) {
      dxfImportButton.disabled = false;
      dxfImportButton.textContent = state.dxfImportText ? "Re-import" : "Import";
    }
  }
}

function collectDxfRoleOverrides() {
  const overrides = {};
  const selects = dxfEntityTable.querySelectorAll("select[data-entity-index]");
  for (const select of selects) {
    const index = select.dataset.entityIndex;
    if (index !== undefined) {
      overrides[index] = select.value;
    }
  }
  return overrides;
}

async function applyDxfAssignments() {
  if (!state.dxfImportText) {
    dxfImportStatus.textContent = "Import a DXF file before applying assignments.";
    return;
  }

  const requestId = ++state.dxfImportRequest;
  dxfApplyButton.disabled = true;
  dxfApplyButton.textContent = "Applying...";
  dxfImportStatus.textContent = "Applying manual assignments...";
  try {
    const response = await fetch("/api/import.dxf", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        source_name: state.dxfImportSourceName,
        dxf_text: state.dxfImportText,
        source_units: dxfSourceUnits.value,
        coordinate_system: dxfCoordinateSystem.value,
        confirm_metadata: true,
        role_overrides: collectDxfRoleOverrides(),
      }),
    });
    if (!response.ok) {
      const errorText = await response.text();
      throw new Error(errorText || `HTTP ${response.status}`);
    }
    const payload = await response.json();
    if (requestId !== state.dxfImportRequest) {
      return;
    }
    renderDxfImportSummary(payload);
    const activated = await activateImportedVehicle(payload);
    renderDxfMetadata(payload, { activated });
    dxfImportStatus.textContent = activated
      ? `Applied assignments and activated the parametric layout from ${state.dxfImportSourceName || "DXF"}.`
      : `Applied manual assignments for ${state.dxfImportSourceName || "DXF"}.`;
  } catch (error) {
    if (requestId === state.dxfImportRequest) {
      dxfImportStatus.textContent = `DXF assignment failed: ${error.message}`;
    }
  } finally {
    if (requestId === state.dxfImportRequest) {
      dxfApplyButton.disabled = false;
      dxfApplyButton.textContent = "Apply assignments";
    }
  }
}

async function retainDxfSource() {
  if (!state.currentProjectId || !state.activeProjectRevisionId || !state.dxfImportText) {
    dxfSourceRetentionStatus.textContent = "Save a confirmed DXF assignment as a revision before retaining its source.";
    return;
  }
  dxfRetainSourceButton.disabled = true;
  dxfSourceRetentionStatus.textContent = "Retaining the exact source bytes on this revision...";
  try {
    const path = `/api/projects/${encodeURIComponent(state.currentProjectId)}/revisions/${encodeURIComponent(state.activeProjectRevisionId)}/cad-source`;
    const response = await fetch(path, {
      method: "POST",
      headers: authHeaders(),
      body: JSON.stringify({
        source_name: state.dxfImportSourceName,
        dxf_text: state.dxfImportText,
      }),
    });
    const payload = await response.json();
    if (!response.ok) {
      throw new Error(payload.message || `HTTP ${response.status}`);
    }
    state.cadSourceArtifact = payload.artifact || null;
    updateDxfSourceRetentionState();
    dxfImportStatus.textContent = `Confirmed DXF source retained on revision ${state.activeProjectRevisionId}.`;
  } catch (error) {
    dxfSourceRetentionStatus.textContent = `CAD source retention failed: ${error.message}`;
    updateDxfSourceRetentionState();
  }
}

let diagnosticPreviewRequest = 0;

function clearDiagnosticPreview(image) {
  const objectUrl = image.dataset.previewObjectUrl;
  if (objectUrl) {
    URL.revokeObjectURL(objectUrl);
    delete image.dataset.previewObjectUrl;
  }
  image.removeAttribute("src");
}

async function loadDiagnosticPreview(image, statusElement, url, label) {
  const requestId = ++diagnosticPreviewRequest;
  clearDiagnosticPreview(image);
  statusElement.dataset.status = "pending";
  statusElement.textContent = `Loading ${label.toLowerCase()}...`;
  try {
    const response = await fetch(url);
    if (!response.ok) {
      let message = `HTTP ${response.status}`;
      try {
        const payload = await response.json();
        message = payload.message || message;
      } catch (_error) {
        // Keep the HTTP status when the failed response is not JSON.
      }
      throw new Error(message);
    }
    const blob = await response.blob();
    if (requestId !== diagnosticPreviewRequest) {
      return;
    }
    const objectUrl = URL.createObjectURL(blob);
    image.src = objectUrl;
    image.dataset.previewObjectUrl = objectUrl;
    statusElement.dataset.status = "ready";
    statusElement.textContent = `${label} generated from the current study.`;
  } catch (error) {
    if (requestId !== diagnosticPreviewRequest) {
      return;
    }
    statusElement.dataset.status = "fail";
    statusElement.textContent = `${label} unavailable: ${error.message}. Resolve the current engineering checks to regenerate it.`;
  }
}

function blockDiagnosticPreview(image, statusElement, label) {
  if (!state.currentPayload || state.currentValidationPass) {
    return false;
  }
  clearDiagnosticPreview(image);
  statusElement.dataset.status = "blocked";
  statusElement.textContent = `${label} unavailable until the current engineering checks pass.`;
  return true;
}

function refreshSteeringCurvesPreview(betaDeg = Number(betaSlider.value)) {
  if (blockDiagnosticPreview(steeringCurvesImage, steeringCurvesStatus, "Steering curve preview")) {
    return;
  }
  const mode = optimizeMode.value;
  const stepDeg = readCurveStep();
  const linkageQuery = state.linkageConfig
    ? `&linkage=${encodeURIComponent(JSON.stringify(serializedLinkageConfig()))}`
    : "";
  const query = `beta_deg=${encodeURIComponent(betaDeg)}&mode=${encodeURIComponent(mode)}&${geometryQuery()}&step_deg=${encodeURIComponent(stepDeg)}&beta_min_deg=${encodeURIComponent(state.betaRange.minDeg)}&beta_max_deg=${encodeURIComponent(state.betaRange.maxDeg)}${linkageQuery}${vehicleConfigQuery()}`;
  void loadDiagnosticPreview(
    steeringCurvesImage,
    steeringCurvesStatus,
    `/api/steering-curves.svg?${query}`,
    "Steering curve preview",
  );
}

function geometryQuery() {
  return `wheelbase_mm=${encodeURIComponent(state.geometry.wheelbaseMm)}&track_mm=${encodeURIComponent(state.geometry.trackMm)}`;
}

function readCurveStep() {
  const value = Number(curveStepInput.value);
  return Number.isFinite(value) && value > 0 ? value : 1;
}

function setBetaRange(minDeg, maxDeg) {
  state.betaRange = { minDeg, maxDeg };
  betaMinInput.value = String(minDeg);
  betaMaxInput.value = String(maxDeg);
  betaMinLabel.textContent = minDeg.toFixed(0);
  betaMaxLabel.textContent = maxDeg > 0 ? `+${maxDeg.toFixed(0)}` : maxDeg.toFixed(0);
  betaSlider.min = String(minDeg);
  betaSlider.max = String(maxDeg);
  const current = Math.max(minDeg, Math.min(maxDeg, Number(betaSlider.value)));
  betaSlider.value = String(current);
}

function defaultCustomAxle(index, count) {
  const spacing = count <= 1 ? 0 : state.geometry.wheelbaseMm / (count - 1);
  return {
    id: `axle_${index + 1}`,
    x_mm: index * spacing,
    y_mm: 0,
    track_mm: state.geometry.trackMm,
    wheel_count: 2,
    wheel_lateral_offsets_mm: "",
    steering_mode: "FORCED_STEER",
    steerable: true,
    heading_rad: 0,
    maximum_steering_angle_deg: null,
    steering_stop_deg: null,
    load_kg: null,
    tire_width_mm: 0,
    outside_diameter_mm: 0,
    heading_deg: 0,
    user_defined_steering_angle_deg: 0,
    sync_mode: "SAME_PHASE",
    sync_source_axle_id: null,
    sync_ratio: 1,
    sync_phase_offset_deg: 0,
    sync_target_curve: [],
  };
}

function targetCurveText(curve) {
  return (Array.isArray(curve) ? curve : []).map((point) => {
    const betaDeg = point.beta_deg ?? Number(point.beta_rad) * 180 / Math.PI;
    const angleDeg = point.steering_angle_deg ?? Number(point.steering_angle_rad) * 180 / Math.PI;
    return `${Number(betaDeg).toFixed(1)}:${Number(angleDeg).toFixed(1)}`;
  }).join(", ");
}

function wheelOffsetText(offsets) {
  return Array.isArray(offsets) ? offsets.map((offset) => Number(offset).toFixed(1)).join(", ") : "";
}

function parseWheelLateralOffsets(value, wheelCount) {
  const text = String(value || "").trim();
  if (!text) {
    if (wheelCount === 2) return null;
    throw new Error("Multi-wheel axles require one lateral offset per wheel, in mm (+left)." );
  }
  const offsets = text.split(/[,;\s]+/).filter(Boolean).map(Number);
  if (offsets.length !== wheelCount) {
    throw new Error(`Enter exactly ${wheelCount} lateral offsets for this axle.`);
  }
  if (offsets.some((offset) => !Number.isFinite(offset) || Math.abs(offset) < 1e-9)) {
    throw new Error("Wheel lateral offsets must be finite and non-zero; positive values are left.");
  }
  if (new Set(offsets).size !== offsets.length) {
    throw new Error("Wheel lateral offsets must be unique.");
  }
  const leftCount = offsets.filter((offset) => offset > 0).length;
  if (leftCount * 2 !== wheelCount) {
    throw new Error("Wheel lateral offsets must contain the same number of left and right positions.");
  }
  return offsets;
}

function wheelIdsForAxle(axle) {
  const wheelCount = Math.max(2, Math.trunc(Number(axle.wheelCount ?? axle.wheel_count ?? 2)));
  const offsets = parseWheelLateralOffsets(
    axle.wheelLateralOffsetsText ?? axle.wheel_lateral_offsets_mm,
    wheelCount,
  );
  if (wheelCount === 2) {
    return [`${axle.id}_left`, `${axle.id}_right`];
  }
  const left = offsets.filter((offset) => offset > 0).sort((a, b) => b - a);
  const right = offsets.filter((offset) => offset < 0).sort((a, b) => a - b);
  return [
    ...left.map((_offset, index) => `${axle.id}_left_${index + 1}`),
    ...right.map((_offset, index) => `${axle.id}_right_${index + 1}`),
  ];
}

function parseTargetCurve(value) {
  if (!value.trim()) {
    return [];
  }
  const points = value.split(",").map((entry) => {
    const parts = entry.trim().split(":");
    if (parts.length !== 2) {
      throw new Error("Use beta:angle pairs separated by commas.");
    }
    const betaDeg = Number(parts[0].trim());
    const angleDeg = Number(parts[1].trim());
    if (!Number.isFinite(betaDeg) || !Number.isFinite(angleDeg)) {
      throw new Error("Target curve beta and angle values must be numeric.");
    }
    return { beta_deg: betaDeg, steering_angle_deg: angleDeg };
  });
  points.sort((left, right) => left.beta_deg - right.beta_deg);
  if (points.some((point, index) => index > 0 && point.beta_deg === points[index - 1].beta_deg)) {
    throw new Error("Target curve beta values must be unique.");
  }
  return points;
}

function renderCustomAxleConfig() {
  const count = Math.max(1, Math.min(12, Number(customAxleCountInput.value) || 1));
  customAxleCountInput.value = String(count);
  const previousAxles = [...state.customAxles];
  const previous = new Map(previousAxles.map((axle) => [axle.id, axle]));
  state.customAxles = Array.from({ length: count }, (_, index) => {
    const fallback = defaultCustomAxle(index, count);
    return previous.get(fallback.id) || previousAxles[index] || fallback;
  });
  customAxleConfig.replaceChildren();

  for (const [index, axle] of state.customAxles.entries()) {
    const row = document.createElement("div");
    row.className = "custom-axle-row";
    const fields = [
      ["x_mm", "X", false],
      ["y_mm", "Y", false],
      ["track_mm", "Track", false],
      ["wheel_count", "Wheel count", false],
      ["wheel_lateral_offsets_mm", "Wheel offsets mm (+left)", false],
      ["heading_deg", "Heading deg", false],
      ["maximum_steering_angle_deg", "Max steer", true],
      ["steering_stop_deg", "Stop", true],
      ["load_kg", "Load kg", true],
      ["tire_width_mm", "Tire width", false],
      ["outside_diameter_mm", "Tire OD", false],
      ["user_defined_steering_angle_deg", "User deg", false],
    ];
    for (const [key, labelText] of fields) {
      const label = document.createElement("label");
      label.textContent = labelText;
      const input = document.createElement("input");
      input.type = key === "wheel_lateral_offsets_mm" ? "text" : "number";
      input.step = "1";
      if (key === "wheel_count") {
        input.min = "2";
        input.max = "12";
        input.step = "2";
      }
      input.value = axle[key] === null || axle[key] === undefined ? "" : String(axle[key]);
      input.dataset.axleIndex = String(index);
      input.dataset.axleKey = key;
      input.addEventListener("input", (event) => {
        markWorkspaceDirty("Axle inputs changed. Recalculate and save a new revision before review.");
        const target = event.target;
        const rawValue = target.value.trim();
        state.customAxles[index][key] = key === "wheel_lateral_offsets_mm"
          ? rawValue
          : fields.find((item) => item[0] === key)?.[2] && rawValue === ""
            ? null
            : Number(rawValue);
        if (key === "heading_deg") {
          state.customAxles[index].heading_rad = Number(rawValue || 0) * Math.PI / 180;
        }
      });
      label.appendChild(input);
      row.appendChild(label);
    }

    const modeLabel = document.createElement("label");
    modeLabel.textContent = "Mode";
    const mode = document.createElement("select");
    mode.dataset.axleIndex = String(index);
    for (const optionValue of ["FIXED", "FORCED_STEER", "SELF_STEER", "USER_DEFINED"]) {
      const option = document.createElement("option");
      option.value = optionValue;
      option.textContent = optionValue.replaceAll("_", " ");
      option.selected = axle.steering_mode === optionValue;
      mode.appendChild(option);
    }
    mode.addEventListener("change", (event) => {
      markWorkspaceDirty("Axle steering mode changed. Recalculate and save a new revision before review.");
      state.customAxles[index].steering_mode = event.target.value;
      state.customAxles[index].steerable = event.target.value !== "FIXED";
    });
    modeLabel.appendChild(mode);
    row.appendChild(modeLabel);

    const syncModeLabel = document.createElement("label");
    syncModeLabel.textContent = "Sync mode";
    const syncMode = document.createElement("select");
    for (const optionValue of ["SAME_PHASE", "OPPOSITE_PHASE", "RATIO", "LINKED_MECHANICALLY", "INDEPENDENT_TARGET"]) {
      const option = document.createElement("option");
      option.value = optionValue;
      option.textContent = optionValue.replaceAll("_", " ");
      option.selected = (axle.sync_mode || "SAME_PHASE") === optionValue;
      syncMode.appendChild(option);
    }
    syncMode.addEventListener("change", (event) => {
      markWorkspaceDirty("Axle synchronization changed. Recalculate and save a new revision before review.");
      state.customAxles[index].sync_mode = event.target.value;
    });
    syncModeLabel.appendChild(syncMode);
    row.appendChild(syncModeLabel);

    const syncSourceLabel = document.createElement("label");
    syncSourceLabel.textContent = "Sync source";
    const syncSource = document.createElement("select");
    const primaryOption = document.createElement("option");
    primaryOption.value = "";
    primaryOption.textContent = "Primary linkage";
    syncSource.appendChild(primaryOption);
    for (const candidate of state.customAxles) {
      if (candidate.id === axle.id) {
        continue;
      }
      const option = document.createElement("option");
      option.value = candidate.id;
      option.textContent = candidate.id;
      option.selected = candidate.id === axle.sync_source_axle_id;
      syncSource.appendChild(option);
    }
    syncSource.value = axle.sync_source_axle_id || "";
    syncSource.addEventListener("change", (event) => {
      markWorkspaceDirty("Axle synchronization source changed. Recalculate and save a new revision before review.");
      state.customAxles[index].sync_source_axle_id = event.target.value || null;
    });
    syncSourceLabel.appendChild(syncSource);
    row.appendChild(syncSourceLabel);

    const syncRatioLabel = document.createElement("label");
    syncRatioLabel.textContent = "Sync ratio";
    const syncRatio = document.createElement("input");
    syncRatio.type = "number";
    syncRatio.step = "0.01";
    syncRatio.value = String(axle.sync_ratio ?? 1);
    syncRatio.addEventListener("input", (event) => {
      markWorkspaceDirty("Axle synchronization ratio changed. Recalculate and save a new revision before review.");
      state.customAxles[index].sync_ratio = Number(event.target.value);
    });
    syncRatioLabel.appendChild(syncRatio);
    row.appendChild(syncRatioLabel);

    const syncPhaseLabel = document.createElement("label");
    syncPhaseLabel.textContent = "Phase offset";
    const syncPhase = document.createElement("input");
    syncPhase.type = "number";
    syncPhase.step = "0.1";
    syncPhase.value = String(axle.sync_phase_offset_deg ?? 0);
    syncPhase.addEventListener("input", (event) => {
      markWorkspaceDirty("Axle synchronization phase changed. Recalculate and save a new revision before review.");
      state.customAxles[index].sync_phase_offset_deg = Number(event.target.value);
    });
    syncPhaseLabel.appendChild(syncPhase);
    row.appendChild(syncPhaseLabel);

    const targetCurveLabel = document.createElement("label");
    targetCurveLabel.textContent = "Target curve beta:angle deg";
    const targetCurveInput = document.createElement("input");
    targetCurveInput.type = "text";
    targetCurveInput.placeholder = "-45:-10, 0:0, 45:10";
    targetCurveInput.value = targetCurveText(axle.sync_target_curve);
    targetCurveInput.title = "Use comma-separated beta:steering-angle degree pairs.";
    targetCurveInput.addEventListener("change", (event) => {
      markWorkspaceDirty("Steering target curve changed. Recalculate and save a new revision before review.");
      try {
        state.customAxles[index].sync_target_curve = parseTargetCurve(event.target.value);
        state.customAxles[index].sync_target_curve_error = null;
        event.target.setCustomValidity("");
      } catch (error) {
        state.customAxles[index].sync_target_curve_error = error.message;
        event.target.setCustomValidity(error.message);
        customAxleStatus.textContent = `Target curve input failed: ${error.message}`;
      }
    });
    targetCurveLabel.appendChild(targetCurveInput);
    row.appendChild(targetCurveLabel);
    customAxleConfig.appendChild(row);
  }
}

function designCaseTarget(caseValue) {
  if (caseValue.outer_diameter_mm !== null && caseValue.outer_diameter_mm !== undefined) {
    return "outer_diameter_mm";
  }
  if (caseValue.turn_radius_mm !== null && caseValue.turn_radius_mm !== undefined) {
    return "turn_radius_mm";
  }
  return "beta_deg";
}

function defaultDesignCase(index) {
  return {
    id: `case_${index + 1}`,
    name: `Case ${index + 1}`,
    beta_deg: 0,
    turn_radius_mm: null,
    outer_diameter_mm: null,
    direction: "left",
    weight: 1,
    enabled: true,
  };
}

function renderDesignCases() {
  designCaseConfig.replaceChildren();
  for (const [index, caseValue] of state.designCases.entries()) {
    const row = document.createElement("div");
    row.className = "design-case-row";
    const target = designCaseTarget(caseValue);

    const nameLabel = document.createElement("label");
    nameLabel.textContent = "Name";
    const nameInput = document.createElement("input");
    nameInput.type = "text";
    nameInput.value = caseValue.name;
    nameInput.addEventListener("input", (event) => {
      markWorkspaceDirty("Design case changed. Save a new revision before review.");
      state.designCases[index].name = event.target.value;
    });
    nameLabel.appendChild(nameInput);
    row.appendChild(nameLabel);

    const targetLabel = document.createElement("label");
    targetLabel.textContent = "Target";
    const targetSelect = document.createElement("select");
    for (const [value, label] of [["beta_deg", "Beta"], ["turn_radius_mm", "Radius"], ["outer_diameter_mm", "Outer dia"]]) {
      const option = document.createElement("option");
      option.value = value;
      option.textContent = label;
      option.selected = target === value;
      targetSelect.appendChild(option);
    }
    targetSelect.addEventListener("change", (event) => {
      markWorkspaceDirty("Design case target changed. Save a new revision before review.");
      const nextTarget = event.target.value;
      const next = { ...state.designCases[index], beta_deg: null, turn_radius_mm: null, outer_diameter_mm: null };
      next[nextTarget] = nextTarget === "beta_deg" ? 0 : 10000;
      state.designCases[index] = next;
      renderDesignCases();
    });
    targetLabel.appendChild(targetSelect);
    row.appendChild(targetLabel);

    const valueLabel = document.createElement("label");
    valueLabel.textContent = target === "beta_deg" ? "Beta deg" : target === "turn_radius_mm" ? "Radius mm" : "Outer mm";
    const valueInput = document.createElement("input");
    valueInput.type = "number";
    valueInput.step = "0.1";
    valueInput.value = String(caseValue[target]);
    valueInput.addEventListener("input", (event) => {
      markWorkspaceDirty("Design case changed. Save a new revision before review.");
      state.designCases[index][target] = Number(event.target.value);
    });
    valueLabel.appendChild(valueInput);
    row.appendChild(valueLabel);

    const directionLabel = document.createElement("label");
    directionLabel.textContent = "Direction";
    const directionSelect = document.createElement("select");
    for (const value of ["left", "right"]) {
      const option = document.createElement("option");
      option.value = value;
      option.textContent = value;
      option.selected = caseValue.direction === value;
      directionSelect.appendChild(option);
    }
    directionSelect.disabled = target !== "outer_diameter_mm";
    directionSelect.addEventListener("change", (event) => {
      markWorkspaceDirty("Design case direction changed. Save a new revision before review.");
      state.designCases[index].direction = event.target.value;
    });
    directionLabel.appendChild(directionSelect);
    row.appendChild(directionLabel);

    const weightLabel = document.createElement("label");
    weightLabel.textContent = "Weight";
    const weightInput = document.createElement("input");
    weightInput.type = "number";
    weightInput.min = "0";
    weightInput.step = "0.1";
    weightInput.value = String(caseValue.weight);
    weightInput.addEventListener("input", (event) => {
      markWorkspaceDirty("Design case weight changed. Save a new revision before review.");
      state.designCases[index].weight = Number(event.target.value);
    });
    weightLabel.appendChild(weightInput);
    row.appendChild(weightLabel);

    const enabledLabel = document.createElement("label");
    enabledLabel.textContent = "On";
    const enabledInput = document.createElement("input");
    enabledInput.className = "design-case-enabled";
    enabledInput.type = "checkbox";
    enabledInput.checked = caseValue.enabled !== false;
    enabledInput.addEventListener("change", (event) => {
      markWorkspaceDirty("Design case selection changed. Save a new revision before review.");
      state.designCases[index].enabled = event.target.checked;
    });
    enabledLabel.appendChild(enabledInput);
    row.appendChild(enabledLabel);

    const removeButton = document.createElement("button");
    removeButton.className = "design-case-remove";
    removeButton.type = "button";
    removeButton.textContent = "x";
    removeButton.title = "Remove design case";
    removeButton.addEventListener("click", () => {
      markWorkspaceDirty("Design case removed. Save a new revision before review.");
      state.designCases.splice(index, 1);
      renderDesignCases();
    });
    row.appendChild(removeButton);
    designCaseConfig.appendChild(row);
  }
  designCaseStatus.textContent = state.designCases.length === 0
    ? "Optional cases add weighted beta, radius, or outer-diameter conditions to the optimizer."
    : `${state.designCases.filter((item) => item.enabled !== false).length} active weighted case(s).`;
}

function serializedDesignCases() {
  return state.designCases.map((caseValue) => ({
    id: String(caseValue.id),
    name: String(caseValue.name),
    beta_deg: caseValue.beta_deg === null || caseValue.beta_deg === undefined ? null : Number(caseValue.beta_deg),
    turn_radius_mm: caseValue.turn_radius_mm === null || caseValue.turn_radius_mm === undefined ? null : Number(caseValue.turn_radius_mm),
    outer_diameter_mm: caseValue.outer_diameter_mm === null || caseValue.outer_diameter_mm === undefined ? null : Number(caseValue.outer_diameter_mm),
    direction: caseValue.direction === "right" ? "right" : "left",
    weight: Number(caseValue.weight),
    enabled: caseValue.enabled !== false,
  }));
}

function serializedVehicleConfig() {
  syncGeometryMetadataFromInputs();
  const invalidTargetCurve = state.customAxles.find((axle) => axle.sync_target_curve_error);
  if (invalidTargetCurve) {
    throw new Error(`Target curve for ${invalidTargetCurve.id} is invalid: ${invalidTargetCurve.sync_target_curve_error}`);
  }
  const serializedAxles = state.customAxles.map((axle) => {
    const wheelCount = Number(axle.wheel_count || 2);
    return {
      id: String(axle.id),
      x_mm: Number(axle.x_mm),
      y_mm: Number(axle.y_mm),
      track_mm: Number(axle.track_mm),
      wheel_count: wheelCount,
      wheel_lateral_offsets_mm: parseWheelLateralOffsets(axle.wheel_lateral_offsets_mm, wheelCount),
      steerable: axle.steerable !== false,
      steering_mode: axle.steering_mode || "FORCED_STEER",
      heading_rad: Number(axle.heading_rad ?? (Number(axle.heading_deg || 0) * Math.PI / 180)),
      maximum_steering_angle_deg: axle.maximum_steering_angle_deg == null ? null : Number(axle.maximum_steering_angle_deg),
      steering_stop_deg: axle.steering_stop_deg == null ? null : Number(axle.steering_stop_deg),
      load_kg: axle.load_kg == null ? null : Number(axle.load_kg),
      tire_width_mm: Number(axle.tire_width_mm || 0),
      outside_diameter_mm: Number(axle.outside_diameter_mm || 0),
      user_defined_steering_angle_deg: Number(axle.user_defined_steering_angle_deg || 0),
    };
  });
  return {
    id: "custom_vehicle_layout",
    name: "Custom axle layout",
    body_length_mm: Number(state.geometry.bodyLengthMm),
    body_width_mm: Number(state.geometry.bodyWidthMm),
    origin: state.geometry.origin || { x_mm: 0, y_mm: 0 },
    body_polygon: Array.isArray(state.geometry.bodyPolygon) ? state.geometry.bodyPolygon : [],
    front_articulation_point: state.geometry.frontArticulationPoint || null,
    rear_articulation_point: state.geometry.rearArticulationPoint || null,
    kingpin_point: state.geometry.kingpinPoint || null,
    maximum_articulation_deg: Number(state.geometry.maximumArticulationDeg ?? 45),
    cad_source: state.vehicleConfig?.cad_source || null,
    axles: serializedAxles,
    steering_synchronizations: state.customAxles.map((axle) => ({
      id: `sync_${String(axle.id)}`,
      target_axle_id: String(axle.id),
      source_axle_id: axle.sync_source_axle_id || null,
      mode: axle.sync_mode || "SAME_PHASE",
      ratio: Number(axle.sync_ratio ?? 1),
      phase_offset_deg: Number(axle.sync_phase_offset_deg ?? 0),
      target_curve: Array.isArray(axle.sync_target_curve) ? axle.sync_target_curve : [],
    })),
  };
}

function storedVehicleConfig(raw) {
  if (!raw || typeof raw !== "object" || !Array.isArray(raw.axles) || raw.axles.length === 0) {
    return null;
  }
  const synchronizations = new Map(
    (Array.isArray(raw.steering_synchronizations)
      ? raw.steering_synchronizations
      : (Array.isArray(raw.steering_sync) ? raw.steering_sync : []))
      .map((item) => [item.target_axle_id, item]),
  );
  state.customAxles = raw.axles.map((axle, index) => {
    const synchronization = synchronizations.get(axle.id) || {};
    return {
    id: String(axle.id || `axle_${index + 1}`),
    x_mm: Number(axle.x_mm ?? axle.center?.x_mm ?? 0),
    y_mm: Number(axle.y_mm ?? axle.center?.y_mm ?? 0),
    track_mm: Number(axle.track_mm),
    wheel_count: Number(axle.wheel_count || 2),
    wheel_lateral_offsets_mm: wheelOffsetText(axle.wheel_lateral_offsets_mm),
    steerable: axle.steerable !== false,
    steering_mode: axle.steering_mode || (axle.steerable === false ? "FIXED" : "FORCED_STEER"),
    heading_rad: Number(axle.heading_rad || 0),
    heading_deg: Number(axle.heading_deg ?? (Number(axle.heading_rad || 0) * 180 / Math.PI)),
    maximum_steering_angle_deg: axle.maximum_steering_angle_deg == null ? null : Number(axle.maximum_steering_angle_deg),
    steering_stop_deg: axle.steering_stop_deg == null ? null : Number(axle.steering_stop_deg),
    load_kg: axle.load_kg == null ? null : Number(axle.load_kg),
    tire_width_mm: Number(axle.tire_width_mm || 0),
    outside_diameter_mm: Number(axle.outside_diameter_mm || 0),
    user_defined_steering_angle_deg: Number(axle.user_defined_steering_angle_deg || 0),
    sync_mode: synchronization.mode || "SAME_PHASE",
    sync_source_axle_id: synchronization.source_axle_id || null,
    sync_ratio: Number(synchronization.ratio ?? 1),
    sync_phase_offset_deg: Number(synchronization.phase_offset_deg ?? 0),
    sync_target_curve: Array.isArray(synchronization.target_curve) ? synchronization.target_curve : [],
  };
  });
  customAxleCountInput.value = String(state.customAxles.length);
  const normalized = serializedVehicleConfig();
  normalized.id = raw.id || normalized.id;
  normalized.name = raw.name || normalized.name;
  normalized.cad_source = raw.cad_source && typeof raw.cad_source === "object"
    ? { ...raw.cad_source }
    : null;
  if (Number.isFinite(Number(raw.body_length_mm)) && Number(raw.body_length_mm) > 0) {
    normalized.body_length_mm = Number(raw.body_length_mm);
    state.geometry.bodyLengthMm = normalized.body_length_mm;
  }
  if (Number.isFinite(Number(raw.body_width_mm)) && Number(raw.body_width_mm) > 0) {
    normalized.body_width_mm = Number(raw.body_width_mm);
    state.geometry.bodyWidthMm = normalized.body_width_mm;
  }
  for (const [rawKey, stateKey, fallback] of [
    ["origin", "origin", { x_mm: 0, y_mm: 0 }],
    ["front_articulation_point", "frontArticulationPoint", null],
    ["rear_articulation_point", "rearArticulationPoint", null],
    ["kingpin_point", "kingpinPoint", null],
  ]) {
    const rawPoint = raw[rawKey];
    const point = rawPoint && typeof rawPoint === "object"
      ? { x_mm: Number(rawPoint.x_mm), y_mm: Number(rawPoint.y_mm) }
      : fallback;
    if (point === null || (Number.isFinite(point.x_mm) && Number.isFinite(point.y_mm))) {
      normalized[rawKey] = point;
      state.geometry[stateKey] = point;
    }
  }
  normalized.body_polygon = Array.isArray(raw.body_polygon) ? raw.body_polygon : [];
  state.geometry.bodyPolygon = normalized.body_polygon;
  normalized.maximum_articulation_deg = Number.isFinite(Number(raw.maximum_articulation_deg))
    && Number(raw.maximum_articulation_deg) >= 0
    ? Number(raw.maximum_articulation_deg)
    : 45;
  state.geometry.maximumArticulationDeg = normalized.maximum_articulation_deg;
  syncGeometryMetadataInputs();
  return normalized;
}

function vehicleConfigQuery() {
  return state.vehicleConfig
    ? `&vehicle_config=${encodeURIComponent(JSON.stringify(state.vehicleConfig))}`
    : "";
}

function makeCombinationField(labelText, value, onInput, options = {}) {
  const label = document.createElement("label");
  label.textContent = labelText;
  const input = document.createElement("input");
  input.type = options.type || "number";
  input.value = String(value);
  if (options.step !== undefined) input.step = String(options.step);
  if (options.min !== undefined) input.min = String(options.min);
  input.addEventListener("input", () => {
    const nextValue = input.type === "number" ? Number(input.value) : input.value;
    onInput(nextValue);
    markWorkspaceDirty("Combination geometry changed. Rebuild the mechanism and save a new revision before review.", {
      invalidateMechanism: true,
    });
  });
  label.appendChild(input);
  return label;
}

function bodyPolygonText(points) {
  return (Array.isArray(points) ? points : [])
    .map((point) => `${Number(point.x_mm).toFixed(1)}, ${Number(point.y_mm).toFixed(1)}`)
    .join("\n");
}

function parseBodyPolygonText(value) {
  const text = String(value || "").trim();
  if (!text) {
    return [];
  }
  const lines = text.split(/\r?\n/).map((line) => line.trim()).filter(Boolean);
  if (lines.length < 3) {
    throw new Error("CAD outlines need at least three perimeter points.");
  }
  return lines.map((line, index) => {
    const values = line.split(/[,;\s]+/).filter(Boolean);
    if (values.length !== 2) {
      throw new Error(`CAD outline point ${index + 1} must be written as x, y.`);
    }
    const xMm = Number(values[0]);
    const yMm = Number(values[1]);
    if (!Number.isFinite(xMm) || !Number.isFinite(yMm)) {
      throw new Error(`CAD outline point ${index + 1} must contain finite millimetres.`);
    }
    return { x_mm: xMm, y_mm: yMm };
  });
}

function makeBodyPolygonEditor(body) {
  const editor = document.createElement("details");
  editor.className = "body-outline-editor";
  const summary = document.createElement("summary");
  summary.textContent = body.bodyPolygon.length >= 3 ? "CAD outline supplied" : "Optional CAD outline";
  const note = document.createElement("p");
  note.className = "curve-note";
  note.textContent = "Leave blank to use the length/width rectangle. Otherwise enter one local x, y point per line in perimeter order; the first and last points are joined automatically.";
  const label = document.createElement("label");
  label.textContent = "Local outline points (mm)";
  const textarea = document.createElement("textarea");
  textarea.rows = 5;
  textarea.spellcheck = false;
  textarea.placeholder = "-3000, -1600\n3000, -1600\n3000, 1600\n-3000, 1600";
  textarea.value = body.bodyPolygonText ?? bodyPolygonText(body.bodyPolygon);
  const status = document.createElement("small");
  status.className = "body-outline-status";
  const clear = document.createElement("button");
  clear.type = "button";
  clear.className = "secondary-button";
  clear.textContent = "Use rectangular envelope";

  const update = () => {
    body.bodyPolygonText = textarea.value;
    try {
      body.bodyPolygon = parseBodyPolygonText(textarea.value);
      body.bodyPolygonError = null;
      textarea.setCustomValidity("");
      summary.textContent = body.bodyPolygon.length >= 3 ? "CAD outline supplied" : "Optional CAD outline";
      status.textContent = body.bodyPolygon.length >= 3
        ? `${body.bodyPolygon.length} perimeter points will replace the rectangular envelope.`
        : "No CAD outline supplied. Length and width define the rectangular envelope.";
    } catch (error) {
      body.bodyPolygon = [];
      body.bodyPolygonError = error.message;
      textarea.setCustomValidity(error.message);
      summary.textContent = "CAD outline needs attention";
      status.textContent = error.message;
    }
    markWorkspaceDirty("Body outline changed. Rebuild the mechanism and save a new revision before review.", {
      invalidateMechanism: true,
    });
  };
  textarea.addEventListener("input", update);
  clear.addEventListener("click", () => {
    textarea.value = "";
    update();
  });
  label.appendChild(textarea);
  editor.append(summary, note, label, status, clear);
  if (body.bodyPolygonError) {
    textarea.setCustomValidity(body.bodyPolygonError);
    status.textContent = body.bodyPolygonError;
  } else {
    status.textContent = body.bodyPolygon.length >= 3
      ? `${body.bodyPolygon.length} perimeter points will replace the rectangular envelope.`
      : "No CAD outline supplied. Length and width define the rectangular envelope.";
  }
  return editor;
}

function newCombinationBody(index) {
  return {
    id: `body_${index + 1}`,
    name: `Body ${index + 1}`,
    lengthMm: 3000,
    widthMm: 3000,
    bodyPolygon: [],
    bodyPolygonText: "",
    bodyPolygonError: null,
    parentBodyId: index === 0 ? null : (state.combinationBodies[index - 1]?.id || `body_${index}`),
    parentJointId: index === 0 ? null : `joint_${index + 1}`,
    articulationDeg: 0,
    articulationMinDeg: -45,
    articulationMaxDeg: 45,
    articulationStepDeg: 5,
    articulationLimitDeg: 45,
    parentAnchorXmm: 1500,
    parentAnchorYmm: 0,
    childAnchorXmm: -1500,
    childAnchorYmm: 0,
    axles: [
      {
        id: `body_${index + 1}_axle_1`,
        xMm: 0,
        yMm: 0,
        trackMm: 2500,
        mode: "FORCED_STEER",
        wheelCount: 2,
        wheelLateralOffsetsText: "",
        maximumSteeringAngleDeg: 45,
        steeringStopDeg: null,
        tireWidthMm: 400,
        outsideDiameterMm: 1000,
      },
    ],
  };
}

function resizeCombinationBodies(count) {
  const normalized = Math.max(1, Math.min(6, Math.trunc(count)));
  while (state.combinationBodies.length < normalized) {
    state.combinationBodies.push(newCombinationBody(state.combinationBodies.length));
  }
  state.combinationBodies.length = normalized;
  const bodyIds = new Set(state.combinationBodies.map((body) => body.id));
  state.combinationBodies.forEach((body, index) => {
    if (index === 0) {
      body.parentBodyId = null;
      return;
    }
    const priorIds = new Set(state.combinationBodies.slice(0, index).map((candidate) => candidate.id));
    if (!body.parentBodyId || !bodyIds.has(body.parentBodyId) || !priorIds.has(body.parentBodyId)) {
      body.parentBodyId = state.combinationBodies[index - 1].id;
    }
  });
  combinationBodyCountInput.value = String(normalized);
  renderCombinationConfig();
}

function resizeCombinationAxles(bodyIndex, count) {
  const body = state.combinationBodies[bodyIndex];
  const normalized = Math.max(1, Math.min(8, Math.trunc(count)));
  while (body.axles.length < normalized) {
    body.axles.push({
      id: `${body.id}_axle_${body.axles.length + 1}`,
      xMm: body.axles.length * -1000,
      yMm: 0,
      trackMm: 2500,
      mode: "FORCED_STEER",
      wheelCount: 2,
      wheelLateralOffsetsText: "",
      maximumSteeringAngleDeg: 45,
      steeringStopDeg: null,
      tireWidthMm: 400,
      outsideDiameterMm: 1000,
    });
  }
  body.axles.length = normalized;
  renderCombinationConfig();
}

function renderCombinationConfig() {
  const active = state.combinationActive;
  if (legacyGeometryCard) {
    legacyGeometryCard.hidden = active;
  }
  if (legacyLinkageCard) {
    legacyLinkageCard.hidden = active;
  }
  if (combinationFields) {
    combinationFields.hidden = !active;
  }
  if (combinationModeState) {
    combinationModeState.dataset.status = active ? "active" : "legacy";
    combinationModeState.textContent = active ? "ACTIVE" : "LEGACY REVISION";
  }
  if (combinationModeNote) {
    combinationModeNote.textContent = active
      ? "Define every rigid body, its parent connection, articulation joint, and mounted axle. A parent may have multiple connected child bodies for branched towing assemblies. Set each physical articulation stop before choosing its sweep range; a range outside the stop is a hard FAIL. Length and width are rectangular safety envelopes unless a CAD-derived outline is supplied. Continue to Maneuver when the physical combination is complete."
      : "This revision uses the legacy single-layout study. Its two-body template is hidden so it cannot be mistaken for saved engineering input. Activate the multi-body workflow to create a new articulated model from the template.";
  }
  if (combinationActivateButton) {
    combinationActivateButton.hidden = active;
  }
  combinationConfig.replaceChildren();
  state.combinationBodies.forEach((body, bodyIndex) => {
    const card = document.createElement("details");
    card.className = "combination-body-row";
    card.open = bodyIndex === 0;
    const header = document.createElement("summary");
    header.className = "combination-body-header";
    const title = document.createElement("strong");
    const parentBody = bodyIndex === 0
      ? null
      : state.combinationBodies.find((candidate) => candidate.id === body.parentBodyId);
    title.textContent = bodyIndex === 0
      ? `${body.name} / root`
      : `${body.name} / connected to ${parentBody?.name || "unassigned parent"}`;
    const summary = document.createElement("small");
    summary.textContent = `${body.axles.length} axle${body.axles.length === 1 ? "" : "s"} / ${body.bodyPolygon.length >= 3 ? "CAD outline" : "rectangular envelope"}${bodyIndex === 0 ? "" : ` / current ${Number(body.articulationDeg).toFixed(1)} deg / sweep ${Number(body.articulationMinDeg).toFixed(0)}..${Number(body.articulationMaxDeg).toFixed(0)} deg / stop +/-${Number(body.articulationLimitDeg).toFixed(0)} deg`}`;
    header.append(title, summary);

    const fields = document.createElement("div");
    fields.className = "combination-field-grid";
    fields.append(
      makeCombinationField("Axles", body.axles.length, (value) => resizeCombinationAxles(bodyIndex, value), { min: 1, step: 1 }),
      makeCombinationField("Body name", body.name, (value) => { body.name = String(value); }, { type: "text" }),
      makeCombinationField("Length mm", body.lengthMm, (value) => { body.lengthMm = value; }, { min: 1, step: 10 }),
      makeCombinationField("Width mm", body.widthMm, (value) => { body.widthMm = value; }, { min: 1, step: 10 }),
    );
    if (bodyIndex > 0) {
      const parentLabel = document.createElement("label");
      parentLabel.textContent = "Parent body";
      const parentSelect = document.createElement("select");
      state.combinationBodies.slice(0, bodyIndex).forEach((candidate) => {
        const option = document.createElement("option");
        option.value = candidate.id;
        option.textContent = `${candidate.name} (${candidate.id})`;
        parentSelect.appendChild(option);
      });
      const selectedParentId = body.parentBodyId || state.combinationBodies[bodyIndex - 1]?.id;
      if (selectedParentId && !state.combinationBodies.slice(0, bodyIndex).some((candidate) => candidate.id === selectedParentId)) {
        const option = document.createElement("option");
        option.value = selectedParentId;
        option.textContent = `Unknown parent (${selectedParentId})`;
        parentSelect.appendChild(option);
      }
      parentSelect.value = selectedParentId || "";
      parentSelect.addEventListener("change", () => {
        body.parentBodyId = parentSelect.value || null;
        markWorkspaceDirty("Body connection changed. Rebuild the mechanism and save a new revision before review.", {
          invalidateMechanism: true,
        });
        renderCombinationConfig();
      });
      parentLabel.appendChild(parentSelect);
      fields.append(
        parentLabel,
        makeCombinationField("Articulation deg", body.articulationDeg, (value) => { body.articulationDeg = value; }, { step: 0.5 }),
        makeCombinationField("Physical limit +/- deg", body.articulationLimitDeg, (value) => { body.articulationLimitDeg = value; }, { min: 0, step: 0.5 }),
        makeCombinationField("Sweep min deg", body.articulationMinDeg, (value) => {
          body.articulationMinDeg = value;
          if (bodyIndex === 1 && Number.isFinite(Number(body.articulationMaxDeg))) {
            setBetaRange(Number(value), Number(body.articulationMaxDeg));
          }
        }, { step: 0.5 }),
        makeCombinationField("Sweep max deg", body.articulationMaxDeg, (value) => {
          body.articulationMaxDeg = value;
          if (bodyIndex === 1 && Number.isFinite(Number(body.articulationMinDeg))) {
            setBetaRange(Number(body.articulationMinDeg), Number(value));
          }
        }, { step: 0.5 }),
        makeCombinationField("Sweep step deg", body.articulationStepDeg, (value) => { body.articulationStepDeg = value; }, { min: 0.1, step: 0.5 }),
        makeCombinationField("Parent anchor X", body.parentAnchorXmm, (value) => { body.parentAnchorXmm = value; }, { step: 10 }),
        makeCombinationField("Parent anchor Y", body.parentAnchorYmm, (value) => { body.parentAnchorYmm = value; }, { step: 10 }),
        makeCombinationField("Child anchor X", body.childAnchorXmm, (value) => { body.childAnchorXmm = value; }, { step: 10 }),
        makeCombinationField("Child anchor Y", body.childAnchorYmm, (value) => { body.childAnchorYmm = value; }, { step: 10 }),
      );
    }
    card.append(header, fields);
    card.appendChild(makeBodyPolygonEditor(body));

    body.axles.forEach((axle, axleIndex) => {
      const heading = document.createElement("p");
      heading.className = "combination-axle-heading";
      heading.textContent = `Axle ${axleIndex + 1}`;
      const axleFields = document.createElement("div");
      axleFields.className = "combination-field-grid";
      const modeLabel = document.createElement("label");
      modeLabel.textContent = "Steering mode";
      const modeSelect = document.createElement("select");
      for (const mode of ["FORCED_STEER", "FIXED", "USER_DEFINED"]) {
        const option = document.createElement("option");
        option.value = mode;
        option.textContent = mode.replaceAll("_", " ");
        modeSelect.appendChild(option);
      }
      modeSelect.value = axle.mode;
      modeSelect.addEventListener("change", () => {
        axle.mode = modeSelect.value;
        markWorkspaceDirty("Combination axle steering mode changed. Rebuild the mechanism and save a new revision before review.", {
          invalidateMechanism: true,
        });
      });
      modeLabel.appendChild(modeSelect);
      axleFields.append(
        makeCombinationField("Local X mm", axle.xMm, (value) => { axle.xMm = value; }, { step: 10 }),
        makeCombinationField("Local Y mm", axle.yMm, (value) => { axle.yMm = value; }, { step: 10 }),
        makeCombinationField("Track mm", axle.trackMm, (value) => { axle.trackMm = value; }, { min: 1, step: 10 }),
        makeCombinationField("Wheel count", axle.wheelCount || 2, (value) => { axle.wheelCount = Math.max(2, Math.trunc(value)); }, { min: 2, step: 2 }),
        makeCombinationField("Wheel offsets mm (+left)", axle.wheelLateralOffsetsText || "", (value) => { axle.wheelLateralOffsetsText = String(value); }, { type: "text" }),
        makeCombinationField("Tire width mm", axle.tireWidthMm || 0, (value) => { axle.tireWidthMm = Math.max(0, value); }, { min: 0, step: 1 }),
        makeCombinationField("Tire OD mm", axle.outsideDiameterMm || 0, (value) => { axle.outsideDiameterMm = Math.max(0, value); }, { min: 0, step: 1 }),
        modeLabel,
      );
      card.append(heading, axleFields);
    });
    combinationConfig.appendChild(card);
  });
  renderCombinationSynchronizationConfig();
}

function combinationAxleOptions() {
  return state.combinationBodies.flatMap((body) => body.axles.map((axle) => ({
    id: axle.id,
    label: `${body.name} / ${axle.id}`,
  })));
}

function renderCombinationSynchronizationConfig() {
  const section = document.createElement("div");
  section.className = "combination-synchronization-section";
  const heading = document.createElement("h4");
  heading.textContent = "Steering coordination";
  const note = document.createElement("p");
  note.className = "curve-note";
  note.textContent = "Define how a target axle follows a source axle. This channel is checked against the ideal steering result.";
  section.append(heading, note);

  const options = combinationAxleOptions();
  state.combinationSynchronizations = state.combinationSynchronizations.filter((sync) =>
    options.some((option) => option.id === sync.targetAxleId)
    && options.some((option) => option.id === sync.sourceAxleId),
  );
  state.combinationSynchronizations.forEach((sync, syncIndex) => {
    const card = document.createElement("div");
    card.className = "combination-synchronization-row";
    const title = document.createElement("strong");
    title.textContent = sync.id || `sync_${syncIndex + 1}`;
    const fields = document.createElement("div");
    fields.className = "combination-field-grid";
    fields.appendChild(makeCombinationField("Channel ID", sync.id, (value) => {
      sync.id = String(value).trim() || `sync_${syncIndex + 1}`;
    }, { type: "text" }));

    const selectField = (labelText, property, allowedOptions) => {
      const label = document.createElement("label");
      label.textContent = labelText;
      const select = document.createElement("select");
      allowedOptions.forEach((option) => {
        const element = document.createElement("option");
        element.value = option.value;
        element.textContent = option.label;
        select.appendChild(element);
      });
      select.value = sync[property];
      select.addEventListener("change", () => {
        sync[property] = select.value;
        markWorkspaceDirty("Steering coordination changed. Rebuild the mechanism and save a new revision before review.", {
          invalidateMechanism: true,
        });
      });
      label.appendChild(select);
      return label;
    };
    fields.append(
      selectField("Target axle", "targetAxleId", options.map((option) => ({ value: option.id, label: option.label }))),
      selectField("Source axle", "sourceAxleId", options.map((option) => ({ value: option.id, label: option.label }))),
      selectField("Mode", "mode", [
        { value: "OPPOSITE_PHASE", label: "Opposite phase" },
        { value: "SAME_PHASE", label: "Same phase" },
        { value: "RATIO", label: "Ratio" },
        { value: "LINKED_MECHANICALLY", label: "Linked mechanically" },
      ]),
      makeCombinationField("Ratio", sync.ratio, (value) => { sync.ratio = value; }, { step: 0.05 }),
      makeCombinationField("Phase offset deg", sync.phaseOffsetDeg, (value) => { sync.phaseOffsetDeg = value; }, { step: 0.5 }),
    );
    const remove = document.createElement("button");
    remove.type = "button";
    remove.className = "secondary-button";
    remove.textContent = "Remove channel";
    remove.addEventListener("click", () => {
      state.combinationSynchronizations.splice(syncIndex, 1);
      markWorkspaceDirty("Steering coordination changed. Rebuild the mechanism and save a new revision before review.", {
        invalidateMechanism: true,
      });
      renderCombinationConfig();
    });
    card.append(title, fields, remove);
    section.appendChild(card);
  });

  const add = document.createElement("button");
  add.type = "button";
  add.className = "secondary-button";
  add.textContent = "Add synchronization channel";
  add.disabled = options.length < 2;
  add.addEventListener("click", () => {
    const source = options[options.length - 1];
    const target = options.find((option) => option.id !== source.id) || source;
    state.combinationSynchronizations.push({
      id: `sync_${state.combinationSynchronizations.length + 1}`,
      targetAxleId: target.id,
      sourceAxleId: source.id,
      mode: "OPPOSITE_PHASE",
      ratio: 1,
      phaseOffsetDeg: 0,
    });
    markWorkspaceDirty("Steering coordination changed. Rebuild the mechanism and save a new revision before review.", {
      invalidateMechanism: true,
    });
    renderCombinationConfig();
  });
  section.appendChild(add);
  combinationConfig.appendChild(section);
}

function serializedCombination() {
  const invalidOutline = state.combinationBodies.find((body) => body.bodyPolygonError);
  if (invalidOutline) {
    throw new Error(`${invalidOutline.name} has an invalid CAD outline: ${invalidOutline.bodyPolygonError}`);
  }
  if (!state.combinationBodies.length) {
    throw new Error("A towing combination requires at least one body.");
  }
  const bodyIds = new Set(state.combinationBodies.map((body) => body.id));
  const parentBodyById = new Map();
  state.combinationBodies.forEach((body, index) => {
    if (index === 0) {
      parentBodyById.set(body.id, null);
      return;
    }
    const parentBodyId = body.parentBodyId || state.combinationBodies[index - 1]?.id;
    const priorIds = new Set(state.combinationBodies.slice(0, index).map((candidate) => candidate.id));
    if (!parentBodyId || !bodyIds.has(parentBodyId) || !priorIds.has(parentBodyId) || parentBodyId === body.id) {
      throw new Error(`${body.name} must connect to a valid earlier parent body.`);
    }
    parentBodyById.set(body.id, parentBodyId);
  });
  const jointIdByBodyId = new Map(
    state.combinationBodies
      .filter((body, index) => index > 0)
      .map((body, index) => [body.id, body.parentJointId || `joint_${index + 2}`]),
  );
  const bodies = state.combinationBodies.map((body, index) => ({
    id: body.id,
    name: body.name,
    pose: index === 0 ? { x_mm: 0, y_mm: 0, yaw_deg: 0 } : undefined,
    body_length_mm: Number(body.lengthMm),
    body_width_mm: Number(body.widthMm),
    body_polygon: Array.isArray(body.bodyPolygon) ? body.bodyPolygon : [],
    parent_joint_id: index === 0 ? null : jointIdByBodyId.get(body.id),
    child_joint_ids: state.combinationBodies
      .filter((candidate) => parentBodyById.get(candidate.id) === body.id)
      .map((candidate) => jointIdByBodyId.get(candidate.id)),
  }));
  const joints = state.combinationBodies.slice(1).map((body, index) => ({
    id: jointIdByBodyId.get(body.id),
    parent_body_id: parentBodyById.get(body.id),
    child_body_id: body.id,
    parent_anchor: { x_mm: Number(body.parentAnchorXmm), y_mm: Number(body.parentAnchorYmm) },
    child_anchor: { x_mm: Number(body.childAnchorXmm), y_mm: Number(body.childAnchorYmm) },
    articulation_deg: Number(body.articulationDeg),
    maximum_articulation_deg: Number(body.articulationLimitDeg),
  }));
  const jointRanges = Object.fromEntries(
    state.combinationBodies.slice(1).map((body, index) => {
      const jointId = jointIdByBodyId.get(body.id);
      return [jointId, {
        min_deg: Number(body.articulationMinDeg),
        max_deg: Number(body.articulationMaxDeg),
        step_deg: Number(body.articulationStepDeg),
      }];
    }),
  );
  const mountedAxles = state.combinationBodies.flatMap((body) => body.axles.map((axle) => ({
    body_id: body.id,
    local_center: { x_mm: Number(axle.xMm), y_mm: Number(axle.yMm) },
      axle: {
      id: axle.id,
      track_mm: Number(axle.trackMm),
      steering_mode: axle.mode,
      steerable: axle.mode !== "FIXED",
      wheel_count: Number(axle.wheelCount || 2),
      wheel_lateral_offsets_mm: parseWheelLateralOffsets(
        axle.wheelLateralOffsetsText,
        Number(axle.wheelCount || 2),
      ),
      maximum_steering_angle_deg: axle.maximumSteeringAngleDeg == null ? null : Number(axle.maximumSteeringAngleDeg),
      steering_stop_deg: axle.steeringStopDeg == null ? null : Number(axle.steeringStopDeg),
      outside_diameter_mm: Number(axle.outsideDiameterMm || 0),
      tire_width_mm: Number(axle.tireWidthMm || 0),
    },
  })));
  const axleIds = new Set(mountedAxles.map((mounted) => mounted.axle.id));
  return {
    id: state.combinationId,
    name: state.combinationName,
    root_body_id: state.combinationBodies[0].id,
    bodies,
    joints,
    joint_ranges: jointRanges,
    mounted_axles: mountedAxles,
    steering_synchronizations: state.combinationSynchronizations
      .filter((sync) => axleIds.has(sync.targetAxleId) && axleIds.has(sync.sourceAxleId))
      .map((sync, index) => ({
        id: String(sync.id || `sync_${index + 1}`),
        target_axle_id: sync.targetAxleId,
        source_axle_id: sync.sourceAxleId,
        mode: sync.mode,
        ratio: Number(sync.ratio),
        phase_offset_deg: Number(sync.phaseOffsetDeg),
      })),
  };
}

function storedJointRange(config, jointId) {
  const rawRanges = config?.joint_ranges;
  if (Array.isArray(rawRanges)) {
    return rawRanges.find((item) => String(item?.joint_id || item?.id || "") === String(jointId)) || null;
  }
  return rawRanges && typeof rawRanges === "object" ? rawRanges[jointId] || null : null;
}

function restoreCombinationConfiguration(config) {
  const rawBodies = Array.isArray(config?.bodies) ? config.bodies : [];
  const rawJoints = Array.isArray(config?.joints) ? config.joints : [];
  const rawMountedAxles = Array.isArray(config?.mounted_axles) ? config.mounted_axles : [];
  if (rawBodies.length === 0 || !config?.root_body_id) {
    throw new Error("Stored combination has no root body.");
  }

  const bodyIds = rawBodies.map((body) => String(body?.id || ""));
  if (bodyIds.some((bodyId) => !bodyId) || new Set(bodyIds).size !== bodyIds.length) {
    throw new Error("Stored combination body IDs must be present and unique.");
  }
  const bodyById = new Map(rawBodies.map((body, index) => [bodyIds[index], body]));
  const bodyOrder = new Map(bodyIds.map((bodyId, index) => [bodyId, index]));
  const parentJointByChild = new Map();
  const childJointsByParent = new Map();
  rawJoints.forEach((joint) => {
    const parentId = String(joint?.parent_body_id || "");
    const childId = String(joint?.child_body_id || "");
    if (!bodyById.has(parentId) || !bodyById.has(childId)) {
      throw new Error(`Stored joint references an unknown body: ${parentId} -> ${childId}.`);
    }
    if (parentId === childId || parentJointByChild.has(childId)) {
      throw new Error(`Stored combination has an invalid or duplicate parent for ${childId}.`);
    }
    parentJointByChild.set(childId, joint);
    const children = childJointsByParent.get(parentId) || [];
    children.push(joint);
    childJointsByParent.set(parentId, children);
  });
  if (!bodyById.has(String(config.root_body_id))) {
    throw new Error(`Stored combination references missing root body ${config.root_body_id}.`);
  }
  if (parentJointByChild.has(String(config.root_body_id))) {
    throw new Error("Stored combination root body cannot have a parent joint.");
  }
  const orderedBodies = [];
  const visited = new Set();
  const visiting = new Set();
  const visitBody = (bodyId) => {
    if (visiting.has(bodyId)) {
      throw new Error("Stored combination contains a body connection cycle.");
    }
    if (visited.has(bodyId)) {
      return;
    }
    const body = bodyById.get(bodyId);
    if (!body) {
      throw new Error(`Stored combination references missing body ${bodyId}.`);
    }
    visiting.add(bodyId);
    visited.add(bodyId);
    orderedBodies.push(body);
    const children = (childJointsByParent.get(bodyId) || [])
      .slice()
      .sort((left, right) => bodyOrder.get(String(left.child_body_id)) - bodyOrder.get(String(right.child_body_id)));
    children.forEach((joint) => visitBody(String(joint.child_body_id)));
    visiting.delete(bodyId);
  };
  visitBody(String(config.root_body_id));
  if (orderedBodies.length !== rawBodies.length) {
    throw new Error("Stored combination contains a disconnected body or missing parent joint.");
  }

  state.combinationId = String(config.id || "workspace_combination");
  state.combinationName = String(config.name || "Workspace vehicle combination");
  const rawSynchronizations = Array.isArray(config.steering_synchronizations)
    ? config.steering_synchronizations
    : [];
  state.combinationSynchronizations = rawSynchronizations.map((sync, index) => ({
    id: String(sync.id || `sync_${index + 1}`),
    targetAxleId: String(sync.target_axle_id || sync.axle_id || ""),
    sourceAxleId: String(sync.source_axle_id || ""),
    mode: String(sync.mode || "SAME_PHASE"),
    ratio: Number(sync.ratio ?? 1),
    phaseOffsetDeg: Number(sync.phase_offset_deg ?? ((Number(sync.phase_offset_rad) || 0) * 180 / Math.PI)),
  }));
  state.combinationBodies = orderedBodies.map((body, index) => {
    const bodyId = String(body.id);
    const parentJoint = parentJointByChild.get(bodyId);
    const jointRange = parentJoint ? storedJointRange(config, parentJoint.id) : null;
    const axles = rawMountedAxles
      .filter((mounted) => String(mounted.body_id || "") === bodyId)
      .map((mounted, axleIndex) => {
        const axle = mounted.axle || mounted;
        const center = mounted.local_center || {};
        return {
          id: String(axle.id || axle.axle_id || `${bodyId}_axle_${axleIndex + 1}`),
          xMm: Number(center.x_mm || 0),
          yMm: Number(center.y_mm || 0),
          trackMm: Number(axle.track_mm || 2500),
          mode: String(axle.steering_mode || (axle.steerable === false ? "FIXED" : "FORCED_STEER")),
          wheelCount: Number(axle.wheel_count || 2),
          wheelLateralOffsetsText: wheelOffsetText(axle.wheel_lateral_offsets_mm),
          maximumSteeringAngleDeg: axle.maximum_steering_angle_deg == null ? null : Number(axle.maximum_steering_angle_deg),
          steeringStopDeg: axle.steering_stop_deg == null ? null : Number(axle.steering_stop_deg),
          tireWidthMm: Number(axle.tire_width_mm ?? 0),
          outsideDiameterMm: Number(axle.outside_diameter_mm ?? 0),
        };
      });
    return {
      id: bodyId,
      name: String(body.name || `Body ${index + 1}`),
      lengthMm: Number(body.body_length_mm || 1800),
      widthMm: Number(body.body_width_mm || 3200),
      bodyPolygon: Array.isArray(body.body_polygon) ? body.body_polygon : [],
      bodyPolygonText: bodyPolygonText(body.body_polygon),
      bodyPolygonError: null,
      parentBodyId: index === 0 ? null : String(parentJoint?.parent_body_id || ""),
      parentJointId: index === 0 ? null : String(parentJoint?.id || `joint_${index + 1}`),
      articulationDeg: Number(parentJoint?.articulation_deg || 0),
      articulationLimitDeg: Number(parentJoint?.maximum_articulation_deg ?? 45),
      parentAnchorXmm: Number(parentJoint?.parent_anchor?.x_mm || 0),
      parentAnchorYmm: Number(parentJoint?.parent_anchor?.y_mm || 0),
      childAnchorXmm: Number(parentJoint?.child_anchor?.x_mm || 0),
      childAnchorYmm: Number(parentJoint?.child_anchor?.y_mm || 0),
      articulationMinDeg: Number(jointRange?.min_deg ?? -45),
      articulationMaxDeg: Number(jointRange?.max_deg ?? 45),
      articulationStepDeg: Number(jointRange?.step_deg ?? 5),
      axles: axles.length > 0
        ? axles
        : [{ id: `${bodyId}_axle_1`, xMm: 0, yMm: 0, trackMm: 2500, mode: "FIXED", wheelCount: 2, maximumSteeringAngleDeg: null, steeringStopDeg: null, tireWidthMm: 0, outsideDiameterMm: 0 }],
    };
  });
  if (state.combinationSynchronizations.length === 0
      && state.combinationBodies.some((body) => body.id === "rear_body")
      && state.combinationBodies.some((body) => body.id === "front_body")) {
    const rearAxle = state.combinationBodies.find((body) => body.id === "rear_body")?.axles[0];
    const frontAxle = state.combinationBodies.find((body) => body.id === "front_body")?.axles[0];
    if (rearAxle && frontAxle) {
      state.combinationSynchronizations = [{
        id: "rear_to_front_sync",
        targetAxleId: rearAxle.id,
        sourceAxleId: frontAxle.id,
        mode: "OPPOSITE_PHASE",
        ratio: 1,
        phaseOffsetDeg: 0,
      }];
    }
  }
  const primaryJointRange = primaryCombinationBody();
  if (primaryJointRange
      && Number.isFinite(Number(primaryJointRange.articulationMinDeg))
      && Number.isFinite(Number(primaryJointRange.articulationMaxDeg))
      && Number(primaryJointRange.articulationMinDeg) < Number(primaryJointRange.articulationMaxDeg)) {
    setBetaRange(
      Number(primaryJointRange.articulationMinDeg),
      Number(primaryJointRange.articulationMaxDeg),
    );
  }
  combinationBodyCountInput.value = String(state.combinationBodies.length);
  renderCombinationConfig();
}

function primaryCombinationBody() {
  return state.combinationBodies.find((body, index) => index > 0 && body.parentJointId)
    || state.combinationBodies[1]
    || null;
}

function primaryCombinationJointId() {
  return primaryCombinationBody()?.parentJointId || "joint_2";
}

function projectCombinationPayload() {
  if (!state.combinationActive) {
    return {};
  }
  return {
    combination_config: serializedCombination(),
    root_turn_radius_mm: Number(combinationTurnRadiusInput.value),
    mechanism_graph_config: state.mechanismGraph,
    mechanism_drivers: state.mechanismDrivers,
    steering_assignments: state.steeringAssignments,
    beta_min_deg: state.betaRange.minDeg,
    beta_max_deg: state.betaRange.maxDeg,
    primary_joint_id: state.combinationBodies.length > 1
      ? primaryCombinationJointId()
      : undefined,
    sweep_step_deg: Number(sweepValidationStepInput.value),
    clearance_target_mm: Number(state.optimizationSettings.clearanceTargetMm),
  };
}

function graphPoint(id, xMm, yMm, mode, bodyId, envelopeRadiusMm = 0) {
  return {
    id,
    mode,
    body_id: bodyId,
    neutral_position: { x_mm: xMm, y_mm: yMm },
    envelope_radius_mm: envelopeRadiusMm,
  };
}

function buildMechanismGraphFromCombination() {
  const config = state.linkageConfig || REFERENCE_LINKAGE_CONFIG;
  if (
    config.companion_enabled === false
    || config.companion_steering_arm_length_mm === null
    || config.companion_tie_rod_length_mm === null
  ) {
    throw new Error("The graph builder requires explicit left and right steering-arm geometry.");
  }
  const points = [];
  const members = [];
  const angleOutputs = [];
  const drivers = [];
  const assignments = [];
  const primaryJointId = primaryCombinationJointId();
  const pointAt = (pivotX, pivotY, angleDeg, lengthMm) => ({
    x: pivotX + Math.cos(angleDeg * Math.PI / 180) * lengthMm,
    y: pivotY + Math.sin(angleDeg * Math.PI / 180) * lengthMm,
  });

  state.combinationBodies.forEach((body, bodyIndex) => {
    body.axles.forEach((axle, axleIndex) => {
      if (axle.mode === "FIXED") {
        return;
      }
      const prefix = `${body.id}_${axle.id}_mechanism`;
      const shiftedX = (value) => Number(axle.xMm) + Number(value);
      const shiftedY = (value) => Number(axle.yMm) + Number(value);
      const bellX = shiftedX(config.bell_crank_pivot_x_mm);
      const bellY = shiftedY(config.bell_crank_pivot_y_mm);
      const steeringX = shiftedX(config.steering_pivot_x_mm);
      const steeringY = shiftedY(config.steering_pivot_y_mm);
      const companionX = shiftedX(config.companion_steering_pivot_x_mm);
      const companionY = shiftedY(config.companion_steering_pivot_y_mm);
      const inputEndpoint = pointAt(
        bellX,
        bellY,
        Number(config.bell_crank_input_neutral_angle_deg),
        Number(config.bell_crank_input_arm_length_mm),
      );
      const outputEndpoint = pointAt(
        bellX,
        bellY,
        Number(config.bell_crank_output_neutral_angle_deg),
        Number(config.bell_crank_output_arm_length_mm),
      );
      const steeringEndpoint = pointAt(
        steeringX,
        steeringY,
        Number(config.steering_arm_neutral_angle_deg),
        Number(config.steering_arm_length_mm),
      );
      const companionEndpoint = pointAt(
        companionX,
        companionY,
        Number(config.companion_steering_arm_neutral_angle_deg),
        Number(config.companion_steering_arm_length_mm),
      );
      const driverCenter = {
        x: shiftedX(config.driver_arc_center_x_mm),
        y: shiftedY(config.driver_arc_center_y_mm),
      };
      const driverNeutral = {
        x: driverCenter.x + Number(config.driver_arc_radius_mm),
        y: driverCenter.y,
      };
      const ids = {
        driver: `${prefix}_driver`,
        bellPivot: `${prefix}_bell_pivot`,
        bellInput: `${prefix}_bell_input`,
        bellOutput: `${prefix}_bell_output`,
        steeringPivot: `${prefix}_left_pivot`,
        steeringEndpoint: `${prefix}_left_endpoint`,
        companionPivot: `${prefix}_right_pivot`,
        companionEndpoint: `${prefix}_right_endpoint`,
        leftOutput: `${prefix}_left_output`,
        rightOutput: `${prefix}_right_output`,
      };
      points.push(
        graphPoint(ids.driver, driverNeutral.x, driverNeutral.y, "driven", body.id),
        graphPoint(ids.bellPivot, bellX, bellY, "fixed", body.id, 28),
        graphPoint(ids.bellInput, inputEndpoint.x, inputEndpoint.y, "free", body.id),
        graphPoint(ids.bellOutput, outputEndpoint.x, outputEndpoint.y, "free", body.id),
        graphPoint(ids.steeringPivot, steeringX, steeringY, "fixed", body.id, 28),
        graphPoint(ids.steeringEndpoint, steeringEndpoint.x, steeringEndpoint.y, "free", body.id),
        graphPoint(ids.companionPivot, companionX, companionY, "fixed", body.id, 28),
        graphPoint(ids.companionEndpoint, companionEndpoint.x, companionEndpoint.y, "free", body.id),
      );
      const member = (id, pointAId, pointBId, lengthMm, kind = "rod", radiusMm = 14, assemblyId = null) => ({
        id: `${prefix}_${id}`,
        point_a_id: pointAId,
        point_b_id: pointBId,
        length_mm: Number(lengthMm),
        kind,
        envelope_radius_mm: radiusMm,
        assembly_id: assemblyId,
      });
      members.push(
        member("input_rod", ids.driver, ids.bellInput, config.input_rod_length_mm),
        member("bell_input_arm", ids.bellPivot, ids.bellInput, config.bell_crank_input_arm_length_mm, "arm", 14, prefix),
        member("bell_output_arm", ids.bellPivot, ids.bellOutput, config.bell_crank_output_arm_length_mm, "arm", 14, prefix),
        member(
          "bell_brace",
          ids.bellInput,
          ids.bellOutput,
          Math.hypot(outputEndpoint.x - inputEndpoint.x, outputEndpoint.y - inputEndpoint.y),
          "rigid_brace",
          0,
          prefix,
        ),
        member("tie_rod", ids.bellOutput, ids.steeringEndpoint, config.tie_rod_length_mm),
        member("left_arm", ids.steeringPivot, ids.steeringEndpoint, config.steering_arm_length_mm, "arm"),
        member("cross_tie_rod", ids.steeringEndpoint, ids.companionEndpoint, config.companion_tie_rod_length_mm),
        member("right_arm", ids.companionPivot, ids.companionEndpoint, config.companion_steering_arm_length_mm, "arm"),
      );
      const stop = config.steering_stop_deg;
      angleOutputs.push(
        {
          id: ids.leftOutput,
          pivot_point_id: ids.steeringPivot,
          endpoint_point_id: ids.steeringEndpoint,
          neutral_angle_deg: Number(config.steering_arm_neutral_angle_deg),
          minimum_angle_deg: stop === null ? undefined : -Number(stop),
          maximum_angle_deg: stop === null ? undefined : Number(stop),
        },
        {
          id: ids.rightOutput,
          pivot_point_id: ids.companionPivot,
          endpoint_point_id: ids.companionEndpoint,
          neutral_angle_deg: Number(config.companion_steering_arm_neutral_angle_deg),
          minimum_angle_deg: stop === null ? undefined : -Number(stop),
          maximum_angle_deg: stop === null ? undefined : Number(stop),
        },
      );
      const inputId = bodyIndex === 0
        ? (state.combinationBodies.length > 1 ? primaryJointId : "articulation")
        : (body.parentJointId || `joint_${bodyIndex + 1}`);
      drivers.push({
        point_id: ids.driver,
        center: { x_mm: driverCenter.x, y_mm: driverCenter.y },
        radius_mm: Number(config.driver_arc_radius_mm),
        neutral_angle_deg: 0,
        input_ratio: bodyIndex === 0 ? -1 : 1,
        input_id: inputId,
      });
      for (const wheelId of wheelIdsForAxle(axle)) {
        assignments.push({
          output_id: wheelId.includes("_left") ? ids.leftOutput : ids.rightOutput,
          wheel_id: wheelId,
        });
      }
    });
  });

  state.mechanismGraph = {
    id: "workspace_mechanism_graph",
    points,
    members,
    angle_outputs: angleOutputs,
  };
  state.mechanismDrivers = drivers;
  state.steeringAssignments = assignments;
  state.workspaceDirty = true;
  resetEngineeringEvidence(
    "Graph built. Solve the design before validating the range.",
    { preserveManeuver: true },
  );
  renderMechanismGraphConfiguration(
    `Graph built with ${drivers.length} articulation drivers and explicit mappings for ${assignments.length} wheels.`,
  );
  // Keep the generated graph summary-first; open the low-level editor only when
  // the engineer explicitly chooses to inspect or change the topology.
  mechanismGraphEditor.open = false;
}

function angleDegreesFromConfig(item, name) {
  if (item[`${name}_deg`] !== undefined && item[`${name}_deg`] !== null) {
    return Number(item[`${name}_deg`]);
  }
  if (item[`${name}_rad`] !== undefined && item[`${name}_rad`] !== null) {
    return Number(item[`${name}_rad`]) * 180 / Math.PI;
  }
  return "";
}

function syncMechanismGraphEditorDraft() {
  if (!state.mechanismGraph) {
    state.mechanismGraphEditorDraft = null;
    return;
  }
  const graph = state.mechanismGraph;
  state.mechanismGraphEditorDraft = {
    graphId: String(graph.id || "workspace_mechanism_graph"),
    points: (graph.points || []).map((point) => ({
      id: String(point.id || ""),
      mode: String(point.mode || "free"),
      body_id: String(point.body_id || ""),
      x_mm: Number(point.neutral_position?.x_mm ?? point.x_mm ?? 0),
      y_mm: Number(point.neutral_position?.y_mm ?? point.y_mm ?? 0),
      envelope_radius_mm: Number(point.envelope_radius_mm ?? 0),
    })),
    members: (graph.members || []).map((member) => ({
      id: String(member.id || ""),
      point_a_id: String(member.point_a_id || ""),
      point_b_id: String(member.point_b_id || ""),
      length_mm: Number(member.length_mm ?? 0),
      kind: String(member.kind || "rod"),
      envelope_radius_mm: Number(member.envelope_radius_mm ?? 0),
      assembly_id: String(member.assembly_id || ""),
    })),
    outputs: (graph.angle_outputs || []).map((output) => ({
      id: String(output.id || ""),
      pivot_point_id: String(output.pivot_point_id || ""),
      endpoint_point_id: String(output.endpoint_point_id || ""),
      neutral_angle_deg: angleDegreesFromConfig(output, "neutral_angle"),
      minimum_angle_deg: angleDegreesFromConfig(output, "minimum_angle"),
      maximum_angle_deg: angleDegreesFromConfig(output, "maximum_angle"),
      reference_body_id: String(output.reference_body_id || ""),
    })),
    drivers: (state.mechanismDrivers || []).map((driver) => ({
      point_id: String(driver.point_id || ""),
      input_id: String(driver.input_id || "articulation"),
      center_x_mm: Number(driver.center?.x_mm ?? 0),
      center_y_mm: Number(driver.center?.y_mm ?? 0),
      radius_mm: Number(driver.radius_mm ?? 0),
      neutral_angle_deg: angleDegreesFromConfig(driver, "neutral_angle"),
      input_ratio: Number(driver.input_ratio ?? 1),
      phase_offset_deg: angleDegreesFromConfig(driver, "phase_offset") === ""
        ? 0
        : angleDegreesFromConfig(driver, "phase_offset"),
    })),
    assignments: (state.steeringAssignments || []).map((assignment) => ({
      output_id: String(assignment.output_id || ""),
      wheel_id: String(assignment.wheel_id || ""),
      ratio: Number(assignment.ratio ?? 1),
      phase_offset_deg: angleDegreesFromConfig(assignment, "phase_offset") === ""
        ? 0
        : angleDegreesFromConfig(assignment, "phase_offset"),
    })),
  };
}

function graphEditorOptions(values) {
  return values.map((value) => ({ value: String(value), label: String(value) }));
}

function mechanismBodyOptions(currentValue = "") {
  const bodyIds = state.combinationBodies.map((body) => String(body.id));
  const options = [{ value: "", label: "Global / no body" }];
  state.combinationBodies.forEach((body) => {
    options.push({ value: String(body.id), label: `${body.name} (${body.id})` });
  });
  const current = String(currentValue || "");
  if (current && !bodyIds.includes(current)) {
    options.push({ value: current, label: `Unknown body (${current})` });
  }
  return options;
}

function createGraphEditorField(collection, index, field) {
  const label = document.createElement("label");
  label.className = "graph-editor-field";
  label.textContent = field.label;
  const input = field.options
    ? document.createElement("select")
    : document.createElement("input");
  input.dataset.graphCollection = collection;
  input.dataset.graphIndex = String(index);
  input.dataset.graphField = field.name;
  input.setAttribute("aria-label", field.label);
  if (field.options) {
    for (const optionData of field.options) {
      const option = document.createElement("option");
      option.value = optionData.value;
      option.textContent = optionData.label;
      input.appendChild(option);
    }
  } else {
    input.type = field.type || "text";
    if (input.type === "number") {
      input.step = "any";
      input.inputMode = "decimal";
    }
  }
  input.value = field.value === null || field.value === undefined ? "" : String(field.value);
  const updateDraft = () => {
    const draft = state.mechanismGraphEditorDraft;
    if (draft?.[collection]?.[index]) {
      draft[collection][index][field.name] = input.value;
      mechanismGraphEditorStatus.textContent = "Unsaved graph edits. Apply the graph before solving or validating.";
    }
  };
  input.addEventListener("input", updateDraft);
  input.addEventListener("change", updateDraft);
  label.appendChild(input);
  return label;
}

function renderGraphEditorRows(container, collection, items, fieldsForItem) {
  container.replaceChildren();
  if (items.length === 0) {
    const empty = document.createElement("p");
    empty.className = "curve-note graph-editor-empty";
    empty.textContent = "No entries. Use the add button above to create one.";
    container.appendChild(empty);
    return;
  }
  items.forEach((item, index) => {
    const row = document.createElement("div");
    row.className = "graph-editor-row";
    row.dataset.graphCollection = collection;
    row.dataset.graphIndex = String(index);
    for (const field of fieldsForItem(item)) {
      row.appendChild(createGraphEditorField(collection, index, field));
    }
    const remove = document.createElement("button");
    remove.type = "button";
    remove.className = "graph-editor-remove";
    remove.textContent = "Remove";
    remove.addEventListener("click", () => {
      state.mechanismGraphEditorDraft[collection].splice(index, 1);
      renderMechanismGraphEditor();
      mechanismGraphEditorStatus.textContent = "Unsaved graph edits. Apply the graph before solving or validating.";
    });
    row.appendChild(remove);
    container.appendChild(row);
  });
}

function renderMechanismGraphEditor() {
  const draft = state.mechanismGraphEditorDraft;
  const enabled = Boolean(state.mechanismGraph && draft);
  for (const button of [
    mechanismGraphApplyButton,
    mechanismGraphAddPointButton,
    mechanismGraphAddMemberButton,
    mechanismGraphAddOutputButton,
    mechanismGraphAddDriverButton,
    mechanismGraphAddAssignmentButton,
  ]) {
    button.disabled = !enabled;
  }
  if (!enabled) {
    for (const container of [
      mechanismPointEditor,
      mechanismMemberEditor,
      mechanismOutputEditor,
      mechanismDriverEditor,
      mechanismAssignmentEditor,
    ]) {
      container.replaceChildren();
    }
    return;
  }
  const pointIds = draft.points.map((point) => point.id);
  const outputIds = draft.outputs.map((output) => output.id);
  renderGraphEditorRows(mechanismPointEditor, "points", draft.points, (point) => [
    { name: "id", label: "ID", value: point.id },
    { name: "mode", label: "Mode", value: point.mode, options: [
      { value: "fixed", label: "Fixed" },
      { value: "driven", label: "Driven" },
      { value: "free", label: "Free" },
    ] },
    { name: "body_id", label: "Body placement", value: point.body_id, options: mechanismBodyOptions(point.body_id) },
    { name: "x_mm", label: "X (mm)", value: point.x_mm, type: "number" },
    { name: "y_mm", label: "Y (mm)", value: point.y_mm, type: "number" },
    { name: "envelope_radius_mm", label: "Envelope (mm)", value: point.envelope_radius_mm, type: "number" },
  ]);
  renderGraphEditorRows(mechanismMemberEditor, "members", draft.members, (member) => [
    { name: "id", label: "ID", value: member.id },
    { name: "point_a_id", label: "Point A", value: member.point_a_id, options: graphEditorOptions(pointIds) },
    { name: "point_b_id", label: "Point B", value: member.point_b_id, options: graphEditorOptions(pointIds) },
    { name: "length_mm", label: "Length (mm)", value: member.length_mm, type: "number" },
    { name: "kind", label: "Kind", value: member.kind, options: [
      { value: "arm", label: "Arm" },
      { value: "rod", label: "Rod" },
      { value: "rigid_brace", label: "Rigid brace" },
    ] },
    { name: "envelope_radius_mm", label: "Envelope (mm)", value: member.envelope_radius_mm, type: "number" },
    { name: "assembly_id", label: "Assembly ID", value: member.assembly_id },
  ]);
  renderGraphEditorRows(mechanismOutputEditor, "outputs", draft.outputs, (output) => [
    { name: "id", label: "ID", value: output.id },
    { name: "pivot_point_id", label: "Pivot", value: output.pivot_point_id, options: graphEditorOptions(pointIds) },
    { name: "endpoint_point_id", label: "Endpoint", value: output.endpoint_point_id, options: graphEditorOptions(pointIds) },
    { name: "reference_body_id", label: "Reference body", value: output.reference_body_id, options: mechanismBodyOptions(output.reference_body_id) },
    { name: "neutral_angle_deg", label: "Neutral (deg)", value: output.neutral_angle_deg, type: "number" },
    { name: "minimum_angle_deg", label: "Min (deg)", value: output.minimum_angle_deg, type: "number" },
    { name: "maximum_angle_deg", label: "Max (deg)", value: output.maximum_angle_deg, type: "number" },
  ]);
  renderGraphEditorRows(mechanismDriverEditor, "drivers", draft.drivers, (driver) => [
    { name: "point_id", label: "Driven point", value: driver.point_id, options: graphEditorOptions(pointIds) },
    { name: "input_id", label: "Input ID", value: driver.input_id },
    { name: "center_x_mm", label: "Center X", value: driver.center_x_mm, type: "number" },
    { name: "center_y_mm", label: "Center Y", value: driver.center_y_mm, type: "number" },
    { name: "radius_mm", label: "Radius (mm)", value: driver.radius_mm, type: "number" },
    { name: "neutral_angle_deg", label: "Neutral (deg)", value: driver.neutral_angle_deg, type: "number" },
    { name: "input_ratio", label: "Input ratio", value: driver.input_ratio, type: "number" },
    { name: "phase_offset_deg", label: "Phase (deg)", value: driver.phase_offset_deg, type: "number" },
  ]);
  renderGraphEditorRows(mechanismAssignmentEditor, "assignments", draft.assignments, (assignment) => [
    { name: "output_id", label: "Output", value: assignment.output_id, options: graphEditorOptions(outputIds) },
    { name: "wheel_id", label: "Wheel ID", value: assignment.wheel_id },
    { name: "ratio", label: "Ratio", value: assignment.ratio, type: "number" },
    { name: "phase_offset_deg", label: "Phase (deg)", value: assignment.phase_offset_deg, type: "number" },
  ]);
}

function graphEditorNumber(value, label, { optional = false, positive = false, nonnegative = false } = {}) {
  if (value === "" || value === null || value === undefined) {
    if (optional) return null;
    throw new Error(`${label} is required.`);
  }
  const parsed = Number(value);
  if (!Number.isFinite(parsed) || (positive && parsed <= 0) || (nonnegative && parsed < 0)) {
    throw new Error(`${label} must be ${positive ? "positive" : "nonnegative"} and finite.`);
  }
  return parsed;
}

function readMechanismGraphEditor() {
  const draft = state.mechanismGraphEditorDraft;
  if (!draft) {
    throw new Error("Build the mechanism graph before editing it.");
  }
  const points = draft.points.map((point, index) => ({
    id: String(point.id || "").trim(),
    mode: String(point.mode || "free"),
    body_id: String(point.body_id || "").trim() || null,
    neutral_position: {
      x_mm: graphEditorNumber(point.x_mm, `Point ${index + 1} X`),
      y_mm: graphEditorNumber(point.y_mm, `Point ${index + 1} Y`),
    },
    envelope_radius_mm: graphEditorNumber(point.envelope_radius_mm, `Point ${index + 1} envelope`, { nonnegative: true }),
  }));
  if (points.some((point) => !point.id)) {
    throw new Error("Every point requires a unique ID.");
  }
  const pointIds = new Set(points.map((point) => point.id));
  if (pointIds.size !== points.length) {
    throw new Error("Point IDs must be unique.");
  }
  const members = draft.members.map((member, index) => ({
    id: String(member.id || "").trim(),
    point_a_id: String(member.point_a_id || "").trim(),
    point_b_id: String(member.point_b_id || "").trim(),
    length_mm: graphEditorNumber(member.length_mm, `Member ${index + 1} length`, { positive: true }),
    kind: String(member.kind || "rod"),
    envelope_radius_mm: graphEditorNumber(member.envelope_radius_mm, `Member ${index + 1} envelope`, { nonnegative: true }),
    assembly_id: String(member.assembly_id || "").trim() || null,
  }));
  if (members.some((member) => !member.id || !member.point_a_id || !member.point_b_id)) {
    throw new Error("Every member requires an ID and two point references.");
  }
  if (members.some((member) => member.point_a_id === member.point_b_id)) {
    throw new Error("A member must connect two different points.");
  }
  if (members.some((member) => !pointIds.has(member.point_a_id) || !pointIds.has(member.point_b_id))) {
    throw new Error("Every member endpoint must reference an existing point.");
  }
  const memberIds = new Set(members.map((member) => member.id));
  if (memberIds.size !== members.length || [...memberIds].some((id) => pointIds.has(id))) {
    throw new Error("Member IDs must be unique and must not overlap point IDs.");
  }
  const outputs = draft.outputs.map((output, index) => {
    const result = {
      id: String(output.id || "").trim(),
      pivot_point_id: String(output.pivot_point_id || "").trim(),
      endpoint_point_id: String(output.endpoint_point_id || "").trim(),
      reference_body_id: String(output.reference_body_id || "").trim() || null,
      neutral_angle_deg: graphEditorNumber(output.neutral_angle_deg, `Output ${index + 1} neutral angle`),
    };
    const minimum = graphEditorNumber(output.minimum_angle_deg, `Output ${index + 1} minimum angle`, { optional: true });
    const maximum = graphEditorNumber(output.maximum_angle_deg, `Output ${index + 1} maximum angle`, { optional: true });
    if (minimum !== null) result.minimum_angle_deg = minimum;
    if (maximum !== null) result.maximum_angle_deg = maximum;
    return result;
  });
  if (outputs.some((output) => !output.id || !output.pivot_point_id || !output.endpoint_point_id)) {
    throw new Error("Every angle output requires an ID, pivot, and endpoint.");
  }
  if (outputs.some((output) => output.pivot_point_id === output.endpoint_point_id)) {
    throw new Error("An angle output pivot and endpoint must be different points.");
  }
  if (outputs.some((output) => !pointIds.has(output.pivot_point_id) || !pointIds.has(output.endpoint_point_id))) {
    throw new Error("Every angle output point must reference an existing point.");
  }
  if (outputs.some((output) => output.minimum_angle_deg !== undefined
      && output.maximum_angle_deg !== undefined
      && output.minimum_angle_deg >= output.maximum_angle_deg)) {
    throw new Error("Angle output minimum limits must be below maximum limits.");
  }
  const outputIds = new Set(outputs.map((output) => output.id));
  if (outputIds.size !== outputs.length) {
    throw new Error("Angle output IDs must be unique.");
  }
  const drivers = draft.drivers.map((driver, index) => ({
    point_id: String(driver.point_id || "").trim(),
    input_id: String(driver.input_id || "").trim(),
    center: {
      x_mm: graphEditorNumber(driver.center_x_mm, `Driver ${index + 1} center X`),
      y_mm: graphEditorNumber(driver.center_y_mm, `Driver ${index + 1} center Y`),
    },
    radius_mm: graphEditorNumber(driver.radius_mm, `Driver ${index + 1} radius`, { positive: true }),
    neutral_angle_deg: graphEditorNumber(driver.neutral_angle_deg, `Driver ${index + 1} neutral angle`),
    input_ratio: graphEditorNumber(driver.input_ratio, `Driver ${index + 1} input ratio`),
    phase_offset_deg: graphEditorNumber(driver.phase_offset_deg, `Driver ${index + 1} phase`),
  }));
  if (drivers.some((driver) => !driver.point_id || !driver.input_id || !pointIds.has(driver.point_id))) {
    throw new Error("Every driver requires an existing driven point and input ID.");
  }
  if (drivers.some((driver) => points.find((point) => point.id === driver.point_id)?.mode !== "driven")) {
    throw new Error("Every driver point must have mode Driven.");
  }
  if (new Set(drivers.map((driver) => driver.point_id)).size !== drivers.length) {
    throw new Error("Each driven point can have only one driver arc.");
  }
  const assignments = draft.assignments.map((assignment, index) => ({
    output_id: String(assignment.output_id || "").trim(),
    wheel_id: String(assignment.wheel_id || "").trim(),
    ratio: graphEditorNumber(assignment.ratio, `Wheel map ${index + 1} ratio`),
    phase_offset_deg: graphEditorNumber(assignment.phase_offset_deg, `Wheel map ${index + 1} phase`),
  }));
  if (assignments.some((assignment) => !assignment.output_id || !assignment.wheel_id)) {
    throw new Error("Every wheel mapping requires an output and wheel ID.");
  }
  if (assignments.some((assignment) => !outputIds.has(assignment.output_id))) {
    throw new Error("Every wheel mapping must reference an existing angle output.");
  }
  if (new Set(assignments.map((assignment) => assignment.wheel_id)).size !== assignments.length) {
    throw new Error("Each wheel can be mapped only once.");
  }
  return {
    graph: {
      id: String(draft.graphId || "workspace_mechanism_graph").trim() || "workspace_mechanism_graph",
      points,
      members,
      angle_outputs: outputs,
    },
    drivers,
    assignments,
  };
}

function applyMechanismGraphEdits() {
  try {
    const configuration = readMechanismGraphEditor();
    state.mechanismGraph = configuration.graph;
    state.mechanismDrivers = configuration.drivers;
    state.steeringAssignments = configuration.assignments;
    state.mechanismGraphEditorDraft = null;
    state.workspaceDirty = true;
    resetEngineeringEvidence(
      "Graph changed. Solve the design again before validating the range.",
      { preserveManeuver: true },
    );
    renderMechanismGraphConfiguration("Graph edits applied. Solve the design to generate new engineering evidence.");
    mechanismGraphEditorStatus.textContent = "Graph edits applied. The saved revision is now out of date; save a new revision after solving.";
    mechanismGraphEditor.open = false;
  } catch (error) {
    mechanismGraphEditorStatus.textContent = `Graph edits rejected: ${error.message}`;
  }
}

function clearEngineeringVisualization() {
  showSimulationDiagram();
  diagram.replaceChildren();
  const placeholder = document.createElementNS("http://www.w3.org/2000/svg", "text");
  placeholder.setAttribute("x", "0");
  placeholder.setAttribute("y", "0");
  placeholder.setAttribute("text-anchor", "middle");
  placeholder.setAttribute("class", "diagram-placeholder");
  placeholder.textContent = "Solve the active design to render engineering evidence";
  diagram.appendChild(placeholder);
  radiusValue.textContent = "n/a";
  radiusChip.textContent = "Awaiting solve";
  maxAngleValue.textContent = "n/a";
  phaseValue.textContent = "n/a";
  actualErrorValue.textContent = "n/a";
  synchronizationErrorValue.textContent = "n/a";
  linkageSteerValue.textContent = "n/a";
  linkageErrorValue.textContent = "n/a";
  linkageResidualValue.textContent = "n/a";
  linkageBranchValue.textContent = "n/a";
  wheelTable.replaceChildren();
  synchronizationTable.replaceChildren();
  bodyChainBodyCountValue.textContent = "n/a";
  bodyChainJointCountValue.textContent = "n/a";
  bodyChainRootValue.textContent = "n/a";
  bodyChainTable.replaceChildren();
  syncDisplayModeUi();
}

function resetEngineeringEvidence(summary, { preserveManeuver = false } = {}) {
  state.currentPayload = null;
  state.maneuverResolved = preserveManeuver && state.maneuverResolved;
  state.currentValidationPass = false;
  state.sweepValidationPayload = null;
  state.acceptanceCriteriaDirty = false;
  state.activeRevisionHasFullRangeEvidence = false;
  state.acceptanceResult = null;
  state.optimizationPayload = null;
  currentValidationCard.dataset.status = "pending";
  currentValidationStatus.textContent = "NOT RUN";
  currentValidationSummary.textContent = summary;
  currentValidationChecks.replaceChildren();
  if (currentSteeringInterpretation && currentSteeringStatus && currentSteeringDetail) {
    currentSteeringInterpretation.dataset.status = "pending";
    currentSteeringStatus.textContent = "STEERING CRITERION PENDING";
    currentSteeringDetail.textContent = "Steering accuracy is not classified until signed-off Monroc limits are entered and evaluated.";
  }
  renderFailureGuidance(currentValidationGuidance, []);
  clearEngineeringVisualization();
  resetOptimizationPanel();
  renderMonrocAcceptance(null);
  renderSweepValidation(null);
  renderReviewControls();
}

function markWorkspaceDirty(summary, { invalidateMechanism = false } = {}) {
  state.workspaceDirty = true;
  if (invalidateMechanism && (state.mechanismGraph || state.mechanismDrivers.length || state.steeringAssignments.length)) {
    state.mechanismGraph = null;
    state.mechanismDrivers = [];
    state.steeringAssignments = [];
    state.mechanismGraphEditorDraft = null;
    renderMechanismGraphConfiguration("Vehicle geometry changed. Rebuild the mechanism graph before solving.");
  }
  resetEngineeringEvidence(summary);
  renderReviewControls();
  updateExportLinks();
  updateDxfSourceRetentionState();
}

function nextGraphEditorId(collection, prefix) {
  const draft = state.mechanismGraphEditorDraft;
  const existing = new Set((draft?.[collection] || []).map((item) => item.id || item.point_id || item.output_id));
  let index = 1;
  while (existing.has(`${prefix}_${index}`)) {
    index += 1;
  }
  return `${prefix}_${index}`;
}

function addMechanismGraphEditorItem(collection) {
  if (!state.mechanismGraphEditorDraft) {
    return;
  }
  const draft = state.mechanismGraphEditorDraft;
  const pointIds = draft.points.map((point) => point.id);
  const outputIds = draft.outputs.map((output) => output.id);
  if (collection === "points") {
    draft.points.push({
      id: nextGraphEditorId("points", "point"),
      mode: "free",
      body_id: "",
      x_mm: 0,
      y_mm: 0,
      envelope_radius_mm: 0,
    });
  } else if (collection === "members") {
    draft.members.push({
      id: nextGraphEditorId("members", "member"),
      point_a_id: pointIds[0] || "",
      point_b_id: pointIds[1] || pointIds[0] || "",
      length_mm: 100,
      kind: "rod",
      envelope_radius_mm: 0,
      assembly_id: "",
    });
  } else if (collection === "outputs") {
    draft.outputs.push({
      id: nextGraphEditorId("outputs", "output"),
      pivot_point_id: pointIds[0] || "",
      endpoint_point_id: pointIds[1] || pointIds[0] || "",
      neutral_angle_deg: 0,
      minimum_angle_deg: "",
      maximum_angle_deg: "",
    });
  } else if (collection === "drivers") {
    const drivenPoint = draft.points.find((point) => point.mode === "driven") || draft.points[0];
    draft.drivers.push({
      point_id: drivenPoint?.id || "",
      input_id: "articulation",
      center_x_mm: 0,
      center_y_mm: 0,
      radius_mm: 100,
      neutral_angle_deg: 0,
      input_ratio: 1,
      phase_offset_deg: 0,
    });
  } else if (collection === "assignments") {
    draft.assignments.push({
      output_id: outputIds[0] || "",
      wheel_id: `wheel_${draft.assignments.length + 1}`,
      ratio: 1,
      phase_offset_deg: 0,
    });
  }
  renderMechanismGraphEditor();
  mechanismGraphEditor.open = true;
  mechanismGraphEditorStatus.textContent = "Unsaved graph edits. Apply the graph before solving or validating.";
}

function renderMechanismGraphConfiguration(statusText = null) {
  syncMechanismGraphEditorDraft();
  const points = state.mechanismGraph?.points || [];
  const members = state.mechanismGraph?.members || [];
  const assignments = state.steeringAssignments || [];
  mechanismGraphPointCount.textContent = String(points.length);
  mechanismGraphMemberCount.textContent = String(members.length);
  mechanismGraphAssignmentCount.textContent = String(assignments.length);
  mechanismGraphMapping.replaceChildren();
  for (const assignment of assignments) {
    const row = document.createElement("div");
    row.className = "wheel-row";
    const target = document.createElement("span");
    target.className = "label";
    target.textContent = assignment.wheel_id;
    const source = document.createElement("span");
    source.className = "value";
    source.textContent = assignment.output_id;
    row.append(target, source);
    mechanismGraphMapping.appendChild(row);
  }
  mechanismGraphSolveButton.disabled = assignments.length === 0;
  mechanismGraphStatus.textContent = statusText
    || `Restored graph with ${state.mechanismDrivers.length} articulation drivers and ${assignments.length} wheel mappings.`;
  renderMechanismGraphEditor();
}

async function solveMechanismGraphDesign() {
  if (!state.mechanismGraph) {
    throw new Error("Build the mechanism graph first.");
  }
  await calculateCombinationStudy();
  mechanismGraphStatus.textContent = state.currentValidationPass
    ? "Graph design solved and all available hard checks pass."
    : "Graph design solved, but the current design fails one or more hard checks.";
}

async function calculateCombinationStudy(betaOverride = null) {
  const primaryBody = primaryCombinationBody();
  if (betaOverride !== null && primaryBody) {
    primaryBody.articulationDeg = Number(betaOverride);
  }
  const radius = Number(combinationTurnRadiusInput.value);
  if (!Number.isFinite(radius) || Math.abs(radius) < 1e-9) {
    throw new Error("Signed root radius must be a non-zero finite number.");
  }
  const betaDeg = primaryBody?.articulationDeg || 0;
  combinationCalculateButton.disabled = true;
  combinationStatus.textContent = "Resolving body poses, rolling constraints, and steering angles...";
  try {
    const response = await fetch("/api/calculate/kinematic", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        beta_deg: betaDeg,
        root_turn_radius_mm: radius,
        combination: serializedCombination(),
        linkage: state.mechanismGraph ? undefined : serializedLinkageConfig(),
        mechanism_graph: state.mechanismGraph,
        mechanism_drivers: state.mechanismGraph ? state.mechanismDrivers : undefined,
        steering_assignments: state.mechanismGraph ? state.steeringAssignments : undefined,
        clearance_target_mm: Number(state.optimizationSettings.clearanceTargetMm),
      }),
    });
    if (!response.ok) {
      const errorPayload = await response.json().catch(() => null);
      throw new Error(errorPayload?.message || `HTTP ${response.status}`);
    }
    const payload = await response.json();
    state.combinationActive = true;
    state.optimizationPayload = null;
    betaSlider.value = String(betaDeg);
    optimizeButton.disabled = !state.mechanismGraph;
    setOptimizationProposalState(false);
    optimizeFeasibilityCard.dataset.status = "pending";
    optimizeFeasibilityStatus.textContent = state.mechanismGraph ? "READY" : "BLOCKED";
    optimizeFeasibilityReasons.textContent = state.mechanismGraph
      ? "Graph-native optimization is available. Run it only after the mechanism graph has been solved."
      : "Build and solve the mechanism graph before running optimization for this combination.";
    optimizeRunStats.textContent = state.mechanismGraph ? "Ready to run" : "Not run for the active combination";
    updateSummary(payload, { refreshCharts: false });
    // Recompute the validation control state after a graph solve. Without this
    // refresh the validation panel can retain the disabled state from the
    // pre-combination project load.
    renderSweepValidation(state.sweepValidationPayload);
    await renderActiveView(payload);
    updateExportLinks();
    const residual = Number(payload.combination_kinematics?.maximum_constraint_residual_mm || 0);
    combinationStatus.textContent = `Resolved ${payload.vehicle_combination.body_count} bodies and ${payload.vehicle.axle_count} axles. Maximum rolling residual ${residual.toFixed(3)} mm.`;
  } finally {
    combinationCalculateButton.disabled = false;
  }
}

function customTurnRadius() {
  const explicitRadius = customTurnRadiusInput.value.trim();
  if (explicitRadius) {
    const value = Number(explicitRadius);
    if (!Number.isFinite(value) || value === 0) {
      throw new Error("Turn radius must be a non-zero finite number.");
    }
    return value;
  }
  const betaDeg = Number(betaSlider.value);
  if (Math.abs(betaDeg) < 1e-9) {
    return null;
  }
  const betaRad = Math.abs(betaDeg) * Math.PI / 180;
  return Math.sign(betaDeg) * state.geometry.wheelbaseMm / Math.tan(betaRad);
}

async function calculateCustomAxleStudy() {
  state.combinationActive = false;
  optimizeButton.disabled = false;
  customAxleApplyButton.disabled = true;
  customAxleStatus.textContent = "Calculating ideal steering...";
  try {
    const vehicleConfig = serializedVehicleConfig();
    const axles = vehicleConfig.axles;
    if (axles.some((axle) => !Number.isFinite(axle.x_mm) || !Number.isFinite(axle.y_mm) || !Number.isFinite(axle.track_mm) || axle.track_mm <= 0)) {
      throw new Error("Every axle needs finite X/Y values and positive track.");
    }
    const response = await fetch("/api/calculate/ideal-steering", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        id: "custom_ideal_study",
        name: "Custom ideal axle study",
        axles,
        steering_synchronizations: vehicleConfig.steering_synchronizations,
        origin: vehicleConfig.origin,
        body_polygon: vehicleConfig.body_polygon,
        front_articulation_point: vehicleConfig.front_articulation_point,
        rear_articulation_point: vehicleConfig.rear_articulation_point,
        kingpin_point: vehicleConfig.kingpin_point,
        maximum_articulation_deg: vehicleConfig.maximum_articulation_deg,
        turn_radius_mm: customTurnRadius(),
        body_length_mm: vehicleConfig.body_length_mm,
        body_width_mm: vehicleConfig.body_width_mm,
      }),
    });
    if (!response.ok) {
      throw new Error(await response.text() || `HTTP ${response.status}`);
    }
    const raw = await response.json();
    state.vehicleConfig = vehicleConfig;
    const axleAngles = Object.values(raw.axle_center_steering_angles_deg || {});
    const customPayload = {
      ...raw,
      beta_deg: Number(betaSlider.value),
      vehicle_combination: null,
      linkage: null,
      clearance: {
        minimum_clearance_mm: null,
        minimum_pair: null,
        collision_detected: false,
        clearance_violation_detected: false,
        items: [],
      },
      metrics: {
        ...raw.metrics,
        front_rear_phase_deg: axleAngles.length > 1 ? Number(axleAngles[axleAngles.length - 1]) - Number(axleAngles[0]) : null,
      },
    };
    state.displayMode = "simulation";
    localStorage.setItem("easytowing_display_mode", state.displayMode);
    linkageSteerValue.textContent = "n/a";
    linkageErrorValue.textContent = "n/a";
    linkageResidualValue.textContent = "n/a";
    linkageBranchValue.textContent = "n/a";
    updateSummary(customPayload, { refreshCharts: false });
    updateExportLinks();
    await renderActiveView(customPayload);
    customAxleStatus.textContent = `Ideal-only study calculated for ${raw.vehicle.axle_count} axles. Reference linkage is not applied.`;
  } finally {
    customAxleApplyButton.disabled = false;
  }
}

async function activateImportedVehicle(payload) {
  const vehicle = payload?.reconstructed_vehicle;
  if (!vehicle || !Array.isArray(vehicle.axles) || vehicle.axles.length === 0) {
    return false;
  }

  markWorkspaceDirty("Imported CAD geometry changed the active study. Save a new revision before review.");

  const axleSpan = Number(vehicle.axle_span_mm);
  const track = Math.max(...vehicle.axles.map((axle) => Number(axle.track_mm) || 0));
  state.geometry = {
    ...state.geometry,
    wheelbaseMm: Number.isFinite(axleSpan) && axleSpan > 0 ? axleSpan : state.geometry.wheelbaseMm,
    trackMm: track > 0 ? track : state.geometry.trackMm,
    bodyLengthMm: Number(vehicle.body_length_mm) || state.geometry.bodyLengthMm,
    bodyWidthMm: Number(vehicle.body_width_mm) || state.geometry.bodyWidthMm,
  };
  wheelbaseInput.value = String(state.geometry.wheelbaseMm);
  trackInput.value = String(state.geometry.trackMm);
  state.vehicleConfig = storedVehicleConfig(vehicle);
  if (!state.vehicleConfig) {
    return false;
  }
  syncGeometryMetadataInputs();
  customAxleCountInput.value = String(state.customAxles.length);
  renderCustomAxleConfig();
  geometryStatus.textContent = `Imported ${vehicle.axles.length} axle(s) into the ideal-study model.`;
  await calculateCustomAxleStudy();
  return true;
}

function refreshSweptPathPreview(betaDeg = Number(betaSlider.value)) {
  if (blockDiagnosticPreview(sweptPathImage, sweptPathStatus, "Swept-path preview")) {
    return;
  }
  const mode = optimizeMode.value;
  const query = `beta_deg=${encodeURIComponent(betaDeg)}&mode=${encodeURIComponent(mode)}&${geometryQuery()}&step_deg=${encodeURIComponent(readCurveStep())}&beta_min_deg=${encodeURIComponent(state.betaRange.minDeg)}&beta_max_deg=${encodeURIComponent(state.betaRange.maxDeg)}${vehicleConfigQuery()}`;
  void loadDiagnosticPreview(
    sweptPathImage,
    sweptPathStatus,
    `/api/swept-path.svg?${query}`,
    "Swept-path preview",
  );
}

function getDisplayModeConfig() {
  return DISPLAY_MODES[state.displayMode] || DISPLAY_MODES.simulation;
}

function syncDisplayModeUi() {
  const config = getDisplayModeConfig();
  if (viewModeSelect.value !== state.displayMode) {
    viewModeSelect.value = state.displayMode;
  }
  statusModeValue.textContent = config.status;
  diagramTitle.textContent = config.title;
  diagramDescription.textContent = config.description;
  radiusChip.textContent = config.chip;
}

function showSimulationDiagram() {
  if (state.dimensionedSketchUrl) {
    URL.revokeObjectURL(state.dimensionedSketchUrl);
    state.dimensionedSketchUrl = null;
  }
  diagram.hidden = false;
  dimensionedSketch.hidden = true;
}

async function loadDimensionedSketch() {
  const requestId = ++state.viewRequest;
  const betaDeg = Number(betaSlider.value);
  const mode = optimizeMode.value;
  const linkageQuery = state.linkageConfig
    ? `&linkage=${encodeURIComponent(JSON.stringify(serializedLinkageConfig()))}`
    : "";
  const response = await fetch(`/api/export.svg?beta_deg=${encodeURIComponent(betaDeg)}&mode=${encodeURIComponent(mode)}&${geometryQuery()}${linkageQuery}${vehicleConfigQuery()}`);
  const svgText = await response.text();
  if (requestId !== state.viewRequest) {
    return;
  }

  const blob = new Blob([svgText], { type: "image/svg+xml" });
  const objectUrl = URL.createObjectURL(blob);
  if (state.dimensionedSketchUrl) {
    URL.revokeObjectURL(state.dimensionedSketchUrl);
  }
  state.dimensionedSketchUrl = objectUrl;
  diagram.hidden = true;
  dimensionedSketch.hidden = false;
  dimensionedSketch.src = objectUrl;
}

async function renderActiveView(payload) {
  state.currentPayload = payload;
  syncDisplayModeUi();
  if (state.displayMode === "dimensions" || state.displayMode === "optimized") {
    await loadDimensionedSketch();
    return;
  }

  showSimulationDiagram();
  renderDiagram(payload, {
    showIcr: state.displayMode === "simulation" || state.displayMode === "rays",
    showError: state.displayMode === "error",
  });
}

function renderLinkageOverlay(payload) {
  if (!payload.linkage) {
    return;
  }

  const { spec, state, driver_point: driverPoint } = payload.linkage;
  const bellCrankPivot = toSvgPoint(spec.bell_crank_pivot);
  const steeringPivot = toSvgPoint(spec.steering_pivot);
  const driver = toSvgPoint(driverPoint);
  const inputEndpoint = toSvgPoint(state.input_endpoint);
  const outputEndpoint = toSvgPoint(state.output_endpoint);
  const steeringEndpoint = toSvgPoint(state.steering_endpoint);

  const segments = [
    [driver, inputEndpoint, "linkage-input-rod"],
    [bellCrankPivot, inputEndpoint, "linkage-input-arm"],
    [bellCrankPivot, outputEndpoint, "linkage-output-arm"],
    [outputEndpoint, steeringEndpoint, "linkage-tie-rod"],
    [steeringPivot, steeringEndpoint, "linkage-steering-arm"],
  ];

  const companionPivot = spec.companion_steering_pivot && toSvgPoint(spec.companion_steering_pivot);
  const companionEndpoint = state.companion_steering_endpoint && toSvgPoint(state.companion_steering_endpoint);
  if (companionPivot && companionEndpoint) {
    segments.push(
      [steeringEndpoint, companionEndpoint, "linkage-companion-tie-rod"],
      [companionPivot, companionEndpoint, "linkage-companion-steering-arm"],
    );
  }

  for (const [start, end, className] of segments) {
    diagram.appendChild(svgEl("line", {
      x1: start.x,
      y1: start.y,
      x2: end.x,
      y2: end.y,
      class: className,
    }));
  }

  const nodes = [
    [driver, "linkage-driver", 18],
    [inputEndpoint, "linkage-node", 20],
    [outputEndpoint, "linkage-node", 20],
    [steeringEndpoint, "linkage-node", 20],
    [bellCrankPivot, "linkage-pivot", 26],
    [steeringPivot, "linkage-pivot", 26],
  ];
  if (companionPivot && companionEndpoint) {
    nodes.push([companionEndpoint, "linkage-node", 20], [companionPivot, "linkage-pivot", 26]);
  }

  for (const [point, className, radius] of nodes) {
    diagram.appendChild(svgEl("circle", {
      cx: point.x,
      cy: point.y,
      r: radius,
      class: className,
    }));
  }

  diagram.appendChild(svgEl("text", {
    x: bellCrankPivot.x + 48,
    y: bellCrankPivot.y - 36,
    class: "linkage-label",
  })).textContent = "Bell crank";

  diagram.appendChild(svgEl("text", {
    x: steeringPivot.x + 48,
    y: steeringPivot.y - 36,
    class: "linkage-label",
  })).textContent = "Knuckle";

  if (companionPivot) {
    diagram.appendChild(svgEl("text", {
      x: companionPivot.x + 48,
      y: companionPivot.y + 52,
      class: "linkage-label",
    })).textContent = "Companion knuckle";
  }
}

function renderMechanismGraphOverlay(payload) {
  const graphPayload = payload.mechanism_graph;
  if (!graphPayload?.mechanism || !graphPayload?.state) {
    return;
  }

  const positions = graphPayload.state.point_positions || {};
  for (const member of graphPayload.mechanism.members || []) {
    const pointA = positions[member.point_a_id];
    const pointB = positions[member.point_b_id];
    if (!pointA || !pointB) {
      continue;
    }
    const start = toSvgPoint(pointA);
    const end = toSvgPoint(pointB);
    diagram.appendChild(svgEl("line", {
      x1: start.x,
      y1: start.y,
      x2: end.x,
      y2: end.y,
      class: `mechanism-member mechanism-member-${member.kind}`,
      "data-member-id": member.id,
    }));
  }

  for (const point of graphPayload.mechanism.points || []) {
    const position = positions[point.id];
    if (!position) {
      continue;
    }
    const marker = toSvgPoint(position);
    diagram.appendChild(svgEl("circle", {
      cx: marker.x,
      cy: marker.y,
      r: point.mode === "fixed" ? 28 : 22,
      class: `mechanism-point mechanism-point-${point.mode}`,
      "data-point-id": point.id,
    }));
  }

  const outputById = new Map(
    (graphPayload.mechanism.angle_outputs || []).map((output) => [output.id, output]),
  );
  for (const assignment of payload.mechanism_mapping?.steering_assignments || []) {
    const output = outputById.get(assignment.output_id);
    const position = output ? positions[output.endpoint_point_id] : null;
    if (!position) {
      continue;
    }
    const labelPoint = toSvgPoint(position);
    diagram.appendChild(svgEl("text", {
      x: labelPoint.x + 48,
      y: labelPoint.y - 40,
      class: "mechanism-output-label",
    })).textContent = assignment.wheel_id;
  }
}

function renderClearanceOverlay(payload) {
  if (!payload.clearance) {
    return;
  }

  const highlightedIds = new Set();
  if (payload.clearance.minimum_pair) {
    highlightedIds.add(payload.clearance.minimum_pair.item_a_id);
    highlightedIds.add(payload.clearance.minimum_pair.item_b_id);
  }

  for (const item of payload.clearance.items) {
    const envelope = item.envelope;
    const highlighted = highlightedIds.has(item.id);
    const highlightClass = highlighted ? " clearance-highlight" : "";

    if (envelope.kind === "circle") {
      const center = toSvgPoint(envelope.center);
      diagram.appendChild(svgEl("circle", {
        cx: center.x,
        cy: center.y,
        r: envelope.radius_mm,
        class: `clearance-circle${highlightClass}`,
      }));
      continue;
    }

    if (envelope.kind === "capsule") {
      const start = toSvgPoint(envelope.start);
      const end = toSvgPoint(envelope.end);
      diagram.appendChild(svgEl("line", {
        x1: start.x,
        y1: start.y,
        x2: end.x,
        y2: end.y,
        class: `clearance-capsule${highlightClass}`,
        "stroke-width": String(envelope.radius_mm * 2),
      }));
      continue;
    }

    if (envelope.kind === "polygon") {
      const points = envelope.points.map((point) => {
        const svgPoint = toSvgPoint(point);
        return `${svgPoint.x},${svgPoint.y}`;
      }).join(" ");
      diagram.appendChild(svgEl("polygon", {
        points,
        class: `clearance-polygon${highlightClass}`,
      }));
    }
  }
}

function renderErrorOverlay(payload) {
  const error = payload.metrics?.max_abs_wheel_error_deg ?? payload.metrics?.linkage_vs_ideal_front_axle_deg;
  if (error === null || error === undefined || Number.isNaN(error)) {
    return;
  }

  diagram.appendChild(svgEl("rect", {
    x: -3900,
    y: -4950,
    width: 2500,
    height: 900,
    rx: 60,
    class: "error-panel",
  }));
  diagram.appendChild(svgEl("text", {
    x: -3700,
    y: -4550,
    class: "error-label",
  })).textContent = "MAX ACTUAL ERROR";
  diagram.appendChild(svgEl("text", {
    x: -3700,
    y: -4200,
    class: "error-value",
  })).textContent = formatAngle(error);
}

function renderDiagram(payload, options = {}) {
  clearSvg(diagram);
  const showIcr = options.showIcr !== false;

  const gridStep = 500;
  for (let x = -4000; x <= 4000; x += gridStep) {
    diagram.appendChild(svgEl("line", {
      x1: x,
      y1: -5200,
      x2: x,
      y2: 5200,
      class: "grid-line",
    }));
  }
  for (let y = -5000; y <= 5000; y += gridStep) {
    diagram.appendChild(svgEl("line", {
      x1: -4200,
      y1: y,
      x2: 4200,
      y2: y,
      class: "grid-line",
    }));
  }

  diagram.appendChild(svgEl("line", {
    x1: -4200,
    y1: 0,
    x2: 4200,
    y2: 0,
    class: "axis",
  }));
  diagram.appendChild(svgEl("line", {
    x1: 0,
    y1: -5200,
    x2: 0,
    y2: 5200,
    class: "axis",
  }));

  if (!payload.vehicle_combination || !Array.isArray(payload.vehicle_combination.bodies)) {
    const body = payload.body_outline.map((point) => `${point.x_mm},${toSvgY(point.y_mm)}`).join(" ");
    diagram.appendChild(svgEl("polygon", {
      points: body,
      class: "body-outline",
    }));
  }

  const vehicleConfig = payload.vehicle_config || {};
  const vehicleOrigin = vehicleConfig.origin || { x_mm: 0, y_mm: 0 };
  for (const [key, label] of [
    ["front_articulation_point", "Front articulation"],
    ["rear_articulation_point", "Rear articulation"],
    ["kingpin_point", "Kingpin"],
  ]) {
    const point = vehicleConfig[key];
    if (!point) {
      continue;
    }
    const marker = toSvgPoint({
      x_mm: Number(point.x_mm) + Number(vehicleOrigin.x_mm || 0),
      y_mm: Number(point.y_mm) + Number(vehicleOrigin.y_mm || 0),
    });
    diagram.appendChild(svgEl("circle", {
      cx: marker.x,
      cy: marker.y,
      r: 42,
      class: "articulation-marker",
    }));
    diagram.appendChild(svgEl("text", {
      x: marker.x + 70,
      y: marker.y - 70,
      class: "articulation-label",
    })).textContent = label;
  }

  renderVehicleCombinationOverlay(payload);

  const actualAxles = new Map(
    (payload.actual_steering?.axles || []).map((axle) => [axle.axle_id, axle]),
  );
  for (const axle of payload.axles) {
    const left = toSvgPoint(axle.left_wheel.center);
    const right = toSvgPoint(axle.right_wheel.center);
    diagram.appendChild(svgEl("line", {
      x1: left.x,
      y1: left.y,
      x2: right.x,
      y2: right.y,
      class: "axle-line",
    }));

    const actualAxle = actualAxles.get(axle.axle_id);
    const wheels = Array.isArray(axle.wheels) && axle.wheels.length > 0
      ? axle.wheels
      : [axle.left_wheel, axle.right_wheel];
    const actualWheels = new Map(
      (actualAxle?.wheels || [actualAxle?.left_wheel, actualAxle?.right_wheel])
        .filter(Boolean)
        .map((wheel) => [wheel.wheel_id, wheel]),
    );
    for (const wheel of wheels.filter(Boolean)) {
      const center = toSvgPoint(wheel.center);
      const heading = lineFromHeading(wheel.center, wheel.heading_rad, 820);
      diagram.appendChild(svgEl("line", {
        x1: center.x,
        y1: center.y,
        x2: heading.x2,
        y2: heading.y2,
        class: "wheel-heading",
      }));
      const actualWheel = actualWheels.get(wheel.wheel_id);
      if (actualWheel) {
        const actualHeading = lineFromHeading(wheel.center, actualWheel.heading_rad, 900);
        diagram.appendChild(svgEl("line", {
          x1: center.x,
          y1: center.y,
          x2: actualHeading.x2,
          y2: actualHeading.y2,
          class: "actual-wheel-heading",
        }));
      }
      diagram.appendChild(svgEl("rect", {
        x: center.x - 110,
        y: center.y - 70,
        width: 220,
        height: 140,
        rx: 22,
        class: "wheel-body",
        transform: `rotate(${-wheel.heading_deg} ${center.x} ${center.y})`,
      }));
      diagram.appendChild(svgEl("circle", {
        cx: center.x,
        cy: center.y,
        r: 22,
        class: "icr-point",
      }));
    }
  }

  renderClearanceOverlay(payload);
  renderLinkageOverlay(payload);
  renderMechanismGraphOverlay(payload);
  if (options.showError) {
    renderErrorOverlay(payload);
  }

  if (showIcr && payload.icr) {
    const icr = toSvgPoint(payload.icr);
    diagram.appendChild(svgEl("circle", {
      cx: icr.x,
      cy: icr.y,
      r: 90,
      class: "icr-point",
    }));
    diagram.appendChild(svgEl("text", {
      x: icr.x + 140,
      y: icr.y - 140,
      class: "wheel-label",
    })).textContent = "ICR";

    for (const axle of payload.axles) {
      const wheels = Array.isArray(axle.wheels) && axle.wheels.length > 0
        ? axle.wheels
        : [axle.left_wheel, axle.right_wheel];
      for (const wheel of wheels.filter(Boolean)) {
        const center = toSvgPoint(wheel.center);
        diagram.appendChild(svgEl("line", {
          x1: center.x,
          y1: center.y,
          x2: icr.x,
          y2: icr.y,
          class: "icr-ray",
        }));
      }
    }
  }
}

function renderCurrentValidation(payload) {
  const linkageState = payload.linkage?.state;
  const graphState = payload.mechanism_graph?.state;
  const linkageResiduals = linkageState
    ? [
      linkageState.input_stage_error_mm,
      linkageState.tie_rod_error_mm,
      linkageState.companion_tie_rod_error_mm,
    ].filter((value) => value !== null && value !== undefined).map((value) => Math.abs(Number(value)))
    : [];
  const maximumLinkageResidual = Math.max(...linkageResiduals, 0);
  const maximumGraphResidual = graphState
    ? Math.abs(Number(graphState.maximum_residual_mm))
    : null;
  const combinationResidual = payload.combination_kinematics?.maximum_constraint_residual_mm;
  const minimumClearance = payload.clearance?.minimum_clearance_mm;
  const clearanceTarget = Number(state.optimizationSettings.clearanceTargetMm);
  const checks = [
    {
      id: "KINEMATICS",
      label: "Kinematics",
      pass: combinationResidual === null || combinationResidual === undefined || Number(combinationResidual) <= 0.01,
      detail: combinationResidual === null || combinationResidual === undefined
        ? "Single-layout solve completed"
        : `Maximum rolling residual ${Number(combinationResidual).toFixed(3)} mm`,
    },
    {
      id: "MECHANISM",
      label: "Mechanism",
      pass: (Boolean(linkageState) && maximumLinkageResidual <= 0.01)
        || (maximumGraphResidual !== null && maximumGraphResidual <= 0.01),
      detail: graphState
        ? `Graph residual ${maximumGraphResidual.toFixed(3)} mm across all rigid members`
        : (linkageState
          ? `Maximum rigid-link residual ${maximumLinkageResidual.toFixed(3)} mm`
          : "No physical mechanism has been solved"),
    },
    {
      id: "COLLISION",
      label: "Collision",
      pass: payload.clearance?.collision_detected === false,
      detail: payload.clearance?.collision_detected === false
        ? "No non-connected component overlap"
        : "Collision detected or not evaluated",
    },
    {
      id: "CLEARANCE",
      label: "Clearance",
      pass: minimumClearance !== null
        && minimumClearance !== undefined
        && Number(minimumClearance) >= clearanceTarget,
      detail: minimumClearance === null || minimumClearance === undefined
        ? `Not evaluated; ${clearanceTarget.toFixed(1)} mm required`
        : `${Number(minimumClearance).toFixed(1)} mm available; ${clearanceTarget.toFixed(1)} mm required`,
    },
  ];

  currentValidationChecks.replaceChildren();
  for (const check of checks) {
    const row = document.createElement("div");
    row.className = "validation-check";
    row.dataset.status = check.pass ? "pass" : "fail";
    const status = document.createElement("strong");
    status.textContent = check.pass ? "PASS" : "FAIL";
    const detail = document.createElement("span");
    detail.textContent = `${check.label}: ${check.detail}`;
    row.append(status, detail);
    currentValidationChecks.appendChild(row);
  }

  renderCurrentSteeringInterpretation(payload);

  const passed = checks.filter((check) => check.pass).length;
  state.currentValidationPass = passed === checks.length;
  currentValidationCard.dataset.status = state.currentValidationPass ? "pass" : "fail";
  currentValidationStatus.textContent = state.currentValidationPass ? "PASS" : "FAIL";
  currentValidationSummary.textContent = `${passed} of ${checks.length} hard checks passed.`;
  renderFailureGuidance(
    currentValidationGuidance,
    checks.filter((check) => !check.pass).map((check) => check.id),
  );
  renderReleaseChecklist();
  renderProjectDashboardStatus();
  updateExportLinks();
}

function renderCurrentSteeringInterpretation(payload) {
  if (!currentSteeringInterpretation || !currentSteeringStatus || !currentSteeringDetail) {
    return;
  }
  const maximumError = Number(payload?.metrics?.max_abs_wheel_error_deg);
  const errorText = Number.isFinite(maximumError)
    ? `Current-pose maximum ideal-versus-actual wheel error: ${maximumError.toFixed(2)} deg.`
    : "Current-pose ideal-versus-actual wheel error is unavailable.";
  const acceptance = state.acceptanceCriteriaDirty ? null : state.acceptanceResult;
  const steeringCheck = acceptance?.checks?.find((check) => check.id === "STEERING_ACCURACY");
  if (steeringCheck?.status === "PASS" && acceptance?.criteria_approval?.status === "APPROVED") {
    currentSteeringInterpretation.dataset.status = "pass";
    currentSteeringStatus.textContent = "STEERING CRITERION PASS";
    currentSteeringDetail.textContent = `${errorText} The steering criterion passed against the approved Monroc profile.`;
    return;
  }
  if (steeringCheck?.status === "FAIL" && acceptance?.criteria_approval?.status === "APPROVED") {
    currentSteeringInterpretation.dataset.status = "fail";
    currentSteeringStatus.textContent = "STEERING CRITERION FAIL";
    currentSteeringDetail.textContent = `${errorText} Correct the steering mechanism or the approved-case limit before review.`;
    return;
  }
  currentSteeringInterpretation.dataset.status = "pending";
  currentSteeringStatus.textContent = "STEERING CRITERION PENDING";
  currentSteeringDetail.textContent = `${errorText} Enter signed-off Monroc limits below and match the approved profile to classify this result; physical feasibility PASS is not steering acceptance.`;
}

function renderSweepValidation(payload) {
  state.sweepValidationPayload = payload || null;
  sweepValidationButton.disabled = !state.combinationActive || !state.mechanismGraph;
  if (!payload) {
    sweepValidationVerdict.textContent = "NOT RUN";
    sweepValidationSolved.textContent = "n/a";
    sweepValidationClearance.textContent = "n/a";
    sweepValidationFailure.textContent = "n/a";
    sweepValidationStatus.textContent = "Build and solve the mechanism graph, then validate every articulation pose.";
    renderFailureGuidance(sweepValidationGuidance, []);
    renderReleaseChecklist();
    renderProjectDashboardStatus();
    updateExportLinks();
    return;
  }
  const status = String(payload.status || "FAIL");
  sweepValidationVerdict.textContent = status;
  sweepValidationSolved.textContent = `${payload.solved_sample_count ?? 0} / ${payload.sample_count ?? 0}`;
  sweepValidationClearance.textContent = payload.minimum_clearance_mm === null
    || payload.minimum_clearance_mm === undefined
    ? "n/a"
    : formatDistance(Number(payload.minimum_clearance_mm));
  const firstFailure = Array.isArray(payload.violations) && payload.violations.length > 0
    ? payload.violations[0]
    : null;
  const formatJointAngles = (sample) => {
    const values = sample?.joint_angles_deg;
    if (!values || typeof values !== "object") return "";
    return Object.entries(values)
      .map(([jointId, value]) => `${jointId} ${Number(value).toFixed(1)} deg`)
      .join(" / ");
  };
  sweepValidationFailure.textContent = firstFailure
    ? `${formatJointAngles(firstFailure) || `${Number(firstFailure.beta_deg).toFixed(1)} deg`} / ${(firstFailure.checks || []).join(", ")}`
    : "none";
  sweepValidationStatus.textContent = status === "PASS"
    ? `All ${payload.sample_count} configured joint poses passed the hard checks.`
    : payload.sampling_complete === false
      ? "Validation was not completed; the configured joint grid is too large."
      : `${payload.violations?.length || 0} of ${payload.sample_count} configured joint poses failed one or more hard checks.`;
  renderFailureGuidance(
    sweepValidationGuidance,
    payload.failure_guidance || firstFailure?.guidance || [],
  );
  renderReleaseChecklist();
  renderProjectDashboardStatus();
  updateExportLinks();
}

async function runCombinationSweepValidation() {
  if (!state.combinationActive || !state.mechanismGraph) {
    throw new Error("Build the multi-body mechanism graph first.");
  }
  const stepDeg = Number(sweepValidationStepInput.value);
  if (!Number.isFinite(stepDeg) || stepDeg <= 0) {
    throw new Error("Full-range step must be a positive number.");
  }
  sweepValidationButton.disabled = true;
  sweepValidationStatus.textContent = "Solving every articulation pose and checking clearance...";
  try {
    const response = await fetch("/api/calculate/combination-sweep", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        combination: serializedCombination(),
        root_turn_radius_mm: Number(combinationTurnRadiusInput.value),
        mechanism_graph: state.mechanismGraph,
        mechanism_drivers: state.mechanismDrivers,
        steering_assignments: state.steeringAssignments,
        beta_min_deg: state.betaRange.minDeg,
        beta_max_deg: state.betaRange.maxDeg,
        step_deg: stepDeg,
        clearance_target_mm: Number(state.optimizationSettings.clearanceTargetMm),
      }),
    });
    if (!response.ok) {
      throw new Error(await response.text() || `HTTP ${response.status}`);
    }
    const payload = await response.json();
    renderSweepValidation(payload);
    updateExportLinks();
  } finally {
    sweepValidationButton.disabled = !state.combinationActive || !state.mechanismGraph;
  }
}

function updateSummary(payload, options = {}) {
  state.currentPayload = payload;
  state.maneuverResolved = Boolean(payload?.vehicle_combination);
  betaValue.textContent = `${payload.beta_deg.toFixed(0)} deg`;
  betaLabel.textContent = `${payload.beta_deg.toFixed(0)} deg`;
  radiusValue.textContent = payload.turn_radius_mm === null ? "Straight" : formatDistance(payload.turn_radius_mm);
  maxAngleValue.textContent = formatAngle(payload.metrics.max_abs_wheel_angle_deg);
  phaseValue.textContent = payload.metrics.front_rear_phase_deg === null
    ? "n/a"
    : formatAngle(payload.metrics.front_rear_phase_deg);
  radiusChip.textContent = payload.turn_radius_mm === null
    ? "Straight"
    : `ICR radius ${Math.abs(payload.turn_radius_mm).toFixed(0)} mm`;
  renderWheelTable(payload.axles, payload.actual_steering);
  renderSynchronizationTable(payload);
  actualErrorValue.textContent = payload.metrics?.max_abs_wheel_error_deg === null
    || payload.metrics?.max_abs_wheel_error_deg === undefined
    ? "n/a"
    : formatAngle(payload.metrics.max_abs_wheel_error_deg);
  synchronizationErrorValue.textContent = payload.metrics?.front_rear_synchronization_error_deg === null
    || payload.metrics?.front_rear_synchronization_error_deg === undefined
    ? "n/a"
    : formatAngle(payload.metrics.front_rear_synchronization_error_deg);
  if (payload.linkage) {
    linkageSteerValue.textContent = `${formatAngle(payload.linkage.state.steering_angle_deg)} / ${formatAngle(payload.linkage.state.companion_steering_angle_deg)}`;
    linkageErrorValue.textContent = `${formatAngle(payload.metrics.linkage_vs_ideal_front_left_deg)} / ${formatAngle(payload.metrics.linkage_vs_ideal_front_right_deg)}`;
    linkageResidualValue.textContent = `${payload.linkage.state.input_stage_error_mm.toFixed(3)} / ${payload.linkage.state.tie_rod_error_mm.toFixed(3)} / ${payload.linkage.state.companion_tie_rod_error_mm === null ? "n/a" : payload.linkage.state.companion_tie_rod_error_mm.toFixed(3)} mm`;
    linkageBranchValue.textContent = `${payload.linkage.state.input_branch_index} / ${payload.linkage.state.steering_branch_index} / ${payload.linkage.state.companion_branch_index ?? "n/a"}`;
  } else if (payload.mechanism_graph?.state) {
    const graphState = payload.mechanism_graph.state;
    const outputAngles = Object.values(graphState.output_angles_deg || {});
    linkageSteerValue.textContent = outputAngles.length === 0
      ? "n/a"
      : outputAngles.slice(0, 2).map((value) => formatAngle(Number(value))).join(" / ");
    linkageErrorValue.textContent = formatAngle(payload.metrics?.max_abs_wheel_error_deg);
    linkageResidualValue.textContent = `${Number(graphState.maximum_residual_mm).toFixed(3)} mm max`;
    linkageBranchValue.textContent = `Graph solve / ${graphState.iterations} iterations`;
  }

  if (payload.clearance) {
    clearanceValue.textContent = payload.clearance.minimum_clearance_mm === null
      ? "n/a"
      : formatDistance(payload.clearance.minimum_clearance_mm);
    clearancePairValue.textContent = payload.clearance.minimum_pair === null
      ? "n/a"
      : `${payload.clearance.minimum_pair.item_a_id} <-> ${payload.clearance.minimum_pair.item_b_id}`;
    if (payload.clearance.collision_detected) {
      clearanceStatusValue.textContent = "Collision";
    } else if (payload.clearance.clearance_violation_detected) {
      clearanceStatusValue.textContent = "Below margin";
    } else {
      clearanceStatusValue.textContent = "Clear";
    }
  }

  renderVehicleCombinationSummary(payload);
  renderCurrentValidation(payload);
  if (options.refreshCharts !== false && payload.beta_deg !== null && payload.beta_deg !== undefined) {
    refreshSteeringCurvesPreview(payload.beta_deg);
    refreshSweptPathPreview(payload.beta_deg);
  }
}

function formatOptimizationMetric(metrics) {
  return metrics === null || metrics === undefined
    ? "n/a"
    : metrics.toFixed(2);
}

function formatOptimizationClearance(metrics) {
  if (!metrics || metrics.minimum_clearance_mm === null || metrics.minimum_clearance_mm === undefined) {
    return "n/a";
  }
  const beta = metrics.minimum_clearance_beta_deg;
  return beta === null || beta === undefined
    ? formatDistance(metrics.minimum_clearance_mm)
    : `${formatDistance(metrics.minimum_clearance_mm)} @ ${beta.toFixed(1)} deg`;
}

function resetOptimizationPanel() {
  state.optimizationPayload = null;
  optimizeFeasibilityCard.dataset.status = "pending";
  optimizeFeasibilityStatus.textContent = "NOT RUN";
  optimizeFeasibilityReasons.textContent = "Optimization is optional. Run it after the current model is defined and the engineering checks are understood.";
  optimizeBaselineScore.textContent = "n/a";
  optimizeOptimizedScore.textContent = "n/a";
  optimizeBaselineRms.textContent = "n/a";
  optimizeOptimizedRms.textContent = "n/a";
  optimizeBaselineClearance.textContent = "n/a";
  optimizeOptimizedClearance.textContent = "n/a";
  optimizeRunStats.textContent = "Not run";
  renderOptimizationVariableConfig([]);
  renderOptimizationVariables([]);
  setOptimizationProposalState(false);
}

function updateOptimizationSummary(payload) {
  state.optimizationPayload = payload;
  if (Array.isArray(payload.design_cases)) {
    state.designCases = payload.design_cases;
    renderDesignCases();
  }
  const objective = payload.objective;
  if (objective && objective.weights) {
    state.optimizationSettings = {
      clearanceTargetMm: Number(objective.clearance_target_mm),
      steeringErrorWeight: Number(objective.weights.steering_error),
      synchronizationErrorWeight: Number(objective.weights.synchronization_error ?? 0.5),
      clearanceWeight: Number(objective.weights.clearance),
      clearanceViolationWeight: Number(objective.weights.clearance_violation),
      failureWeight: Number(objective.weights.failure),
      preferredWeight: Number(objective.weights.preferred),
      complexityWeight: Number(objective.weights.complexity),
    };
    optimizeClearanceTarget.value = String(state.optimizationSettings.clearanceTargetMm);
    optimizeSteeringWeight.value = String(state.optimizationSettings.steeringErrorWeight);
    optimizeSynchronizationWeight.value = String(state.optimizationSettings.synchronizationErrorWeight);
    optimizeClearanceWeight.value = String(state.optimizationSettings.clearanceWeight);
    optimizeClearanceViolationWeight.value = String(state.optimizationSettings.clearanceViolationWeight);
    optimizeFailureWeight.value = String(state.optimizationSettings.failureWeight);
    optimizePreferredWeight.value = String(state.optimizationSettings.preferredWeight);
    optimizeComplexityWeight.value = String(state.optimizationSettings.complexityWeight);
  }
  optimizeBaselineScore.textContent = payload.baseline ? payload.baseline.score.toFixed(2) : "n/a";
  optimizeOptimizedScore.textContent = payload.optimized ? payload.optimized.score.toFixed(2) : "n/a";
  optimizeBaselineRms.textContent = payload.baseline ? formatAngle(payload.baseline.rms_error_deg) : "n/a";
  optimizeOptimizedRms.textContent = payload.optimized ? formatAngle(payload.optimized.rms_error_deg) : "n/a";
  optimizeBaselineClearance.textContent = formatOptimizationClearance(payload.baseline);
  optimizeOptimizedClearance.textContent = formatOptimizationClearance(payload.optimized);
  const proposalFeasible = payload.optimized?.feasible === true;
  const violations = Array.isArray(payload.optimized?.violations)
    ? payload.optimized.violations
    : [];
  optimizeFeasibilityCard.dataset.status = proposalFeasible ? "pass" : "fail";
  optimizeFeasibilityStatus.textContent = proposalFeasible ? "PASS" : "FAIL";
  optimizeFeasibilityReasons.textContent = proposalFeasible
    ? `All hard constraints satisfied, including ${formatDistance(objective?.clearance_target_mm)} minimum clearance.`
    : (violations.join(", ") || "No feasible proposal was produced.");
  optimizeRunStats.textContent = `${proposalFeasible ? "PASS" : "FAIL"} / ${payload.mode} / ${payload.iterations} it / ${payload.evaluations} eval`;
  renderOptimizationVariableConfig(payload.variables_before || payload.variables_after || []);
  renderOptimizationVariables(payload.variables_after || []);
  const hasProposal = Boolean(payload.optimized) && proposalFeasible;
  setOptimizationProposalState(hasProposal);
}

function getActiveRevision(project) {
  if (!project || !Array.isArray(project.revisions) || project.revisions.length === 0) {
    return null;
  }
  return project.revisions.find((revision) => revision.id === project.active_revision_id) || project.revisions[0];
}

function authHeaders() {
  return {
    "Content-Type": "application/json",
    ...(state.authToken ? { Authorization: `Bearer ${state.authToken}` } : {}),
  };
}

function renderAuthStatus() {
  authLogoutButton.disabled = !state.authToken;
  authLoginButton.disabled = Boolean(state.authToken);
  const canManageUsers = hasPermission("user:manage");
  userProvisioning.hidden = !canManageUsers;
  userCreateButton.disabled = !canManageUsers;
  if (state.authPrincipal) {
    authStatus.textContent = `Signed in as ${state.authPrincipal.display_name} (${state.authPrincipal.role}) in ${state.authPrincipal.organization_id}.`;
  } else {
    authStatus.textContent = state.authRequired
      ? "Sign in is required for project changes and approvals."
      : "Local development mode. Authentication is available but not required.";
  }
  renderAccessControlledControls();
  renderReviewControls();
}

async function createUser() {
  if (!hasPermission("user:manage")) {
    return;
  }
  userCreateButton.disabled = true;
  userCreateStatus.textContent = "Creating user...";
  try {
    const response = await fetch("/api/users", {
      method: "POST",
      headers: authHeaders(),
      body: JSON.stringify({
        email: userCreateEmail.value.trim(),
        password: userCreatePassword.value,
        display_name: userCreateName.value.trim(),
        role: userCreateRole.value,
      }),
    });
    const payload = await response.json();
    if (!response.ok) {
      throw new Error(payload.message || `HTTP ${response.status}`);
    }
    const user = payload.user;
    userCreateStatus.textContent = `Created ${user.display_name} (${user.role}).`;
    userCreateForm.reset();
    userCreateRole.value = "designer";
    await loadReviewerUsers();
  } finally {
    renderAuthStatus();
  }
}

async function loadReviewerUsers() {
  if (!hasPermission("user:manage")) {
    state.reviewerUsers = [];
    renderReviewControls();
    return;
  }
  try {
    const response = await fetch("/api/users", { headers: authHeaders() });
    const payload = await response.json();
    if (!response.ok) {
      throw new Error(payload.message || `HTTP ${response.status}`);
    }
    state.reviewerUsers = Array.isArray(payload.users) ? payload.users : [];
  } catch (error) {
    state.reviewerUsers = [];
  }
  renderReviewControls();
}

async function loadSaaSStatus() {
  try {
    const response = await fetch("/api/saas/status");
    const status = await response.json();
    state.authRequired = status.auth_required === true;
    state.artifactStorageBackend = status.artifact_storage || "response-only";
    if (state.authToken) {
      const sessionResponse = await fetch("/api/auth/session", {
        headers: { Authorization: `Bearer ${state.authToken}` },
      });
      if (sessionResponse.ok) {
        state.authPrincipal = (await sessionResponse.json()).principal;
      } else {
        state.authToken = null;
        sessionStorage.removeItem("easytowing_auth_token");
      }
    }
    await loadReviewerUsers();
  } catch (error) {
    authStatus.textContent = `Workspace security status unavailable: ${error.message}`;
  }
  renderAuthStatus();
  updateDxfSourceRetentionState();
}

async function signIn() {
  authLoginButton.disabled = true;
  authStatus.textContent = "Signing in...";
  try {
    const response = await fetch("/api/auth/login", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        organization_id: authOrganizationInput.value.trim(),
        email: authEmailInput.value.trim(),
        password: authPasswordInput.value,
      }),
    });
    const payload = await response.json();
    if (!response.ok) {
      throw new Error(payload.message || `HTTP ${response.status}`);
    }
    state.authToken = payload.token;
    state.authPrincipal = payload.principal;
    sessionStorage.setItem("easytowing_auth_token", state.authToken);
    authPasswordInput.value = "";
    await loadReviewerUsers();
    renderAuthStatus();
    await refreshProjectPanel();
  } finally {
    renderAuthStatus();
  }
}

async function signOut() {
  try {
    if (state.authToken) {
      await fetch("/api/auth/logout", {
        method: "POST",
        headers: { Authorization: `Bearer ${state.authToken}` },
      });
    }
  } finally {
    state.authToken = null;
    state.authPrincipal = null;
    state.currentProjectId = null;
    state.activeProjectRevisionId = null;
    state.approvalStatus = null;
    state.approvalHistory = [];
    state.workspaceDirty = false;
    state.activeRevisionHasFullRangeEvidence = false;
    localStorage.removeItem("easytowing_project_id");
    sessionStorage.removeItem("easytowing_auth_token");
    window.location.reload();
  }
}

async function loadProjectList() {
  const response = await fetch("/api/projects", { headers: authHeaders() });
  const payload = await response.json();
  if (!response.ok) {
    throw new Error(payload.message || `HTTP ` + response.status);
  }
  return payload;
}

async function loadProjectDetail(projectId) {
  const response = await fetch(`/api/projects/${encodeURIComponent(projectId)}`, { headers: authHeaders() });
  const payload = await response.json();
  if (!response.ok) {
    throw new Error(payload.message || `HTTP ` + response.status);
  }
  return payload;
}

async function loadRevisionArtifacts() {
  state.cadSourceArtifact = null;
  if (!state.currentProjectId || !state.activeProjectRevisionId) {
    updateDxfSourceRetentionState();
    return;
  }
  try {
    const path = `/api/projects/${encodeURIComponent(state.currentProjectId)}/revisions/${encodeURIComponent(state.activeProjectRevisionId)}/artifacts`;
    const response = await fetch(path, { headers: authHeaders() });
    const payload = await response.json();
    if (!response.ok) {
      throw new Error(payload.message || `HTTP ${response.status}`);
    }
    state.cadSourceArtifact = (payload.artifacts || []).find(
      (artifact) => artifact.artifact_type === "cad-source-dxf",
    ) || null;
  } catch (error) {
    state.cadSourceArtifact = null;
  }
  updateDxfSourceRetentionState();
}

async function renderProjectFromDetail(project) {
  state.currentProjectId = project?.id ?? null;
  state.maneuverResolved = false;
  state.acceptanceCriteriaDirty = false;
  state.workspaceDirty = false;
  resetOptimizationPanel();
  if (state.currentProjectId) {
    localStorage.setItem("easytowing_project_id", state.currentProjectId);
  }
  renderProjectSummary(project);
  if (!project) {
    return;
  }

  await loadApprovalStatus();
  await loadRevisionArtifacts();

  projectNameInput.value = project.name || "Reference Demo Project";
  const activeRevision = getActiveRevision(project);
  if (!activeRevision) {
    state.acceptanceResult = null;
    renderMonrocAcceptance(null);
    state.activeRevisionHasFullRangeEvidence = false;
    renderReleaseChecklist();
    return;
  }

  const acceptanceRecord = activeRevision.snapshot?.monroc_acceptance;
  state.acceptanceResult = acceptanceRecord?.result || null;
  const storedAcceptanceCriteria = acceptanceRecord?.criteria;
  if (storedAcceptanceCriteria) {
    acceptanceCaseIdInput.value = storedAcceptanceCriteria.case_id || "";
    acceptanceMinClearanceInput.value = String(storedAcceptanceCriteria.minimum_clearance_mm ?? "");
    acceptanceMaxWheelErrorInput.value = String(storedAcceptanceCriteria.maximum_wheel_error_deg ?? "");
    acceptanceMaxSyncErrorInput.value = String(storedAcceptanceCriteria.maximum_synchronization_error_deg ?? "");
    acceptanceMaxResidualInput.value = String(storedAcceptanceCriteria.maximum_mechanism_residual_mm ?? "0.01");
    acceptanceRequireFullRangeInput.checked = storedAcceptanceCriteria.require_full_range !== false;
  } else {
    acceptanceCaseIdInput.value = "";
    acceptanceMinClearanceInput.value = "";
    acceptanceMaxWheelErrorInput.value = "";
    acceptanceMaxSyncErrorInput.value = "";
    acceptanceMaxResidualInput.value = "0.01";
    acceptanceRequireFullRangeInput.checked = true;
  }
  renderMonrocAcceptance(state.acceptanceResult);

  state.activeRevisionHasFullRangeEvidence = activeRevision.combination_config
    ? activeRevision.snapshot?.sweep_validation?.status === "PASS"
    : Boolean(activeRevision.accepted_optimization);

  projectNoteInput.value = activeRevision.note || "Current design snapshot";
  setBetaRange(
    Number(activeRevision.beta_min_deg ?? -45),
    Number(activeRevision.beta_max_deg ?? 45),
  );
  const storedWheelbase = Number(activeRevision.wheelbase_mm ?? state.geometry.wheelbaseMm);
  const storedTrack = Number(activeRevision.track_mm ?? state.geometry.trackMm);
  if (Number.isFinite(storedWheelbase) && storedWheelbase > 0 && Number.isFinite(storedTrack) && storedTrack > 0) {
    state.geometry = {
      ...state.geometry,
      wheelbaseMm: storedWheelbase,
      trackMm: storedTrack,
      bodyLengthMm: Math.max(storedWheelbase + 1800, 1800),
      bodyWidthMm: storedTrack + 700,
    };
    wheelbaseInput.value = String(storedWheelbase);
    trackInput.value = String(storedTrack);
  }
  betaSlider.value = String(activeRevision.beta_deg);
  optimizeMode.value = activeRevision.optimization_mode || "quick";
  state.optimizationEnabledIds = Array.isArray(activeRevision.optimization_enabled_ids)
    ? new Set(activeRevision.optimization_enabled_ids)
    : null;
  state.designCases = Array.isArray(activeRevision.design_cases)
    ? activeRevision.design_cases
    : [];
  state.vehicleConfig = storedVehicleConfig(activeRevision.vehicle_config);
  if (state.vehicleConfig) {
    customAxleCountInput.value = String(state.customAxles.length);
  } else {
    customAxleCountInput.value = "2";
    state.customAxles = [];
  }
  renderCustomAxleConfig();
  updateDxfSourceRetentionState();
  state.linkageConfig = storedLinkageConfig(activeRevision.linkage_config);
  renderLinkageConfig();
  renderDesignCases();
  if (activeRevision.combination_config) {
    restoreCombinationConfiguration(activeRevision.combination_config);
    combinationTurnRadiusInput.value = String(activeRevision.root_turn_radius_mm ?? 9000);
    state.mechanismGraph = activeRevision.mechanism_graph_config || null;
    state.mechanismDrivers = Array.isArray(activeRevision.mechanism_drivers)
      ? activeRevision.mechanism_drivers
      : [];
    state.steeringAssignments = Array.isArray(activeRevision.steering_assignments)
      ? activeRevision.steering_assignments
      : [];
    state.combinationActive = true;
    state.displayMode = "simulation";
    localStorage.setItem("easytowing_display_mode", state.displayMode);
    renderMechanismGraphConfiguration();
    updateExportLinks();
    await calculateCombinationStudy(Number(activeRevision.beta_deg));
    renderSweepValidation(activeRevision.snapshot?.sweep_validation || null);
    renderReleaseChecklist();
    return;
  }

  state.combinationActive = false;
  state.mechanismGraph = null;
  state.mechanismDrivers = [];
  state.steeringAssignments = [];
  renderCombinationConfig();
  renderMechanismGraphConfiguration("Build the graph after defining the active vehicle combination.");
  renderSweepValidation(null);
  if (activeRevision.accepted_optimization) {
    state.displayMode = "optimized";
    localStorage.setItem("easytowing_display_mode", state.displayMode);
    geometryStatus.textContent = "Applied optimized design is active in this revision.";
  }
  updateExportLinks();
  await loadState(Number(activeRevision.beta_deg));
  renderReleaseChecklist();
}

async function refreshProjectPanel() {
  const listPayload = await loadProjectList();
  const preferredProjectId = localStorage.getItem("easytowing_project_id");
  const projects = Array.isArray(listPayload.projects) ? listPayload.projects : [];
  const preferredProjectBelongsToWorkspace = projects.some((project) => project.id === preferredProjectId);
  const projectId = preferredProjectBelongsToWorkspace
    ? preferredProjectId
    : (listPayload.active_project_id || projects[0]?.id || null);
  renderProjectSelector(projects, projectId);
  if (!projectId) {
    return;
  }
  const detail = await loadProjectDetail(projectId);
  await renderProjectFromDetail(detail);
}

async function createProject() {
  const response = await fetch("/api/projects", {
    method: "POST",
    headers: authHeaders(),
    body: JSON.stringify({
      name: projectNameInput.value || "Reference Demo Project",
      beta_deg: Number(betaSlider.value),
      beta_min_deg: state.betaRange.minDeg,
      beta_max_deg: state.betaRange.maxDeg,
      optimization_mode: optimizeMode.value,
      wheelbase_mm: state.geometry.wheelbaseMm,
      track_mm: state.geometry.trackMm,
      design_cases: serializedDesignCases(),
      linkage_config: state.linkageConfig ? serializedLinkageConfig() : null,
      vehicle_config: state.vehicleConfig,
      ...projectCombinationPayload(),
      note: projectNoteInput.value || "Initial revision",
    }),
  });
  if (!response.ok) {
    throw new Error(await response.text() || `HTTP ${response.status}`);
  }
  const payload = await response.json();
  await renderProjectFromDetail(payload.project);
}

async function saveProjectRevision() {
  if (!state.currentProjectId) {
    await createProject();
    return;
  }
  const response = await fetch(`/api/projects/${encodeURIComponent(state.currentProjectId)}/revisions`, {
    method: "POST",
    headers: authHeaders(),
    body: JSON.stringify({
      beta_deg: Number(betaSlider.value),
      beta_min_deg: state.betaRange.minDeg,
      beta_max_deg: state.betaRange.maxDeg,
      optimization_mode: optimizeMode.value,
      wheelbase_mm: state.geometry.wheelbaseMm,
      track_mm: state.geometry.trackMm,
      enabled_ids: state.optimizationEnabledIds === null ? undefined : [...state.optimizationEnabledIds],
      design_cases: serializedDesignCases(),
      linkage_config: state.linkageConfig ? serializedLinkageConfig() : null,
      vehicle_config: state.vehicleConfig,
      ...projectCombinationPayload(),
      note: projectNoteInput.value || "Revision",
    }),
  });
  if (!response.ok) {
    throw new Error(await response.text() || `HTTP ${response.status}`);
  }
  const payload = await response.json();
  await renderProjectFromDetail(payload.project);
}

function setOptimizationProposalState(enabled) {
  optimizeCompareButton.disabled = !enabled;
  optimizeApplyButton.disabled = !enabled || !state.currentProjectId || !hasPermission("project:write");
  optimizeRejectButton.disabled = !enabled;
}

async function compareOptimization() {
  if (!state.optimizationPayload || !state.currentPayload) {
    return;
  }
  state.displayMode = "optimized";
  localStorage.setItem("easytowing_display_mode", state.displayMode);
  await renderActiveView(state.currentPayload);
}

async function applyOptimizedDesign() {
  if (!state.currentProjectId || !state.optimizationPayload) {
    geometryStatus.textContent = "Create a project and run optimization before applying a design.";
    return;
  }
  optimizeApplyButton.disabled = true;
  try {
    if (state.optimizationPayload.optimized?.feasible !== true) {
      throw new Error("Only a hard-feasible optimization proposal can be applied.");
    }
    const graphOptimization = state.combinationActive && state.mechanismGraph;
    const optimizedDrivers = graphOptimization
      ? state.optimizationPayload.mechanism_drivers_after
      : null;
    const optimizedAssignments = graphOptimization
      ? state.optimizationPayload.steering_assignments_after
      : null;
    if (graphOptimization && (!Array.isArray(optimizedDrivers) || !Array.isArray(optimizedAssignments))) {
      throw new Error("The graph optimization result has no traceable driver or wheel mapping.");
    }
    const appliedLinkageConfig = serializedLinkageConfig(optimizedLinkageConfig());
    const response = await fetch(`/api/projects/${encodeURIComponent(state.currentProjectId)}/optimization`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        action: "apply",
        beta_deg: Number(betaSlider.value),
        beta_min_deg: state.betaRange.minDeg,
        beta_max_deg: state.betaRange.maxDeg,
        optimization_mode: optimizeMode.value,
        clearance_target_mm: state.optimizationSettings.clearanceTargetMm,
        steering_error_weight: state.optimizationSettings.steeringErrorWeight,
        synchronization_error_weight: state.optimizationSettings.synchronizationErrorWeight,
        clearance_weight: state.optimizationSettings.clearanceWeight,
        clearance_violation_weight: state.optimizationSettings.clearanceViolationWeight,
        failure_weight: state.optimizationSettings.failureWeight,
        preferred_weight: state.optimizationSettings.preferredWeight,
        complexity_weight: state.optimizationSettings.complexityWeight,
        wheelbase_mm: state.geometry.wheelbaseMm,
        track_mm: state.geometry.trackMm,
        enabled_ids: state.optimizationEnabledIds === null ? [] : [...state.optimizationEnabledIds],
        design_cases: serializedDesignCases(),
        linkage_config: graphOptimization ? null : appliedLinkageConfig,
        vehicle_config: state.vehicleConfig,
        ...(graphOptimization
          ? {
            ...projectCombinationPayload(),
            mechanism_drivers: optimizedDrivers,
            steering_assignments: optimizedAssignments,
          }
          : {}),
        note: projectNoteInput.value || "Applied optimized design",
      }),
    });
    if (!response.ok) {
      throw new Error(await response.text() || `HTTP ${response.status}`);
    }
    const payload = await response.json();
    geometryStatus.textContent = "Optimized design applied as a new revision.";
    await renderProjectFromDetail(payload.project);
  } finally {
    setOptimizationProposalState(state.optimizationPayload?.optimized?.feasible === true);
  }
}

function rejectOptimization() {
  state.optimizationPayload = null;
  optimizeRunStats.textContent = "Proposal rejected";
  setOptimizationProposalState(false);
}

function wait(milliseconds) {
  return new Promise((resolve) => window.setTimeout(resolve, milliseconds));
}

async function runBackgroundOptimization(requestBody, requestId) {
  const response = await fetch("/api/jobs/optimization", {
    method: "POST",
    headers: authHeaders(),
    body: JSON.stringify({
      ...requestBody,
      project_id: state.currentProjectId || undefined,
    }),
  });
  if (!response.ok) {
    throw new Error(await response.text() || `HTTP ${response.status}`);
  }
  let job = await response.json();
  optimizeFeasibilityReasons.textContent = `Full optimization queued (${job.id}).`;
  optimizeRunStats.textContent = `QUEUED / ${job.progress ?? 0}%`;
  for (let attempt = 0; attempt < 600; attempt += 1) {
    if (requestId !== state.optimizationRequest) {
      return null;
    }
    if (["queued", "running"].includes(job.status)) {
      if (attempt > 0) {
        optimizeFeasibilityReasons.textContent = `Full optimization ${job.status} (${job.progress ?? 0}%).`;
        optimizeRunStats.textContent = `${String(job.status).toUpperCase()} / ${job.progress ?? 0}%`;
      }
      await wait(250);
      const jobResponse = await fetch(`/api/jobs/${encodeURIComponent(job.id)}`, {
        headers: authHeaders(),
      });
      if (!jobResponse.ok) {
        throw new Error(await jobResponse.text() || `HTTP ${jobResponse.status}`);
      }
      job = await jobResponse.json();
      continue;
    }
    if (job.status === "succeeded") {
      return job.result;
    }
    throw new Error(job.error || `Optimization job ended with status ${job.status}.`);
  }
  throw new Error("Optimization job polling timed out after 150 seconds.");
}

async function restoreProjectRevision(revisionId) {
  if (!state.currentProjectId) {
    return;
  }
  const response = await fetch(`/api/projects/${encodeURIComponent(state.currentProjectId)}/restore`, {
    method: "POST",
    headers: authHeaders(),
    body: JSON.stringify({ revision_id: revisionId }),
  });
  const payload = await response.json();
  if (!response.ok) {
    throw new Error(payload.message || `HTTP ` + response.status);
  }
  await renderProjectFromDetail(payload.project);
}

async function loadOptimization(mode) {
  if (state.combinationActive && !state.mechanismGraph) {
    optimizeFeasibilityCard.dataset.status = "fail";
    optimizeFeasibilityStatus.textContent = "BLOCKED";
    optimizeFeasibilityReasons.textContent = "Build and solve the mechanism graph before running graph-native optimization.";
    setOptimizationProposalState(false);
    return;
  }
  const requestId = ++state.optimizationRequest;
  state.optimizationPayload = null;
  setOptimizationProposalState(false);
  optimizeFeasibilityCard.dataset.status = "pending";
  optimizeFeasibilityStatus.textContent = "CHECKING";
  optimizeFeasibilityReasons.textContent = "Evaluating mechanism solvability, collisions, and minimum clearance.";
  optimizeButton.disabled = true;
  optimizeButton.textContent = "Running...";
  try {
    const enabledIds = state.optimizationEnabledIds === null
      ? null
      : [...state.optimizationEnabledIds].sort();
    const optimizationBody = {
      mode,
      wheelbase_mm: state.geometry.wheelbaseMm,
      track_mm: state.geometry.trackMm,
      clearance_target_mm: state.optimizationSettings.clearanceTargetMm,
      steering_error_weight: state.optimizationSettings.steeringErrorWeight,
      synchronization_error_weight: state.optimizationSettings.synchronizationErrorWeight,
      clearance_weight: state.optimizationSettings.clearanceWeight,
      clearance_violation_weight: state.optimizationSettings.clearanceViolationWeight,
      failure_weight: state.optimizationSettings.failureWeight,
      preferred_weight: state.optimizationSettings.preferredWeight,
      complexity_weight: state.optimizationSettings.complexityWeight,
      enabled_ids: enabledIds,
      design_cases: serializedDesignCases(),
      linkage: state.linkageConfig ? serializedLinkageConfig() : null,
      vehicle_config: state.vehicleConfig,
      ...(state.combinationActive ? projectCombinationPayload() : {}),
    };
    let payload;
    if (mode === "full") {
      payload = await runBackgroundOptimization(optimizationBody, requestId);
      if (payload === null) {
        return;
      }
    } else {
      const response = state.combinationActive || state.linkageConfig || state.vehicleConfig
        ? await fetch("/api/calculate/optimization", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(optimizationBody),
        })
        : await fetch(`/api/optimize?mode=${encodeURIComponent(mode)}${enabledIds === null ? "" : `&enabled=${encodeURIComponent(enabledIds.join(","))}`}`
          + `&clearance_target_mm=${encodeURIComponent(state.optimizationSettings.clearanceTargetMm)}`
          + `&steering_error_weight=${encodeURIComponent(state.optimizationSettings.steeringErrorWeight)}`
          + `&synchronization_error_weight=${encodeURIComponent(state.optimizationSettings.synchronizationErrorWeight)}`
          + `&clearance_weight=${encodeURIComponent(state.optimizationSettings.clearanceWeight)}`
          + `&clearance_violation_weight=${encodeURIComponent(state.optimizationSettings.clearanceViolationWeight)}`
          + `&failure_weight=${encodeURIComponent(state.optimizationSettings.failureWeight)}`
          + `&preferred_weight=${encodeURIComponent(state.optimizationSettings.preferredWeight)}`
          + `&complexity_weight=${encodeURIComponent(state.optimizationSettings.complexityWeight)}`
          + (state.designCases.length === 0 ? "" : `&cases=${encodeURIComponent(JSON.stringify(serializedDesignCases()))}`));
      if (!response.ok) {
        throw new Error(await response.text() || `HTTP ${response.status}`);
      }
      payload = await response.json();
    }
    if (requestId !== state.optimizationRequest) {
      return;
    }
    updateOptimizationSummary(payload);
  } catch (error) {
    if (requestId === state.optimizationRequest) {
      optimizeFeasibilityCard.dataset.status = "fail";
      optimizeFeasibilityStatus.textContent = "FAIL";
      optimizeFeasibilityReasons.textContent = error.message;
    }
    throw error;
  } finally {
    if (requestId === state.optimizationRequest) {
      optimizeButton.disabled = state.combinationActive && !state.mechanismGraph;
      optimizeButton.textContent = "Run";
    }
  }
}

async function loadState(betaDeg) {
  state.combinationActive = false;
  optimizeButton.disabled = false;
  const requestId = ++state.activeRequest;
  const response = state.linkageConfig || state.vehicleConfig
    ? await fetch("/api/calculate/kinematic", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        beta_deg: betaDeg,
        wheelbase_mm: state.geometry.wheelbaseMm,
        track_mm: state.geometry.trackMm,
        linkage: serializedLinkageConfig(),
        vehicle_config: state.vehicleConfig,
      }),
    })
    : await fetch(`/api/ideal-steering?beta_deg=${encodeURIComponent(betaDeg)}&${geometryQuery()}`);
  if (!response.ok) {
    throw new Error(await response.text() || `HTTP ${response.status}`);
  }
  const payload = await response.json();
  if (requestId !== state.activeRequest) {
    return;
  }
  updateSummary(payload);
  updateExportLinks();
  await renderActiveView(payload);
}

async function applyLinkageConfiguration() {
  if (state.combinationActive) {
    throw new Error("This is a multi-body graph study. Edit the mechanism graph instead of the legacy linkage panel.");
  }
  const previous = state.linkageConfig;
  let next;
  try {
    next = readLinkageConfig();
  } catch (error) {
    linkageConfigStatus.textContent = `Linkage input failed: ${error.message}`;
    return;
  }
  state.linkageConfig = next;
  markWorkspaceDirty("Linkage configuration applied. Save a new revision before review.");
  linkageApplyButton.disabled = true;
  linkageConfigStatus.textContent = "Solving custom linkage...";
  try {
    if (state.combinationActive) {
      await calculateCombinationStudy(Number(betaSlider.value));
    } else {
      await loadState(Number(betaSlider.value));
    }
    linkageConfigStatus.textContent = "Custom linkage active. Slider results use the applied component dimensions.";
  } catch (error) {
    state.linkageConfig = previous;
    renderLinkageConfig();
    linkageConfigStatus.textContent = `Linkage solve failed: ${error.message}`;
  } finally {
    linkageApplyButton.disabled = false;
  }
}

async function resetLinkageConfiguration() {
  if (state.combinationActive) {
    throw new Error("This is a multi-body graph study. Edit the mechanism graph instead of the legacy linkage panel.");
  }
  const previous = state.linkageConfig;
  state.linkageConfig = null;
  markWorkspaceDirty("Linkage configuration reset. Save a new revision before review.");
  renderLinkageConfig();
  try {
    if (state.combinationActive) {
      await calculateCombinationStudy(Number(betaSlider.value));
    } else {
      await loadState(Number(betaSlider.value));
    }
  } catch (error) {
    state.linkageConfig = previous;
    renderLinkageConfig();
    linkageConfigStatus.textContent = `Linkage reset failed: ${error.message}`;
  }
}

betaSlider.addEventListener("input", (event) => {
  markWorkspaceDirty("Selected maneuver changed. Save a new revision before review.");
  if (state.combinationActive) {
    renderSweepValidation(null);
  }
  const calculation = state.combinationActive
    ? calculateCombinationStudy(Number(event.target.value))
    : loadState(Number(event.target.value));
  void calculation.catch((error) => {
    geometryStatus.textContent = `Simulation failed: ${error.message}`;
  });
});

geometryApplyButton.addEventListener("click", () => {
  const wheelbaseMm = Number(wheelbaseInput.value);
  const trackMm = Number(trackInput.value);
  const minBetaDeg = Number(betaMinInput.value);
  const maxBetaDeg = Number(betaMaxInput.value);
  if (!Number.isFinite(wheelbaseMm) || wheelbaseMm <= 0 || !Number.isFinite(trackMm) || trackMm <= 0) {
    geometryStatus.textContent = "Wheelbase and track must be positive numbers.";
    return;
  }
  if (!Number.isFinite(minBetaDeg) || !Number.isFinite(maxBetaDeg) || minBetaDeg >= maxBetaDeg || minBetaDeg > 0 || maxBetaDeg < 0) {
    geometryStatus.textContent = "Articulation bounds must straddle zero with min below max.";
    return;
  }
  try {
    syncGeometryMetadataFromInputs();
  } catch (error) {
    geometryStatus.textContent = `Geometry metadata failed: ${error.message}`;
    return;
  }
  if (Math.max(Math.abs(minBetaDeg), Math.abs(maxBetaDeg)) > state.geometry.maximumArticulationDeg + 1e-9) {
    geometryStatus.textContent = "Articulation slider bounds cannot exceed the vehicle maximum articulation.";
    return;
  }
  markWorkspaceDirty("Vehicle geometry changed. Recalculate and save a new revision before review.");
  state.geometry = {
    ...state.geometry,
    wheelbaseMm,
    trackMm,
    bodyLengthMm: Math.max(wheelbaseMm + 1800, 1800),
    bodyWidthMm: trackMm + 700,
  };
  if (state.vehicleConfig) {
    state.vehicleConfig = serializedVehicleConfig();
  }
  setBetaRange(minBetaDeg, maxBetaDeg);
  renderCustomAxleConfig();
  syncGeometryMetadataInputs();
  geometryStatus.textContent = `Applied ${wheelbaseMm.toFixed(0)} x ${trackMm.toFixed(0)} mm geometry and ${minBetaDeg.toFixed(0)} to ${maxBetaDeg.toFixed(0)} deg cases.`;
  void loadState(Number(betaSlider.value)).catch((error) => {
    geometryStatus.textContent = `Geometry failed: ${error.message}`;
  });
});

customAxleCountInput.addEventListener("input", () => {
  markWorkspaceDirty("Axle count changed. Recalculate and save a new revision before review.");
  renderCustomAxleConfig();
});

curveStepInput.addEventListener("change", () => {
  const value = Number(curveStepInput.value);
  if (!Number.isFinite(value) || value <= 0) {
    curveStepInput.setCustomValidity("Curve step must be a positive number.");
    return;
  }
  curveStepInput.setCustomValidity("");
  markWorkspaceDirty("Preview sampling step changed. Save a new revision before review.");
  refreshSteeringCurvesPreview();
  refreshSweptPathPreview();
});

designCaseAddButton.addEventListener("click", () => {
  markWorkspaceDirty("Design case added. Save a new revision before review.");
  state.designCases.push(defaultDesignCase(state.designCases.length));
  renderDesignCases();
});

customAxleApplyButton.addEventListener("click", () => {
  markWorkspaceDirty("Axle study inputs changed. Save a new revision before review.");
  void calculateCustomAxleStudy().catch((error) => {
    customAxleStatus.textContent = `Ideal axle study failed: ${error.message}`;
    customAxleApplyButton.disabled = false;
  });
});

linkageCompanionEnabled.addEventListener("change", () => {
  markWorkspaceDirty("Linkage configuration changed. Apply the linkage and save a new revision before review.");
  renderLinkageConfig();
});

for (const workflowStep of workflowSteps) {
  workflowStep.addEventListener("click", () => {
    setWorkflowStep(workflowStep.dataset.workflowStep);
  });
}

workflowNextButton.addEventListener("click", () => {
  if (isLegacyRevisionMode()) {
    combinationActivateButton.click();
    return;
  }
  const action = workflowNextButton.dataset.workflowAction;
  if (action) {
    runWorkflowNextAction(action);
    return;
  }
  setWorkflowStep(workflowNextButton.dataset.workflowTarget || "project");
});

projectStartButton.addEventListener("click", () => {
  if (!state.currentProjectId) {
    if (projectCreateButton.disabled) {
      workspaceAccessCard.open = true;
      authStatus.textContent = "Sign in with project-write permission before creating a project.";
      authEmailInput?.focus();
      return;
    }
    projectCreateButton.click();
    return;
  }
  if (isLegacyRevisionMode()) {
    combinationActivateButton.click();
    return;
  }
  setWorkflowStep(nextWorkflowAction().step);
});

combinationBodyCountInput.addEventListener("change", () => {
  markWorkspaceDirty("Body count changed. Rebuild the mechanism and save a new revision before review.", {
    invalidateMechanism: true,
  });
  resizeCombinationBodies(Number(combinationBodyCountInput.value));
});

combinationActivateButton.addEventListener("click", () => {
  state.combinationActive = true;
  state.vehicleConfig = null;
  state.mechanismGraph = null;
  state.mechanismDrivers = [];
  state.steeringAssignments = [];
  state.mechanismGraphEditorDraft = null;
  resetEngineeringEvidence("Multi-body workflow activated. Define and solve the combination before validation.");
  markWorkspaceDirty("Multi-body workflow activated. Define and solve the combination, then save a new revision.");
  renderCombinationConfig();
  renderMechanismGraphConfiguration("Define the active vehicle combination, then build the mechanism graph.");
  setWorkflowStep("vehicle");
});

combinationTurnRadiusInput.addEventListener("input", () => {
  markWorkspaceDirty("Maneuver radius changed. Recalculate and save a new revision before review.");
});

sweepValidationStepInput.addEventListener("change", () => {
  markWorkspaceDirty("Full-range sample step changed. Save a new revision before review.");
});

combinationCalculateButton.addEventListener("click", () => {
  markWorkspaceDirty("Maneuver pose changed. Save a new revision before review.");
  void calculateCombinationStudy().catch((error) => {
    combinationStatus.textContent = `Combination failed: ${error.message}`;
  });
});

mechanismGraphBuildButton.addEventListener("click", () => {
  try {
    buildMechanismGraphFromCombination();
  } catch (error) {
    mechanismGraphStatus.textContent = `Graph build failed: ${error.message}`;
  }
});

mechanismGraphApplyButton.addEventListener("click", () => {
  applyMechanismGraphEdits();
});

mechanismGraphAddPointButton.addEventListener("click", () => {
  addMechanismGraphEditorItem("points");
});

mechanismGraphAddMemberButton.addEventListener("click", () => {
  addMechanismGraphEditorItem("members");
});

mechanismGraphAddOutputButton.addEventListener("click", () => {
  addMechanismGraphEditorItem("outputs");
});

mechanismGraphAddDriverButton.addEventListener("click", () => {
  addMechanismGraphEditorItem("drivers");
});

mechanismGraphAddAssignmentButton.addEventListener("click", () => {
  addMechanismGraphEditorItem("assignments");
});

mechanismGraphSolveButton.addEventListener("click", () => {
  mechanismGraphSolveButton.disabled = true;
  void solveMechanismGraphDesign()
    .catch((error) => {
      mechanismGraphStatus.textContent = `Graph solve failed: ${error.message}`;
    })
    .finally(() => {
      mechanismGraphSolveButton.disabled = state.mechanismGraph === null;
    });
});

sweepValidationButton.addEventListener("click", () => {
  void runCombinationSweepValidation().catch((error) => {
    sweepValidationStatus.textContent = `Full-range validation failed: ${error.message}`;
    sweepValidationButton.disabled = !state.combinationActive || !state.mechanismGraph;
  });
});

acceptanceEvaluateButton.addEventListener("click", () => {
  void evaluateMonrocAcceptance().catch((error) => {
    acceptanceStatusNote.dataset.status = "fail";
    acceptanceStatusNote.textContent = `Acceptance evaluation failed: ${error.message}`;
    renderMonrocAcceptance(state.acceptanceResult);
  });
});

for (const acceptanceInput of [
  acceptanceCaseIdInput,
  acceptanceMinClearanceInput,
  acceptanceMaxWheelErrorInput,
  acceptanceMaxSyncErrorInput,
  acceptanceMaxResidualInput,
  acceptanceRequireFullRangeInput,
]) {
  acceptanceInput.addEventListener("input", markAcceptanceCriteriaDirty);
  acceptanceInput.addEventListener("change", markAcceptanceCriteriaDirty);
}

linkageApplyButton.addEventListener("click", () => {
  void applyLinkageConfiguration();
});

linkageResetButton.addEventListener("click", () => {
  void resetLinkageConfiguration();
});

optimizeButton.addEventListener("click", () => {
  const values = [
    Number(optimizeClearanceTarget.value),
    Number(optimizeSteeringWeight.value),
    Number(optimizeSynchronizationWeight.value),
    Number(optimizeClearanceWeight.value),
    Number(optimizeClearanceViolationWeight.value),
    Number(optimizeFailureWeight.value),
    Number(optimizePreferredWeight.value),
    Number(optimizeComplexityWeight.value),
  ];
  if (values.some((value) => !Number.isFinite(value) || value < 0)) {
    optimizeRunStats.textContent = "Invalid objective settings";
    return;
  }
  [
    state.optimizationSettings.clearanceTargetMm,
    state.optimizationSettings.steeringErrorWeight,
    state.optimizationSettings.synchronizationErrorWeight,
    state.optimizationSettings.clearanceWeight,
    state.optimizationSettings.clearanceViolationWeight,
    state.optimizationSettings.failureWeight,
    state.optimizationSettings.preferredWeight,
    state.optimizationSettings.complexityWeight,
  ] = values;
  void loadOptimization(optimizeMode.value).catch((error) => {
    optimizeRunStats.textContent = `Optimization failed: ${error.message}`;
  });
});

optimizeMode.addEventListener("change", () => {
  updateExportLinks();
  refreshSteeringCurvesPreview();
  refreshSweptPathPreview();
  if (state.currentPayload) {
    void renderActiveView(state.currentPayload).catch((error) => {
      console.error(error);
    });
  }
});

optimizeCompareButton.addEventListener("click", () => {
  void compareOptimization().catch((error) => {
    geometryStatus.textContent = `Compare failed: ${error.message}`;
  });
});

optimizeApplyButton.addEventListener("click", () => {
  void applyOptimizedDesign().catch((error) => {
    geometryStatus.textContent = `Apply failed: ${error.message}`;
    setOptimizationProposalState(state.optimizationPayload?.optimized?.feasible === true);
  });
});

optimizeRejectButton.addEventListener("click", rejectOptimization);

viewModeSelect.addEventListener("change", () => {
  const selectedMode = viewModeSelect.value;
  state.displayMode = Object.prototype.hasOwnProperty.call(DISPLAY_MODES, selectedMode)
    ? selectedMode
    : "simulation";
  localStorage.setItem("easytowing_display_mode", state.displayMode);
  if (state.currentPayload) {
    void renderActiveView(state.currentPayload).catch((error) => {
      console.error(error);
    });
  } else {
    syncDisplayModeUi();
  }
});

projectCreateButton.addEventListener("click", () => {
  void createProject().catch((error) => {
    geometryStatus.textContent = `Project creation failed: ${error.message}`;
  });
});

projectSelector.addEventListener("change", () => {
  const projectId = projectSelector.value;
  if (!projectId) {
    return;
  }
  projectSelector.disabled = true;
  void loadProjectDetail(projectId)
    .then((project) => renderProjectFromDetail(project))
    .catch((error) => {
      geometryStatus.textContent = `Project load failed: ${error.message}`;
    })
    .finally(() => {
      projectSelector.disabled = false;
    });
});

projectSaveButton.addEventListener("click", () => {
  void saveProjectRevision().catch((error) => {
    geometryStatus.textContent = `Revision save failed: ${error.message}`;
  });
});

reviewSubmitButton.addEventListener("click", () => {
  void submitRevisionForReview().catch((error) => {
    reviewStatusNote.textContent = `Review submission failed: ${error.message}`;
    renderReviewControls();
  });
});

reviewerAssignButton.addEventListener("click", () => {
  void assignReviewer().catch((error) => {
    reviewerAssignmentStatus.textContent = `Reviewer assignment failed: ${error.message}`;
    renderReviewControls();
  });
});

reviewApproveButton.addEventListener("click", () => {
  void decideRevision(true).catch((error) => {
    reviewStatusNote.textContent = `Approval failed: ${error.message}`;
    renderReviewControls();
  });
});

reviewRejectButton.addEventListener("click", () => {
  void decideRevision(false).catch((error) => {
    reviewStatusNote.textContent = `Rejection failed: ${error.message}`;
    renderReviewControls();
  });
});

authForm.addEventListener("submit", (event) => {
  event.preventDefault();
  void signIn().catch((error) => {
    authStatus.textContent = `Sign-in failed: ${error.message}`;
    renderAuthStatus();
  });
});

authLogoutButton.addEventListener("click", () => {
  void signOut().catch((error) => {
    authStatus.textContent = `Sign-out failed: ${error.message}`;
  });
});

userCreateForm.addEventListener("submit", (event) => {
  event.preventDefault();
  void createUser().catch((error) => {
    userCreateStatus.textContent = `User creation failed: ${error.message}`;
    renderAuthStatus();
  });
});

dxfImportButton.addEventListener("click", () => {
  importSelectedDxfFile();
});

dxfSourceUnits.addEventListener("change", () => {
  renderDxfMetadata(state.dxfImportPayload || {});
  updateDxfApplyButtonState(Boolean(state.dxfImportText));
});

dxfCoordinateSystem.addEventListener("change", () => {
  renderDxfMetadata(state.dxfImportPayload || {});
  updateDxfApplyButtonState(Boolean(state.dxfImportText));
});

dxfApplyButton.addEventListener("click", () => {
  applyDxfAssignments();
});

dxfRetainSourceButton.addEventListener("click", () => {
  void retainDxfSource();
});

interactiveEditToggle.addEventListener("click", () => {
  state.interactiveEdit = !state.interactiveEdit;
  interactiveEditToggle.setAttribute("aria-pressed", String(state.interactiveEdit));
  interactiveEditToggle.textContent = state.interactiveEdit ? "Finish axle edit" : "Edit axle layout";
  diagram.classList.toggle("interactive-edit-active", state.interactiveEdit);
  interactiveEditStatus.textContent = state.interactiveEdit
    ? "Drag an axle centerline in the SVG. Release to recalculate ideal steering."
    : "Enable Edit axle layout to drag axle centerlines directly in the live SVG.";
});

diagram.addEventListener("pointerdown", (event) => {
  if (!state.interactiveEdit || !state.currentPayload?.axles?.length) {
    return;
  }
  if (!hydrateEditableVehicleFromPayload(state.currentPayload)) {
    interactiveEditStatus.textContent = "This view has no editable vehicle configuration.";
    return;
  }
  const point = diagramWorldPoint(event);
  const bounds = diagram.getBoundingClientRect();
  const tolerance = Math.max(250, (diagram.viewBox.baseVal.width / Math.max(bounds.width, 1)) * 32);
  const nearest = state.currentPayload.axles
    .map((axle) => ({
      axle,
      distance: Math.hypot(axle.center.x_mm - point.x_mm, axle.center.y_mm - point.y_mm),
    }))
    .sort((left, right) => left.distance - right.distance)[0];
  if (!nearest || nearest.distance > tolerance) {
    interactiveEditStatus.textContent = "Click closer to an axle centerline to move it.";
    return;
  }
  state.draggingAxleId = nearest.axle.axle_id;
  diagram.classList.add("interactive-edit-dragging");
  interactiveEditStatus.textContent = `Dragging ${state.draggingAxleId}. Release to solve.`;
  try {
    diagram.setPointerCapture(event.pointerId);
  } catch (error) {
    // Pointer capture is not available in a few embedded SVG implementations.
  }
});

diagram.addEventListener("pointermove", (event) => {
  if (!state.draggingAxleId || !state.currentPayload) {
    return;
  }
  const nextCenter = diagramWorldPoint(event);
  const editableAxle = state.customAxles.find((axle) => axle.id === state.draggingAxleId);
  if (!editableAxle) {
    return;
  }
  editableAxle.x_mm = nextCenter.x_mm;
  editableAxle.y_mm = nextCenter.y_mm;
  updateDraggedAxlePreview(state.draggingAxleId, nextCenter);
});

diagram.addEventListener("pointerup", (event) => {
  const axleId = state.draggingAxleId;
  if (!axleId) {
    return;
  }
  state.draggingAxleId = null;
  diagram.classList.remove("interactive-edit-dragging");
  try {
    diagram.releasePointerCapture(event.pointerId);
  } catch (error) {
    // Pointer capture is not available in a few embedded SVG implementations.
  }
  interactiveEditStatus.textContent = `Applying ${axleId} position...`;
  try {
    markWorkspaceDirty("Axle layout changed. Save a new revision before review.");
    state.vehicleConfig = serializedVehicleConfig();
    renderCustomAxleConfig();
    void calculateCustomAxleStudy()
      .then(() => {
        interactiveEditStatus.textContent = "Axle layout applied. Drag another centerline or disable edit mode.";
      })
      .catch((error) => {
        interactiveEditStatus.textContent = `Axle layout failed: ${error.message}`;
      });
  } catch (error) {
    interactiveEditStatus.textContent = `Axle layout failed: ${error.message}`;
  }
});

diagram.addEventListener("pointercancel", () => {
  state.draggingAxleId = null;
  diagram.classList.remove("interactive-edit-dragging");
  interactiveEditStatus.textContent = "Axle edit cancelled.";
});

async function initializeApp() {
  state.displayMode = Object.prototype.hasOwnProperty.call(DISPLAY_MODES, state.displayMode)
    ? state.displayMode
    : "simulation";
  syncDisplayModeUi();
  initializeWorkflowPanels();
  setWorkflowStep(state.activeWorkflowStep);
  setBetaRange(state.betaRange.minDeg, state.betaRange.maxDeg);
  updateExportLinks();
  dxfApplyButton.disabled = true;
  dxfImportButton.textContent = "Import";
  syncGeometryMetadataInputs();
  renderCustomAxleConfig();
  renderCombinationConfig();
  renderLinkageConfig();
  renderDesignCases();
  renderDxfEntities({ entities: [], role_options: getDxfRoleOptions() });
  dxfImportStatus.textContent = "Choose a DXF file to parse supported entities and rebuild a rough layout preview.";
  await loadSaaSStatus();
  if (state.authRequired && !state.authPrincipal) {
    return;
  }
  await refreshProjectPanel();
  if (state.combinationActive) {
    return;
  }
  void loadState(Number(betaSlider.value)).catch((error) => {
    geometryStatus.textContent = `Simulation failed: ${error.message}`;
  });
}

initializeApp();
