const state = {
  activeRequest: 0,
  optimizationRequest: 0,
  projectRequest: 0,
  viewRequest: 0,
  dxfImportRequest: 0,
  dxfImportText: "",
  dxfImportSourceName: "",
  dxfImportPayload: null,
  interactiveEdit: false,
  draggingAxleId: null,
  currentProjectId: null,
  currentPayload: null,
  optimizationPayload: null,
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
const dxfFileInput = document.getElementById("dxf-file-input");
const dxfImportButton = document.getElementById("dxf-import-button");
const dxfApplyButton = document.getElementById("dxf-apply-button");
const dxfImportStatus = document.getElementById("dxf-import-status");
const dxfEntityCount = document.getElementById("dxf-entity-count");
const dxfSupportedCount = document.getElementById("dxf-supported-count");
const dxfBoundsValue = document.getElementById("dxf-bounds-value");
const dxfLayoutValue = document.getElementById("dxf-layout-value");
const dxfParametricValue = document.getElementById("dxf-parametric-value");
const dxfEntityTable = document.getElementById("dxf-entity-table");
const steeringCurvesImage = document.getElementById("steering-curves-image");
const sweptPathImage = document.getElementById("swept-path-image");
const projectNameInput = document.getElementById("project-name-input");
const projectCreateButton = document.getElementById("project-create-button");
const projectSaveButton = document.getElementById("project-save-button");
const projectNoteInput = document.getElementById("project-note-input");
const projectIdValue = document.getElementById("project-id-value");
const projectActiveRevisionValue = document.getElementById("project-active-revision-value");
const projectRevisionCountValue = document.getElementById("project-revision-count-value");
const projectRevisionList = document.getElementById("project-revision-list");
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

function formatAngle(value) {
  if (value === null || Number.isNaN(value)) {
    return "n/a";
  }
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
  const corners = [
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
  for (const wheel of [payloadAxle.left_wheel, payloadAxle.right_wheel]) {
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
    (actualSteering?.axles || []).flatMap((axle) => [
      [axle.left_wheel?.wheel_id, axle.left_wheel],
      [axle.right_wheel?.wheel_id, axle.right_wheel],
    ]),
  );
  for (const axle of axles) {
    for (const side of ["left_wheel", "right_wheel"]) {
      const wheel = axle[side];
      const actualWheel = actualWheels.get(wheel.wheel_id);
      const row = document.createElement("div");
      row.className = "wheel-row";
      const label = document.createElement("span");
      label.className = "label";
      label.textContent = `${axle.axle_id} ${wheel.side}`;
      const value = document.createElement("span");
      value.className = "value";
      const idealAngle = wheel.steering_angle_deg ?? wheel.heading_deg;
      const actualAngle = actualWheel?.steering_angle_deg ?? actualSteering?.wheel_angles_deg?.[wheel.wheel_id];
      const error = actualSteering?.errors_deg?.[wheel.wheel_id];
      value.textContent = actualAngle === undefined
        ? formatAngle(idealAngle)
        : `I ${formatAngle(idealAngle)} / A ${formatAngle(actualAngle)} / E ${formatAngle(error)}`;
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
    value.textContent = `I ${formatAngle(idealTarget)} / A ${formatAngle(actualTarget)} / E ${formatAngle(error)}`;
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
  dxfApplyButton.disabled = !enabled;
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

  dxfApplyButton.disabled = entities.length === 0 || !state.dxfImportText;
}

function renderDxfImportSummary(payload) {
  state.dxfImportPayload = payload;
  const bounds = payload.bounds_mm;
  const vehicle = payload.reconstructed_vehicle;
  const parametric = payload.parametric_mechanism;
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
  dxfApplyButton.disabled = !state.dxfImportText || !(payload.entities && payload.entities.length);
  renderDxfEntities(payload);
}

function formatTimestamp(isoString) {
  const value = new Date(isoString);
  if (Number.isNaN(value.getTime())) {
    return isoString;
  }
  return value.toLocaleString();
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
    action.addEventListener("click", () => restoreProjectRevision(revision.id));

    row.append(text, action);
    projectRevisionList.appendChild(row);
  }
}

function renderProjectSummary(project) {
  if (!project) {
    projectIdValue.textContent = "n/a";
    projectActiveRevisionValue.textContent = "n/a";
    projectRevisionCountValue.textContent = "0";
    projectRevisionList.replaceChildren();
    return;
  }

  projectIdValue.textContent = project.id;
  projectActiveRevisionValue.textContent = project.active_revision_id || "n/a";
  projectRevisionCountValue.textContent = String(project.revision_count ?? project.revisions?.length ?? 0);
  renderProjectRevisions(project);
}

function updateExportLinks() {
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
    dxfApplyButton.disabled = false;
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

function refreshSteeringCurvesPreview(betaDeg = Number(betaSlider.value)) {
  const mode = optimizeMode.value;
  const stepDeg = readCurveStep();
  const linkageQuery = state.linkageConfig
    ? `&linkage=${encodeURIComponent(JSON.stringify(serializedLinkageConfig()))}`
    : "";
  const query = `beta_deg=${encodeURIComponent(betaDeg)}&mode=${encodeURIComponent(mode)}&${geometryQuery()}&step_deg=${encodeURIComponent(stepDeg)}&beta_min_deg=${encodeURIComponent(state.betaRange.minDeg)}&beta_max_deg=${encodeURIComponent(state.betaRange.maxDeg)}${linkageQuery}${vehicleConfigQuery()}`;
  steeringCurvesImage.src = `/api/steering-curves.svg?${query}`;
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
      ["wheel_count", "Wheels", false],
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
      input.type = "number";
      input.step = "1";
      if (key === "wheel_count") {
        input.min = "2";
      }
      input.value = axle[key] === null || axle[key] === undefined ? "" : String(axle[key]);
      input.dataset.axleIndex = String(index);
      input.dataset.axleKey = key;
      input.addEventListener("input", (event) => {
        const target = event.target;
        const rawValue = target.value.trim();
        state.customAxles[index][key] = fields.find((item) => item[0] === key)?.[2] && rawValue === ""
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
  const serializedAxles = state.customAxles.map((axle) => ({
    id: String(axle.id),
    x_mm: Number(axle.x_mm),
    y_mm: Number(axle.y_mm),
    track_mm: Number(axle.track_mm),
    wheel_count: Number(axle.wheel_count || 2),
    steerable: axle.steerable !== false,
    steering_mode: axle.steering_mode || "FORCED_STEER",
    heading_rad: Number(axle.heading_rad ?? (Number(axle.heading_deg || 0) * Math.PI / 180)),
    maximum_steering_angle_deg: axle.maximum_steering_angle_deg == null ? null : Number(axle.maximum_steering_angle_deg),
    steering_stop_deg: axle.steering_stop_deg == null ? null : Number(axle.steering_stop_deg),
    load_kg: axle.load_kg == null ? null : Number(axle.load_kg),
    tire_width_mm: Number(axle.tire_width_mm || 0),
    outside_diameter_mm: Number(axle.outside_diameter_mm || 0),
    user_defined_steering_angle_deg: Number(axle.user_defined_steering_angle_deg || 0),
  }));
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
  const mode = optimizeMode.value;
  const query = `beta_deg=${encodeURIComponent(betaDeg)}&mode=${encodeURIComponent(mode)}&${geometryQuery()}&step_deg=${encodeURIComponent(readCurveStep())}&beta_min_deg=${encodeURIComponent(state.betaRange.minDeg)}&beta_max_deg=${encodeURIComponent(state.betaRange.maxDeg)}${vehicleConfigQuery()}`;
  sweptPathImage.src = `/api/swept-path.svg?${query}`;
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
    for (const [side, wheel] of [["left_wheel", axle.left_wheel], ["right_wheel", axle.right_wheel]]) {
      const center = toSvgPoint(wheel.center);
      const heading = lineFromHeading(wheel.center, wheel.heading_rad, 820);
      diagram.appendChild(svgEl("line", {
        x1: center.x,
        y1: center.y,
        x2: heading.x2,
        y2: heading.y2,
        class: "wheel-heading",
      }));
      const actualWheel = actualAxle?.[side];
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
      for (const wheel of [axle.left_wheel, axle.right_wheel]) {
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

function updateSummary(payload, options = {}) {
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
  optimizeRunStats.textContent = `${payload.mode} / ${payload.iterations} it / ${payload.evaluations} eval / ${payload.improved ? "improved" : "no change"}`;
  renderOptimizationVariableConfig(payload.variables_before || payload.variables_after || []);
  renderOptimizationVariables(payload.variables_after || []);
  const hasProposal = Boolean(payload.optimized);
  optimizeCompareButton.disabled = !hasProposal;
  optimizeApplyButton.disabled = !hasProposal || !state.currentProjectId;
  optimizeRejectButton.disabled = !hasProposal;
}

function getActiveRevision(project) {
  if (!project || !Array.isArray(project.revisions) || project.revisions.length === 0) {
    return null;
  }
  return project.revisions.find((revision) => revision.id === project.active_revision_id) || project.revisions[0];
}

async function loadProjectList() {
  const response = await fetch("/api/projects");
  return response.json();
}

async function loadProjectDetail(projectId) {
  const response = await fetch(`/api/projects/${encodeURIComponent(projectId)}`);
  return response.json();
}

async function renderProjectFromDetail(project) {
  state.currentProjectId = project?.id ?? null;
  if (state.currentProjectId) {
    localStorage.setItem("easytowing_project_id", state.currentProjectId);
  }
  renderProjectSummary(project);
  if (!project) {
    return;
  }

  projectNameInput.value = project.name || "Reference Demo Project";
  const activeRevision = getActiveRevision(project);
  if (!activeRevision) {
    return;
  }

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
  state.linkageConfig = storedLinkageConfig(activeRevision.linkage_config);
  renderLinkageConfig();
  renderDesignCases();
  if (activeRevision.accepted_optimization) {
    state.displayMode = "optimized";
    localStorage.setItem("easytowing_display_mode", state.displayMode);
    geometryStatus.textContent = "Applied optimized design is active in this revision.";
  }
  updateExportLinks();
  await loadState(Number(activeRevision.beta_deg));
  await loadOptimization(activeRevision.optimization_mode || "quick");
}

async function refreshProjectPanel() {
  const listPayload = await loadProjectList();
  const preferredProjectId = localStorage.getItem("easytowing_project_id");
  let projectId = preferredProjectId;
  if (!projectId) {
    projectId = listPayload.active_project_id || listPayload.projects?.[0]?.id || null;
  }
  if (!projectId) {
    return;
  }
  const detail = await loadProjectDetail(projectId);
  await renderProjectFromDetail(detail);
}

async function createProject() {
  const response = await fetch("/api/projects", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
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
      note: projectNoteInput.value || "Initial revision",
    }),
  });
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
    headers: {
      "Content-Type": "application/json",
    },
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
      note: projectNoteInput.value || "Revision",
    }),
  });
  const payload = await response.json();
  await renderProjectFromDetail(payload.project);
}

function setOptimizationProposalState(enabled) {
  optimizeCompareButton.disabled = !enabled;
  optimizeApplyButton.disabled = !enabled || !state.currentProjectId;
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
    const appliedLinkageConfig = state.linkageConfig
      ? serializedLinkageConfig(optimizedLinkageConfig())
      : null;
    const response = await fetch(`/api/projects/${encodeURIComponent(state.currentProjectId)}/optimization`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        action: "apply",
        beta_deg: Number(betaSlider.value),
        beta_min_deg: state.betaRange.minDeg,
        beta_max_deg: state.betaRange.maxDeg,
        optimization_mode: optimizeMode.value,
        wheelbase_mm: state.geometry.wheelbaseMm,
        track_mm: state.geometry.trackMm,
        enabled_ids: state.optimizationEnabledIds === null ? [] : [...state.optimizationEnabledIds],
        design_cases: serializedDesignCases(),
        linkage_config: appliedLinkageConfig,
        vehicle_config: state.vehicleConfig,
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
    setOptimizationProposalState(Boolean(state.optimizationPayload));
  }
}

function rejectOptimization() {
  state.optimizationPayload = null;
  optimizeRunStats.textContent = "Proposal rejected";
  setOptimizationProposalState(false);
}

async function restoreProjectRevision(revisionId) {
  if (!state.currentProjectId) {
    return;
  }
  const response = await fetch(`/api/projects/${encodeURIComponent(state.currentProjectId)}/restore`, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ revision_id: revisionId }),
  });
  const payload = await response.json();
  await renderProjectFromDetail(payload.project);
}

async function loadOptimization(mode) {
  const requestId = ++state.optimizationRequest;
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
    };
    const response = state.linkageConfig || state.vehicleConfig
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
    const payload = await response.json();
    if (requestId !== state.optimizationRequest) {
      return;
    }
    updateOptimizationSummary(payload);
  } finally {
    if (requestId === state.optimizationRequest) {
      optimizeButton.disabled = false;
      optimizeButton.textContent = "Run";
    }
  }
}

async function loadState(betaDeg) {
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
  const previous = state.linkageConfig;
  let next;
  try {
    next = readLinkageConfig();
  } catch (error) {
    linkageConfigStatus.textContent = `Linkage input failed: ${error.message}`;
    return;
  }
  state.linkageConfig = next;
  linkageApplyButton.disabled = true;
  linkageConfigStatus.textContent = "Solving custom linkage...";
  try {
    await loadState(Number(betaSlider.value));
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
  const previous = state.linkageConfig;
  state.linkageConfig = null;
  renderLinkageConfig();
  try {
    await loadState(Number(betaSlider.value));
  } catch (error) {
    state.linkageConfig = previous;
    renderLinkageConfig();
    linkageConfigStatus.textContent = `Linkage reset failed: ${error.message}`;
  }
}

betaSlider.addEventListener("input", (event) => {
  void loadState(Number(event.target.value)).catch((error) => {
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

customAxleCountInput.addEventListener("input", renderCustomAxleConfig);

curveStepInput.addEventListener("change", () => {
  const value = Number(curveStepInput.value);
  if (!Number.isFinite(value) || value <= 0) {
    curveStepInput.setCustomValidity("Curve step must be a positive number.");
    return;
  }
  curveStepInput.setCustomValidity("");
  refreshSteeringCurvesPreview();
  refreshSweptPathPreview();
});

designCaseAddButton.addEventListener("click", () => {
  state.designCases.push(defaultDesignCase(state.designCases.length));
  renderDesignCases();
});

customAxleApplyButton.addEventListener("click", () => {
  void calculateCustomAxleStudy().catch((error) => {
    customAxleStatus.textContent = `Ideal axle study failed: ${error.message}`;
    customAxleApplyButton.disabled = false;
  });
});

linkageCompanionEnabled.addEventListener("change", renderLinkageConfig);

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
    setOptimizationProposalState(Boolean(state.optimizationPayload));
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
  createProject();
});

projectSaveButton.addEventListener("click", () => {
  saveProjectRevision();
});

dxfImportButton.addEventListener("click", () => {
  importSelectedDxfFile();
});

dxfApplyButton.addEventListener("click", () => {
  applyDxfAssignments();
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
  setBetaRange(state.betaRange.minDeg, state.betaRange.maxDeg);
  updateExportLinks();
  dxfApplyButton.disabled = true;
  dxfImportButton.textContent = "Import";
  syncGeometryMetadataInputs();
  renderCustomAxleConfig();
  renderLinkageConfig();
  renderDesignCases();
  refreshSteeringCurvesPreview();
  refreshSweptPathPreview();
  renderDxfEntities({ entities: [], role_options: getDxfRoleOptions() });
  dxfImportStatus.textContent = "Choose a DXF file to parse supported entities and rebuild a rough layout preview.";
  await refreshProjectPanel();
  void loadState(Number(betaSlider.value)).catch((error) => {
    geometryStatus.textContent = `Simulation failed: ${error.message}`;
  });
}

initializeApp();
