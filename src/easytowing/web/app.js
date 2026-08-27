const state = {
  activeRequest: 0,
  optimizationRequest: 0,
  projectRequest: 0,
  viewRequest: 0,
  dxfImportRequest: 0,
  dxfImportText: "",
  dxfImportSourceName: "",
  dxfImportPayload: null,
  currentProjectId: null,
  currentPayload: null,
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
const betaLabel = document.getElementById("beta-label");
const statusModeValue = document.getElementById("status-mode-value");
const diagramTitle = document.getElementById("diagram-title");
const diagramDescription = document.getElementById("diagram-description");
const viewModeSelect = document.getElementById("view-mode-select");
const radiusChip = document.getElementById("radius-chip");
const wheelTable = document.getElementById("wheel-table");
const linkageSteerValue = document.getElementById("linkage-steer-value");
const linkageErrorValue = document.getElementById("linkage-error-value");
const linkageResidualValue = document.getElementById("linkage-residual-value");
const linkageBranchValue = document.getElementById("linkage-branch-value");
const clearanceValue = document.getElementById("clearance-value");
const clearancePairValue = document.getElementById("clearance-pair-value");
const clearanceStatusValue = document.getElementById("clearance-status-value");
const optimizeMode = document.getElementById("optimize-mode");
const optimizeButton = document.getElementById("optimize-button");
const optimizeBaselineScore = document.getElementById("opt-baseline-score");
const optimizeOptimizedScore = document.getElementById("opt-optimized-score");
const optimizeBaselineRms = document.getElementById("opt-baseline-rms");
const optimizeOptimizedRms = document.getElementById("opt-optimized-rms");
const optimizeBaselineClearance = document.getElementById("opt-baseline-clearance");
const optimizeOptimizedClearance = document.getElementById("opt-optimized-clearance");
const optimizeRunStats = document.getElementById("opt-run-stats");
const optimizeVariableTable = document.getElementById("opt-variable-table");
const exportJsonLink = document.getElementById("export-json");
const exportCsvLink = document.getElementById("export-csv");
const exportPdfLink = document.getElementById("export-pdf");
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

const DISPLAY_MODES = {
  simulation: {
    label: "Simulation",
    status: "Simulation",
    title: "Top-view kinematics",
    description: "Live articulation, linkage, and clearance over the ideal steering core.",
    chip: "Live view",
  },
  clearance: {
    label: "Clearance focus",
    status: "Clearance focus",
    title: "Clearance focus",
    description: "Minimum-clearance envelopes and the active linkage state.",
    chip: "Clearance",
  },
  dimensions: {
    label: "Dimensioned sketch",
    status: "Sketch preview",
    title: "Dimensioned engineering sketch",
    description: "Before/after linkage comparison with dimension callouts and optimized overlays.",
    chip: "Sketch preview",
  },
};

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

function lineFromHeading(point, headingRad, lengthMm) {
  return {
    x2: point.x_mm + Math.cos(headingRad) * lengthMm,
    y2: toSvgY(point.y_mm + Math.sin(headingRad) * lengthMm),
  };
}

function renderWheelTable(axles) {
  wheelTable.replaceChildren();
  for (const axle of axles) {
    for (const side of ["left_wheel", "right_wheel"]) {
      const wheel = axle[side];
      const row = document.createElement("div");
      row.className = "wheel-row";
      const label = document.createElement("span");
      label.className = "label";
      label.textContent = `${axle.axle_id} ${wheel.side}`;
      const value = document.createElement("span");
      value.className = "value";
      value.textContent = formatAngle(wheel.heading_deg);
      row.append(label, value);
      wheelTable.appendChild(row);
    }
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
  dxfEntityCount.textContent = String(payload.entity_count ?? 0);
  dxfSupportedCount.textContent = String(payload.supported_entity_count ?? 0);
  dxfBoundsValue.textContent = bounds
    ? `${formatMillimeters(bounds.min_x_mm)} .. ${formatMillimeters(bounds.max_x_mm)} / ${formatMillimeters(bounds.min_y_mm)} .. ${formatMillimeters(bounds.max_y_mm)}`
    : "n/a";
  dxfLayoutValue.textContent = vehicle
    ? `${formatMillimeters(vehicle.body_length_mm)} x ${formatMillimeters(vehicle.body_width_mm)} | ${vehicle.axles?.length ?? 0} axles`
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
    meta.textContent = `${revision.id} | ${formatTimestamp(revision.created_at)} | beta ${Number(revision.beta_deg).toFixed(1)} deg | ${revision.optimization_mode}`;
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
  const query = `beta_deg=${encodeURIComponent(betaDeg)}&mode=${encodeURIComponent(mode)}`;
  exportJsonLink.href = `/api/export.json?${query}`;
  exportCsvLink.href = `/api/export.csv?${query}`;
  exportPdfLink.href = `/api/export.pdf?${query}`;
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
    dxfImportStatus.textContent = `Applied manual assignments for ${state.dxfImportSourceName || "DXF"}.`;
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
  const query = `beta_deg=${encodeURIComponent(betaDeg)}&mode=${encodeURIComponent(mode)}&step_deg=1`;
  steeringCurvesImage.src = `/api/steering-curves.svg?${query}`;
}

function refreshSweptPathPreview(betaDeg = Number(betaSlider.value)) {
  const mode = optimizeMode.value;
  const query = `beta_deg=${encodeURIComponent(betaDeg)}&mode=${encodeURIComponent(mode)}&step_deg=1`;
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
  const response = await fetch(`/api/export.svg?beta_deg=${encodeURIComponent(betaDeg)}&mode=${encodeURIComponent(mode)}`);
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
  if (state.displayMode === "dimensions") {
    await loadDimensionedSketch();
    return;
  }

  showSimulationDiagram();
  renderDiagram(payload, { showIcr: state.displayMode !== "clearance" });
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

  const body = payload.body_outline.map((point) => `${point.x_mm},${toSvgY(point.y_mm)}`).join(" ");
  diagram.appendChild(svgEl("polygon", {
    points: body,
    class: "body-outline",
  }));

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

    for (const wheel of [axle.left_wheel, axle.right_wheel]) {
      const center = toSvgPoint(wheel.center);
      const heading = lineFromHeading(wheel.center, wheel.heading_rad, 820);
      diagram.appendChild(svgEl("line", {
        x1: center.x,
        y1: center.y,
        x2: heading.x2,
        y2: heading.y2,
        class: "wheel-heading",
      }));
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

function updateSummary(payload) {
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
  renderWheelTable(payload.axles);
  if (payload.linkage) {
    linkageSteerValue.textContent = formatAngle(payload.linkage.state.steering_angle_deg);
    linkageErrorValue.textContent = formatAngle(payload.metrics.linkage_vs_ideal_front_axle_deg);
    linkageResidualValue.textContent = `${payload.linkage.state.input_stage_error_mm.toFixed(3)} / ${payload.linkage.state.tie_rod_error_mm.toFixed(3)} mm`;
    linkageBranchValue.textContent = `${payload.linkage.state.input_branch_index} / ${payload.linkage.state.steering_branch_index}`;
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

  refreshSteeringCurvesPreview(payload.beta_deg);
  refreshSweptPathPreview(payload.beta_deg);
}

function formatOptimizationMetric(metrics) {
  return metrics === null || metrics === undefined
    ? "n/a"
    : metrics.toFixed(2);
}

function updateOptimizationSummary(payload) {
  optimizeBaselineScore.textContent = payload.baseline ? payload.baseline.score.toFixed(2) : "n/a";
  optimizeOptimizedScore.textContent = payload.optimized ? payload.optimized.score.toFixed(2) : "n/a";
  optimizeBaselineRms.textContent = payload.baseline ? formatAngle(payload.baseline.rms_error_deg) : "n/a";
  optimizeOptimizedRms.textContent = payload.optimized ? formatAngle(payload.optimized.rms_error_deg) : "n/a";
  optimizeBaselineClearance.textContent = payload.baseline && payload.baseline.minimum_clearance_mm !== null
    ? formatDistance(payload.baseline.minimum_clearance_mm)
    : "n/a";
  optimizeOptimizedClearance.textContent = payload.optimized && payload.optimized.minimum_clearance_mm !== null
    ? formatDistance(payload.optimized.minimum_clearance_mm)
    : "n/a";
  optimizeRunStats.textContent = `${payload.mode} / ${payload.iterations} it / ${payload.evaluations} eval / ${payload.improved ? "improved" : "no change"}`;
  renderOptimizationVariables(payload.variables_after || []);
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
  betaSlider.value = String(activeRevision.beta_deg);
  optimizeMode.value = activeRevision.optimization_mode || "quick";
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
      optimization_mode: optimizeMode.value,
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
      optimization_mode: optimizeMode.value,
      note: projectNoteInput.value || "Revision",
    }),
  });
  const payload = await response.json();
  await renderProjectFromDetail(payload.project);
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
    const response = await fetch(`/api/optimize?mode=${encodeURIComponent(mode)}`);
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
  const response = await fetch(`/api/ideal-steering?beta_deg=${encodeURIComponent(betaDeg)}`);
  const payload = await response.json();
  if (requestId !== state.activeRequest) {
    return;
  }
  updateSummary(payload);
  updateExportLinks();
  await renderActiveView(payload);
}

betaSlider.addEventListener("input", (event) => {
  loadState(Number(event.target.value));
});

optimizeButton.addEventListener("click", () => {
  loadOptimization(optimizeMode.value);
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

async function initializeApp() {
  state.displayMode = Object.prototype.hasOwnProperty.call(DISPLAY_MODES, state.displayMode)
    ? state.displayMode
    : "simulation";
  syncDisplayModeUi();
  updateExportLinks();
  dxfApplyButton.disabled = true;
  dxfImportButton.textContent = "Import";
  refreshSteeringCurvesPreview();
  refreshSweptPathPreview();
  renderDxfEntities({ entities: [], role_options: getDxfRoleOptions() });
  dxfImportStatus.textContent = "Choose a DXF file to parse supported entities and rebuild a rough layout preview.";
  await refreshProjectPanel();
}

initializeApp();
