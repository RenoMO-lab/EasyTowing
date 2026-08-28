from pathlib import Path
import json
import os
import re
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from easytowing.acceptance import MonrocAcceptanceCriteria
from easytowing.demo_server import (
    _monroc_acceptance_profile_status,
    _parse_required_bool,
    _require_engineering_pass_for_approval,
    _should_seed_reference_project,
)


ROOT = Path(__file__).resolve().parents[1]


class WebWorkflowContractTests(unittest.TestCase):
    def test_every_navigation_step_has_a_progressive_panel(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        steps = set(re.findall(r'data-workflow-step="([^"]+)"', html))
        panels = set(re.findall(r'data-workflow-panel="([^"]+)"', html))
        self.assertEqual(
            steps,
            {"project", "vehicle", "maneuver", "mechanism", "validate", "optimize", "results"},
        )
        self.assertTrue(steps.issubset(panels))

    def test_multi_joint_validation_method_is_explained_in_the_ui(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        self.assertIn("Cartesian product of every configured joint range", html)
        self.assertIn("not a continuous proof between samples", html)
        self.assertNotIn('data-workflow-panel="vehicle">"', html)

    def test_results_explain_the_engineering_decision_and_auth_form(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn('id="results-decision-card"', html)
        self.assertIn("Can this design be released?", html)
        self.assertIn('id="auth-form"', html)
        self.assertIn("renderResultsDecision", javascript)
        self.assertIn("acceptanceIsReleaseApproved", javascript)
        self.assertIn("UNAPPROVED", javascript)
        self.assertIn('sweepValidationButton.disabled = !state.combinationActive || !state.mechanismGraph;', javascript)

    def test_bootstrap_endpoint_is_first_admin_only(self) -> None:
        server = (ROOT / "easytowing" / "demo_server.py").read_text(encoding="utf-8")
        self.assertIn("SaaSBootstrapError", server)
        self.assertIn("SAAS_CONTROL.bootstrap_admin", server)
        self.assertIn('"BOOTSTRAP_ALREADY_COMPLETED"', server)
        self.assertNotIn('role=UserRole(str(body.get("role"', server)

    def test_api_boolean_fields_do_not_use_truthiness_coercion(self) -> None:
        self.assertTrue(_parse_required_bool({"approved": True}, "approved"))
        self.assertFalse(_parse_required_bool({"approved": False}, "approved"))
        for value in ("false", "true", 0, 1, None):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "JSON boolean"):
                    _parse_required_bool({"approved": value}, "approved")

        server = (ROOT / "easytowing" / "demo_server.py").read_text(encoding="utf-8")
        self.assertIn("JSON request body must be an object.", server)
        self.assertIn("approved = _parse_required_bool(body, \"approved\")", server)

    def test_ui_exposes_reference_data_boundary_and_result_reading_guide(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        styles = (ROOT / "easytowing" / "web" / "styles.css").read_text(encoding="utf-8")
        self.assertIn("Simulation and reference data", html)
        self.assertIn("Not a manufacturing release", html)
        self.assertIn('class="decision-reading-guide"', html)
        self.assertIn("PASS</strong> means every configured hard check", html)
        self.assertIn("Physical feasibility checks", html)
        self.assertIn('id="current-steering-interpretation"', html)
        self.assertIn("STEERING CRITERION PENDING", html)
        self.assertIn(".trust-banner", styles)

    def test_results_label_ideal_actual_and_error_explicitly(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn('class="result-legend"', html)
        self.assertIn("Ideal</strong> is the geometric target", html)
        self.assertIn("Ideal: ${formatAngle(idealAngle)} / Actual:", javascript)
        self.assertIn("Error: ${formatAngle(error)}", javascript)

    def test_ideal_maneuver_output_is_separate_from_physical_mechanism_evidence(self) -> None:
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn('const idealOnly = state.combinationActive && !state.mechanismGraph;', javascript)
        self.assertIn('ideal_only: idealOnly', javascript)
        self.assertIn('payload.result_scope === "ideal_kinematics"', javascript)
        self.assertIn("Ideal targets only. Build and solve the mechanism", javascript)

    def test_release_checklist_distinguishes_pending_evidence_from_failure(self) -> None:
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn(
            "const currentChecksStatus = !hasRevision || state.workspaceDirty || !state.currentPayload",
            javascript,
        )
        self.assertIn('state.workspaceDirty ? "PENDING"', javascript)
        self.assertIn('state.workspaceDirty\n      ? "INCOMPLETE"', javascript)

    def test_combination_edits_clear_stale_maneuver_status(self) -> None:
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn(
            'combinationStatus.textContent = "Model changed. Resolve the maneuver before interpreting results."',
            javascript,
        )

    def test_workspace_explains_the_next_action_and_step_state(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn('id="workflow-next-action"', html)
        self.assertIn('id="workflow-next-button"', html)
        self.assertIn("function renderWorkflowProgress", javascript)
        self.assertIn("workflowStepStates", javascript)
        self.assertIn("button.dataset.workflowState = status", javascript)

    def test_restored_project_opens_on_the_first_incomplete_step(self) -> None:
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertGreaterEqual(javascript.count("setWorkflowStep(nextWorkflowAction().step);"), 2)

    def test_each_workflow_step_explains_inputs_outputs_and_release_rule(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn('id="workflow-guide-question"', html)
        self.assertIn('id="workflow-guide-steps"', html)
        self.assertIn('id="workflow-guide-result"', html)
        self.assertIn("const WORKFLOW_GUIDANCE", javascript)
        self.assertIn("What physical towing combination is being analyzed?", javascript)
        self.assertIn("function renderWorkflowGuide(step)", javascript)
        self.assertNotIn("workflowGuide.open = false", javascript)

    def test_next_action_button_executes_the_current_step_operation(self) -> None:
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn('action: "resolve-maneuver"', javascript)
        self.assertIn('activeButton: "Resolve maneuver"', javascript)
        self.assertIn('action: "confirm-vehicle-layout"', javascript)
        self.assertIn('activeButton: "Confirm vehicle layout"', javascript)
        self.assertIn("function runWorkflowNextAction(action)", javascript)
        self.assertIn('workflowNextButton.dataset.workflowAction = actionIsAvailable', javascript)
        self.assertIn("runWorkflowNextAction(action);", javascript)

    def test_legacy_controls_are_hidden_when_multi_body_mode_is_active(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn('class="wheel-table-card geometry-card legacy-geometry-card"', html)
        self.assertIn('class="wheel-table-card linkage-card legacy-linkage-card"', html)
        self.assertIn("legacyGeometryCard.hidden = active", javascript)
        self.assertIn("legacyLinkageCard.hidden = active", javascript)
        self.assertIn('results: legacyRevision ? "WAIT" : resultState', javascript)
        self.assertIn('"INCOMPLETE"', javascript)

    def test_project_dashboard_exposes_engineering_review_and_model_scope(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        for element_id in (
            "project-engineering-state",
            "project-review-state",
            "project-model-scope",
        ):
            self.assertIn(f'id="{element_id}"', html)
        self.assertIn("function renderProjectDashboardStatus", javascript)
        self.assertIn("Current pose and required range checks pass.", javascript)
        self.assertIn("Explicit articulated combination is active.", javascript)

    def test_project_dashboard_explains_inputs_calculation_and_decision(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        styles = (ROOT / "easytowing" / "web" / "styles.css").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn('id="project-start-card"', html)
        self.assertIn("Define a steering mechanism for a towing combination", html)
        self.assertIn("Ideal versus actual", html)
        self.assertIn("PASS or FAIL", html)
        self.assertIn('id="workflow-guide"', html)
        self.assertIn("function renderProjectStartCard", javascript)
        self.assertIn("isLegacyRevisionMode", javascript)
        self.assertIn(".project-start-card", styles)

    def test_validation_separates_physical_feasibility_from_steering_acceptance(self) -> None:
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn("function renderCurrentSteeringInterpretation", javascript)
        self.assertIn("physical feasibility PASS is not steering acceptance", javascript)
        self.assertIn('currentSteeringStatus.textContent = "STEERING CRITERION PASS"', javascript)
        self.assertIn('currentSteeringStatus.textContent = "STEERING CRITERION FAIL"', javascript)
        self.assertIn(
            'steeringCheck?.status === "FAIL" && acceptance?.criteria_approval?.status === "APPROVED"',
            javascript,
        )
        self.assertIn('acceptance?.criteria_approval?.status === "APPROVED"', javascript)

    def test_legacy_revision_is_a_migration_action_not_a_completed_workflow(self) -> None:
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn('title: "Switch this revision to multi-body workflow"', javascript)
        self.assertIn('engineeringStatus = "LEGACY"', javascript)
        self.assertIn('projectModelDetail.textContent = "Legacy single-layout study.', javascript)
        self.assertIn('combinationActivateButton.click();', javascript)

    def test_new_workspace_defaults_to_the_explicit_multi_body_path(self) -> None:
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn("combinationActive: true", javascript)
        self.assertIn("legacy revision still switches this back to false", javascript)

    def test_new_workspace_does_not_auto_open_seeded_reference_data(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn('value="New Monroc steering study"', html)
        self.assertIn('newStudy.textContent = "Start a new study"', javascript)
        self.assertIn("const projectId = preferredProjectBelongsToWorkspace ? preferredProjectId : null;", javascript)
        self.assertIn("renderProjectSummary(null);", javascript)
        self.assertIn("window.location.reload();", javascript)

    def test_vehicle_confirmation_gates_the_downstream_workflow(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn('id="combination-confirm-button"', html)
        self.assertIn("function hasConfirmedVehicleDefinition", javascript)
        self.assertIn("state.vehicleDefinitionConfirmed = false", javascript)
        self.assertIn("serializedCombination();", javascript)
        self.assertIn("Vehicle layout confirmed. Resolve the maneuver", javascript)
        self.assertIn("state.vehicleDefinitionConfirmed = true", javascript)

    def test_combination_editor_preserves_body_tree_connections(self) -> None:
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn('parentLabel.textContent = "Connected to / parent"', javascript)
        self.assertIn("parentBodyId", javascript)
        self.assertIn("childJointsByParent", javascript)
        self.assertIn("Stored combination contains a body connection cycle.", javascript)
        self.assertIn("Stored combination contains a disconnected body or missing parent joint.", javascript)
        self.assertIn("function primaryCombinationJointId", javascript)

    def test_legacy_revision_mode_is_explicit_before_multi_body_activation(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn('id="combination-mode-state"', html)
        self.assertIn('id="combination-activate-button"', html)
        self.assertIn("LEGACY REVISION", javascript)
        self.assertIn("Use multi-body workflow", html)
        self.assertIn("state.combinationActive = true;", javascript)

    def test_workflow_surface_tokens_are_defined(self) -> None:
        styles = (ROOT / "easytowing" / "web" / "styles.css").read_text(encoding="utf-8")
        for token in ("--surface:", "--surface-strong:", "--ink:", "--accent-soft:", "--display-font:"):
            self.assertIn(token, styles)

    def test_mobile_workflow_navigation_keeps_all_steps_visible(self) -> None:
        styles = (ROOT / "easytowing" / "web" / "styles.css").read_text(encoding="utf-8")
        self.assertIn("@media (max-width: 600px)", styles)
        self.assertIn("grid-template-columns: repeat(2, minmax(0, 1fr));", styles)
        self.assertIn("overflow: visible;", styles)
        self.assertIn(".metric-card strong", styles)
        self.assertIn("overflow-wrap: anywhere;", styles)

    def test_mobile_workflow_explains_the_task_before_showing_the_canvas(self) -> None:
        styles = (ROOT / "easytowing" / "web" / "styles.css").read_text(encoding="utf-8")
        self.assertIn("display: flex;", styles)
        self.assertIn(".info-panel {\n    order: 1;", styles)
        self.assertIn(".canvas-card {\n    order: 2;", styles)
        self.assertIn(".combination-card {\n  order: 1;", styles)

    def test_multi_body_outline_input_has_an_explicit_rectangular_fallback(self) -> None:
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        styles = (ROOT / "easytowing" / "web" / "styles.css").read_text(encoding="utf-8")
        self.assertIn("function parseBodyPolygonText", javascript)
        self.assertIn("Use rectangular envelope", javascript)
        self.assertIn("invalid CAD outline", javascript)
        self.assertIn("body-outline-editor", styles)

    def test_multi_wheel_axle_inputs_and_wheel_end_mapping_are_exposed(self) -> None:
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        self.assertIn("wheel_lateral_offsets_mm", javascript)
        self.assertIn("function wheelIdsForAxle", javascript)
        self.assertIn("wheelId.includes(\"_left\")", javascript)
        self.assertIn("const wheels = Array.isArray(axle.wheels)", javascript)
        self.assertIn("const wheels = Array.isArray(payloadAxle.wheels)", javascript)
        self.assertIn("one output may drive a wheel end", html)

    def test_multi_vehicle_mapping_uses_engineer_facing_terminology(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        styles = (ROOT / "easytowing" / "web" / "styles.css").read_text(encoding="utf-8")
        self.assertIn("Vehicle / trailer combination", html)
        self.assertIn("How to map your combination", html)
        self.assertIn("Vehicles / trailers", html)
        self.assertIn("root reference", javascript)
        self.assertIn("Connected to / parent", javascript)
        self.assertIn(".combination-model-guide", styles)

    def test_dxf_activation_requires_units_and_coordinate_frame_confirmation(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        server = (ROOT / "easytowing" / "demo_server.py").read_text(encoding="utf-8")
        dxf_import = (ROOT / "easytowing" / "dxf_import.py").read_text(encoding="utf-8")
        self.assertIn("CAD source confirmation", html)
        self.assertIn('id="dxf-source-units"', html)
        self.assertIn('id="dxf-coordinate-system"', html)
        self.assertIn("source_units: dxfSourceUnits.value", javascript)
        self.assertIn("coordinate_system: dxfCoordinateSystem.value", javascript)
        self.assertIn("confirm_metadata: true", javascript)
        self.assertIn('"CAD activation requires confirmed source units', server)
        self.assertIn("DXF_UNIT_OPTIONS", dxf_import)
        self.assertIn("source_sha256", dxf_import)
        self.assertIn("unsupported DXF", javascript)
        self.assertIn("unsupportedCount > 0", javascript)
        self.assertIn("const layoutReady = Boolean(state.dxfImportPayload?.reconstructed_vehicle)", javascript)
        self.assertIn("No valid vehicle layout was reconstructed", javascript)

    def test_diagnostic_previews_render_structured_failures_in_page(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn('id="steering-curves-status"', html)
        self.assertIn('id="swept-path-status"', html)
        self.assertIn("async function loadDiagnosticPreview", javascript)
        self.assertIn("Resolve the current engineering checks to regenerate it.", javascript)

    def test_multi_body_diagnostic_exports_are_available_in_the_saved_revision(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        server = (ROOT / "easytowing" / "demo_server.py").read_text(encoding="utf-8")
        for extension in ("png", "svg", "dxf"):
            self.assertIn(f'id="export-{extension}"', html)
            self.assertIn(f'[export{extension.title()}Link, "{extension}"]', javascript)
            self.assertIn(f'"export.{extension}"', server)
        self.assertIn("Diagnostic JSON, CSV, PDF, SVG, DXF, and PNG are available", javascript)

    def test_controlled_artifact_storage_and_download_contract_is_exposed(self) -> None:
        server = (ROOT / "easytowing" / "demo_server.py").read_text(encoding="utf-8")
        saas = (ROOT / "easytowing" / "saas.py").read_text(encoding="utf-8")
        self.assertIn("EASYTOWING_ARTIFACT_STORAGE_DIR", server)
        self.assertIn("EASYTOWING_REQUIRE_ARTIFACT_STORAGE", server)
        self.assertIn('"ARTIFACT_NOT_RETAINED"', server)
        self.assertIn("len(parts) == 7", server)
        self.assertIn("class FileArtifactStore", saas)
        self.assertIn("class S3ArtifactStore", saas)
        self.assertIn("expected_sha256", saas)

    def test_revision_scoped_cad_source_retention_contract_is_exposed(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        server = (ROOT / "easytowing" / "demo_server.py").read_text(encoding="utf-8")
        self.assertIn('id="dxf-retain-source-button"', html)
        self.assertIn('id="dxf-source-retention-status"', html)
        self.assertIn("async function retainDxfSource", javascript)
        self.assertIn("/cad-source`", javascript)
        self.assertIn('artifactStorageBackend,', javascript)
        self.assertIn('"response-only", "unavailable"', javascript)
        self.assertIn('"cad-source-dxf"', server)
        self.assertIn("reconstructed_vehicle?.cad_source", javascript)
        self.assertIn("CAD_SOURCE_MISMATCH", server)
        self.assertIn("MAX_CAD_SOURCE_BYTES", server)
        self.assertIn("_send_download(lambda: content, content_type, artifact.filename)", server)

    def test_reviewer_assignment_contract_is_exposed(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        server = (ROOT / "easytowing" / "demo_server.py").read_text(encoding="utf-8")
        saas = (ROOT / "easytowing" / "saas.py").read_text(encoding="utf-8")
        schema = (ROOT / "easytowing" / "postgres_schema.sql").read_text(encoding="utf-8")
        self.assertIn('id="reviewer-selector"', html)
        self.assertIn('id="reviewer-assign-button"', html)
        self.assertIn("assignReviewer", javascript)
        self.assertIn('parts[5] == "reviewer"', server)
        self.assertIn("assigned_reviewer_id", saas)
        self.assertIn("REVIEWER_ASSIGNED", saas)
        self.assertIn("REVIEWER_UNASSIGNED", saas)
        self.assertIn("ADD COLUMN IF NOT EXISTS assigned_reviewer_id", schema)

    def test_admin_user_provisioning_contract_is_exposed(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        server = (ROOT / "easytowing" / "demo_server.py").read_text(encoding="utf-8")
        saas = (ROOT / "easytowing" / "saas.py").read_text(encoding="utf-8")
        self.assertIn('id="user-provisioning"', html)
        self.assertIn('id="user-create-form"', html)
        self.assertIn('id="user-create-role"', html)
        self.assertIn("async function createUser", javascript)
        self.assertIn('fetch("/api/users"', javascript)
        self.assertIn('parsed.path == "/api/users"', server)
        self.assertIn("created_by=principal", server)
        self.assertIn("Users can only be created in the administrator's organization.", saas)
        self.assertIn("actor_user_id=created_by.user_id", saas)

    def test_postgres_release_metadata_is_revision_scoped_and_locked(self) -> None:
        saas = (ROOT / "easytowing" / "saas.py").read_text(encoding="utf-8")
        schema = (ROOT / "easytowing" / "postgres_schema.sql").read_text(encoding="utf-8")
        self.assertGreaterEqual(saas.count("FOR UPDATE"), 2)
        self.assertIn("Revision does not exist for this project.", saas)
        self.assertGreaterEqual(
            schema.count("FOREIGN KEY (organization_id, project_id, revision_id)"),
            2,
        )

    def test_postgres_schema_enforces_tenant_scoped_relationships(self) -> None:
        schema = (ROOT / "easytowing" / "postgres_schema.sql").read_text(encoding="utf-8")
        for relationship in (
            "FOREIGN KEY (organization_id, user_id) REFERENCES users(organization_id, id)",
            "FOREIGN KEY (organization_id, project_id) REFERENCES projects(organization_id, id)",
            "FOREIGN KEY (organization_id, project_id, revision_id)",
            "FOREIGN KEY (organization_id, assigned_reviewer_id) REFERENCES users(organization_id, id)",
            "FOREIGN KEY (organization_id, submitted_by) REFERENCES users(organization_id, id)",
            "FOREIGN KEY (organization_id, created_by) REFERENCES users(organization_id, id)",
            "FOREIGN KEY (organization_id, actor_user_id) REFERENCES users(organization_id, id)",
        ):
            self.assertIn(relationship, schema)
        self.assertIn("projects_active_revision_tenant_fk", schema)
        self.assertIn("DEFERRABLE INITIALLY DEFERRED", schema)
        self.assertIn("-- Add the tenant-scoped keys and relationships", schema)
        self.assertIn("CREATE OR REPLACE FUNCTION reject_append_only_mutation()", schema)
        self.assertIn("audit_events_append_only", schema)
        self.assertIn("artifact_records_append_only", schema)

    def test_combination_solve_refreshes_full_range_validation_control(self) -> None:
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn("renderSweepValidation(state.sweepValidationPayload);", javascript)

    def test_combination_save_carries_the_configured_clearance_target(self) -> None:
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn(
            'clearance_target_mm: Number(state.optimizationSettings.clearanceTargetMm)',
            javascript,
        )

    def test_server_exposes_a_liveness_endpoint(self) -> None:
        server = (ROOT / "easytowing" / "demo_server.py").read_text(encoding="utf-8")
        self.assertIn('if parsed.path == "/api/health":', server)
        self.assertIn('"status": "ok"', server)
        self.assertIn('"artifact_storage":', server)

    def test_server_exposes_a_database_readiness_endpoint(self) -> None:
        server = (ROOT / "easytowing" / "demo_server.py").read_text(encoding="utf-8")
        projects = (ROOT / "easytowing" / "projects.py").read_text(encoding="utf-8")
        self.assertIn('if parsed.path == "/api/ready":', server)
        self.assertIn('"status": "not_ready"', server)
        self.assertIn('"artifact_storage_required"', server)
        self.assertIn('"EASYTOWING_REQUIRE_WORKER"', server)
        self.assertIn('SAAS_CONTROL.worker_health', server)
        self.assertIn('"worker_max_age_seconds"', server)
        self.assertIn('def health_check(self)', projects)

    def test_postgres_project_list_does_not_seed_reference_customer_data(self) -> None:
        server = (ROOT / "easytowing" / "demo_server.py").read_text(encoding="utf-8")
        self.assertIn("_should_seed_reference_project(DATABASE_URL)", server)
        self.assertIn("reference data cannot be mistaken for a", server)
        self.assertTrue(_should_seed_reference_project(None))
        self.assertTrue(_should_seed_reference_project("  "))
        self.assertFalse(_should_seed_reference_project("postgresql://db/easytowing"))

    def test_postgres_worker_heartbeat_schema_and_append_only_guards_exist(self) -> None:
        schema = (ROOT / "easytowing" / "postgres_schema.sql").read_text(encoding="utf-8")
        saas = (ROOT / "easytowing" / "saas.py").read_text(encoding="utf-8")
        self.assertIn("CREATE TABLE IF NOT EXISTS worker_heartbeats", schema)
        self.assertIn("idx_worker_heartbeats_last_seen", schema)
        self.assertIn("def record_worker_heartbeat", saas)
        self.assertIn("def worker_health", saas)
        self.assertIn("status=\"running\"", saas)

    def test_mechanism_graph_editor_supports_shared_components_and_invalidates_evidence(self) -> None:
        html = (ROOT / "easytowing" / "web" / "index.html").read_text(encoding="utf-8")
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn("Edit graph topology and outputs", html)
        self.assertIn("Shared point IDs connect components", html)
        self.assertIn("generated graph is shown as a summary first", html)
        self.assertIn("function readMechanismGraphEditor", javascript)
        self.assertIn("function mechanismBodyOptions", javascript)
        self.assertIn("Global / no body", javascript)
        self.assertIn("resetEngineeringEvidence", javascript)
        self.assertIn("state.workspaceDirty = true", javascript)
        self.assertIn("mechanismGraphEditor.open = false", javascript)

    def test_mechanism_build_preserves_maneuver_but_never_stale_evidence(self) -> None:
        javascript = (ROOT / "easytowing" / "web" / "app.js").read_text(encoding="utf-8")

        self.assertIn("maneuverResolved: false", javascript)
        self.assertIn("{ preserveManeuver: true }", javascript)
        self.assertIn("const hasSolvedMechanism", javascript)
        self.assertIn("resetEngineeringEvidence(summary);", javascript)

    def test_approval_requires_explicit_complete_full_range_evidence(self) -> None:
        revision = SimpleNamespace(
            combination_config={"id": "combination"},
            accepted_optimization=False,
            snapshot={
                "monroc_acceptance": {"result": {"status": "PASS"}},
                "combination_kinematics": {"maximum_constraint_residual_mm": 0.0},
                "mechanism_graph": {"state": {"maximum_residual_mm": 0.0}},
                "clearance": {"collision_detected": False, "minimum_clearance_mm": 25.0},
                "sweep_validation": {
                    "status": "PASS",
                    "sample_count": 1,
                    "solved_sample_count": 1,
                    "violations": [],
                },
            },
        )

        with self.assertRaisesRegex(ValueError, "full-range"):
            _require_engineering_pass_for_approval(revision)

    def test_approval_uses_the_saved_clearance_target(self) -> None:
        revision = SimpleNamespace(
            combination_config={"id": "combination"},
            accepted_optimization=False,
            snapshot={
                "monroc_acceptance": {"result": {"status": "PASS"}},
                "combination_kinematics": {"maximum_constraint_residual_mm": 0.0},
                "mechanism_graph": {"state": {"maximum_residual_mm": 0.0}},
                "clearance": {"collision_detected": False, "minimum_clearance_mm": 25.0},
                "sweep_validation": {
                    "status": "PASS",
                    "sampling_complete": True,
                    "sample_count": 1,
                    "solved_sample_count": 1,
                    "clearance_target_mm": 30.0,
                    "violations": [],
                },
            },
        )

        with self.assertRaisesRegex(ValueError, "full-range"):
            _require_engineering_pass_for_approval(revision)

    def test_acceptance_profile_requires_exact_protected_criteria_match(self) -> None:
        criteria = MonrocAcceptanceCriteria(
            case_id="MONROC-01",
            minimum_clearance_mm=20.0,
            maximum_wheel_error_deg=2.0,
            maximum_synchronization_error_deg=1.0,
        )
        configured = {
            "monroc:MONROC-01": criteria.to_dict(),
        }
        with patch.dict(
            os.environ,
            {"EASYTOWING_MONROC_ACCEPTANCE_PROFILES_JSON": json.dumps(configured)},
        ):
            approved = _monroc_acceptance_profile_status("monroc", criteria)
            changed = _monroc_acceptance_profile_status(
                "monroc",
                MonrocAcceptanceCriteria(
                    case_id="MONROC-01",
                    minimum_clearance_mm=25.0,
                    maximum_wheel_error_deg=2.0,
                    maximum_synchronization_error_deg=1.0,
                ),
            )
        self.assertEqual(approved["status"], "APPROVED")
        self.assertEqual(changed["status"], "UNAPPROVED")

    def test_approval_requires_a_server_approved_acceptance_profile(self) -> None:
        revision = SimpleNamespace(
            combination_config={"id": "combination"},
            accepted_optimization=False,
            snapshot={
                "monroc_acceptance": {
                    "result": {"status": "PASS"},
                    "criteria_approval": {"status": "UNAPPROVED"},
                },
                "combination_kinematics": {"maximum_constraint_residual_mm": 0.0},
                "mechanism_graph": {"state": {"maximum_residual_mm": 0.0}},
                "clearance": {"collision_detected": False, "minimum_clearance_mm": 25.0},
                "sweep_validation": {
                    "status": "PASS",
                    "sampling_complete": True,
                    "sample_count": 1,
                    "solved_sample_count": 1,
                    "clearance_target_mm": 20.0,
                    "violations": [],
                },
            },
        )

        with self.assertRaisesRegex(ValueError, "approved Monroc acceptance profile"):
            _require_engineering_pass_for_approval(revision)


if __name__ == "__main__":
    unittest.main()
