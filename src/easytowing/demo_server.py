from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import argparse
import json
import math
from urllib.parse import parse_qs, urlparse

from .collision import (
    CapsuleEnvelope,
    CircleEnvelope,
    CollisionItem,
    ClearancePair,
    ClearanceReport,
    PolygonEnvelope,
    analyze_clearance,
)
from .geometry import Point2D
from .dxf_import import analyze_dxf_import, apply_dxf_role_overrides, serialize_dxf_import_report
from .linkage import build_reference_linkage_demo, solve_reference_linkage_demo
from .optimization import (
    OptimizationMetrics,
    OptimizedVariable,
    build_reference_optimization_problem,
    optimize_linkage_problem,
)
from .projects import ProjectStore, serialize_project, serialize_revision
from .reporting import (
    build_dimensioned_svg,
    build_export_bundle,
    build_export_csv,
    build_export_dxf,
    build_export_pdf,
    build_steering_curves_svg,
    build_swept_path_svg,
)
from .steering import build_demo_solution

WEB_DIR = Path(__file__).resolve().parent / "web"
PROJECT_STORE = ProjectStore.default()


def _point_payload(point: Point2D) -> dict[str, float]:
    return {"x_mm": point.x_mm, "y_mm": point.y_mm}


def _wheel_payload(wheel_solution) -> dict[str, object]:
    return {
        "wheel_id": wheel_solution.wheel_id,
        "axle_id": wheel_solution.axle_id,
        "side": wheel_solution.side,
        "center": _point_payload(wheel_solution.center),
        "heading_rad": wheel_solution.heading_rad,
        "heading_deg": wheel_solution.heading_deg,
    }


def _axle_payload(axle_solution) -> dict[str, object]:
    return {
        "axle_id": axle_solution.axle_id,
        "center": _point_payload(axle_solution.center),
        "center_heading_rad": axle_solution.center_heading_rad,
        "center_heading_deg": axle_solution.center_heading_deg,
        "left_wheel": _wheel_payload(axle_solution.left_wheel),
        "right_wheel": _wheel_payload(axle_solution.right_wheel),
    }


def _envelope_payload(envelope) -> dict[str, object]:
    if isinstance(envelope, CircleEnvelope):
        return {
            "kind": "circle",
            "center": _point_payload(envelope.center),
            "radius_mm": envelope.radius_mm,
        }
    if isinstance(envelope, CapsuleEnvelope):
        return {
            "kind": "capsule",
            "start": _point_payload(envelope.start),
            "end": _point_payload(envelope.end),
            "radius_mm": envelope.radius_mm,
        }
    if isinstance(envelope, PolygonEnvelope):
        return {
            "kind": "polygon",
            "points": [_point_payload(point) for point in envelope.points],
        }
    raise TypeError(f"Unsupported envelope type: {type(envelope)!r}")


def _collision_item_payload(item: CollisionItem) -> dict[str, object]:
    return {
        "id": item.id,
        "margin_mm": item.margin_mm,
        "envelope": _envelope_payload(item.envelope),
    }


def _clearance_pair_payload(pair: ClearancePair) -> dict[str, object]:
    return {
        "item_a_id": pair.item_a_id,
        "item_b_id": pair.item_b_id,
        "raw_clearance_mm": pair.raw_clearance_mm,
        "required_margin_mm": pair.required_margin_mm,
        "clearance_mm": pair.clearance_mm,
        "overlaps": pair.overlaps,
        "violates_margin": pair.violates_margin,
        "description": pair.description,
    }


def _clearance_report_payload(report: ClearanceReport) -> dict[str, object]:
    return {
        "minimum_clearance_mm": report.minimum_clearance_mm,
        "collision_detected": report.collision_detected,
        "clearance_violation_detected": report.clearance_violation_detected,
        "items": [_collision_item_payload(item) for item in report.items],
        "pairs": [_clearance_pair_payload(pair) for pair in report.pairs],
        "minimum_pair": None if report.minimum_pair is None else _clearance_pair_payload(report.minimum_pair),
    }


def _optimization_metrics_payload(metrics: OptimizationMetrics) -> dict[str, object]:
    return {
        "score": metrics.score,
        "rms_error_deg": metrics.rms_error_deg,
        "mean_abs_error_deg": metrics.mean_abs_error_deg,
        "max_abs_error_deg": metrics.max_abs_error_deg,
        "minimum_clearance_mm": metrics.minimum_clearance_mm,
        "failure_index": metrics.failure_index,
        "solved_samples": metrics.solved_samples,
        "sample_count": metrics.sample_count,
    }


def _optimized_variable_payload(variable: OptimizedVariable) -> dict[str, object]:
    return {
        "id": variable.id,
        "current": variable.current,
        "minimum": variable.minimum,
        "maximum": variable.maximum,
        "enabled": variable.enabled,
        "preferred": variable.preferred,
        "optimized": variable.optimized,
        "delta": variable.delta,
    }


def _optimization_payload(mode: str) -> dict[str, object]:
    problem = build_reference_optimization_problem(mode=mode)
    result = optimize_linkage_problem(problem)
    return {
        "mode": result.mode,
        "iterations": result.iterations,
        "evaluations": result.evaluations,
        "improved": result.improved,
        "improvement": result.improvement,
        "baseline": _optimization_metrics_payload(result.baseline_metrics),
        "optimized": _optimization_metrics_payload(result.optimized_metrics),
        "variables_before": [_optimized_variable_payload(variable) for variable in result.baseline_variables],
        "variables_after": [_optimized_variable_payload(variable) for variable in result.optimized_variables],
    }


def _dxf_import_payload(dxf_text: str, source_name: str = "") -> dict[str, object]:
    report = analyze_dxf_import(dxf_text, source_name=source_name)
    return serialize_dxf_import_report(report)


def _parse_role_overrides(raw_overrides) -> dict[int, str | None]:
    if raw_overrides is None:
        return {}
    if not isinstance(raw_overrides, dict):
        raise ValueError("role_overrides must be an object mapping entity indexes to roles")

    parsed: dict[int, str | None] = {}
    for key, value in raw_overrides.items():
        try:
            index = int(key)
        except (TypeError, ValueError) as exc:
            raise ValueError("role_overrides keys must be numeric entity indexes") from exc
        if value is None:
            parsed[index] = None
        elif isinstance(value, str):
            parsed[index] = value
        else:
            raise ValueError("role_overrides values must be strings or null")
    return parsed


def _project_summary_payload(project) -> dict[str, object]:
    return serialize_project(project, include_snapshots=False)


def _project_detail_payload(project) -> dict[str, object]:
    return serialize_project(project, include_snapshots=True)


def _revision_payload(revision, include_snapshot: bool = False) -> dict[str, object]:
    return serialize_revision(revision, include_snapshot=include_snapshot)


def _linkage_payload(rig, state) -> dict[str, object]:
    return {
        "driver_point": _point_payload(state.driver_point),
        "spec": {
            "id": rig.spec.id,
            "bell_crank_pivot": _point_payload(rig.spec.bell_crank_pivot),
            "steering_pivot": _point_payload(rig.spec.steering_pivot),
            "input_rod_length_mm": rig.spec.input_rod_length_mm,
            "tie_rod_length_mm": rig.spec.tie_rod_length_mm,
            "bell_crank_input_arm_length_mm": rig.spec.bell_crank_input_arm_length_mm,
            "bell_crank_output_arm_length_mm": rig.spec.bell_crank_output_arm_length_mm,
            "steering_arm_length_mm": rig.spec.steering_arm_length_mm,
        },
        "state": {
            "input_endpoint": _point_payload(state.input_endpoint),
            "output_endpoint": _point_payload(state.output_endpoint),
            "steering_endpoint": _point_payload(state.steering_endpoint),
            "bell_crank_angle_rad": state.bell_crank_angle_rad,
            "bell_crank_angle_deg": state.bell_crank_angle_deg,
            "steering_angle_rad": state.steering_angle_rad,
            "steering_angle_deg": state.steering_angle_deg,
            "input_stage_error_mm": state.input_stage_error_mm,
            "tie_rod_error_mm": state.tie_rod_error_mm,
            "input_branch_index": state.input_branch_index,
            "steering_branch_index": state.steering_branch_index,
        },
    }


def _project_state_payload() -> dict[str, object]:
    projects = PROJECT_STORE.list_projects()
    active_project = next((project for project in projects if project.active_revision_id is not None), None)
    return {
        "projects": [_project_summary_payload(project) for project in projects],
        "active_project_id": None if active_project is None else active_project.id,
        "active_project": None if active_project is None else _project_detail_payload(active_project),
    }


def _clearance_payload(vehicle, rig, state) -> dict[str, object]:
    rear_axle = next(axle for axle in vehicle.axles if axle.id == "rear_axle")
    front_axle = next(axle for axle in vehicle.axles if axle.id == "front_axle")
    rear_left, rear_right = rear_axle.wheels()
    front_left, front_right = front_axle.wheels()

    items = (
        CollisionItem(
            id="input_rod",
            envelope=CapsuleEnvelope(
                start=state.driver_point,
                end=state.input_endpoint,
                radius_mm=14.0,
            ),
        ),
        CollisionItem(
            id="tie_rod",
            envelope=CapsuleEnvelope(
                start=state.output_endpoint,
                end=state.steering_endpoint,
                radius_mm=14.0,
            ),
        ),
        CollisionItem(
            id="bell_crank_pivot",
            envelope=CircleEnvelope(
                center=rig.spec.bell_crank_pivot,
                radius_mm=28.0,
            ),
        ),
        CollisionItem(
            id="steering_pivot",
            envelope=CircleEnvelope(
                center=rig.spec.steering_pivot,
                radius_mm=28.0,
            ),
        ),
        CollisionItem(
            id="front_axle_beam",
            envelope=CapsuleEnvelope(
                start=front_left.center,
                end=front_right.center,
                radius_mm=70.0,
            ),
        ),
        CollisionItem(
            id="rear_axle_beam",
            envelope=CapsuleEnvelope(
                start=rear_left.center,
                end=rear_right.center,
                radius_mm=70.0,
            ),
        ),
    )
    report = analyze_clearance(items)
    return _clearance_report_payload(report)


def build_demo_payload(beta_deg: float) -> dict[str, object]:
    vehicle, solution, radius = build_demo_solution(beta_deg)
    rig = build_reference_linkage_demo()
    linkage_state = solve_reference_linkage_demo(beta_deg)
    linkage = _linkage_payload(rig, linkage_state)
    clearance = _clearance_payload(vehicle, rig, linkage_state)
    body_half_length = vehicle.body_length_mm / 2.0
    body_half_width = vehicle.body_width_mm / 2.0

    body_outline = [
        {"x_mm": -body_half_length, "y_mm": -body_half_width},
        {"x_mm": body_half_length, "y_mm": -body_half_width},
        {"x_mm": body_half_length, "y_mm": body_half_width},
        {"x_mm": -body_half_length, "y_mm": body_half_width},
    ]

    wheel_angles_deg = solution.wheel_angles_deg()
    axle_angles_deg = solution.axle_center_angles_deg()

    metrics = {
        "max_abs_wheel_angle_deg": max((abs(value) for value in wheel_angles_deg.values()), default=0.0),
        "front_axle_center_angle_deg": axle_angles_deg.get("front_axle"),
        "rear_axle_center_angle_deg": axle_angles_deg.get("rear_axle"),
        "linkage_actual_steering_deg": linkage["state"]["steering_angle_deg"],
        "linkage_vs_ideal_front_axle_deg": None,
        "minimum_clearance_mm": clearance["minimum_clearance_mm"],
    }

    if "front_axle" in axle_angles_deg and "rear_axle" in axle_angles_deg:
        metrics["front_rear_phase_deg"] = axle_angles_deg["front_axle"] - axle_angles_deg["rear_axle"]
    else:
        metrics["front_rear_phase_deg"] = None

    if metrics["front_axle_center_angle_deg"] is not None:
        metrics["linkage_vs_ideal_front_axle_deg"] = (
            linkage["state"]["steering_angle_deg"] - metrics["front_axle_center_angle_deg"]
        )

    return {
        "beta_deg": beta_deg,
        "beta_rad": math.radians(beta_deg),
        "turn_radius_mm": radius,
        "icr": None if solution.icr is None else _point_payload(solution.icr),
        "vehicle": {
            "id": vehicle.id,
            "name": vehicle.name,
            "body_length_mm": vehicle.body_length_mm,
            "body_width_mm": vehicle.body_width_mm,
            "origin": _point_payload(vehicle.origin),
        },
        "body_outline": body_outline,
        "axles": [_axle_payload(axle_solution) for axle_solution in solution.axles],
        "linkage": linkage,
        "clearance": clearance,
        "metrics": metrics,
    }


class DemoRequestHandler(BaseHTTPRequestHandler):
    server_version = "EasyTowingDemo/0.1"

    def _send_bytes(self, content: bytes, content_type: str, status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _send_file(self, filename: str, content_type: str) -> None:
        path = WEB_DIR / filename
        if not path.exists():
            self.send_error(404, "File not found")
            return
        self._send_bytes(path.read_bytes(), content_type)

    def _send_json(self, payload: dict[str, object], status: int = 200) -> None:
        content = json.dumps(payload, indent=2).encode("utf-8")
        self._send_bytes(content, "application/json; charset=utf-8", status=status)

    def _read_json_body(self) -> dict[str, object]:
        content_length = self.headers.get("Content-Length", "0")
        try:
            length = int(content_length)
        except ValueError as exc:
            raise ValueError("Invalid Content-Length header") from exc
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        if not raw:
            return {}
        return json.loads(raw.decode("utf-8"))

    def log_message(self, format: str, *args) -> None:  # noqa: A003 - match BaseHTTPRequestHandler
        return

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler interface
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self._send_file("index.html", "text/html; charset=utf-8")
            return
        if parsed.path == "/favicon.ico":
            self.send_response(204)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        if parsed.path == "/app.js":
            self._send_file("app.js", "application/javascript; charset=utf-8")
            return
        if parsed.path == "/styles.css":
            self._send_file("styles.css", "text/css; charset=utf-8")
            return
        if parsed.path == "/api/ideal-steering":
            query = parse_qs(parsed.query)
            beta_values = query.get("beta_deg", ["0"])
            try:
                beta_deg = float(beta_values[0])
            except ValueError:
                self.send_error(400, "beta_deg must be numeric")
                return
            payload = build_demo_payload(beta_deg)
            self._send_json(payload)
            return
        if parsed.path == "/api/optimize":
            query = parse_qs(parsed.query)
            mode = query.get("mode", ["quick"])[0]
            if mode not in {"quick", "full"}:
                self.send_error(400, "mode must be quick or full")
                return
            payload = _optimization_payload(mode)
            self._send_json(payload)
            return
        if parsed.path == "/api/import.dxf":
            self.send_error(405, "POST required")
            return
        if parsed.path in {
            "/api/export.json",
            "/api/export.csv",
            "/api/export.pdf",
            "/api/export.svg",
            "/api/export.dxf",
            "/api/steering-curves.svg",
            "/api/swept-path.svg",
        }:
            query = parse_qs(parsed.query)
            mode = query.get("mode", ["quick"])[0]
            if mode not in {"quick", "full"}:
                self.send_error(400, "mode must be quick or full")
                return
            beta_values = query.get("beta_deg", ["0"])
            try:
                beta_deg = float(beta_values[0])
            except ValueError:
                self.send_error(400, "beta_deg must be numeric")
                return

            if parsed.path == "/api/export.json":
                payload = build_export_bundle(beta_deg, mode)
                content = json.dumps(payload, indent=2).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Disposition", 'attachment; filename="easytowing-report.json"')
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
                return

            if parsed.path == "/api/export.csv":
                content = build_export_csv(beta_deg, mode).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/csv; charset=utf-8")
                self.send_header("Content-Disposition", 'attachment; filename="easytowing-report.csv"')
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
                return

            if parsed.path == "/api/export.pdf":
                content = build_export_pdf(beta_deg, mode)
                self.send_response(200)
                self.send_header("Content-Type", "application/pdf")
                self.send_header("Content-Disposition", 'attachment; filename="easytowing-report.pdf"')
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
                return

            if parsed.path == "/api/export.svg":
                content = build_dimensioned_svg(beta_deg, mode).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "image/svg+xml; charset=utf-8")
                self.send_header("Content-Disposition", 'attachment; filename="easytowing-sketch.svg"')
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
                return

            if parsed.path == "/api/export.dxf":
                content = build_export_dxf(beta_deg, mode).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/dxf; charset=utf-8")
                self.send_header("Content-Disposition", 'attachment; filename="easytowing-sketch.dxf"')
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
                return

            if parsed.path == "/api/steering-curves.svg":
                content = build_steering_curves_svg(beta_deg, mode).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "image/svg+xml; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
                return

            if parsed.path == "/api/swept-path.svg":
                content = build_swept_path_svg(beta_deg, mode).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "image/svg+xml; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
                return

        if parsed.path == "/api/projects":
            PROJECT_STORE.ensure_seed_project()
            self._send_json(_project_state_payload())
            return

        parts = [part for part in parsed.path.split("/") if part]
        if len(parts) == 3 and parts[0] == "api" and parts[1] == "projects":
            project = PROJECT_STORE.get_project(parts[2])
            if project is None:
                self.send_error(404, "Project not found")
                return
            self._send_json(_project_detail_payload(project))
            return
        if len(parts) == 4 and parts[0] == "api" and parts[1] == "projects" and parts[3] == "revisions":
            project = PROJECT_STORE.get_project(parts[2])
            if project is None:
                self.send_error(404, "Project not found")
                return
            self._send_json(
                {
                    "project": _project_detail_payload(project),
                    "revisions": [_revision_payload(revision) for revision in project.revisions],
                }
            )
            return
        if len(parts) == 5 and parts[0] == "api" and parts[1] == "projects" and parts[3] == "revisions":
            project = PROJECT_STORE.get_project(parts[2])
            if project is None:
                self.send_error(404, "Project not found")
                return
            revision = project.get_revision(parts[4])
            if revision is None:
                self.send_error(404, "Revision not found")
                return
            self._send_json(
                {
                    "project": _project_detail_payload(project),
                    "revision": _revision_payload(revision, include_snapshot=True),
                }
            )
            return

        self.send_error(404, "Not found")

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler interface
        parsed = urlparse(self.path)
        parts = [part for part in parsed.path.split("/") if part]
        try:
            body = self._read_json_body()
        except Exception as exc:  # noqa: BLE001 - return a clear HTTP 400
            self.send_error(400, str(exc))
            return

        if len(parts) == 2 and parts[0] == "api" and parts[1] == "projects":
            name = str(body.get("name", "Reference Demo Project"))
            beta_deg = float(body.get("beta_deg", 0.0))
            optimization_mode = str(body.get("optimization_mode", "quick"))
            note = str(body.get("note", "Initial revision"))
            if optimization_mode not in {"quick", "full"}:
                self.send_error(400, "optimization_mode must be quick or full")
                return
            project = PROJECT_STORE.create_project(
                name,
                beta_deg=beta_deg,
                optimization_mode=optimization_mode,
                note=note,
            )
            self._send_json({"project": _project_detail_payload(project)}, status=201)
            return

        if len(parts) == 2 and parts[0] == "api" and parts[1] == "import.dxf":
            dxf_text = str(body.get("dxf_text", ""))
            if not dxf_text.strip():
                self.send_error(400, "dxf_text is required")
                return
            source_name = str(body.get("source_name", ""))
            try:
                role_overrides = _parse_role_overrides(body.get("role_overrides"))
            except ValueError as exc:
                self.send_error(400, str(exc))
                return
            report = analyze_dxf_import(dxf_text, source_name=source_name)
            if role_overrides:
                report = apply_dxf_role_overrides(report, role_overrides)
            payload = serialize_dxf_import_report(report)
            self._send_json(payload)
            return

        if len(parts) == 4 and parts[0] == "api" and parts[1] == "projects" and parts[3] == "revisions":
            project = PROJECT_STORE.get_project(parts[2])
            if project is None:
                self.send_error(404, "Project not found")
                return
            try:
                beta_deg = float(body.get("beta_deg", 0.0))
            except (TypeError, ValueError):
                self.send_error(400, "beta_deg must be numeric")
                return
            optimization_mode = str(body.get("optimization_mode", "quick"))
            note = str(body.get("note", "Revision"))
            if optimization_mode not in {"quick", "full"}:
                self.send_error(400, "optimization_mode must be quick or full")
                return
            revision = PROJECT_STORE.append_revision(
                project.id,
                beta_deg=beta_deg,
                optimization_mode=optimization_mode,
                note=note,
            )
            latest_project = PROJECT_STORE.get_project(project.id) or project
            self._send_json(
                {
                    "project": _project_detail_payload(latest_project),
                    "revision": _revision_payload(revision),
                },
                status=201,
            )
            return

        if len(parts) == 4 and parts[0] == "api" and parts[1] == "projects" and parts[3] == "restore":
            project = PROJECT_STORE.get_project(parts[2])
            if project is None:
                self.send_error(404, "Project not found")
                return
            revision_id = str(body.get("revision_id", ""))
            if not revision_id:
                self.send_error(400, "revision_id is required")
                return
            try:
                revision = PROJECT_STORE.restore_revision(project.id, revision_id)
            except KeyError:
                self.send_error(404, "Revision not found")
                return
            latest_project = PROJECT_STORE.get_project(project.id) or project
            self._send_json(
                {
                    "project": _project_detail_payload(latest_project),
                    "revision": _revision_payload(revision),
                }
            )
            return

        self.send_error(404, "Not found")


def run_server(host: str = "127.0.0.1", port: int = 8000) -> None:
    if not WEB_DIR.exists():
        raise RuntimeError(f"Web assets not found at {WEB_DIR}")
    PROJECT_STORE.ensure_seed_project()
    server = ThreadingHTTPServer((host, port), DemoRequestHandler)
    address = f"http://{host}:{port}"
    print(f"EasyTowing demo server running at {address}")
    print("Open the URL in a browser to inspect the ideal steering prototype.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the EasyTowing demo server.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8000, type=int)
    args = parser.parse_args()
    run_server(host=args.host, port=args.port)


if __name__ == "__main__":
    main()
