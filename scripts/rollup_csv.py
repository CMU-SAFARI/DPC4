#!/usr/bin/env python3
"""Rewrite of rollup.pl in Python."""

from __future__ import annotations

import argparse
import ast
import csv
import json
import re
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Set


@dataclass
class Metric:
    name: str
    path: List[Any] | None = None
    expression: str | None = None
    dependencies: List[str] = field(default_factory=list)


@dataclass
class Experiment:
    name: str
    knobs: str


@dataclass
class Trace:
    data: Dict[str, str]

    @property
    def name(self) -> str:
        return self.data.get("NAME", "")


def read_csv_ignore_hash_comments(csv_path: Path, label: str) -> List[Dict[str, str]]:
    kept_lines: List[str] = []
    with csv_path.open(newline="") as f:
        for line in f:
            if not line.strip():
                continue
            if line.lstrip().startswith("#"):
                continue
            kept_lines.append(line)
    reader = csv.DictReader(kept_lines)
    rows = list(reader)
    if not rows:
        raise ValueError(f"No rows found in {label} file {csv_path}")
    return rows


def parse_metric_path(expr: str) -> List[Any]:
    tokens: List[Any] = []
    for match in re.finditer(r"\[(.*?)\]", expr):
        inner = match.group(1).strip()
        if not inner:
            continue
        try:
            tokens.append(ast.literal_eval(inner))
        except (SyntaxError, ValueError):
            tokens.append(inner)
    return tokens


def extract_expression_dependencies(expr: str) -> List[str]:
    try:
        tree = ast.parse(expr, mode="eval")
    except SyntaxError as exc:
        raise ValueError(f"Invalid derived metric expression '{expr}'") from exc

    allowed_nodes = (
        ast.Expression,
        ast.BinOp,
        ast.UnaryOp,
        ast.Name,
        ast.Load,
        ast.Add,
        ast.Sub,
        ast.Mult,
        ast.Div,
        ast.Pow,
        ast.Mod,
        ast.FloorDiv,
        ast.USub,
        ast.UAdd,
        ast.Constant,
        ast.Call,
    )

    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            raise ValueError(f"Function calls are not allowed in derived metric expression '{expr}'")
        if not isinstance(node, allowed_nodes):
            raise ValueError(f"Unsupported syntax in derived metric expression '{expr}'")

    names: Set[str] = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    return sorted(names)


def parse_metrics(path: Path) -> List[Metric]:
    metrics: List[Metric] = []
    defined: Set[str] = set()
    with path.open() as fh:
        for raw_line in fh:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                raise ValueError(f"Invalid metric line: {line}")
            name, expr = line.split("=", 1)
            metric_name = name.strip()
            expression = expr.strip()
            if expression.startswith("["):
                metric_path = parse_metric_path(expression)
                metrics.append(Metric(name=metric_name, path=metric_path))
            else:
                dependencies = extract_expression_dependencies(expression)
                undefined = [dep for dep in dependencies if dep not in defined]
                if undefined:
                    missing = ", ".join(undefined)
                    raise ValueError(
                        f"Derived metric '{metric_name}' references undefined metrics: {missing}"
                    )
                metrics.append(
                    Metric(name=metric_name, expression=expression, dependencies=dependencies)
                )
            defined.add(metric_name)
    return metrics


def parse_experiments(path: Path) -> List[Experiment]:
    rows = read_csv_ignore_hash_comments(path, "experiments")
    if "ExpName" not in rows[0]:
        raise ValueError("Experiment CSV must contain an 'ExpName' column")
    experiments: List[Experiment] = []
    for row in rows:
        name = (row.get("ExpName") or "").strip()
        if not name:
            continue
        knobs_parts: List[str] = []
        for key, value in row.items():
            if key == "ExpName":
                continue
            if value is None:
                continue
            value_str = str(value).strip()
            if value_str:
                knobs_parts.append(value_str)
        experiments.append(Experiment(name=name, knobs=" ".join(knobs_parts)))
    if not experiments:
        raise ValueError(f"No valid experiments found in {path}")
    return experiments


def parse_traces(path: Path) -> List[Trace]:
    rows = read_csv_ignore_hash_comments(path, "traces")
    if "TraceName" not in rows[0]:
        raise ValueError("Trace CSV must contain a 'TraceName' column")
    traces: List[Trace] = []
    for row in rows:
        name = (row.get("TraceName") or "").strip()
        if not name:
            continue
        data = {"NAME": name}
        for key, value in row.items():
            if key == "TraceName":
                continue
            if value is None:
                continue
            data[key] = value
        traces.append(Trace(data=data))
    if not traces:
        raise ValueError(f"No valid traces found in {path}")
    return traces


def load_stats(log_path: Path) -> Any:
    with log_path.open() as fh:
        return json.load(fh)


def extract_metric_value(stats: Any, metric: Metric) -> Any:
    value: Any = stats
    for step in metric.path:
        if isinstance(step, int):
            value = value[step]
        else:
            if isinstance(value, dict) and step in value:
                value = value[step]
            elif isinstance(value, dict):
                matched_key = next(
                    (k for k in value.keys() if isinstance(k, str) and k.lower() == str(step).lower()),
                    None,
                )
                if matched_key is None:
                    raise KeyError(step)
                value = value[matched_key]
            else:
                raise KeyError(step)
    return value


def format_metric_value(value: Any) -> str:
    if isinstance(value, (dict, list)):
        return json.dumps(value)
    return str(value)


def ensure_numeric(value: Any) -> float:
    if isinstance(value, bool):
        return float(value)
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError as exc:
            raise TypeError(f"Value '{value}' is not numeric") from exc
    raise TypeError(f"Unsupported value type '{type(value).__name__}' for derived metrics")


def evaluate_expression(expression: str, context: Dict[str, Any]) -> Any:
    numeric_context = {name: ensure_numeric(value) for name, value in context.items()}
    code = compile(expression, "<metric>", "eval")
    return eval(code, {"__builtins__": {}}, numeric_context)


def format_trace_results(
    trace: Trace, exps: Sequence[Experiment], metrics: Sequence[Metric], ext: str, results_dir: Path
) -> Iterable[str]:
    trace_name = trace.name
    per_trace_result: Dict[str, str] = {}
    all_exps_passed = True
    for exp in exps:
        exp_name = exp.name
        log_file = results_dir / f"{trace_name}.{exp_name}.{ext}"
        metric_values: List[str] = []
        if log_file.exists():
            try:
                stats = load_stats(log_file)
            except (OSError, json.JSONDecodeError):
                all_exps_passed = False
                metric_values.extend("0" for _ in metrics)
            else:
                computed: Dict[str, Any] = {}
                for metric in metrics:
                    try:
                        if metric.path is not None:
                            value = extract_metric_value(stats, metric)
                        else:
                            ctx = {dep: computed[dep] for dep in metric.dependencies}
                            value = evaluate_expression(metric.expression or "", ctx)
                        computed[metric.name] = value
                        metric_values.append(format_metric_value(value))
                    except (KeyError, IndexError, TypeError, ZeroDivisionError, ValueError) as exc:
                        all_exps_passed = False
                        computed[metric.name] = 0
                        metric_values.append("0")
        else:
            all_exps_passed = False
            metric_values.extend("0" for _ in metrics)
        per_trace_result[exp_name] = ",".join(metric_values)
    for exp in exps:
        yield f"{trace_name},{exp.name},{per_trace_result[exp.name]},{int(all_exps_passed)}"


def main() -> None:
    parser = argparse.ArgumentParser(description="Aggregate metric rollups")
    parser.add_argument("--tlist", required=True, help="Trace list file")
    parser.add_argument("--exp", required=True, help="Experiment file")
    parser.add_argument("--mfile", required=True, help="Metrics file")
    parser.add_argument("--ext", default="json", help="Log file extension")
    parser.add_argument(
        "--results-dir",
        required=True,
        help="Directory containing TraceName.ExpName.<ext> result files",
    )
    parser.add_argument(
        "--threads",
        type=int,
        default=1,
        help="Number of worker threads to use for per-trace processing (default: 1)",
    )
    args = parser.parse_args()

    traces = parse_traces(Path(args.tlist))
    experiments = parse_experiments(Path(args.exp))
    metrics = parse_metrics(Path(args.mfile))
    results_dir = Path(args.results_dir)
    threads = args.threads
    if threads <= 0:
        raise SystemExit("--threads must be >= 1")

    header_fields = ["Trace", "Exp", *(metric.name for metric in metrics), "Filter"]
    print(",".join(header_fields))
    if threads == 1:
        for trace in traces:
            for row in format_trace_results(trace, experiments, metrics, args.ext, results_dir):
                print(row)
    else:
        def process_trace_task(idx: int, trace: Trace) -> tuple[int, List[str]]:
            rows = list(format_trace_results(trace, experiments, metrics, args.ext, results_dir))
            return idx, rows

        with ThreadPoolExecutor(max_workers=threads) as pool:
            futures = [
                pool.submit(process_trace_task, idx, trace) for idx, trace in enumerate(traces)
            ]
            ordered_results: List[tuple[int, List[str]]] = []
            for fut in as_completed(futures):
                ordered_results.append(fut.result())
        for _, rows in sorted(ordered_results, key=lambda x: x[0]):
            for row in rows:
                print(row)


if __name__ == "__main__":
    main()
