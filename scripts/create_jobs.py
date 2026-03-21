#!/usr/bin/env python3
import argparse
import csv
import re
from pathlib import Path


_TRACEPATH_COL_RE = re.compile(r"^TracePath(\d+)$")


def read_csv(path):
    rows = []
    with open(path, newline="") as f:
        # Filter out commented or empty lines
        filtered_lines = (
            line for line in f
            if line.strip() and not line.lstrip().startswith("#")
        )
        reader = csv.DictReader(filtered_lines)
        rows.extend(reader)
    return rows


def get_tracepath_columns(fieldnames):
    """
    Return TracePath columns sorted by numeric suffix:
    TracePath0, TracePath1, ...
    """
    cols = []
    for name in fieldnames:
        m = _TRACEPATH_COL_RE.match(name.strip())
        if m:
            cols.append((int(m.group(1)), name.strip()))
    cols.sort(key=lambda x: x[0])
    return [c for _, c in cols]


def main():
    parser = argparse.ArgumentParser(
        description="Generate command list: each experiment runs on each trace."
    )
    parser.add_argument("--tlist", required=True, help="Path to trace.csv")
    parser.add_argument("--exp", required=True, help="Path to experiments.csv")
    parser.add_argument("--out", default="jobfile.sh", help="Output file (one command per line)")
    args = parser.parse_args()

    traces = read_csv(args.tlist)
    experiments = read_csv(args.exp)

    if not traces:
        raise RuntimeError("trace.csv has no data rows")
    if not experiments:
        raise RuntimeError("experiments.csv has no data rows")

    # Sanity checks: TraceName + at least one TracePath<i>
    if "TraceName" not in traces[0]:
        raise RuntimeError("trace.csv missing required column: TraceName")

    tracepath_cols = get_tracepath_columns(traces[0].keys())
    if not tracepath_cols:
        raise RuntimeError(
            "trace.csv missing required TracePath columns: expected TracePath0, TracePath1, ..."
        )

    # Sanity checks: experiments
    for col in ("ExpName", "Exe", "Cmd"):
        if experiments and col not in experiments[0]:
            raise RuntimeError(f"experiments.csv missing required column: {col}")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    preamble = "sbatch -p cpu_part --mincpus=1 -c 1"
    commands = []

    for exp in experiments:
        exp_name = exp["ExpName"].strip()
        exe = exp["Exe"].strip()
        cmd = exp["Cmd"].strip()

        for tr in traces:
            trace_name = (tr.get("TraceName") or "").strip()

            # Collect all TracePath<i> fields present for this row (skip empty)
            trace_paths = []
            for c in tracepath_cols:
                v = (tr.get(c) or "").strip()
                if v:
                    trace_paths.append(v)

            if not trace_name or not trace_paths:
                continue

            trace_paths_str = " ".join(trace_paths)

            header = (
                f"{preamble} -J {trace_name}.{exp_name} "
                f"-e {trace_name}.{exp_name}.err "
                f"-o {trace_name}.{exp_name}.out "
                f"/home/rahbera/dpc4/submissions/scripts/wrapper.sh"
            )

            if cmd:
                full_cmd = (
                    f"{header} {exe} "
                    f"\"{cmd} --json {trace_name}.{exp_name}.json {trace_paths_str}\""
                )
            else:
                full_cmd = f"{header} {exe} \"{trace_paths_str}\""

            commands.append(full_cmd)

    out_path.write_text("\n".join(commands) + "\n")


if __name__ == "__main__":
    main()
