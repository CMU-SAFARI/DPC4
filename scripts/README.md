# Script Documentation

This directory contains some scripts that were used to generate slurm jobs to evaluate all competing prefetchers, as well as rolling up statistics from output files.

## Creating Slurm Jobs

`create_jobs.py` creates slurm jobs to run a list of experiments on a list of traces. The script supports the following commandline arguments:

| Argument | Description | Default Value |
| -------- | ----------- | --------------|
| `tlist` | A list of traces as a CSV file. These traces will be used to run the experiments. `dpc.1c.csv` is an example for single-core traces, `dpc.4c.csv` is an example for four-core traces. | "" |
| `exp` | A list of experiments as a CSV file. Each experiment runs on all traces. `exp.csv` is an example of list of experiments. Line starting with # gets ignored. | "" |
| `out` | Name of the output jobfile. | jobfile.sh |

## Rolling-Up Statistics

`rollup_csv.py` parses a list of output file, extracts a list of metrics, and dumps into a CSV format. The script supports the following commandline arguments.

| Argument | Description | Default Value |
| -------- | ----------- | --------------|
| `tlist` | A list of traces as a CSV file. These traces will be used to rollup stats. `dpc.1c.csv` is an example for single-core traces, `dpc.4c.csv` is an example for four-core traces. | "" |
| `exp` | A list of experiments as a CSV file. `exp.csv` is an example of list of experiments. Line starting with # gets ignored. | "" |
| `mfile` | A list of metrics to rollup. `perf.mfile` and `coverage.mfile` are example mfiles | "" |
| `ext` | The extension of the stat file. Typically JSON. | JSON |
| `results-dir` | The directory where all the output files are expected. | "" |
| `threads` | The number of threads to launch. Useful when rolling up stats for a long list of experiments and traces. | 1 |