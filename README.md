<p align="center">
  <picture>
  	<source media="(prefers-color-scheme: dark)" srcset="logo/logo-light.png">
  	<source media="(prefers-color-scheme: light)" srcset="logo/logo-dark.png">
  <img alt="athena-logo" src="logo/logo-dark.png" width="800">
</picture>
  <!-- <h2 align="center">The 4th Data Prefetching Championship (DPC4)</h2>
  <h4 align="center">Co-located with HPCA 2026, February 1 2026, Sydney, Australia</h4> -->
</p>

This repository is the one-stop destination of everything related to DPC4. It contains, among many things, (1) the codes of the competing prefetchers, (2) the simulation infrastructure, (3) workload traces, (4) evaluation metrics, and (5) any other auxiliary scripts used in DPC4. More information about the championship can be found at https://sites.google.com/view/dpc4-2026/home

<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#competing-prefetchers">Competing Prefetchers</a></li>
    <li><a href="#simulation-infrastructure">Simulation Infrastructure</a></li>
    <li><a href="#workload-traces">Workload Traces</a></li>
    <li><a href="#evaluation-metric">Evaluation Metric</a></li>
    <li><a href="#auxiliary-scripts">Auxiliary Scripts</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
    <li><a href="#organizers">Organizers</a></li>
  </ol>
</details>

## Competing Prefetchers

In total, eight submissions were part of the DPC4 main program. Their codes can be found inside `submissions/`. The eight submissions were:

- [Crossing the Boundary: Virtual-Address Based Inter-Page Prefetching for Lower Level Caches](submissions/VIP/)
- [SPPAM: Signature Pattern Prediction and Access-Map Prefetcher](submissions/SPPAM/)
- [Emender: Optimizing Prefetch Priority and Throttling in VBerti+Pythia](submissions/Emender/)
- [sBerti: Enhancing Berti with a Smart Stride Prefetcher for Better Coverage](submissions/sBerti/)
- [Pushing the limits of the Berti Prefetcher](submissions/BertiGO/)
- [The Entangling Data Prefetcher](submissions/EDP/)
- [Performance-Driven Composite Prefetching with Bandits](submissions/uMAMA/)
- [Global Berti: Simultaneous Streaming and Spatial Prefetching](submissions/gBerti/)

## Simulation Infrastructure

DPC4 used ChampSim as the simulation infrastructure to evaluate all submissions. The infra can be found [here](https://github.com/CMU-SAFARI/ChampSim/tree/master).

## Workload Traces

All 610 workload traces used in DPC4 are hosted in a public Cloudflare R2 bucket, taking ~1.5 TB of storage in compressed form. More information about the composition of the traces can be found in [our slides](https://github.com/CMU-SAFARI/DPC4/blob/main/presentations/dpc4-conclusion.pptx).

The bucket cannot be listed by opening its base URL in a browser. Instead, [`manifest.txt`](https://pub-c31f67d79d1b4cd28ff320612b1a9f84.r2.dev/manifest.txt) is the index of the dataset: it lists every object with its path, size, last-modified time, and upload checksum. Downloading needs no prior knowledge of the directory layout — the manifest supplies it.

To download a single trace, append its `path` column to the base URL:

```bash
wget https://pub-c31f67d79d1b4cd28ff320612b1a9f84.r2.dev/SPEC17/602.gcc_s-1850B.champsimtrace.xz
```

To download in bulk, turn the manifest into a URL list:

```bash
BASE=https://pub-c31f67d79d1b4cd28ff320612b1a9f84.r2.dev

# 1. Fetch the index
curl -sO $BASE/manifest.txt

# 2. Build a URL list (skip the '#' preamble and the header row)
grep -v '^#' manifest.txt | tail -n +2 | cut -f1 | sed "s|^|$BASE/|" > urls.txt

# 3. Download, recreating the directory layout. -c resumes an interrupted run,
#    so re-running the same command picks up where it left off.
wget -x -nH -c -i urls.txt
```

To fetch just one collection, filter the `path` column first — e.g. only the 92 `SPEC17` traces:

```bash
grep -v '^#' manifest.txt | tail -n +2 | awk -F'\t' '$1 ~ /^SPEC17\//{print $1}' \
  | sed "s|^|$BASE/|" > spec17.txt
wget -x -nH -c -i spec17.txt
```

The collections are `SPEC17` (92 traces), `gtrace_v2` (359), `ai-ml` (80), and `Graph` (79, split further into `GAP`, `GMS`, and `Ligra`).

> The full repository is sized ~1.5 TB. Please be considerate while downloading.

##### Acknowledgements
- `SPEC17` traces were open-sourced by [DPC3](https://dpc3.compas.cs.stonybrook.edu). We are grateful to [Prof. Daniel Jiménez](https://people.engr.tamu.edu/djimenez/index.html) and [Prof. Mike Ferdman](https://compas.cs.stonybrook.edu/~mferdman/) for capturing and maintaining these traces.
- `Graph/GAP` traces were open-sourced by [ML-Based Data Prefetching Competition](https://sites.google.com/view/mlarchsys/isca-2021/ml-prefetching-competition).
- `gtrace_v2` traces were originally [open-sourced by Google](https://dynamorio.org/google_workload_traces.html) and later converted to ChampSim. We are grateful to [Matthew Giordano](https://homes.cs.washington.edu/~mgiordan/) and [Akanksha Jain](https://research.google/people/akankshajain/) for this effort.

## Evaluation Metric

Every submission was evaluated to receive three scores:
- **FullBW score**: measured in a single-core system with 4800 MTPS main memory bandwidth
- **LimitBW score**: measured in a single-core system with 800 MTPS main memory bandwidth
- **MC score**: measured in a four-core system with 4800 MTPS main memory bandwidth

The FullBW and LimitBW scores are calculated as the non-weighted geomean of speedup (i.e., the ratio of IPC in with submitted prefetcher and the IPC of the baseline) across all workload traces.

The MC Score is defined in three steps as follows:

> The definition uses the following convention: `IPC <i,shared,baseline>` denotes the IPC of the i-th core running together with other traces with baseline prefetchers.

1. For a given trace mix k, compute the HarmonicSpeedup of the baseline ($BaselineHS_k$) as $\frac{IPC<i,shared,baseline>}{IPC<i,alone,baseline>}$.
2. For the same trace mix, compute the HarmonicSpeedup of the submission ($SubmissionHS_k$) as $\frac{IPC<i,shared,submission>}{IPC<i,alone,baseline>}$.
3. Compute the MC Score as the non-weighted geometric mean of the ratio $\frac{SubmissionHS_k}{BaselineHS_k}$ across all trace mixes.

The overall score of a submission is calculated as the non-weighted geometric mean of all three individual scores.

## Auxiliary Scripts

Check inside the `scripts` directory. [scripts/README.md](https://github.com/CMU-SAFARI/DPC4/blob/main/scripts/README.md) provides a documentation on how to use the scripts. The evaluation results can be found in [eval.xlsx](https://github.com/CMU-SAFARI/DPC4/blob/main/scripts/eval.xlsx) file.

## Acknowledgements

We are thankful to all the competitors for submitting their ideas and pushing the the state-of-the-art in prefetching. We are grateful to all the program committee members for their valuable feedback on the submissions: [Akanksha Jain](https://research.google/people/akankshajain/), [Alaa Alameldeen](http://www.cs.sfu.ca/~alaa/alaa_home.shtml), [Alberto Ros](https://webs.um.es/aros/), [Anant Nori](https://dblp.org/pid/221/2867.html), [Biswabandan Panda](https://www.cse.iitb.ac.in/~biswa/), [Leeor Peled](https://dblp.org/pid/163/0025.html), [Mike Ferdman](https://compas.cs.stonybrook.edu/~mferdman/), [Paul Gratz](https://cesg.tamu.edu/faculty/paul-gratz/), and [Pierre Michaud](https://team.inria.fr/pacap/members/pierre-michaud/). Thanks goes to the HPCA 2026 organizers for giving us the opportunity to host the workshop.

## Organizers

- [Rahul Bera](mailto:write2bera@gmail.com)
- [Konstantinos Kanellopoulos](mailto:konkanello@gmail.com)
- [Onur Mutlu](mailto:omutlu@gmail.com)
