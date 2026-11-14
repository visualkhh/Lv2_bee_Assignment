# Resource recording on Jetson

`resource_watch.py` reads Linux `/proc` using only Python's standard library. It does not
import ROS, NumPy or CUDA, and never publishes commands, terminates processes or
restarts services. It records UTC timestamps, aggregate CPU utilization, available
RAM, swap counters and the largest processes by RSS. Process names are read from
`stat`; command lines and environment variables are not recorded.

Run from `/home/ghj/Lv2_bee_Assignment/lv2_module6/tools`:

```sh
python3 resource_watch.py --duration 1800 --output /tmp/bee-resources.jsonl
```

The default file limit is 8 MiB plus two rotated backups, bounding total retained
output to 24 MiB. Each sample is flushed immediately. One collector should own an
output path; do not run simultaneous writers against the same file.

To preserve evidence when Jetson freezes, run the following **on pa27**. The JSONL
file is then written on pa27, while the small sampler runs on Jetson:

```sh
ssh bee.local 'python3 /home/ghj/Lv2_bee_Assignment/lv2_module6/tools/resource_watch.py --duration 1800' > bee-resources.jsonl
```

Use a one-shot preflight check without changing robot state:

```sh
python3 resource_watch.py --check --min-available-mib 1536
```

Exit codes: `0` sufficient memory; `2` available RAM below threshold; `3` available
RAM could not be read; `1` output or recording error. This does not inhibit startup
unless the caller explicitly checks the exit code. Normal recording does not stop
when memory is low. `--samples N` provides a bounded recording for smoke tests.

The first CPU sample is `null` because CPU percentages require a counter delta.
Aggregate CPU percent is 0–100 across the machine; per-process CPU percent uses
one logical CPU as 100% and can therefore exceed 100%. Swap totals are allocated
swap, while `swap_*_pages_per_s` measures current traffic after the first sample.
RSS can include shared pages and does not attribute all CUDA allocations: do not
sum the process RSS column and treat it as physical RAM use. Jetson CPU/GPU share
system RAM, so `mem_available_mib` is the primary memory-headroom signal. Kernel
OOM/power/thermal logs and control timing are still needed to establish the cause
of a crash. A low-memory threshold is an operational target, not proof of safety.

Run regression tests without ROS or a GPU:

```sh
python3 -m unittest -v test_resource_watch.py
```
