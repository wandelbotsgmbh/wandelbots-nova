# Nova Benchmark Suite

This suite provides tools to benchmark and visualize motion planning strategies in the Nova framework.

```bash
# install the dependencies
uv sync --extra "benchmark"

# run a benchmark
uv run python -m nova_rerun_bridge/benchmark/run_collision_free_benchmark.py
```

## Overview

The benchmark suite evaluates different motion planning strategies against a standardized set of problems from the [robometrics](https://github.com/fishbotics/robometrics) dataset. It measures:

- Planning success rate
- Computation time

## Available Strategies

- `collision_free_magma`: Collision-free point-to-point motion planning using Magma
- `collision_free`: Collision-free point-to-point motion planning using MidpointInsertion

### MagmaP2P Prerequisites

To run the benchmark suite, you need to have the following installed:

```bash
nova catalog install magma-p2p
```
