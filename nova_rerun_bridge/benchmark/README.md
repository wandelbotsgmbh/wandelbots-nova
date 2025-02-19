# Nova Benchmark Suite

This suite provides tools to benchmark and visualize motion planning strategies in the Nova framework.

## Overview

The benchmark suite evaluates different motion planning strategies against a standardized set of problems from the [robometrics](https://github.com/fishbotics/robometrics) dataset. It measures:

- Planning success rate
- Computation time

## Available Strategies

- `collision_free_magma`: Collision-free point-to-point motion planning using Magma
- `collision_free`: Collision-free point-to-point motion planning using MidpointInsertion
