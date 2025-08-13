# System and Controls Library

Collection of different tools in the area of systems and controls.

In a **first step** the library will be filled mostly
with **discrete-time model-based** controllers.

## Usage

```python
from sysco import Controller # subclass not already implemented

my_controller = Controller(system_param, task_param)
next_input = my_controller.get_input()
```

## Future development

To avoid bottlenecks later on, Rust is already built in 

## Controller (in planning)

### LQR

- First one that will be implemented
- Infinite horizon without constraints

Returns optimal input $u^\star$ that drives the System to the origin.

### MPC

#### Nominal Case

#### Economic MPC

#### Tracking MPC

#### Robust MPC

### Data-driven approaches

#### DeePC

#### RL-based
