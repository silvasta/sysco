# System and Controls Library

Collection of various tools in the field of systems and controls.

> Systems can mean just about anything.
> Here, we are talking about **dynamic systems**
> and how to find the **right input**
> to achieve a desired **state of equilibrium**.

As a first step, the library implements the **base controller class**
that defines the basic requirements for any **discrete-time** controller.

The **next step** is to derive **model-based** controllers
and then also **data-driven** controllers.

_At some point, the fundamentals of control systems such as PID could be integrated,
presumably as wrappers for existing libraries and tools._

## Usage

```python
from sysco import BaseController # Controller subclasses not already implemented

# custom controller example
class MyController(BaseController):
    def get_input(self):
        return 1

# instantiate custom class and get input
my_controller = MyController()
print(my_controller.get_input()) # --> 1

# try to instantiate base class
controller = BaseController()
next_input = controller.get_input() # --> NotImplementedError
```

## Future development

To avoid bottlenecks later on, Rust is already built into the structure 

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
