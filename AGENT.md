# RoboticArm Solver Notes

## Solver Architecture

The kinematics and dynamics code is under `src/arm_controller/lib/calculate`.
The current design uses a user-defined task space. It must not assume that the
external command is the traditional 6D Cartesian pose
`[x, y, z, roll, pitch, yaw]`.

The main relationships are:

```text
ArmSolve
  owns an IKSolver
  receives a ModelBase and a user-owned TaskMapping

IKSolver
  calls ModelBase::forward_kinematics()
  calls ModelBase::geometric_jacobian()
  delegates task-space conversion to TaskMapping
  performs iterative damped least-squares IK

ModelBase
  supplies the robot model state, geometric Jacobian,
  dynamics, and joint limits

TaskMapping
  maps the model state to user-defined task coordinates
  maps the model geometric Jacobian to the matching task Jacobian
```

## Core Classes

### `ModelBase`

File: `src/arm_controller/lib/calculate/modelbase.hpp`

`ModelBase` is the robot-model abstraction. Implementations provide:

- `dof()`
- `forward_kinematics(q)`: returns the end-effector pose as
  `Eigen::Isometry3d`
- `geometric_jacobian(q)`: returns the model Jacobian, currently expected by
  `IKSolver` to have shape `6 x dof`
- `inverse_dynamic(q, dq, ddq)`
- `lower_jointLimit()` and `upper_jointLimit()`

`ModelFromURDF` is the current Pinocchio-backed implementation.

### `TaskMapping`

File: `src/arm_controller/lib/calculate/task.hpp`

`TaskMapping` is an abstract strategy supplied by the user. The mapping defines
the task space used by IK and dynamics:

```cpp
class TaskMapping {
public:
    virtual ~TaskMapping() = default;

    virtual bool position_map(
        const Eigen::VectorXd& joint_pos,
        const Eigen::Isometry3d& pose,
        Eigen::VectorXd* task_position) = 0;

    virtual bool jacobian_map(
        const Eigen::VectorXd& joint_pos,
        const Eigen::MatrixXd& jacobian,
        Eigen::MatrixXd* task_jacobian) = 0;
};
```

`position_map()` and `jacobian_map()` must describe the same task coordinates:

- `position_map(q)` returns the current task position `x(q)`.
- `jacobian_map(q, J)` returns the Jacobian `J_task = dx/dq`.
- Both outputs must use the same task dimension and coordinate ordering.
- The mapped Jacobian must have `dof` columns.
- Return `false` when the mapping cannot be evaluated.

The mapping object is not owned or deleted by `IKSolver`/`ArmSolve`; its
lifetime must exceed the solver objects.

### `IKSolver`

Files:

- `src/arm_controller/lib/calculate/kinamic.hpp`
- `src/arm_controller/lib/calculate/kinamic.cpp`

Construction:

```cpp
IKSolver(ModelBase* robot, TaskMapping* task_mapping);
```

`solve(target, joint_pos)` parameter semantics:

- `target`: user-defined task-space target. Its dimension is determined by
  `TaskMapping::position_map()`.
- `joint_pos`: input initial joint configuration and output solution on
  success. It must have size `robot->dof()`.
- The solver uses the supplied initial configuration; it does not assume zero
  as the initial state.
- On failure, the caller's `joint_pos` is left unchanged.

At every iteration, `IKSolver`:

1. Gets the current model pose through `ModelBase`.
2. Calls `TaskMapping::position_map()` to obtain the current task position.
3. Computes `error = target - current_task_position`.
4. Calls `TaskMapping::jacobian_map()` to obtain the task Jacobian.
5. Computes a damped least-squares joint step.
6. Applies the fixed per-joint step limit and model joint limits.

`IKSolver` contains no task-unit selection logic, no built-in Euler-angle
conversion, and no built-in axis/dot-product mapping. Those behaviors belong in
the user's `TaskMapping` subclass.

The public helper methods are:

- `task_position(q)`: calls `position_map()`.
- `jacobian(q)`: obtains the model's geometric Jacobian and calls
  `jacobian_map()`.

### `ArmSolve`

Current files:

- `src/arm_controller/lib/calculate/arm.hpp`
- `src/arm_controller/lib/calculate/arm.cpp`

`ArmSolve` is the higher-level façade. It stores the model pointer and an
`IKSolver` pointer, then exposes:

- `inverse_kinamic(task_pos)`: solves a user-defined task-space target. The
  current implementation starts from a zero joint vector.
- `forward_kinamic(joint_pos)`: returns `IKSolver::task_position()`, so the
  result is user-defined task coordinates, not necessarily Cartesian pose data.
- `inverse_dynamic(joint_pos, task_vel, task_acc, task_force)`: converts task
  velocity and acceleration to joint velocity and acceleration using the
  mapped task Jacobian, then calls `ModelBase::inverse_dynamic()` and adds
  `J_task.transpose() * task_force`.
- `static_force(joint_pos, joint_torque_residual)`: solves
  `J_task.transpose() * task_force = joint_torque_residual`.

The dynamic interfaces use the same task dimension returned by
`TaskMapping::jacobian_map()`. They must not silently fall back to a
traditional 6D Cartesian interface or direct joint-space inputs.

## Current Mapping Implementations

`default6dof_task.hpp/.cpp` declares
`Default6DofTaskSpaceMapping`, but it is currently only a placeholder and does
not yet implement the pure virtual mapping methods. It cannot be instantiated
until both `position_map()` and `jacobian_map()` are implemented.

## Build Integration

The intended calculation sources are:

```text
kinamic.cpp
arm.cpp
model_from_urdf.cpp
```

The current `src/arm_controller/CMakeLists.txt` still needs to be checked when
the `solve.cpp` to `arm.cpp` rename is finalized. It must compile `arm.cpp`
and must not reference the deleted `solve.cpp`, `solve.hpp`, `armsolve.cpp`, or
`armsolve.hpp`.

## Maintenance Rules

- Keep task-space semantics in `TaskMapping`, not in `IKSolver`.
- Keep model-specific kinematics and dynamics in `ModelBase` implementations.
- When changing a task coordinate definition, update both `position_map()` and
  `jacobian_map()` together.
- Validate task dimensions and Jacobian dimensions at the API boundary.
- Do not reintroduce `TaskUnit` or hard-coded Cartesian task branches.
- After solver changes, build the `arm_controller` package and run at least
  `cppcheck` and `lint_cmake`.
