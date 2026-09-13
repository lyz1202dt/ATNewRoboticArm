# RoboticArm Solver Notes

Kinematics/dynamics code lives in `src/arm_controller/lib/calculate`. The task
space is user-defined and must not assume the classic 6D pose
`[x, y, z, roll, pitch, yaw]`.

```text
ArmSolve -> IKSolver -> ModelBase   (FK / geometric Jacobian / dynamics / limits)
                     -> TaskMapping (task coordinates + task Jacobian)
```

## Core Classes

### `ModelBase` — `modelbase.hpp`
Robot-model abstraction: `dof()`, `forward_kinematics(q)` (→ `Isometry3d`),
`geometric_jacobian(q)` (6×dof), `inverse_dynamic(q,dq,ddq)`,
`lower_jointLimit()` / `upper_jointLimit()`. `ModelFromURDF` is the
Pinocchio-backed implementation.

### `TaskMapping` — `task.hpp`
User-owned strategy mapping model state to task space; must outlive the solver.
`position_map(q, pose, *task_position)` → `x(q)`;
`jacobian_map(q, J, *task_jacobian)` → `dx/dq` (dof columns). Both must share
the task dimension/ordering and return `false` when unevaluable.

### `IKSolver` — `kinamic.hpp/.cpp`
`IKSolver(ModelBase*, TaskMapping*)`. `solve(target, joint_pos)` runs iterative
damped least-squares: pose → task position → error → task Jacobian → joint step
→ per-joint/joint limits. `joint_pos` is the initial guess and the output
(unchanged on failure). Helpers: `task_position(q)`, `jacobian(q)`. No
task-unit, Euler-angle, or axis/dot-product logic here — that belongs in
`TaskMapping`.

### `ArmSolve` — `arm.hpp/.cpp`
Façade over model + IKSolver: `inverse_kinamic(task_pos)`,
`forward_kinamic(q)`, `inverse_dynamic(q, task_vel, task_acc, task_force)`
(adds `J_task^T * task_force`), `static_force(q, tau_residual)`.

### `Trajectory` — `trajectory.hpp/.cpp`
Point interpolation over `std::vector<Point>`:
`add_point(point, time_from_start)`, `start(time)`, `update(time, point)`, plus
`operator+` for accumulating points. Each segment is a quintic Bézier (≡
quintic Hermite) matching endpoint position, velocity, and acceleration
(missing vel/acc treated as zero), so adjacent segments are C²-continuous.

## Mappings

`default6dof_task.hpp/.cpp` declares `Default6DofTaskSpaceMapping` but is a
placeholder — pure virtual methods unimplemented.

## Build

Intended sources: `kinamic.cpp`, `arm.cpp`, `model_from_urdf.cpp`,
`trajectory.cpp`. `CMakeLists.txt` still lists the deleted `solve.cpp` and does
not yet list `arm.cpp` / `trajectory.cpp`.

## Rules

- Task-space semantics → `TaskMapping`; model math → `ModelBase`.
- Update `position_map()` and `jacobian_map()` together.
- Validate task/Jacobian dimensions at the API boundary.
- Don't reintroduce `TaskUnit` or hard-coded Cartesian branches.
- After changes: build `arm_controller`, run `cppcheck` and `lint_cmake`.
