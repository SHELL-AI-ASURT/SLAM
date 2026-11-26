# C++ Engineering & Agent Working Rules

This document defines the mandatory engineering standards for all AI agents and contributors working in this repository. Every change—whether a new feature, algorithm, refactor, or bug fix—must satisfy these rules before work is considered complete.

If a repository configuration file (such as `CMakeLists.txt`, `.clang-format`, or `.clang-tidy`) specifies stricter or tool-specific settings, treat that configuration as the authoritative enforcement mechanism alongside these rules.

---

## 1. Core C++ Engineering Standards

### 1.1 Language Standard & Modern Idioms
- **C++ Standard**: Target the project's configured standard in `CMakeLists.txt` (default to **C++20** if not yet configured). Prefer standard library facilities over hand-rolled equivalents.
- **RAII & Ownership**: Manage all resources (memory, files, locks, handles) via RAII.
  - Prefer stack/value semantics first.
  - Use `std::unique_ptr` for sole ownership and `std::shared_ptr` only when shared ownership is genuinely required.
  - **Never** use raw owning `new` / `delete` or `malloc` / `free`. Non-owning raw pointers (`T*`) or references (`T&` / `const T&`) are acceptable only when lifetime is clearly guaranteed and documented.
- **Rule of Zero / Five**: Prefer the Rule of Zero. When custom resource management is unavoidable, explicitly define or `= delete` all five special member functions.
- **Type & Interface Safety**:
  - Mark single-argument constructors `explicit` unless implicit conversion is deliberately designed and documented.
  - Use `enum class` instead of unscoped `enum`.
  - Use `nullptr`, `override`, and `final` consistently.
  - Use `[[nodiscard]]` on functions whose return values represent errors, statuses, or computed results that should not be silently ignored.
  - Use `const` and `constexpr` aggressively (methods, parameters, local variables, views).
  - Prefer `std::span` and `std::string_view` for non-owning contiguous views.

### 1.2 Safety, Conversions & Undefined Behavior
- **Zero Undefined Behavior**: Never rely on UB (out-of-bounds access, dangling references/views, uninitialized reads, signed integer overflow, strict aliasing violations, data races).
- **Explicit Conversions**: Avoid C-style casts; use `static_cast` (or narrow-checked helpers) and avoid implicit narrowing conversions between signed/unsigned or floating-point/integer types.
- **Initialization**: Initialize all variables and class members at declaration or via member initializer lists.

### 1.3 Error Handling & Contracts
- **Distinguish Bugs from Runtime Failures**:
  - Use assertions/contract checks for programmer errors and internal invariants that must never be violated.
  - Use explicit error-reporting mechanisms (e.g., `std::expected`, status/error types, or documented exceptions—consistent with the module's convention) for recoverable runtime/input failures.
- **Validate at Boundaries**: Validate external inputs, file/sensor data, matrix dimensions, and API preconditions at module boundaries.
- **No Silent Failures**: Never swallow errors, ignore return codes, or catch-all (`catch (...)`) without handling, logging, or propagating cleanly.

### 1.4 Numerical, Algorithmic & Performance Discipline
- **Precision**: Default to `double` for mathematical, geometric, and state-estimation computations unless `float` is justified by memory/SIMD/hardware constraints and validated against accuracy requirements.
- **Numerical Stability**: Guard against division by zero, `NaN`/`Inf` propagation, catastrophic cancellation, poorly conditioned matrices, and gimbal/angle wraparound singularities. Document the chosen numerical strategy (e.g., QR/Cholesky/SVD over explicit matrix inversion).
- **Units & Conventions**: Use SI units by default (meters, radians, seconds) and keep coordinate-frame conventions explicit in types, variable names, and documentation (e.g., `T_world_body`, `p_body`).
- **Determinism**: Any stochastic algorithm must accept an explicit RNG seed/engine so results are reproducible in tests and debugging.
- **Clarity First, Zero-Cost Idioms**: Write clear, maintainable code first; avoid gratuitous copies and heap allocations inside hot loops, and optimize further only with profiling evidence.

### 1.5 Code Organization & Header Hygiene
- **Self-Contained Headers**: Every header must include everything it directly uses and compile cleanly on its own.
- **Include Guards**: Use `#pragma once` (or a consistent include-guard convention across the repo).
- **Minimal Coupling**: Keep headers minimal; use forward declarations where appropriate and place implementation details in `.cpp` files or internal `detail` namespaces.
- **Namespaces**: Never place `using namespace ...;` in header files or at global scope.
- **Modularity**: Keep functions, classes, and files cohesive and focused on a single clear responsibility.

### 1.6 Naming & Style
- Follow `.clang-format` and the repository's naming convention consistently across all files.
- Choose clear, domain-accurate names. Where code directly implements mathematical notation, keep identifiers closely aligned with the corresponding symbols in `docs/theory/` and document the mapping.

---

## 2. Code Documentation Standards

Code must be understandable by a new engineer or agent without reverse-engineering every line.

### 2.1 Public API Documentation (Doxygen Style)
Document every public class, struct, function, type alias, enum, and constant using consistent Doxygen comments (`///` or `/** ... */`). Include:
- **Brief summary** of what the symbol represents or computes.
- **Parameters (`@param`)**: meaning, units, coordinate frame, and valid range/constraints.
- **Return value (`@return`)**: meaning, units, and error/empty semantics.
- **Preconditions, Postconditions & Invariants**: assumptions required from callers and guarantees provided.
- **Edge cases & Failure behavior**: how invalid inputs, degenerate geometry, or non-convergence are handled.
- **Complexity / Thread-safety**: whenever non-trivial.
- **Theory Cross-Reference**: link to the relevant theory or architecture document and equation/section (e.g., `See docs/theory/icp.md, Section 3, Eq. (7)`).

### 2.2 Implementation Comments
- Explain **why**, not **what**: capture design rationale, mathematical tricks, numerical safeguards, coordinate transformations, and non-obvious invariants.
- Avoid noisy comments that merely restate obvious C++ syntax.
- Annotate non-trivial formulas inline with equation/section references to the corresponding `docs/theory/` document.

### 2.3 Documentation Freshness
- Whenever code behavior, interfaces, invariants, or equations change, update all related comments and `docs/` files **in the same change**. Stale documentation is treated as a bug.

---

## 3. Theory, Architecture & Math Documentation (Required)

Agents must maintain a living technical knowledge base in `docs/` explaining **how the big picture works**, **the underlying mathematics and algorithmic logic**, and **the authoritative sources** behind every non-trivial design choice.

### 3.1 Required Directory Structure
Organize technical documentation cleanly under `docs/`:

```text
docs/
├── architecture/
│   ├── overview.md           # System-wide big picture, pipeline, data flow, coordinate conventions
│   └── <subsystem>.md        # Module-level architecture, interfaces, state & design trade-offs
├── theory/
│   └── <algorithm_or_topic>.md  # Deep mathematical & theoretical write-up per algorithm/model
└── references.md             # Optional central bibliography (in addition to per-doc citations)
```

### 3.2 When to Create or Update `docs/` Files
An agent **must** create or update documentation in `docs/` whenever it:
1. Introduces a new module, subsystem, or pipeline stage (`docs/architecture/`).
2. Implements a new algorithm, mathematical model, estimator, optimizer, filter, transform, or heuristic (`docs/theory/`).
3. Modifies mathematical formulations, coordinate conventions, assumptions, or system data flow.

### 3.3 Mandatory Structure for Architecture Files (`docs/architecture/*.md`)
Each architecture document must cover:
1. **Purpose & Big Picture**: What problem this system/module solves and how it connects to upstream and downstream components.
2. **System / Data-Flow Diagram**: A clear Mermaid or ASCII diagram showing components, data structures, and control/data flow.
3. **Key Abstractions & State**: Core types, state representations, ownership boundaries, and invariants.
4. **Conventions**: Coordinate frames, units, time representation, and error-handling strategy.
5. **Design Decisions & Trade-offs**: Why this approach was chosen over alternatives, known limitations, and assumptions.
6. **Code Map**: Links to the primary source files and corresponding `docs/theory/` documents.

### 3.4 Mandatory Structure for Theory & Math Files (`docs/theory/*.md`)
Every theory file must use clean Markdown + LaTeX math (`$...$` for inline, `$$...$$` for numbered/block equations) and include all of the following sections:

```markdown
# <Algorithm / Topic Title>

## 1. Big Picture & Role in the System
- What problem this solves and where it sits in the overall pipeline (link to `docs/architecture/overview.md`).
- High-level intuition before diving into equations.
- Inputs, outputs, assumptions, and known failure modes / limitations.

## 2. Notation, Units & Coordinate Conventions
| Symbol | Meaning | Dimension / Type | Units / Frame |
| :--- | :--- | :--- | :--- |
| $\mathbf{x}_k$ | State vector at step $k$ | $\mathbb{R}^n$ | SI / World frame |

## 3. Mathematical Formulation & Derivations
- State the core model, objective function, or probabilistic/geometric formulation.
- Derive non-trivial steps explicitly (e.g., linearization, Jacobians, update equations, covariance propagation, Lie algebra/manifold retractions, optimality conditions).
- Explain numerical stability choices (e.g., square-root form, Cholesky vs SVD, epsilon guards, angle normalization) and edge/degenerate cases.

## 4. Algorithm Walkthrough
- Step-by-step algorithm or pseudocode bridging the math to execution.
- Time and space complexity analysis.

## 5. Code-to-Math Traceability
| Math Symbol / Equation | Code Location | Identifier / Function | Notes |
| :--- | :--- | :--- | :--- |
| Eq. (1): State update | `src/.../filter.cpp` | `Filter::predict()` | Uses RK4 discretization |

## 6. Sources & References
- Authoritative references supporting the theoretical logic (see Section 3.5).
```

### 3.5 Source & Citation Integrity Rules
- **Cite Primary, Authoritative Sources**: Reference peer-reviewed papers (authors, year, title, venue, DOI/arXiv/URL), established textbooks (authors, edition, chapter, section/equation numbers), or official algorithm/library specifications.
- **Map Claims to Sources**: Note which equations, theorems, or design choices come from which reference.
- **Zero Fabrication Policy**: **Never** invent, guess, or hallucinate paper titles, authors, years, DOIs, URLs, or section/equation numbers.
  - If a derivation is worked out from first principles without citing a specific paper, label it explicitly as **`Derived from first principles`**.
  - If a step is an engineering heuristic or approximation, label it explicitly as **`Engineering heuristic`** and explain the rationale and limitations.
  - If a source is known conceptually but exact bibliographic details cannot be verified in the current environment, state that transparently instead of guessing metadata.

---

## 4. Testing Standards

Tests are first-class engineering artifacts. No feature, algorithm, or bug fix is complete without automated tests.

### 4.1 Required Test Coverage
- **Unit Tests**: Test every public class, function, and non-trivial helper in isolation.
- **Mathematical & Numerical Verification**:
  - Test against known analytical / closed-form ground-truth solutions.
  - Verify analytical Jacobians/gradients against numerical finite differences within justified tolerances.
  - Assert mathematical invariants (e.g., orthogonality $R^\top R = I$, $\det(R) = +1$, symmetry/positive semi-definiteness of covariances, energy/norm preservation, convergence on synthetic problems).
  - Always justify numerical tolerances (`abs` / `rel`) rather than picking arbitrary loose bounds.
- **Edge Cases & Failure Modes**: Test empty inputs, single-element cases, zero vectors, collinear/singular/ill-conditioned inputs, `NaN`/`Inf` rejection, angle wraparound ($\pm\pi$), and boundary conditions.
- **Integration Tests**: Verify end-to-end interactions across module boundaries and pipeline stages.
- **Regression Tests**: Every bug fix must include a test that fails before the fix and passes after it.

### 4.2 Test Quality & Hygiene
- **Deterministic**: Pin RNG seeds explicitly; avoid wall-clock timing flakiness or filesystem/environment dependencies.
- **Clear Structure**: Follow Arrange–Act–Assert with descriptive test names conveying `<Unit>_<Scenario>_<ExpectedOutcome>`.
- **Fast & Isolated**: Keep unit tests fast and independent of execution order.
- **Test Integrity**: **Never** weaken assertions, loosen tolerances blindly, skip/disable failing tests, or tailor production code only to pass hardcoded test cases. Fix the underlying defect or justify any spec change explicitly.

---

## 5. Code Quality & Validation Workflow

Before marking any task complete, agents must build, check, and verify the codebase using the repository's configured toolchain.

### 5.1 Compiler Warnings
- Build with strict warnings enabled (recommended baseline for GCC/Clang: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wnon-virtual-dtor -Wold-style-cast`; treat warnings as errors `-Werror` in CI/dev presets when configured).
- **Zero new warnings**: Fix warnings at the root cause. Do not silence warnings with pragma/cast suppressions unless unavoidable, narrowly scoped, and explained in a comment.

### 5.2 Formatting & Static Analysis
- **Formatting**: Run `clang-format` on all created or modified C++ headers and sources using the repository's `.clang-format` rules.
- **Static Analysis**: Run `clang-tidy` (and `cppcheck` if configured) against modified targets and resolve all actionable diagnostics.

### 5.3 Dynamic Analysis & Sanitizers
- Build and run the test suite with **AddressSanitizer** and **UndefinedBehaviorSanitizer** (`-fsanitize=address,undefined`) whenever supported by the local toolchain.
- For concurrent/multithreaded code, additionally validate under **ThreadSanitizer** (`-fsanitize=thread`).
- Fix all memory leaks, out-of-bounds accesses, UB reports, and race conditions before finishing.

### 5.4 Test Execution & Verification Honesty
- Run the full affected test suite (e.g., `ctest --output-on-failure`) **after the final edit**.
- Inspect actual command output; never assume a build or test passes without running it.
- If a tool (e.g., `clang-tidy` or a sanitizer) is unavailable in the environment, explicitly report what was skipped and why.

---

## 6. ROS 2 Engineering Standards (Humble / C++)

All ROS 2 packages, nodes, launch files, and configurations in this repository must follow these principles.

### 6.1 Core-vs-Middleware Decoupling
- **Keep Algorithms ROS-Free**: Implement mathematical, estimation, geometry, and perception algorithms in pure C++/Eigen libraries with **zero** `rclcpp`, `tf2`, or ROS message dependencies.
- **Thin Node Wrappers**: ROS 2 nodes (`rclcpp::Node` or lifecycle nodes) must serve only as adapters: parameter loading, subscription/publication, TF2 lookups/broadcasts, message conversion, throttling, and diagnostics.
- **Standalone Testability**: Core libraries must compile and pass unit tests independently of the ROS 2 runtime.

### 6.2 Parameters, Topics & Configuration
- **No Hardcoded Interfaces**: Never hardcode topic names, frame IDs, covariance matrices, gating thresholds, or sensor toggles in C++ source files. Declare every configurable setting as a ROS 2 parameter with sensible defaults and load deployments from versioned YAML files under `config/`.
- **Validate Parameters at Startup**: Reject invalid parameter values (e.g., negative covariances, invalid fusion modes, non-positive timeouts, empty frame IDs) immediately on node construction with clear error messages.
- **Standard Message Types First**: Prefer standard ROS 2 interfaces (`sensor_msgs/Imu`, `nav_msgs/Odometry`, `geometry_msgs/PoseWithCovarianceStamped`, `geometry_msgs/TwistWithCovarianceStamped`, `nav_msgs/Path`) unless a custom message is explicitly approved and documented in `docs/interfaces/`.

### 6.3 Coordinate Frames, Units & TF2 Discipline
- **REP-103 & REP-105 Compliance**: Follow REP-103 strictly (SI units, right-handed ENU world frame, FLU vehicle body frame: $+X$ forward, $+Y$ left, $+Z$ up; CCW positive yaw) and REP-105 frame hierarchy (`map -> odom -> base_link -> <sensor_link>`).
- **No Duplicate TF Broadcasters**: A transform edge in `/tf` must have **exactly one** publisher at any time. TF broadcasting in estimation nodes must be explicitly toggleable (e.g., `publish_tf` parameter) so it never collides with a simulator's built-in odometry or static TF publishers.
- **Sanitize Frame IDs**: Strip any accidental leading `/` from incoming `frame_id` / `child_frame_id` strings before passing them to `tf2` (ROS 2 TF2 forbids leading slashes).
- **Transform Extrinsics Properly**: Never assume a sensor frame coincides with `base_link` without verifying or applying the `base_link <- sensor_frame` transform from TF2 (including lever-arm kinematics and covariance rotation).

### 6.4 Time, QoS & Runtime Hygiene
- **Honor Simulation Time**: Always use node clocks (`this->get_clock()`) and message header timestamps (`msg->header.stamp`), never `std::chrono::system_clock` or wall-clock time for estimation math. Every launch file must expose `use_sim_time`.
- **Guard Clock Anomalies**: Handle paused simulation (`dt == 0`), large time jumps (`dt > max_dt`), simulation resets (large backward time jump), and minor out-of-order timestamps explicitly and deterministically.
- **Appropriate QoS Profiles**: Match QoS intentionally — use `rclcpp::SensorDataQoS()` (or explicit reliability/depth) where required by high-rate sensor publishers, and standard reliable QoS for fused state / TF / control outputs. Document any non-default QoS choice.
- **Non-Blocking Callbacks & Bounded Memory**: Keep subscription callbacks short and lock scopes minimal. Never allow debug buffers, measurement queues, or visualization `nav_msgs/Path` histories to grow unbounded.
- **Throttled Logging**: Never spam `RCLCPP_INFO`/`WARN`/`ERROR` inside high-rate callbacks (e.g., 400 Hz IMU); use `RCLCPP_WARN_THROTTLE` / `RCLCPP_ERROR_THROTTLE` with the node clock.

### 6.5 Package & Build Hygiene
- **Third-Party Header Isolation**: Include ROS 2, Eigen, and GTest headers as `SYSTEM` includes in CMake so strict project warning flags (`-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wold-style-cast`) apply cleanly to repository code without third-party noise.
- **Clean Launch Files**: Provide self-contained Python launch files under `launch/` exposing `params_file` and `use_sim_time` arguments.

---

## 7. SLAM & Sensor Fusion Working Principles

Every localization, mapping, scan-matching, odometry, and sensor-fusion component must satisfy these domain principles.

### 7.1 Mathematical & Filtering Invariants
- **Manifold-Aware Operations**: Always normalize planar angles and angular innovations into $[-\pi, \pi)$ (`wrapAngle`). Never subtract raw Euler angles without wrapping; for 3D extensions, represent rotations on $SO(3)$ (quaternions / Lie algebra $\mathfrak{so}(3)$) rather than raw Euler integration.
- **Covariance Stability & Validity**:
  - Maintain symmetry ($P = \frac{1}{2}(P + P^\top)$) and positive semi-definiteness across every predict and update step.
  - Use the **Joseph form** $P^+ = (I - KH)P(I - KH)^\top + KRK^\top$ (or a square-root filter) instead of the algebraically simplified $(I - KH)P$ to prevent loss of positive definiteness under floating-point roundoff.
  - Solve linear systems with **Cholesky (`LDLT` / `LLT`)**, QR, or SVD — never call explicit `.inverse()` on innovation or information matrices.
- **Coriolis & Specific-Force Physics**: When fusing body-frame IMU linear acceleration with body-frame velocities on a turning vehicle, account for the rotating-frame term $\boldsymbol{\omega} \times \mathbf{v}_b$ (e.g., lateral centripetal acceleration $+\omega v_x$ in 2D) and gravity projection on sloped/pitched frames.

### 7.2 Multi-Sensor Fusion & Outlier Robustness
- **Prevent Drift Fighting & Double-Counting**: Never naively fuse multiple independent drifting absolute poses (`odom`-frame estimates) directly into the same pose state; use **differential / relative updates** or velocity/twist measurements for secondary odometry sources, reserving absolute pose updates for drift-free map-frame localization (or a single designated pose reference).
- **Validate Incoming Covariances**: Never trust raw covariance arrays blindly. Check finiteness, symmetry, and positive definiteness before every update. Because simulators and prototype nodes often publish all-zero covariances, fall back cleanly to configured YAML covariances with a throttled warning rather than dividing by zero or blowing up the filter.
- **Statistical Outlier Rejection**: Gate every measurement update using the **Mahalanobis distance** $d_M^2 = \mathbf{y}^\top S^{-1} \mathbf{y}$ against an appropriate $\chi^2$ threshold for the measurement dimension, and expose rejection counters for diagnostics.
- **Graceful Sensor Degradation**: A sensor dropout (e.g., LiDAR stall or visual odometry tracking loss) must never crash, freeze, or destabilize estimation. Track per-sensor last-seen timestamps, detect timeouts cleanly, and continue running with remaining sensors.

### 7.3 Observability, Extrinsics & Verification Discipline
- **Verify Observability & Jacobians**: Document which state variables are observable under each sensor subset, and test every analytical Jacobian against central finite differences in automated unit tests.
- **Keep Extrinsics Explicit**: Document all sensor mounting transforms relative to `base_link` from the authoritative URDF/SDF model and verify sign conventions before fusion.
- **Deterministic Offline Evaluation**: Validate estimation quality on deterministic synthetic trajectories first (known ground truth, pinned RNG seed) and on recorded rosbags during integration. Report quantitative metrics (position/heading RMSE, ATE/RPE, innovation consistency / NEES) rather than judging by eye alone.

---

## 8. Agent Working Principles & Definition of Done

### 8.1 Scope & Change Discipline
- Keep changes focused on the requested task; avoid unrelated refactors or formatting churn in untouched files.
- Prefer clean, incremental, reviewable commits/edits.
- Remove dead code, commented-out blocks, temporary files, and leftover debug prints before completing work.
- Surface assumptions, trade-offs, and open questions clearly when requirements are ambiguous.

### 8.2 Pre-Completion Checklist
An agent may report a task as complete only after verifying every applicable item:

- [ ] **Modern & Safe C++**: Code adheres to Section 1 (RAII, `const`-correctness, explicit types/units, numerical stability, no UB).
- [ ] **ROS 2 & SLAM Standards**: Code adheres to Sections 6 and 7 (pure C++/Eigen core decoupled from ROS 2, YAML-configured topics/frames/covariances, REP-103/105 & TF2 hygiene, Joseph-form covariance, Mahalanobis gating, zero-covariance fallback, `use_sim_time` support).
- [ ] **Code Documentation**: Public APIs have complete Doxygen comments; non-trivial implementation details explain *why* and link to theory docs.
- [ ] **Theory, Architecture & Sources**: Relevant `docs/architecture/` and/or `docs/theory/` files are created or updated with the big picture, notation, full math/derivations, code-to-math mapping, and honest, verified references.
- [ ] **Tests**: Unit, mathematical/invariant (including finite-difference Jacobian verification), edge-case, ROS 2 adapter/integration, and regression tests are written and meaningful.
- [ ] **Build & Warnings**: Project compiles cleanly with strict warnings enabled and zero new warnings.
- [ ] **Formatting & Static Analysis**: `clang-format`, `clang-tidy`, and/or `cppcheck` (as available) pass cleanly on modified files.
- [ ] **Sanitizers & Tests Pass**: Test suite passes cleanly (including ASan/UBSan on pure C++ core where available) after the final edit.
- [ ] **Clean Diff & Honest Report**: Diff is clean and focused; final summary states exactly what changed, what checks ran, and any limitations or unrun checks.
