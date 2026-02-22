# Copilot Instructions — 2026 REBUILT FRC Robot Code

## Project Overview
FRC Java robot code for the 2026 REBUILT game. WPILib Command-based with AdvantageKit logging, CTRE Phoenix 6 swerve drive, PathPlanner navigation, Limelight vision, and MapleSim physics simulation. Inspired heavily by Team 254's architecture.

## Architecture

### Three-Mode Runtime (`Constants.Mode`)
Every subsystem and IO layer branches on `Constants.currentMode` (`REAL`, `SIM`, `REPLAY`). See `RobotContainer` constructor for the canonical switch. REPLAY mode uses no-op IO interfaces (`new IntakeIO() {}`).

### Subsystem IO Pattern
Each subsystem follows a 4-file pattern (e.g. `subsystems/intake/`):
- **`IntakeIO.java`** — Interface with `@AutoLog` inputs class and default no-op methods
- **`IntakeIOTalonFX.java`** — Hardware implementation (Phoenix 6 TalonFX)
- **`IntakeIOSim.java`** — Simulation implementation (simple math, no physics engine)
- **`IntakeSubsystem.java`** — Command-based subsystem that calls `io.updateInputs()` + `Logger.processInputs()` in `periodic()`

When adding a new subsystem, replicate this exact pattern. The `@AutoLog` annotation on the inputs class auto-generates `*AutoLogged` classes at compile time.

### Drive System (CTRE SwerveDrivetrain)
- `DriveIOHardware` extends `SwerveDrivetrain<TalonFX, TalonFX, CANcoder>` directly (254's approach, NOT the AdvantageKit IO pattern)
- `DriveIOSim` extends `DriveIOHardware` and adds MapleSim world sync
- `DriveSwerveDrivetrain` is the WPILib `SubsystemBase` wrapper that owns the IO
- Odometry runs at **250Hz** on a high-priority thread; poses flow into `RobotState` via telemetry consumer

### RobotState (254-style)
`RobotState.java` is the single source of truth for robot pose and turret angle history. Uses `TreeMap<Double, Pose2d>` keyed by FPGA timestamp with a 2-second buffer. All pose consumers (vision, auto, aiming) read from here, never from the drive directly.

### Superstructure (254-style "Wanted State")
`Superstructure.java` coordinates all scoring subsystems via a state machine. `periodic()` runs every 20ms and continuously applies the `wantedState` to all subsystems using direct void methods (e.g. `turret.applyAiming()`, `intake.applyIntake()`). State transitions are instant `runOnce` commands — no long-running `run()` commands needed. Both auto and teleop use the same `setWantedState()` API.

States: `IDLE`, `ONLY_INTAKE`, `ONLY_AIMING`, `ONLY_SHOOTING`, `AIMING_WHILE_INTAKING`, `SHOOTING_WHILE_INTAKING`, `EJECT`, `CLIMB_MODE`, `EMERGENCY`.

### Vision System
`VisionSubsystem` processes 3 Limelight cameras (front, back, turret-mounted). Supports both MegaTag1 (multi-tag) and MegaTag2 (gyro-fused). Vision estimates are consumed via a `Consumer<VisionFieldPoseEstimate>` that feeds into CTRE's pose estimator. **Vision is only active in REAL mode** — SIM and REPLAY use no-op IO.

### Pose Initialization (254-style)
The robot's pose is **pre-seeded during disabled**, not hard-reset at autonomousInit. This preserves vision corrections accumulated before the match starts.

- **`RobotState` constructor** seeds a zero pose at t=0 so the pose history is never empty.
- **`disabledPeriodic()`** (every 50 iterations ≈ 1 sec): when dashboard auto settings change, pre-seeds odometry via `drive.setPose(startingPose)`. Also publishes a `"Near Auto Starting Pose"` boolean (0.25 m / 8° tolerance, matching 254).
- **`autonomousInit()`** does NOT reset the pose — it trusts the pre-seeded + vision-refined estimate and just schedules the auto command.
- **`AutoCommandBuilder.buildSetStartPose()`** logs the expected vs actual start pose for verification instead of resetting odometry.
- **`teleopInit()`** has a `hasBeenEnabled` fallback: if the robot was never enabled (e.g. practice), it resets heading to the alliance wall orientation (0° blue / 180° red) while preserving translation.

### ShooterSetpoint
`util/ShooterSetpoint.java` computes a coordinated aiming solution (turret angle, hood angle, flywheel speed, feedforward) from distance-based interpolation maps. A single `Supplier<ShooterSetpoint>` is shared by turret, hood, and shooter subsystems to ensure they aim at the same target.

## Auto System (`auto/dashboard/`)
A dashboard-driven planning system inspired by 254/6328:

| File | Role |
|------|------|
| `AutoSettings.java` | Reads SmartDashboard choosers (start pose, intake priority, climb level, risk) |
| `AutoPlanner.java` | Stateless planner: settings → `List<AutoAction>` (deterministic, runs in µs) |
| `AutoAction.java` | Sealed hierarchy of action types: `SetStartPose`, `ScoreAt`, `IntakeAt`, `DriveTo`, `Climb` |
| `AutoCommandBuilder.java` | Converts action list → WPILib Command tree with runtime time-budgeting |
| `DashboardAutoManager.java` | Lifecycle manager: calls `update()` in `disabledPeriodic`, provides command at `autonomousInit` |
| `AutoTuning.java` | Behavioral tuning knobs (SWD gating, current detection thresholds) |
| `FieldConstants.java` | Field geometry, zone definitions, trench detection, alliance mirroring |

**Key pattern:** The planner generates an optimistic plan; the command builder makes runtime time decisions using FPGA countdown. Shoot-while-driving (SWD) is zone-based: D/O intakes → SWD, U/L intakes → stop-and-shoot.

## Climb System
4-motor cable-driven 2-link arm with inverse kinematics (`ClimbIK.java`) and Cartesian path planning (`ClimbPathPlanner.java`). Uses `ClimbState` enum for state machine transitions. REV Smart Robot Servos release passive hooks.

## Build & Tooling
- **Java 17**, GradleRIO 2026.1.1
- `./gradlew build` — compile + spotless format check
- `./gradlew deploy` — deploy to roboRIO
- `./gradlew simulateJava` — run physics simulation (MapleSim + AdvantageKit)
- **Spotless** auto-formats on compile (`googleJavaFormat`). Run `./gradlew spotlessApply` to fix formatting.
- **gversion** plugin generates `BuildConstants.java` with git metadata
- **Event deploy**: on `event*` branches, `eventDeploy` task auto-commits before deploy
- Vendor deps: Phoenix 6, PathPlannerLib, AdvantageKit, REVLib, PhotonLib, MapleSim, Studica

## Conventions
- All coordinates use **WPILib blue-alliance origin** (0,0 = bottom-left of blue DS wall). Red alliance poses are mirrored at runtime via `FieldConstants.flipPose()`.
- Constants live in `Constants.java` nested classes (`DriveConstants`, `TurretConstants`, etc.), NOT in subsystem files.
- CAN IDs and bus names are in `Constants.*Constants` classes. Superstructure devices use `"Superstructure"` CAN bus.
- Swerve module configs are in `generated/TunerConstants.java` (CTRE Tuner X generated — do not hand-edit).
- Use `Logger.recordOutput()` (AdvantageKit) for all telemetry, not `SmartDashboard.put*()`.
- Use `ChezySequenceCommandGroup` (254's optimized version) instead of `SequentialCommandGroup` in auto sequences for faster execution.
- Trench detection uses hysteresis (`TRENCH_EXIT_HOLDOFF_SECONDS`) to prevent state flutter at zone boundaries.
