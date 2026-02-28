# Copilot Instructions — 2026 REBUILT FRC Robot (Team 0, Amped)

## Architecture
WPILib Command-based robot on **AdvantageKit** (logging/replay), **CTRE Phoenix 6** swerve, **Maple-Sim** physics sim. Follows **Team 254** patterns.

| Layer | Role |
|---|---|
| `Robot.java` | `LoggedRobot`; mode transitions, 254-style pose pre-seeding during disabled, sim init |
| `RobotContainer.java` | Subsystem construction (REAL/SIM/REPLAY switch), auto chooser, PathPlanner config, button bindings |
| `Superstructure.java` | 254-style "wanted state" coordinator — `periodic()` applies state to all subsystems every 20ms via void calls |
| `RobotState.java` | Timestamped pose + turret buffer shared across subsystems and vision |

### Subsystem IO Pattern
`SubsystemIO` (interface) → `SubsystemIOTalonFX` / `SubsystemIOSim` → `SubsystemSubsystem`. Example: `subsystems/turret/{TurretIO, TurretIOTalonFX, TurretIOSim, TurretSubsystem}`.
- `@AutoLog` on `*IOInputs` inner class generates `*IOInputsAutoLogged` at compile time.
- IO interfaces use `default` empty methods — REPLAY mode needs no implementation.
- Subsystems expose Command-returning methods (teleop) **and** `void apply*()` methods (Superstructure periodic).

## Superstructure State Machine
`periodic()` continuously applies `wantedState` — does NOT use commands internally.
- `setWantedState(state)` → instant `Command` (for bindings/sequences)
- `forceWantedState(state)` → void (for `Commands.run()` polling loops)
- `forceIdleState()` → bypasses EMERGENCY guard (use at auto start)
- States: `IDLE`, `ONLY_INTAKE`, `ONLY_AIMING`, `ONLY_SHOOTING`, `AIMING_WHILE_INTAKING`, `SHOOTING_WHILE_INTAKING`, `EJECT`, `EMERGENCY`
- **Intake shake:** Operator toggles `intakeHalfDeployed` flag via `setIntakeHalfDeploy()`/`setIntakeFullDeploy()`. When true, `periodic()` calls `intakePivot.applyHalfDeploy()` instead of `applyDeploy()`, oscillating the pivot to dislodge stuck FUEL.

## ShooterSetpoint Pipeline
`ShooterSetpoint.createSupplier(robotState)` returns a shared `Supplier<ShooterSetpoint>` wired to turret, hood, and shooter in `RobotContainer`. Each cycle it computes:
- Turret angle (field-relative, accounts for turret offset from robot center)
- Hood angle + flywheel speed (distance-based interpolation maps)
- Motion compensation (predicts future pose for shoot-while-driving)
- Feedforward angular rates (filtered) for turret/hood tracking
- Neutral zone detection — aims at alternate targets when robot is past `AIMING_ZONE_MAX_X`

## Climb Subsystem
Independent from Superstructure — operator controls climb at any time while intake/shooting continues.
- **State machine:** `ClimbState` enum defines waypoints as `Translation2d` paths. Teleop advances one step per POV press: `STOWED → EXTEND_L1 → RETRACT_L1 → RELEASE_ANGLE_L1 → ... → EXTEND_L3 → RETRACT_L3`.
- **Modes:** Normal (POV-driven), Calibration (`operator.leftBumper()` toggle — raw motor voltage via POV/buttons), Manual (`operator.rightBumper()` toggle — stick-controlled end-effector velocity).
- Auto climb: `AutoCommandBuilder` gates `climb.nextClimbStep()` behind intake-stowed check.

## Vision
`VisionSubsystem` uses Limelight MegaTag2 with two cameras: front drivetrain camera (`limelight-right`) and turret-mounted camera (`limelight-turret`). Turret camera has a 6× std-dev multiplier (timing mismatch between image capture and turret heading). Pose estimates feed into CTRE's pose estimator via `drive.getDriveIO().addVisionMeasurement()`.

## Autonomous System
Registered via `autoChooser.addOption()` in `RobotContainer`:

1. **Dashboard Auto** (`auto/dashboard/`) — `AutoSettings` → `AutoPlanner` → `AutoCommandBuilder` → `DashboardAutoManager`. Shoot-while-driving, current-based FUEL detection, runtime time budgeting.
2. **Hardcoded Auto** (`auto/HardcodedAutos.java`) — Lane-based fallback (UPPER/CENTER/LOWER).
3. **Custom Autos** (`auto/SweepAuto.java`, etc.) — Standalone commands.

**Adding a new auto:** Create class in `frc.robot.auto`, accept `(DriveSwerveDrivetrain, Superstructure, DashboardAutoManager)`, expose `getCommand()` returning a `Commands.defer()`-wrapped command, register with `autoChooser.addOption()` in `RobotContainer`.

### Field Geometry
All coordinates **blue-alliance origin**. REBUILT uses **point symmetry** (180° rotation about field center) — use `FieldConstants.flipPose()`/`flipTranslation()` for red. Key files: `FieldConstants.java` (zones, waypoints, trenches), `AutoTuning.java` (behavioral thresholds).

## Build & Development
- **Java 17**, **GradleRIO 2026.1.1**, Windows: `.\gradlew.bat`
- `.\gradlew.bat build` — Spotless (google-java-format) + BOM stripping + gversion
- `.\gradlew.bat compileJava` — compile only
- `.\gradlew.bat deploy` — deploy to roboRIO
- `.\gradlew.bat simulateJava` — Maple-Sim arena with FUEL game pieces
- Spotless auto-runs before compile. Code is formatted on build.
- Vendor deps: Phoenix6, REVLib, PathplannerLib, AdvantageKit, maple-sim, PhotonLib, Studica

## Conventions
- **Logging:** `Logger.recordOutput("Subsystem/Key", value)` for telemetry; `Logger.processInputs()` for IO.
- **Constants:** `Constants.java` (nested `XxxConstants` per subsystem) or `AutoTuning.java` (auto thresholds). No magic numbers.
- **Drive:** `drive.driveFieldRelative(vx, vy, omega)`, `drive.getPose()` from RobotState, `AutoBuilder.pathfindToPose()` for pathing.
- **Deferred commands:** `Commands.defer(() -> ..., Set.of(drive))` for alliance-aware auto commands.
- **ChezySequenceCommandGroup:** Use in performance-critical auto paths (runs multiple commands per cycle).
- **Trenches:** Snap heading to cardinal near trenches (22.25in ceiling). PathPlanner rotation override + teleop trench assist in `FieldConstants`.
