# Architecture Codemap

## High-Level Shape
- Single Gradle project with Java robot code under `src/main/java/frc/robot`.
- Command-based WPILib structure with AdvantageKit logging.
- Main runtime chain is `Main` -> `Robot` -> `RobotContainer`.

## Major Repository Areas
- `src/main/java/frc/robot` — top-level lifecycle, constants, shared state.
- `src/main/java/frc/robot/commands` — reusable command builders such as `DriveCommands`, `AimCommand`, `PassCommand`, `FollowPoint`.
- `src/main/java/frc/robot/subsystems/drive` — swerve drive, odometry, gyro/module IO, trajectory following.
- `src/main/java/frc/robot/subsystems/vision` — camera IO, observation filtering, vision pose handoff.
- `src/main/java/frc/robot/subsystems/intake|flywheel|hood|feeder|clamber` — mechanism control and IO layers.
- `src/main/java/frc/robot/subsystems/aim` — computes shooter-related setpoints from robot state.
- `src/main/java/frc/robot/generated` — generated constants.
- `src/main/deploy` — Choreo and PathPlanner assets deployed to the robot.
- `vendordeps` — AdvantageKit, Phoenix 6, PhotonLib, ChoreoLib, and other vendor descriptors.

## Core Runtime Classes
- `src/main/java/frc/robot/Main.java` — starts WPILib with `Robot::new`.
- `src/main/java/frc/robot/Robot.java` — configures AdvantageKit, owns lifecycle methods, runs `CommandScheduler`.
- `src/main/java/frc/robot/RobotContainer.java` — constructs subsystems for REAL / SIM / REPLAY, configures controller bindings, builds autonomous routines.
- `src/main/java/frc/robot/RobotState.java` — singleton shared by estimation and aiming logic.

## Repeated Design Pattern
Subsystem packages commonly follow this split:
- subsystem class: `Drive`, `Vision`, `Hood`, `Flywheel`, `Intake`, `Feeder`, `Clamber`
- abstraction/interface: `*IO`
- simulation implementation: `*IOSim`
- hardware implementation: `*IOTalonFX`, `GyroIOPigeon2`, `VisionIOPhotonVision`

This means most hardware changes start in the subsystem package, not in `Robot`.

## Where to Look
- Change startup or mode behavior -> `src/main/java/frc/robot/Robot.java`
- Change controller bindings or auto selection -> `src/main/java/frc/robot/RobotContainer.java`
- Change driving/odometry/trajectory following -> `src/main/java/frc/robot/subsystems/drive`
- Change vision filtering or pose injection -> `src/main/java/frc/robot/subsystems/vision`
- Change aiming/pass setpoint logic -> `src/main/java/frc/robot/subsystems/aim`
- Change deploy-time path assets -> `src/main/deploy/choreo` and `src/main/deploy/pathplanner`
