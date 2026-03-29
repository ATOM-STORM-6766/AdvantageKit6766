# Project Instructions

## Repo Snapshot
- Java 17 robot project built with GradleRIO/WPILib and AdvantageKit.
- Main package: `src/main/java/frc/robot`.
- Runtime starts at `src/main/java/frc/robot/Main.java`, then `Robot`, then `RobotContainer`.

## Where to Start
- `src/main/java/frc/robot/Main.java` — JVM entry point.
- `src/main/java/frc/robot/Robot.java` — lifecycle hooks, logging/replay setup, scheduler loop.
- `src/main/java/frc/robot/RobotContainer.java` — subsystem wiring, controller bindings, autonomous chooser.
- `src/main/java/frc/robot/RobotState.java` — shared pose/speed/aim state.

## Key Directories
- `src/main/java/frc/robot/commands` — command composition for driving, aiming, passing, path following.
- `src/main/java/frc/robot/subsystems` — mechanism code grouped by domain (`drive`, `vision`, `intake`, `flywheel`, `hood`, `feeder`, `clamber`, `aim`).
- `src/main/java/frc/robot/generated` — generated constants such as `TunerConstants` / `BuildConstants`.
- `src/main/deploy` — deploy-time assets including `choreo/` and `pathplanner/`.
- `vendordeps` — vendor dependency descriptors.

## Confirmed Commands
- Build + CI baseline: `./gradlew build`
- Run tests: `./gradlew test`
- Check formatting: `./gradlew spotlessCheck`
- Apply formatting: `./gradlew spotlessApply`
- Run simulator: `./gradlew simulateJava`
- Replay helper: `./gradlew replayWatch`

## Conventions
- Formatting is enforced with Spotless and `googleJavaFormat()` for Java.
- Package layout is domain-first under `frc.robot`.
- Subsystems commonly use an IO split: main subsystem class plus `*IO`, `*IOSim`, and hardware-backed implementations such as `*IOTalonFX` or `GyroIOPigeon2`.
- `robotPeriodic()` is kept thin; most behavior lives in subsystems, commands, and `RobotContainer` wiring.
- No `src/test` tree is currently checked in, even though JUnit 5 is configured.

## Cautions
- `build.gradle` contains an `eventDeploy` task that auto-stages and attempts a git commit when deploy-related tasks run on branches prefixed with `event`.
- Avoid changing generated files unless the generation source is understood.
- Prefer evidence from code/config over assumptions; mode-specific behavior is split across REAL / SIM / REPLAY paths.

## Codemaps
- `docs/CODEMAPS/architecture.md`
- `docs/CODEMAPS/runtime-flow.md`
- `docs/CODEMAPS/build-test-style.md`
