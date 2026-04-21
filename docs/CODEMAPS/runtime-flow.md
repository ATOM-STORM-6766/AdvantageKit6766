# Runtime Flow Codemap

## Startup Flow
1. `src/main/java/frc/robot/Main.java` calls `RobotBase.startRobot(Robot::new)`.
2. `src/main/java/frc/robot/Robot.java` records build metadata and configures AdvantageKit logging.
3. `Robot` selects REAL / SIM / REPLAY logging behavior from `Constants.currentMode`.
4. `Robot` starts the logger and constructs `RobotContainer`.
5. `src/main/java/frc/robot/RobotContainer.java` wires subsystems and input devices, then initializes the auto chooser.

## Mode-Specific Wiring
- REAL: hardware-backed IO such as `GyroIOPigeon2`, `ModuleIOTalonFX`, `VisionIOPhotonVision`, `HoodIOTalonFX`.
- SIM: sim IO such as `ModuleIOSim`, `VisionIOPhotonVisionSim`, `HoodIOSim`, `FlywheelIOSim`.
- REPLAY/default branch: mostly inert or sim-backed IO so logic can run without live hardware.

## Periodic Loop
1. `Robot.robotPeriodic()` runs `CommandScheduler.getInstance().run()`.
2. The scheduler polls buttons, schedules commands, runs active commands, and invokes subsystem `periodic()` methods.
3. `disabledPeriodic()` refreshes the autonomous chooser once alliance information is available.

## Data / Control Flow
1. Driver and operator bindings are declared in `RobotContainer`.
2. Default manual driving uses `DriveCommands.joystickDrive(...)`.
3. Command code sends chassis requests into `subsystems/drive/Drive.java` via `runVelocity(...)`.
4. `Drive.periodic()` updates gyro/module inputs, advances odometry, and writes pose + field-relative speeds into `RobotState`.
5. `subsystems/vision/Vision.java` filters pose observations and forwards accepted measurements back into drive via `drive::addVisionMeasurement`.
6. Aim/pass logic reads `RobotState` and target suppliers to compute hood/flywheel/yaw setpoints.

## Autonomous Flow
1. `RobotContainer.configureAutoChooser()` builds a `choreo.auto.AutoFactory`.
2. The factory uses drive pose getters/setters plus `drive::followTrajectory`.
3. Named autos are added to `AutoChooser` as command factories.
4. `Robot.autonomousInit()` fetches the selected command and schedules it.
5. Autonomous routines compose reset/homing, path following, intake/feed/shoot, and stop commands.

## Teleop Flow
1. `Robot.teleopInit()` cancels any active autonomous command.
2. Teleop startup schedules hood/intake homing if needed and clears feeder/flywheel outputs.
3. The match timer is used to compute and log shift-state outputs during teleop.

## Shared State Notes
- `RobotState` is the cross-cutting state bus for pose, field-relative speeds, and aim setpoint.
- `Robot` stays thin by design; most operational behavior lives in `RobotContainer`, command classes, and subsystem `periodic()` methods.
