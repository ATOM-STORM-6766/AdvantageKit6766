# Top-Level Entry Thinning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Slim `Robot` and `RobotContainer` by extracting mode-based hardware assembly, controls wiring, autonomous wiring, and teleop phase logic into focused collaborators without changing match behavior.

**Architecture:** Keep `frc.robot.Robot` as the lifecycle shell and `frc.robot.RobotContainer` as the application assembly entrypoint. Introduce `RobotHardware` + `RobotHardwareFactory` for mode-specific subsystem creation, `ControlsConfigurator` for bindings, `AutoConfigurator` for chooser/factory registration, and `TeleopPhaseCalculator` + `TeleopPhaseTracker` for teleop phase logging.

**Tech Stack:** Java 17, WPILib command-based framework, AdvantageKit, Choreo `AutoFactory`, GradleRIO, JUnit 5.

---

## File Structure

### New files
- `src/main/java/frc/robot/config/RobotHardware.java` — immutable aggregate of top-level subsystems created at startup.
- `src/main/java/frc/robot/config/RobotHardwareFactory.java` — the only place that switches over `Constants.currentMode` to build REAL/SIM/REPLAY hardware.
- `src/main/java/frc/robot/controls/ControlsConfigurator.java` — owns all driver/operator bindings currently in `RobotContainer.configureButtonBindings()`.
- `src/main/java/frc/robot/auto/AutoConfigurator.java` — owns `AutoFactory` creation, trajectory logging hook, and all `autoChooser.addCmd(...)` registration.
- `src/main/java/frc/robot/match/TeleopPhaseSnapshot.java` — immutable value object for teleop phase status.
- `src/main/java/frc/robot/match/TeleopPhaseCalculator.java` — pure logic extracted from `Robot.updateTeleopStatus()`.
- `src/main/java/frc/robot/match/TeleopPhaseTracker.java` — stateful timer/logging wrapper used by `Robot`.
- `src/test/java/frc/robot/match/TeleopPhaseCalculatorTest.java` — focused tests for phase calculation and blink behavior.

### Modified files
- `src/main/java/frc/robot/Robot.java:39-291` — replace embedded teleop phase logic with `TeleopPhaseTracker` and add small top-level observability logs.
- `src/main/java/frc/robot/RobotContainer.java:75-769` — replace inline mode wiring, control wiring, and auto wiring with focused collaborators; rename `m_intake` to `intake`.

### Verification commands
- `./gradlew test --tests frc.robot.match.TeleopPhaseCalculatorTest`
- `./gradlew build`

---

### Task 1: Extract pure teleop phase calculation logic

**Files:**
- Create: `src/main/java/frc/robot/match/TeleopPhaseSnapshot.java`
- Create: `src/main/java/frc/robot/match/TeleopPhaseCalculator.java`
- Test: `src/test/java/frc/robot/match/TeleopPhaseCalculatorTest.java`

- [ ] **Step 1: Write the failing test for the extracted phase logic**

```java
package frc.robot.match;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class TeleopPhaseCalculatorTest {
  @Test
  void returnsIdleStateWhenTimerHasNotStarted() {
    TeleopPhaseSnapshot snapshot = TeleopPhaseCalculator.calculate(0.0, false, true);

    assertFalse(snapshot.isOurShiftActive());
    assertEquals(0.0, snapshot.currentPhaseTimeRemainingSeconds(), 1e-9);
    assertEquals(140.0, snapshot.teleopTimeRemainingSeconds(), 1e-9);
  }

  @Test
  void keepsStartingAllianceActiveDuringShiftOneOutsideBlinkWindow() {
    TeleopPhaseSnapshot snapshot = TeleopPhaseCalculator.calculate(15.0, true, true);

    assertTrue(snapshot.isOurShiftActive());
    assertEquals(20.0, snapshot.currentPhaseTimeRemainingSeconds(), 1e-9);
    assertEquals(125.0, snapshot.teleopTimeRemainingSeconds(), 1e-9);
  }

  @Test
  void flipsOutputInsideBlinkWindowBeforeEndgame() {
    TeleopPhaseSnapshot snapshot = TeleopPhaseCalculator.calculate(30.0, true, true);

    assertFalse(snapshot.isOurShiftActive());
    assertEquals(5.0, snapshot.currentPhaseTimeRemainingSeconds(), 1e-9);
    assertEquals(110.0, snapshot.teleopTimeRemainingSeconds(), 1e-9);
  }

  @Test
  void keepsBothAlliancesActiveInEndgameWithoutBlinking() {
    TeleopPhaseSnapshot snapshot = TeleopPhaseCalculator.calculate(135.0, true, false);

    assertTrue(snapshot.isOurShiftActive());
    assertEquals(5.0, snapshot.currentPhaseTimeRemainingSeconds(), 1e-9);
    assertEquals(5.0, snapshot.teleopTimeRemainingSeconds(), 1e-9);
  }
}
```

- [ ] **Step 2: Run the test to verify the missing implementation fails**

Run: `./gradlew test --tests frc.robot.match.TeleopPhaseCalculatorTest`
Expected: FAIL with class-not-found or symbol-not-found errors for `TeleopPhaseSnapshot` / `TeleopPhaseCalculator`.

- [ ] **Step 3: Add the value object and pure calculator**

`src/main/java/frc/robot/match/TeleopPhaseSnapshot.java`

```java
package frc.robot.match;

public record TeleopPhaseSnapshot(
    boolean isOurShiftActive,
    double currentPhaseTimeRemainingSeconds,
    double teleopTimeRemainingSeconds) {}
```

`src/main/java/frc/robot/match/TeleopPhaseCalculator.java`

```java
package frc.robot.match;

public final class TeleopPhaseCalculator {
  private static final double TELEOP_SECONDS = 140.0;
  private static final double BLINK_WINDOW_SECONDS = 7.0;
  private static final double BLINK_PERIOD_SECONDS = 0.5;

  private TeleopPhaseCalculator() {}

  public static TeleopPhaseSnapshot calculate(
      double elapsedTeleopSeconds, boolean timerRunning, boolean startsActive) {
    if (elapsedTeleopSeconds == 0.0 && !timerRunning) {
      return new TeleopPhaseSnapshot(false, 0.0, TELEOP_SECONDS);
    }

    double remainingTime = Math.max(0.0, TELEOP_SECONDS - elapsedTeleopSeconds);
    boolean isOurShiftActive;
    double currentPhaseTimeRemainingSeconds;

    if (remainingTime > 130.0) {
      isOurShiftActive = true;
      currentPhaseTimeRemainingSeconds = remainingTime - 130.0;
    } else if (remainingTime > 105.0) {
      isOurShiftActive = startsActive;
      currentPhaseTimeRemainingSeconds = remainingTime - 105.0;
    } else if (remainingTime > 80.0) {
      isOurShiftActive = !startsActive;
      currentPhaseTimeRemainingSeconds = remainingTime - 80.0;
    } else if (remainingTime > 55.0) {
      isOurShiftActive = startsActive;
      currentPhaseTimeRemainingSeconds = remainingTime - 55.0;
    } else if (remainingTime > 30.0) {
      isOurShiftActive = !startsActive;
      currentPhaseTimeRemainingSeconds = remainingTime - 30.0;
    } else if (remainingTime > 0.0) {
      isOurShiftActive = true;
      currentPhaseTimeRemainingSeconds = remainingTime;
    } else {
      isOurShiftActive = false;
      currentPhaseTimeRemainingSeconds = 0.0;
    }

    if (currentPhaseTimeRemainingSeconds <= BLINK_WINDOW_SECONDS
        && currentPhaseTimeRemainingSeconds > 0.0
        && remainingTime > 30.0
        && (elapsedTeleopSeconds % BLINK_PERIOD_SECONDS) < (BLINK_PERIOD_SECONDS / 2.0)) {
      isOurShiftActive = !isOurShiftActive;
    }

    return new TeleopPhaseSnapshot(
        isOurShiftActive,
        Math.max(0.0, currentPhaseTimeRemainingSeconds),
        remainingTime);
  }
}
```

- [ ] **Step 4: Re-run the focused test and confirm it passes**

Run: `./gradlew test --tests frc.robot.match.TeleopPhaseCalculatorTest`
Expected: PASS for all four tests.

- [ ] **Step 5: Commit the isolated logic extraction**

```bash
git add src/main/java/frc/robot/match/TeleopPhaseSnapshot.java src/main/java/frc/robot/match/TeleopPhaseCalculator.java src/test/java/frc/robot/match/TeleopPhaseCalculatorTest.java
git commit -m "refactor: extract teleop phase calculation"
```

### Task 2: Slim `Robot` into a lifecycle shell

**Files:**
- Create: `src/main/java/frc/robot/match/TeleopPhaseTracker.java`
- Modify: `src/main/java/frc/robot/Robot.java:39-291`
- Test: `src/test/java/frc/robot/match/TeleopPhaseCalculatorTest.java`

- [ ] **Step 1: Add a tracker that owns the timer and logs outputs**

`src/main/java/frc/robot/match/TeleopPhaseTracker.java`

```java
package frc.robot.match;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class TeleopPhaseTracker {
  private final Timer timer = new Timer();
  private final BooleanSupplier startsActiveSupplier;

  public TeleopPhaseTracker(BooleanSupplier startsActiveSupplier) {
    this.startsActiveSupplier = startsActiveSupplier;
  }

  public void start() {
    timer.start();
  }

  public void stopAndReset() {
    timer.stop();
    timer.reset();
  }

  public void logOutputs() {
    TeleopPhaseSnapshot snapshot =
        TeleopPhaseCalculator.calculate(timer.get(), timer.isRunning(), startsActiveSupplier.getAsBoolean());
    Logger.recordOutput("Game/IsOurShiftActive", snapshot.isOurShiftActive());
    Logger.recordOutput(
        "Game/CurrentShiftTimeRemaining", snapshot.currentPhaseTimeRemainingSeconds());
    Logger.recordOutput("Game/TeleopTimeRemaining", snapshot.teleopTimeRemainingSeconds());
  }
}
```

- [ ] **Step 2: Replace the embedded phase logic in `Robot` with the tracker**

Update the fields near `src/main/java/frc/robot/Robot.java:39-44` to this shape:

```java
private Command autonomousCommand;
private RobotContainer robotContainer;
private static final LoggedTunableBoolean ourIsFirstActive =
    new LoggedTunableBoolean("OurIsFirstActive", true);
private final TeleopPhaseTracker teleopPhaseTracker =
    new TeleopPhaseTracker(ourIsFirstActive::get);
```

Delete the entire `updateTeleopStatus()` method at `src/main/java/frc/robot/Robot.java:124-192` and replace the lifecycle hooks with:

```java
@Override
public void teleopPeriodic() {
  teleopPhaseTracker.logOutputs();
}

@Override
public void teleopExit() {
  teleopPhaseTracker.stopAndReset();
}

@Override
public void teleopInit() {
  if (autonomousCommand != null) {
    autonomousCommand.cancel();
  }

  Command resetCommand =
      new ParallelCommandGroup(
              robotContainer.getHood().resetToLimitCommand().unless(robotContainer.getHood()::isInitialized),
              robotContainer.getIntake().resetToLimitCommand().unless(robotContainer.getIntake()::isInitialized))
          .withName("Reset To Limit");

  var feederAndFlywheelReset =
      new ParallelCommandGroup(robotContainer.getFeeder().stopCommand(), robotContainer.getFlywheel().stopCommand())
          .withName("Reset Feeder and Flywheel");

  CommandScheduler.getInstance().schedule(resetCommand.andThen(feederAndFlywheelReset));
  teleopPhaseTracker.start();
}
```

Also add two tiny observability logs:

```java
Logger.recordOutput("Robot/CurrentMode", Constants.currentMode.name());
```

right after `Logger.start();`, and

```java
Logger.recordOutput(
    "Robot/AutonomousCommand",
    autonomousCommand != null ? autonomousCommand.getName() : "None");
```

at the end of `autonomousInit()`.

- [ ] **Step 3: Re-run the focused calculator test to confirm the refactor did not break the pure logic**

Run: `./gradlew test --tests frc.robot.match.TeleopPhaseCalculatorTest`
Expected: PASS.

- [ ] **Step 4: Compile the robot project after the lifecycle refactor**

Run: `./gradlew build`
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 5: Commit the lifecycle-shell refactor**

```bash
git add src/main/java/frc/robot/Robot.java src/main/java/frc/robot/match/TeleopPhaseTracker.java src/main/java/frc/robot/match/TeleopPhaseSnapshot.java src/main/java/frc/robot/match/TeleopPhaseCalculator.java src/test/java/frc/robot/match/TeleopPhaseCalculatorTest.java
git commit -m "refactor: slim robot lifecycle logic"
```

### Task 3: Centralize mode-based hardware assembly

**Files:**
- Create: `src/main/java/frc/robot/config/RobotHardware.java`
- Create: `src/main/java/frc/robot/config/RobotHardwareFactory.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java:75-174`

- [ ] **Step 1: Add a single aggregate for the top-level subsystems**

`src/main/java/frc/robot/config/RobotHardware.java`

```java
package frc.robot.config;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;

public record RobotHardware(
    Drive drive,
    Vision vision,
    Flywheel flywheel,
    Feeder feeder,
    Hood hood,
    Intake intake) {}
```

- [ ] **Step 2: Move the `Constants.currentMode` switch into a dedicated factory**

`src/main/java/frc/robot/config/RobotHardwareFactory.java`

```java
package frc.robot.config;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.flywheel.LimitSwitchDIO;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

public final class RobotHardwareFactory {
  private RobotHardwareFactory() {}

  public static RobotHardware create(Constants.Mode mode) {
    return switch (mode) {
      case REAL -> createRealHardware();
      case SIM -> createSimHardware();
      case REPLAY -> createReplayHardware();
    };
  }

  private static RobotHardware createRealHardware() {
    Drive drive =
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));
    Vision vision =
        new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVision(camera0Name, robotToCamera0),
            new VisionIOPhotonVision(camera1Name, robotToCamera1));
    Hood hood = new Hood(new HoodIOTalonFX());
    Flywheel flywheel = new Flywheel(new FlywheelIOTalonFX(), new LimitSwitchDIO());
    Feeder feeder = new Feeder(new FeederIOTalonFX());
    Intake intake = new Intake(new IntakeIOTalonFX());
    return new RobotHardware(drive, vision, flywheel, feeder, hood, intake);
  }

  private static RobotHardware createSimHardware() {
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));
    Vision vision =
        new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
    Hood hood = new Hood(new HoodIOSim());
    Flywheel flywheel = new Flywheel(new FlywheelIOSim(), new LimitSwitchDIO());
    Feeder feeder = new Feeder(new FeederIOSim());
    Intake intake = new Intake(new IntakeIOSim());
    return new RobotHardware(drive, vision, flywheel, feeder, hood, intake);
  }

  private static RobotHardware createReplayHardware() {
    Drive drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
    Vision vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
    Hood hood = new Hood(new HoodIOSim());
    Flywheel flywheel = new Flywheel(new FlywheelIOSim(), new LimitSwitchDIO());
    Feeder feeder = new Feeder(new FeederIOSim());
    Intake intake = new Intake(new IntakeIOTalonFX());
    return new RobotHardware(drive, vision, flywheel, feeder, hood, intake);
  }
}
```

- [ ] **Step 3: Replace the inline switch in `RobotContainer` with the aggregate**

Update the field section near `src/main/java/frc/robot/RobotContainer.java:76-92` to:

```java
private final RobotHardware hardware;
private final Drive drive;
@SuppressWarnings("unused")
private final Vision vision;
private final Flywheel flywheel;
private final Feeder feeder;
private final Hood hood;
private final Intake intake;
private final AimSubsystem aimSubsystem = new AimSubsystem();
private final PassSubsystem passSubsystem = new PassSubsystem();
```

Then replace the constructor block at `src/main/java/frc/robot/RobotContainer.java:99-164` with:

```java
public RobotContainer() {
  hardware = RobotHardwareFactory.create(Constants.currentMode);
  drive = hardware.drive();
  vision = hardware.vision();
  flywheel = hardware.flywheel();
  feeder = hardware.feeder();
  hood = hardware.hood();
  intake = hardware.intake();

  autoChooser = new AutoChooser();
  autoFactory = null;
  SmartDashboard.putData("Auto Choices", autoChooser);
  updateAutoChooser();
  configureButtonBindings();
}
```

Finally, rename every `m_intake` reference in `RobotContainer.java` to `intake`, including the getter:

```java
public Intake getIntake() {
  return intake;
}
```

- [ ] **Step 4: Build after the constructor simplification**

Run: `./gradlew build`
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 5: Commit the mode-factory extraction**

```bash
git add src/main/java/frc/robot/config/RobotHardware.java src/main/java/frc/robot/config/RobotHardwareFactory.java src/main/java/frc/robot/RobotContainer.java
git commit -m "refactor: extract mode-based hardware factory"
```

### Task 4: Move controller bindings out of `RobotContainer`

**Files:**
- Create: `src/main/java/frc/robot/controls/ControlsConfigurator.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java:588-759`

- [ ] **Step 1: Create a focused configurator for driver and operator bindings**

`src/main/java/frc/robot/controls/ControlsConfigurator.java`

```java
package frc.robot.controls;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.AimCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FollowPoint;
import frc.robot.commands.PassCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelSetpoint;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;

public class ControlsConfigurator {
  private final RobotContainer robotContainer;
  private final Drive drive;
  private final Flywheel flywheel;
  private final Feeder feeder;
  private final Intake intake;
  private final CommandXboxController controller;
  private final CommandPS5Controller operator;

  public ControlsConfigurator(
      RobotContainer robotContainer,
      Drive drive,
      Flywheel flywheel,
      Feeder feeder,
      Intake intake,
      CommandXboxController controller,
      CommandPS5Controller operator) {
    this.robotContainer = robotContainer;
    this.drive = drive;
    this.flywheel = flywheel;
    this.feeder = feeder;
    this.intake = intake;
    this.controller = controller;
    this.operator = operator;
  }

  public void configure() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Move the remaining command chains from RobotContainer.configureButtonBindings()
    // into this method unchanged, replacing only:
    // - `this` with `robotContainer`
    // - `m_intake` with `intake`
  }
}
```

- [ ] **Step 2: Move the existing bindings without changing behavior**

Copy the current binding chains from `src/main/java/frc/robot/RobotContainer.java:596-758` into `ControlsConfigurator.configure()`.

Specific replacements that must be made while copying:

```java
AimCommand.autoAimAtTarget(
    robotContainer,
    () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint),
    () -> -controller.getLeftY(),
    () -> -controller.getLeftX())
```

```java
PassCommand.passAtTarget(
    robotContainer,
    () -> {
      var target = new Translation3d(FieldConstants.Hub.passPoint);
      target = AllianceFlipUtil.apply(target);
      target =
          AllianceFlipUtil.apply(drive.getPose()).getTranslation().getY() > FieldConstants.fieldWidth / 2
              ? target
              : AllianceFlipUtil.mirror(target);
      return target;
    },
    () -> -controller.getLeftY(),
    () -> -controller.getLeftX())
```

Every `m_intake` call must become `intake`, for example:

```java
Commands.parallel(intake.setIntakeVelocityCommand(Volts.of(10)), intake.setPosCommand(() -> Degrees.of(0)))
    .withName("Intake")
```

- [ ] **Step 3: Replace the inline method in `RobotContainer` with a one-line delegation**

Replace `configureButtonBindings()` in `src/main/java/frc/robot/RobotContainer.java:588-759` with:

```java
private void configureButtonBindings() {
  new ControlsConfigurator(this, drive, flywheel, feeder, intake, controller, operator).configure();
}
```

Also remove no-longer-used imports from `RobotContainer.java` after the move.

- [ ] **Step 4: Build after extracting the controls wiring**

Run: `./gradlew build`
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 5: Commit the controls extraction**

```bash
git add src/main/java/frc/robot/controls/ControlsConfigurator.java src/main/java/frc/robot/RobotContainer.java
git commit -m "refactor: extract controller bindings"
```

### Task 5: Move auto wiring out of `RobotContainer`

**Files:**
- Create: `src/main/java/frc/robot/auto/AutoConfigurator.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java:176-549`

- [ ] **Step 1: Create a dedicated auto configurator**

`src/main/java/frc/robot/auto/AutoConfigurator.java`

```java
package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.AimCommand;
import frc.robot.commands.FollowPoint;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelSetpoint;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class AutoConfigurator {
  private final RobotContainer robotContainer;
  private final Drive drive;
  private final Flywheel flywheel;
  private final Feeder feeder;
  private final Hood hood;
  private final Intake intake;

  public AutoConfigurator(
      RobotContainer robotContainer,
      Drive drive,
      Flywheel flywheel,
      Feeder feeder,
      Hood hood,
      Intake intake) {
    this.robotContainer = robotContainer;
    this.drive = drive;
    this.flywheel = flywheel;
    this.feeder = feeder;
    this.hood = hood;
    this.intake = intake;
  }

  public AutoFactory configure(AutoChooser autoChooser) {
    AutoFactory autoFactory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            drive,
            (trajectory, isFinished) -> {
              Logger.recordOutput(
                  "Odometry/Trajectory",
                  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                      ? trajectory.flipped().getPoses()
                      : trajectory.getPoses());
              Logger.recordOutput("Odometry/TrajectoryIsFinished", isFinished);
            });

    // Move the existing command fragment definitions and autoChooser.addCmd(...) registrations
    // from RobotContainer.configureAutoChooser() into this method unchanged, replacing only:
    // - `this` with `robotContainer`
    // - `m_intake` with `intake`

    return autoFactory;
  }
}
```

- [ ] **Step 2: Move the existing auto definitions without changing names or sequencing**

Copy the current command fragment definitions and chooser registrations from `src/main/java/frc/robot/RobotContainer.java:208-548` into `AutoConfigurator.configure(...)`.

Mandatory mechanical replacements while copying:

```java
var resetCmd = Commands.parallel(intake.resetToLimitCommand().alongWith(hood.resetToLimitCommand()));
```

```java
var intakeCmd =
    Commands.parallel(intake.setIntakeVelocityCommand(Volts.of(10)), intake.setPosCommand(() -> Degrees.of(0)))
        .withName("Intake");
```

```java
AimCommand.autoAimAtTarget(
    robotContainer,
    () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint),
    () -> 0.0,
    () -> 0.0)
```

Keep all chooser names unchanged: `"P2"`, `"P3"`, `"P4"`, `"P6"`, `"P6_mirror"`, and `"p2_1"`.

- [ ] **Step 3: Replace `RobotContainer.configureAutoChooser()` with a thin delegation and add top-level auto logs**

Update `RobotContainer` so `updateAutoChooser()` becomes:

```java
public void updateAutoChooser() {
  if (autoFactory == null && DriverStation.getAlliance().isPresent()) {
    autoFactory =
        new AutoConfigurator(this, drive, flywheel, feeder, hood, intake).configure(autoChooser);
    SmartDashboard.putData("Auto Choices", autoChooser);
    Logger.recordOutput("Robot/Auto/ChooserReady", true);
  } else if (autoFactory == null) {
    Logger.recordOutput("Robot/Auto/ChooserReady", false);
  }
}
```

Delete the old `configureAutoChooser()` method entirely.

- [ ] **Step 4: Build after the auto extraction**

Run: `./gradlew build`
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 5: Commit the auto extraction**

```bash
git add src/main/java/frc/robot/auto/AutoConfigurator.java src/main/java/frc/robot/RobotContainer.java
git commit -m "refactor: extract autonomous configuration"
```

### Task 6: Final cleanup and verification

**Files:**
- Modify: `src/main/java/frc/robot/Robot.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java`
- Modify: `src/main/java/frc/robot/config/RobotHardwareFactory.java`
- Modify: `src/main/java/frc/robot/controls/ControlsConfigurator.java`
- Modify: `src/main/java/frc/robot/auto/AutoConfigurator.java`

- [ ] **Step 1: Remove dead imports and keep the top-level APIs small**

After the extractions, make sure these APIs remain intact in `RobotContainer`:

```java
public Hood getHood() { return hood; }
public Flywheel getFlywheel() { return flywheel; }
public Feeder getFeeder() { return feeder; }
public Intake getIntake() { return intake; }
public AimSubsystem getAimSubsystem() { return aimSubsystem; }
public PassSubsystem getPassSubsystem() { return passSubsystem; }
public Drive getDrive() { return drive; }
public Command getAutonomousCommand() { return autoChooser.selectedCommand(); }
```

- [ ] **Step 2: Run formatting and the full build**

Run: `./gradlew spotlessApply build`
Expected: Spotless may rewrite imports/formatting, then the build finishes with `BUILD SUCCESSFUL`.

- [ ] **Step 3: Sanity-check the extracted responsibilities in the final diff**

Run:

```bash
git diff -- src/main/java/frc/robot/Robot.java src/main/java/frc/robot/RobotContainer.java src/main/java/frc/robot/config/RobotHardware.java src/main/java/frc/robot/config/RobotHardwareFactory.java src/main/java/frc/robot/controls/ControlsConfigurator.java src/main/java/frc/robot/auto/AutoConfigurator.java src/main/java/frc/robot/match/TeleopPhaseSnapshot.java src/main/java/frc/robot/match/TeleopPhaseCalculator.java src/main/java/frc/robot/match/TeleopPhaseTracker.java src/test/java/frc/robot/match/TeleopPhaseCalculatorTest.java
```

Expected review points:
- `Robot.java` no longer contains teleop phase branching logic.
- `RobotContainer.java` no longer contains the mode switch or large inline binding/auto blocks.
- `RobotHardwareFactory` is the only top-level location switching on `Constants.currentMode` for subsystem assembly.
- `ControlsConfigurator` and `AutoConfigurator` preserve existing behavior and names.

- [ ] **Step 4: Create the final refactor commit**

```bash
git add src/main/java/frc/robot/Robot.java src/main/java/frc/robot/RobotContainer.java src/main/java/frc/robot/config/RobotHardware.java src/main/java/frc/robot/config/RobotHardwareFactory.java src/main/java/frc/robot/controls/ControlsConfigurator.java src/main/java/frc/robot/auto/AutoConfigurator.java src/main/java/frc/robot/match/TeleopPhaseSnapshot.java src/main/java/frc/robot/match/TeleopPhaseCalculator.java src/main/java/frc/robot/match/TeleopPhaseTracker.java src/test/java/frc/robot/match/TeleopPhaseCalculatorTest.java
git commit -m "refactor: thin top-level robot entrypoints"
```

## Self-Review

- **Spec coverage:** The plan covers the four agreed improvements: slim `Robot`, slim `RobotContainer`, centralize mode-specific assembly, and improve top-level debugging visibility with explicit `Robot/*` outputs.
- **Placeholder scan:** No `TODO`, `TBD`, or unnamed tasks remain. Every task lists exact files and verification commands.
- **Type consistency:** `RobotHardware`, `ControlsConfigurator`, `AutoConfigurator`, `TeleopPhaseSnapshot`, `TeleopPhaseCalculator`, and `TeleopPhaseTracker` use the same names and signatures across all tasks.
