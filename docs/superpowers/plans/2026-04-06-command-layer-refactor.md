# Command Layer Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将自动模式配置从 `AutoConfigurator` 中拆出，按执行功能重组复用 command，回收 `RobotHardware` 到 `RobotContainer`，并在不改变比赛行为的前提下整理 `RobotControl`。

**Architecture:** 顶层由 `frc.robot.AutoModeSelector` 负责 `AutoFactory` 创建、auto routine 注册和 chooser 对外接口。自动流程拆到 `frc.robot.commands.auto.routines`，而 intake/feed/shoot/follow-point 这类可复用动作上提到 `frc.robot.commands` 下按功能聚合，使 teleop 与 auto 共用同一套 command builder。`RobotContainer` 改回直接按模式构造硬件，`RobotControl` 只负责绑定并按 controller/operator 两段组织。

**Tech Stack:** Java 17, WPILib command-based v2, Choreo `AutoFactory`/`AutoChooser`, AdvantageKit, GradleRIO, Spotless/google-java-format

---

## 文件结构

### 新建文件
- `src/main/java/frc/robot/AutoModeSelector.java` — 持有 `AutoChooser`，延迟创建 `AutoFactory`，注册所有 auto routines，并向 `RobotContainer` 暴露 `update()` / `selectedCommand()`。
- `src/main/java/frc/robot/commands/GamePieceCommands.java` — 聚合 intake、wave、feed、stop feeding、机构归零等进球/喂料相关 command。
- `src/main/java/frc/robot/commands/ShootingCommands.java` — 聚合自动瞄准射球、定点射球、停止射球等射球类 command。
- `src/main/java/frc/robot/commands/AutoDriveCommands.java` — 聚合 auto 使用的 follow point、停底盘、轨迹停转等移动类 command。
- `src/main/java/frc/robot/commands/auto/routines/P2Auto.java` — `P2` routine。
- `src/main/java/frc/robot/commands/auto/routines/P3Auto.java` — `P3` routine。
- `src/main/java/frc/robot/commands/auto/routines/P4Auto.java` — `P4` routine。
- `src/main/java/frc/robot/commands/auto/routines/P6Auto.java` — `P6` routine。
- `src/main/java/frc/robot/commands/auto/routines/P6MirrorAuto.java` — `P6_mirror` routine。
- `src/main/java/frc/robot/commands/auto/routines/P21Auto.java` — chooser 名称仍为 `p2_1`，内部继续走 `t5` 轨迹。

### 修改文件
- `src/main/java/frc/robot/RobotContainer.java` — 删除 `RobotHardware`/`AutoConfigurator` 依赖，内联硬件构造，接入 `AutoModeSelector`。
- `src/main/java/frc/robot/RobotControl.java` — 改成 `configureControllerBindings()` / `configureOperatorBindings()`，复用新的 command builder，并补中文注释。

### 删除文件
- `src/main/java/frc/robot/RobotHardware.java`
- `src/main/java/frc/robot/match/AutoConfigurator.java`

### 验证文件
- `src/main/java/frc/robot/Robot.java` — 确认 `updateAutoChooser()` / `getAutonomousCommand()` 调用在重构后仍兼容。
- `src/main/java/frc/robot/commands/AimCommand.java`
- `src/main/java/frc/robot/commands/PassCommand.java`
- `src/main/java/frc/robot/commands/FollowPoint.java`

## 测试策略
- 当前仓库没有 `src/test`，并且该项目更看重可读性、可扩展性和调试体验，因此本次不额外创建测试树。
- 每个逻辑块完成后用 `./gradlew build` 做编译级验证。
- 最终再跑一次 `./gradlew spotlessCheck` 确认格式。

---

### Task 1: 提取进球/射球/移动复用 command builder

**Files:**
- Create: `src/main/java/frc/robot/commands/GamePieceCommands.java`
- Create: `src/main/java/frc/robot/commands/ShootingCommands.java`
- Create: `src/main/java/frc/robot/commands/AutoDriveCommands.java`
- Verify: `src/main/java/frc/robot/commands/AimCommand.java`
- Verify: `src/main/java/frc/robot/commands/PassCommand.java`
- Verify: `src/main/java/frc/robot/commands/FollowPoint.java`

- [ ] **Step 1: 新建 `GamePieceCommands.java`**

```java
package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;

public final class GamePieceCommands {
  private GamePieceCommands() {}

  /** 自动与手动共用的机构归零动作。 */
  public static Command resetMechanisms(Intake intake, Hood hood) {
    return Commands.parallel(
            intake.resetToLimitCommand().unless(intake::isInitialized),
            hood.resetToLimitCommand().unless(hood::isInitialized))
        .withName("Reset Mechanisms");
  }

  /** 保持 intake 下放并正转吸球。 */
  public static Command runIntake(Intake intake) {
    return Commands.parallel(
            intake.setIntakeVelocityCommand(Volts.of(10)),
            intake.setPosCommand(() -> Degrees.of(0)))
        .withName("Run Intake");
  }

  /** 手动退球时使用的反转动作。 */
  public static Command reverseFeed(Intake intake, Feeder feeder) {
    return Commands.parallel(
            intake.setIntakeVelocityCommand(Volts.of(-10)),
            feeder.setFeederVelocityCommand(
                () -> RotationsPerSecond.of(-90), () -> RotationsPerSecond.of(-40)))
        .withName("Reverse Feed");
  }

  /** 喂料与 intake wave 联动，可用于 auto 与手动出球。 */
  public static Command feedWithWave(Intake intake, Feeder feeder, double delaySeconds) {
    return Commands.race(
            feeder.setFeederVelocityCommand(
                () -> RotationsPerSecond.of(48), () -> RotationsPerSecond.of(90)),
            intake.WaveIntakeCommand())
        .beforeStarting(Commands.waitSeconds(delaySeconds))
        .withName("Feed With Wave");
  }

  /** 手动取消喂料后，先轻微反转再停机。 */
  public static Command clearAndStopFeed(Intake intake, Feeder feeder) {
    return Commands.parallel(
            feeder
                .setFeederVelocityCommand(
                    () -> RotationsPerSecond.of(-20.0), () -> RotationsPerSecond.of(-80.0))
                .withTimeout(1)
                .andThen(feeder.stopCommand()),
            intake.stopCommand())
        .withName("Clear And Stop Feed");
  }

  /** 停止 intake 与 feeder。 */
  public static Command stopIntakeAndFeeder(Intake intake, Feeder feeder) {
    return Commands.parallel(intake.stopCommand(), feeder.stopCommand())
        .withName("Stop Intake And Feeder");
  }

  /** 停止射球后收回 intake。 */
  public static Command stowIntakeAndStop(Intake intake, Feeder feeder) {
    return Commands.parallel(
            feeder.stopCommand(),
            intake.setPosCommand(() -> Degrees.of(0)).andThen(intake.stopCommand()))
        .withName("Stow Intake And Stop");
  }
}
```

- [ ] **Step 2: 新建 `ShootingCommands.java`**

```java
package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelSetpoint;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import java.util.function.Supplier;

public final class ShootingCommands {
  private ShootingCommands() {}

  /** 自动瞄准准备并在给定时长内持续运行。 */
  public static Command autoAimShot(
      RobotContainer container, Supplier<Translation3d> targetSupplier, double timeoutSeconds) {
    return AimCommand.autoAimAtTarget(container, targetSupplier, () -> 0.0, () -> 0.0)
        .withTimeout(timeoutSeconds)
        .withName("Auto Aim Shot");
  }

  /** 固定 hood 与 flywheel 参数的定点射球。 */
  public static Command shootAtFixedPosition(
      Flywheel flywheel, Hood hood, Command feedCommand, double timeoutSeconds) {
    return Commands.parallel(
            flywheel.setVelocity(
                () ->
                    new FlywheelSetpoint(
                        RotationsPerSecond.of(49),
                        RotationsPerSecond.of(49),
                        RotationsPerSecond.of(49))),
            hood.positionSetpointCommand(() -> Degrees.of(30)),
            feedCommand)
        .withTimeout(timeoutSeconds)
        .withName("Shoot At Fixed Position");
  }

  /** 停止飞轮、喂料并收回 intake。 */
  public static Command stopShooting(Flywheel flywheel, Feeder feeder, Intake intake) {
    return Commands.parallel(
            flywheel.stopCommand(),
            feeder.stopCommand(),
            intake.setPosCommand(() -> Degrees.of(0)).andThen(intake.stopCommand()))
        .withName("Stop Shooting");
  }
}
```

- [ ] **Step 3: 新建 `AutoDriveCommands.java`**

```java
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public final class AutoDriveCommands {
  private AutoDriveCommands() {}

  /** 自动阶段定点跟随，内部统一做 alliance flip。 */
  public static Command followPoint(Drive drive, Pose2d targetPose) {
    return new FollowPoint(drive, () -> AllianceFlipUtil.apply(targetPose))
        .withName("Follow Point");
  }

  /** 通过清零底盘速度结束当前轨迹段。 */
  public static Command zeroChassisSpeeds(Drive drive) {
    return Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive)
        .withName("Zero Chassis Speeds");
  }

  /** 直接调用底盘 stop。 */
  public static Command stopDrive(Drive drive) {
    return Commands.runOnce(drive::stop, drive).withName("Stop Drive");
  }
}
```

- [ ] **Step 4: 跑一次编译，确认 3 个 builder 文件本身可通过**

Run: `./gradlew build`
Expected: `BUILD SUCCESSFUL`

- [ ] **Step 5: 提交 builder 抽取**

```bash
git add \
  src/main/java/frc/robot/commands/GamePieceCommands.java \
  src/main/java/frc/robot/commands/ShootingCommands.java \
  src/main/java/frc/robot/commands/AutoDriveCommands.java
git commit -m "refactor: extract reusable gamepiece and auto commands"
```

---

### Task 2: 拆分 auto routines 并引入 `AutoModeSelector`

**Files:**
- Create: `src/main/java/frc/robot/AutoModeSelector.java`
- Create: `src/main/java/frc/robot/commands/auto/routines/P2Auto.java`
- Create: `src/main/java/frc/robot/commands/auto/routines/P3Auto.java`
- Create: `src/main/java/frc/robot/commands/auto/routines/P4Auto.java`
- Create: `src/main/java/frc/robot/commands/auto/routines/P6Auto.java`
- Create: `src/main/java/frc/robot/commands/auto/routines/P6MirrorAuto.java`
- Create: `src/main/java/frc/robot/commands/auto/routines/P21Auto.java`
- Delete: `src/main/java/frc/robot/match/AutoConfigurator.java`

- [ ] **Step 1: 新建 `AutoModeSelector.java` 并迁移 chooser/factory 创建逻辑**

```java
package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.routines.P21Auto;
import frc.robot.commands.auto.routines.P2Auto;
import frc.robot.commands.auto.routines.P3Auto;
import frc.robot.commands.auto.routines.P4Auto;
import frc.robot.commands.auto.routines.P6Auto;
import frc.robot.commands.auto.routines.P6MirrorAuto;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class AutoModeSelector {
  private final RobotContainer robotContainer;
  private final Drive drive;
  private final Flywheel flywheel;
  private final Feeder feeder;
  private final Hood hood;
  private final Intake intake;
  private final AutoChooser autoChooser = new AutoChooser();

  private AutoFactory autoFactory;

  public AutoModeSelector(
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

    SmartDashboard.putData("Auto Choices", autoChooser);
  }

  public void update() {
    if (autoFactory != null) {
      Logger.recordOutput("Robot/Auto/ChooserReady", true);
      return;
    }

    if (DriverStation.getAlliance().isEmpty()) {
      Logger.recordOutput("Robot/Auto/ChooserReady", false);
      return;
    }

    autoFactory =
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

    autoChooser.addCmd("P2", () -> P2Auto.create(autoFactory, robotContainer, drive, flywheel, feeder, hood, intake));
    autoChooser.addCmd("P3", () -> P3Auto.create(autoFactory, robotContainer, drive, flywheel, feeder, hood, intake));
    autoChooser.addCmd("P4", () -> P4Auto.create(autoFactory, robotContainer, drive, flywheel, feeder, hood, intake));
    autoChooser.addCmd("P6", () -> P6Auto.create(autoFactory, robotContainer, drive, flywheel, feeder, hood, intake));
    autoChooser.addCmd(
        "P6_mirror",
        () -> P6MirrorAuto.create(autoFactory, robotContainer, drive, flywheel, feeder, hood, intake));
    autoChooser.addCmd("p2_1", () -> P21Auto.create(autoFactory));

    SmartDashboard.putData("Auto Choices", autoChooser);
    Logger.recordOutput("Robot/Auto/ChooserReady", true);
  }

  public Command selectedCommand() {
    return autoChooser.selectedCommand();
  }
}
```

- [ ] **Step 2: 新建 `P2Auto.java` 与 `P3Auto.java`，把原有 sequence 原样迁移并改用 builder**

```java
package frc.robot.commands.auto.routines;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDriveCommands;
import frc.robot.commands.GamePieceCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;

public final class P2Auto {
  private P2Auto() {}

  public static Command create(
      AutoFactory autoFactory,
      RobotContainer container,
      Drive drive,
      Flywheel flywheel,
      Feeder feeder,
      Hood hood,
      Intake intake) {
    return Commands.sequence(
        GamePieceCommands.resetMechanisms(intake, hood),
        GamePieceCommands.runIntake(intake),
        autoFactory.trajectoryCmd("p2").beforeStarting(autoFactory.resetOdometry("p2")),
        AutoDriveCommands.zeroChassisSpeeds(drive),
        Commands.race(
            GamePieceCommands.feedWithWave(intake, feeder, 2),
            ShootingCommands.autoAimShot(
                container, () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint), 6)),
        ShootingCommands.stopShooting(flywheel, feeder, intake));
  }
}
```

```java
package frc.robot.commands.auto.routines;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDriveCommands;
import frc.robot.commands.GamePieceCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;

public final class P3Auto {
  private P3Auto() {}

  public static Command create(
      AutoFactory autoFactory,
      RobotContainer container,
      Drive drive,
      Flywheel flywheel,
      Feeder feeder,
      Hood hood,
      Intake intake) {
    return Commands.sequence(
        GamePieceCommands.resetMechanisms(intake, hood),
        GamePieceCommands.runIntake(intake),
        autoFactory.trajectoryCmd("p3", 0).beforeStarting(autoFactory.resetOdometry("p3")),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(0.8837900161743164, 5.983599662780762, Rotation2d.fromDegrees(180))),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(1.3281044721603394, 5.964677333831787, Rotation2d.fromDegrees(180))),
        autoFactory.trajectoryCmd("p3", 1),
        AutoDriveCommands.zeroChassisSpeeds(drive),
        Commands.parallel(
            GamePieceCommands.feedWithWave(intake, feeder, 2),
            ShootingCommands.autoAimShot(
                container, () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint), 6)),
        ShootingCommands.stopShooting(flywheel, feeder, intake),
        autoFactory.trajectoryCmd("p3", 2),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(1.505340428352356, 3.7539849281311035, Rotation2d.fromDegrees(180))));
  }
}
```

- [ ] **Step 3: 新建 `P4Auto.java`，迁移固定点射球与后续 auto aim 逻辑**

```java
package frc.robot.commands.auto.routines;

import static edu.wpi.first.units.Units.Degrees;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDriveCommands;
import frc.robot.commands.GamePieceCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;

public final class P4Auto {
  private P4Auto() {}

  public static Command create(
      AutoFactory autoFactory,
      RobotContainer container,
      Drive drive,
      Flywheel flywheel,
      Feeder feeder,
      Hood hood,
      Intake intake) {
    Command feedCommand = GamePieceCommands.feedWithWave(intake, feeder, 1);

    return Commands.sequence(
        GamePieceCommands.resetMechanisms(intake, hood),
        GamePieceCommands.runIntake(intake),
        ShootingCommands.shootAtFixedPosition(flywheel, hood, feedCommand, 3.5),
        Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()),
        intake.setPosCommand(() -> Degrees.of(0)),
        autoFactory.trajectoryCmd("p4", 0).beforeStarting(autoFactory.resetOdometry("p4")),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(7.664889812469482, 5.9223713874816895, Rotation2d.fromDegrees(-90))),
        autoFactory.trajectoryCmd("p4", 1),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(7.531137466430664, 2.8975112438201904, Rotation2d.fromDegrees(-90))),
        autoFactory.trajectoryCmd("p4", 2),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(2.695476770401001, 2.67116117477417, Rotation2d.fromRadians(0.6565357014663004))),
        Commands.parallel(
                ShootingCommands.autoAimShot(
                    container, () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint), 6),
                GamePieceCommands.feedWithWave(intake, feeder, 1))
            .withTimeout(6),
        ShootingCommands.stopShooting(flywheel, feeder, intake));
  }
}
```

- [ ] **Step 4: 新建 `P6Auto.java`、`P6MirrorAuto.java`、`P21Auto.java`，完整迁移剩余 routine**

```java
package frc.robot.commands.auto.routines;

import static edu.wpi.first.units.Units.Degrees;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDriveCommands;
import frc.robot.commands.GamePieceCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;

public final class P6Auto {
  private P6Auto() {}

  public static Command create(
      AutoFactory autoFactory,
      RobotContainer container,
      Drive drive,
      Flywheel flywheel,
      Feeder feeder,
      Hood hood,
      Intake intake) {
    return Commands.sequence(
        GamePieceCommands.resetMechanisms(intake, hood),
        GamePieceCommands.runIntake(intake),
        autoFactory.trajectoryCmd("p6", 0).beforeStarting(autoFactory.resetOdometry("p6")),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(7.8589396476745605, 7.029058456420898, Rotation2d.fromDegrees(-90))),
        autoFactory.trajectoryCmd("p6", 1),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(7.532529354095459, 5.936294078826904, Rotation2d.fromDegrees(-90))),
        autoFactory.trajectoryCmd("p6", 2),
        AutoDriveCommands.followPoint(
            drive,
            new Pose2d(
                3.5140202045440674,
                5.620737552642822,
                Rotation2d.fromRadians(-0.9667225715055064))),
        AutoDriveCommands.stopDrive(drive),
        Commands.parallel(
                ShootingCommands.autoAimShot(
                    container, () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint), 6),
                GamePieceCommands.feedWithWave(intake, feeder, 1))
            .withTimeout(6),
        ShootingCommands.stopShooting(flywheel, feeder, intake)
            .alongWith(
                AutoDriveCommands.followPoint(
                    drive,
                    new Pose2d(
                        5.5501532554626465,
                        5.673675060272217,
                        Rotation2d.fromRadians(-1.2068177253516055)))),
        GamePieceCommands.runIntake(intake).withName("Auto Intake Reload"),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(7.664889812469482, 5.1983232498168945, Rotation2d.fromDegrees(-90))));
  }
}
```

```java
package frc.robot.commands.auto.routines;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDriveCommands;
import frc.robot.commands.GamePieceCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;

public final class P6MirrorAuto {
  private P6MirrorAuto() {}

  public static Command create(
      AutoFactory autoFactory,
      RobotContainer container,
      Drive drive,
      Flywheel flywheel,
      Feeder feeder,
      Hood hood,
      Intake intake) {
    return Commands.sequence(
        GamePieceCommands.resetMechanisms(intake, hood),
        GamePieceCommands.runIntake(intake),
        autoFactory
            .trajectoryCmd("p6_mirror", 0)
            .beforeStarting(autoFactory.resetOdometry("p6_mirror")),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(7.664889812469482, 2.1466286125183114, Rotation2d.fromDegrees(90))),
        autoFactory.trajectoryCmd("p6_mirror", 1),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(7.664889812469482, 2.1466286125183114, Rotation2d.fromDegrees(90))),
        autoFactory.trajectoryCmd("p6_mirror", 2),
        AutoDriveCommands.followPoint(
            drive,
            new Pose2d(
                3.5140202045440674,
                2.4482624473571786,
                Rotation2d.fromRadians(0.9667225715055064))),
        AutoDriveCommands.stopDrive(drive),
        Commands.parallel(
                ShootingCommands.autoAimShot(
                    container, () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint), 6),
                GamePieceCommands.feedWithWave(intake, feeder, 1))
            .withTimeout(6),
        ShootingCommands.stopShooting(flywheel, feeder, intake)
            .alongWith(
                AutoDriveCommands.followPoint(
                    drive,
                    new Pose2d(
                        5.5501532554626465,
                        2.395324939727784,
                        Rotation2d.fromRadians(1.2068177253516055)))),
        GamePieceCommands.runIntake(intake).withName("Auto Intake Reload"),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(7.664889812469482, 2.1466286125183114, Rotation2d.fromDegrees(90))));
  }
}
```

```java
package frc.robot.commands.auto.routines;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;

public final class P21Auto {
  private P21Auto() {}

  public static Command create(AutoFactory autoFactory) {
    return autoFactory
        .trajectoryCmd("t5", 0)
        .beforeStarting(autoFactory.resetOdometry("t5"))
        .andThen(autoFactory.trajectoryCmd("t5", 1));
  }
}
```

- [ ] **Step 5: 删除 `AutoConfigurator.java` 并做一次编译检查**

Run: `./gradlew build`
Expected: `BUILD SUCCESSFUL`

- [ ] **Step 6: 提交 auto 拆分**

```bash
git add \
  src/main/java/frc/robot/AutoModeSelector.java \
  src/main/java/frc/robot/commands/auto/routines/P2Auto.java \
  src/main/java/frc/robot/commands/auto/routines/P3Auto.java \
  src/main/java/frc/robot/commands/auto/routines/P4Auto.java \
  src/main/java/frc/robot/commands/auto/routines/P6Auto.java \
  src/main/java/frc/robot/commands/auto/routines/P6MirrorAuto.java \
  src/main/java/frc/robot/commands/auto/routines/P21Auto.java \
  src/main/java/frc/robot/match/AutoConfigurator.java
git commit -m "refactor: split autonomous routines and selector"
```

---

### Task 3: 将 `RobotHardware` 回收到 `RobotContainer`

**Files:**
- Modify: `src/main/java/frc/robot/RobotContainer.java`
- Delete: `src/main/java/frc/robot/RobotHardware.java`
- Verify: `src/main/java/frc/robot/Robot.java`

- [ ] **Step 1: 把 `RobotHardware` 中的按模式构造逻辑原样搬回 `RobotContainer`**

```java
private final AutoModeSelector autoModeSelector;

public RobotContainer() {
  switch (Constants.currentMode) {
    case REAL:
      drive =
          new Drive(
              new GyroIOPigeon2(),
              new ModuleIOTalonFX(TunerConstants.FrontLeft),
              new ModuleIOTalonFX(TunerConstants.FrontRight),
              new ModuleIOTalonFX(TunerConstants.BackLeft),
              new ModuleIOTalonFX(TunerConstants.BackRight));
      vision =
          new Vision(
              drive::addVisionMeasurement,
              new VisionIOPhotonVision(camera0Name, robotToCamera0),
              new VisionIOPhotonVision(camera1Name, robotToCamera1));
      flywheel = new Flywheel(new FlywheelIOTalonFX(), new LimitSwitchDIO());
      feeder = new Feeder(new FeederIOTalonFX());
      hood = new Hood(new HoodIOTalonFX());
      intake = new Intake(new IntakeIOTalonFX());
      clamber = new Clamber(new ClamberIOTalonFX());
      break;

    case SIM:
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIOSim(TunerConstants.FrontLeft),
              new ModuleIOSim(TunerConstants.FrontRight),
              new ModuleIOSim(TunerConstants.BackLeft),
              new ModuleIOSim(TunerConstants.BackRight));
      vision =
          new Vision(
              drive::addVisionMeasurement,
              new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
      flywheel = new Flywheel(new FlywheelIOSim(), new LimitSwitchDIO());
      feeder = new Feeder(new FeederIOSim());
      hood = new Hood(new HoodIOSim());
      intake = new Intake(new IntakeIOSim());
      clamber = new Clamber(new ClamberIOSim());
      break;

    default:
      drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
      vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
      flywheel = new Flywheel(new FlywheelIOSim(), new LimitSwitchDIO());
      feeder = new Feeder(new FeederIOSim());
      hood = new Hood(new HoodIOSim());
      intake = new Intake(new IntakeIOTalonFX());
      clamber = new Clamber(new ClamberIO() {});
      break;
  }

  autoModeSelector = new AutoModeSelector(this, drive, flywheel, feeder, hood, intake);
  autoModeSelector.update();
  configureButtonBindings();
}
```

- [ ] **Step 2: 改写 `updateAutoChooser()` 与 `getAutonomousCommand()`，只保留 selector 委托**

```java
public void updateAutoChooser() {
  autoModeSelector.update();
}

public Command getAutonomousCommand() {
  return autoModeSelector.selectedCommand();
}
```

- [ ] **Step 3: 清理旧字段与 import，删除 `RobotHardware.java`**

```java
// 删除这几项：
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.match.AutoConfigurator;
import org.littletonrobotics.junction.Logger;

private AutoFactory autoFactory;
private final AutoChooser autoChooser;
```

- [ ] **Step 4: 跑编译确认 `Robot` 侧调用无需改动**

Run: `./gradlew build`
Expected: `BUILD SUCCESSFUL`

- [ ] **Step 5: 提交 `RobotContainer` 装配回收**

```bash
git add src/main/java/frc/robot/RobotContainer.java src/main/java/frc/robot/RobotHardware.java
git commit -m "refactor: inline robot hardware setup in container"
```

---

### Task 4: 重构 `RobotControl` 为按功能分组的绑定文件

**Files:**
- Modify: `src/main/java/frc/robot/RobotControl.java`
- Verify: `src/main/java/frc/robot/commands/GamePieceCommands.java`
- Verify: `src/main/java/frc/robot/commands/ShootingCommands.java`
- Verify: `src/main/java/frc/robot/commands/AutoDriveCommands.java`

- [ ] **Step 1: 把 `configure()` 改成分组入口，并加中文注释**

```java
public void configure() {
  // 配置驾驶手柄相关绑定。
  configureControllerBindings();

  // 配置操作手柄相关绑定。
  configureOperatorBindings();
}

private void configureControllerBindings() {
  drive.setDefaultCommand(
      DriveCommands.joystickDrive(
          drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

  controller
      .leftBumper()
      .whileTrue(
          DriveCommands.joystickRobotDrive(
              drive,
              () -> -controller.getLeftY(),
              () -> -controller.getLeftX(),
              () -> -controller.getRightX() * 0.9));

  controller
      .rightTrigger()
      .whileTrue(DriveCommands.snapToNearest30Degrees(drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

  controller.back().onTrue(Commands.runOnce(drive::stopWithX, drive));
  controller
      .start()
      .onTrue(
          Commands.runOnce(
                  () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                  drive)
              .ignoringDisable(true));
}

private void configureOperatorBindings() {
  operator.L1().whileTrue(clamber.runPercentCommand(0.6));
  operator.R1().whileTrue(clamber.runPercentCommand(-0.6));
}
```

- [ ] **Step 2: 将 intake/feed/reverse/shoot 相关内联 command 改用新 builder，但不改变按键行为**

```java
controller
    .rightBumper()
    .whileTrue(GamePieceCommands.runIntake(intake))
    .onFalse(GamePieceCommands.stopIntakeAndFeeder(intake, feeder));

controller
    .x()
    .or(operator.square())
    .whileTrue(GamePieceCommands.feedWithWave(intake, feeder, 0))
    .onFalse(GamePieceCommands.clearAndStopFeed(intake, feeder));

controller
    .y()
    .whileTrue(GamePieceCommands.reverseFeed(intake, feeder))
    .onFalse(GamePieceCommands.stopIntakeAndFeeder(intake, feeder));

controller
    .a()
    .whileTrue(
        AimCommand.autoAimAtTarget(
            robotContainer,
            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint),
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX()))
    .onFalse(ShootingCommands.stopShooting(flywheel, feeder, intake));
```

- [ ] **Step 3: 保留 pass/follow-point 行为，仅整理位置与中文注释**

```java
// 配置需要自动对准或定点移动的驾驶绑定。
controller
    .b()
    .whileTrue(
        new FollowPoint(drive, () -> AllianceFlipUtil.apply(FieldConstants.Hub.tower))
            .onlyIf(
                () ->
                    drive
                            .getPose()
                            .getTranslation()
                            .getDistance(AllianceFlipUtil.apply(FieldConstants.Hub.tower.getTranslation()))
                        < 1.6));

controller
    .pov(90)
    .whileTrue(
        AutoDriveCommands.followPoint(drive, FieldConstants.Hub.blink)
            .raceWith(
                flywheel.setVelocity(
                    () ->
                        new FlywheelSetpoint(
                            RotationsPerSecond.of(20),
                            RotationsPerSecond.of(20),
                            RotationsPerSecond.of(20))))
            .andThen(
                Commands.parallel(
                    Commands.run(drive::stopWithX, drive),
                    AimCommand.noMoveShootAtTarget(
                        robotContainer,
                        () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint)))))
    .onFalse(ShootingCommands.stopShooting(flywheel, feeder, intake));
```

- [ ] **Step 4: 跑编译，确认按钮绑定行为保持一致**

Run: `./gradlew build`
Expected: `BUILD SUCCESSFUL`

- [ ] **Step 5: 提交 `RobotControl` 整理**

```bash
git add src/main/java/frc/robot/RobotControl.java
git commit -m "refactor: regroup robot control bindings"
```

---

### Task 5: 清理旧文件并做最终验证

**Files:**
- Delete: `src/main/java/frc/robot/RobotHardware.java`
- Delete: `src/main/java/frc/robot/match/AutoConfigurator.java`
- Verify: `src/main/java/frc/robot/RobotContainer.java`
- Verify: `src/main/java/frc/robot/AutoModeSelector.java`
- Verify: `src/main/java/frc/robot/RobotControl.java`

- [ ] **Step 1: 确认旧文件没有剩余引用**

Run: `rg "RobotHardware|AutoConfigurator" src/main/java/frc/robot`
Expected: 只剩删除前的文件路径本身，清理后应无结果

- [ ] **Step 2: 删除旧文件并检查工作区差异**

Run: `git diff -- src/main/java/frc/robot src/main/java/frc/robot/match`
Expected: 只包含本次重构相关 diff，没有额外行为改动

- [ ] **Step 3: 跑最终格式与构建验证**

Run: `./gradlew spotlessCheck`
Expected: `BUILD SUCCESSFUL`

Run: `./gradlew build`
Expected: `BUILD SUCCESSFUL`

- [ ] **Step 4: 提交最终清理**

```bash
git add \
  src/main/java/frc/robot/AutoModeSelector.java \
  src/main/java/frc/robot/RobotContainer.java \
  src/main/java/frc/robot/RobotControl.java \
  src/main/java/frc/robot/commands \
  src/main/java/frc/robot/commands/auto/routines \
  src/main/java/frc/robot/RobotHardware.java \
  src/main/java/frc/robot/match/AutoConfigurator.java
git commit -m "refactor: reorganize command and autonomous layers"
```

---

## 自检
- spec 要求的 3 个核心点都覆盖了：`AutoModeSelector`、可复用 command 按功能聚合、`RobotHardware` 回收至 `RobotContainer`。
- 计划没有再把原子 command 限制在 `commands/auto`；auto routines 与 teleop builder 的边界已经分开。
- `Main.java` 不需要修改，只要求 `AutoModeSelector` 位于同层 package，这一点已经反映在文件结构中。
- 当前仓库没有测试树，因此执行计划采用编译与格式校验，不额外添加与项目习惯冲突的测试任务。
