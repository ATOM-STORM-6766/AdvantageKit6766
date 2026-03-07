// 版权 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// 本程序是自由软件；您可以根据自由软件基金会发布的 GNU 通用公共许可证
// 第 3 版的条款进行再发布和/或修改，或在本项目根目录中查阅该许可证。
//
// 本程序的发布目的是希望它有用，但不提供任何担保；
// 甚至没有适销性或特定用途适用性的默示担保。
// 详细信息请参阅 GNU 通用公共许可证。

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.vision.VisionConstants.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FollowPoint;
import frc.robot.commands.PassCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.aim.AimSubsystem;
import frc.robot.subsystems.aim.PassSubsystem;
import frc.robot.subsystems.clamber.Clamber;
import frc.robot.subsystems.clamber.ClamberIO;
import frc.robot.subsystems.clamber.ClamberIOSim;
import frc.robot.subsystems.clamber.ClamberIOTalonFX;
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
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelSetpoint;
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
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/**
 * 该类用于声明机器人中的主要内容。由于 Command-based 属于“声明式”范式， {@link Robot} 的 periodic 方法中（除调度器调用外）不应包含太多机器人逻辑。
 * 相反，机器人结构（子系统、指令和按键映射等）应在此处声明。
 */
public class RobotContainer {
  // 子系统
  private final Drive drive;
  private final Clamber clamber;

  private AutoFactory autoFactory;
  private final AutoChooser autoChooser;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Flywheel flywheel;
  private final Feeder feeder;

  private final Hood hood;
  private final Intake m_intake;
  private final AimSubsystem aimSubsystem = new AimSubsystem();
  private final PassSubsystem passSubsystem = new PassSubsystem();

  // 控制器
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandPS5Controller operator = new CommandPS5Controller(1);

  /** 机器人容器，包含子系统、操作接口设备以及指令。 */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // 真机模式：实例化硬件 IO 实现
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

        hood = new Hood(new HoodIOTalonFX());
        flywheel = new Flywheel(new FlywheelIOTalonFX(), new LimitSwitchDIO());
        feeder = new Feeder(new FeederIOTalonFX());
        m_intake = new Intake(new IntakeIOTalonFX());
        clamber = new Clamber(new ClamberIOTalonFX());
        break;

      case SIM:
        // 仿真模式：实例化物理仿真 IO 实现
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
                // new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        hood = new Hood(new HoodIOSim());
        flywheel = new Flywheel(new FlywheelIOSim(), new LimitSwitchDIO());
        feeder = new Feeder(new FeederIOSim());
        m_intake = new Intake(new IntakeIOSim());
        clamber = new Clamber(new ClamberIOSim());
        break;

      default:
        // 回放模式：禁用 IO 实现
        // （使用与真机相同数量的虚拟实现）
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        hood = new Hood(new HoodIOSim());
        flywheel = new Flywheel(new FlywheelIOSim(), new LimitSwitchDIO());
        feeder = new Feeder(new FeederIOSim());
        m_intake = new Intake(new IntakeIOTalonFX());
        clamber = new Clamber(new ClamberIO() {});
        break;
    }

    // 设置自动例程
    autoChooser = new AutoChooser();
    autoFactory = null;
    SmartDashboard.putData("Auto Choices", autoChooser);
    updateAutoChooser();

    // 配置按键绑定
    configureButtonBindings();
  }

  public void updateAutoChooser() {
    if (autoFactory == null && DriverStation.getAlliance().isPresent()) {
      configureAutoChooser();
      SmartDashboard.putData("Auto Choices", autoChooser);
    }
  }

  private void configureAutoChooser() {
    autoFactory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            drive,
            (trajectory, isFinsh) -> {
              Logger.recordOutput(
                  "Odometry/Trajectory",
                  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                      ? trajectory.flipped().getPoses()
                      : trajectory.getPoses());
              Logger.recordOutput("Odometry/TrajectoryIsFinished", isFinsh);
            });

    // autoFactory.bind(
    //     "intake",
    //     Commands.parallel(
    //         m_intake.setIntakeVelocityCommand(Volts.of(10)),
    //         m_intake.setPosCommand(() -> Degrees.of(0))));

    // autoFactory.bind("stopIntake", m_intake.stopCommand());

    var resetCmd =
        Commands.parallel(m_intake.resetToLimitCommand().alongWith(hood.resetToLimitCommand()));
    var intakeCmd =
        Commands.parallel(
                m_intake.setIntakeVelocityCommand(Volts.of(10)),
                m_intake.setPosCommand(() -> Degrees.of(0)))
            .withName("Intake");
    var feedCmd =
        Commands.race(
                feeder.setFeederVelocityCommand(
                    () -> RotationsPerSecond.of(48), () -> RotationsPerSecond.of(90)),
                m_intake.WaveIntakeCommand())
            .beforeStarting(Commands.waitSeconds(1));
    var shootCmd =
        // Commands.parallel(
        AimCommand.autoAimAtTarget(
            this,
            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint),
            () -> 0.0,
            () -> 0.0);
    // Commands.waitSeconds(1).andThen(feedCmd));
    var stopShootCmd =
        Commands.parallel(
            flywheel.stopCommand(),
            feeder.stopCommand(),
            m_intake.setPosCommand(() -> Degrees.of(0)).andThen(m_intake.stopCommand()));
    var climbCmd =
        clamber.runPercentCommand(0.6).withTimeout(5).andThen(clamber.runPercentCommand(0.1));
    var shootAtPosition =
        Commands.parallel(
                flywheel.setVelocity(
                    () ->
                        new FlywheelSetpoint(
                            RotationsPerSecond.of(49),
                            RotationsPerSecond.of(49),
                            RotationsPerSecond.of(49))),
                hood.positionSetpointCommand(() -> Degrees.of(30)),
                feedCmd)
            .withTimeout(3.5);

    var p2Cmd =
        autoFactory
            .trajectoryCmd("p2")
            .beforeStarting(autoFactory.resetOdometry("p2"))
            .withName("P2 Trajectory");
    var p3_0Cmd =
        autoFactory
            .trajectoryCmd("p3", 0)
            .beforeStarting(autoFactory.resetOdometry("p3"))
            .withName("P3 Trajectory");
    var p3_1Cmd = autoFactory.trajectoryCmd("p3", 1).withName("P3 Trajectory Part 2");
    var p3_2Cmd = autoFactory.trajectoryCmd("p3", 2).withName("P3 Trajectory Part 3");
    var p4_0Cmd =
        autoFactory
            .trajectoryCmd("p4", 0)
            .beforeStarting(autoFactory.resetOdometry("p4"))
            .withName("P4 Trajectory Part 1");
    var p4_1Cmd = autoFactory.trajectoryCmd("p4", 1).withName("P4 Trajectory Part 2");
    var p4_2Cmd = autoFactory.trajectoryCmd("p4", 2).withName("P4 Trajectory Part 3");

    var p6_0Cmd =
        autoFactory
            .trajectoryCmd("p6", 0)
            .beforeStarting(autoFactory.resetOdometry("p6"))
            .withName("P6 Trajectory Part 1");
    var p6_1Cmd = autoFactory.trajectoryCmd("p6", 1).withName("P6 Trajectory Part 2");
    var p6_2Cmd = autoFactory.trajectoryCmd("p6", 2).withName("P6 Trajectory Part 3");

    var p6_0_mirrorCmd =
        autoFactory
            .trajectoryCmd("p6_mirror", 0)
            .beforeStarting(autoFactory.resetOdometry("p6_mirror"))
            .withName("P6 Mirror Trajectory Part 1");
    var p6_1_mirrorCmd =
        autoFactory.trajectoryCmd("p6_mirror", 1).withName("P6 Mirror Trajectory Part 2");
    var p6_2_mirrorCmd =
        autoFactory.trajectoryCmd("p6_mirror", 2).withName("P6 Mirror Trajectory Part 3");

    autoChooser.addCmd(
        "P2",
        () ->
            Commands.sequence(
                resetCmd,
                intakeCmd,
                p2Cmd,
                Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive),
                Commands.race(
                    Commands.race(
                            feeder.setFeederVelocityCommand(
                                () -> RotationsPerSecond.of(48), () -> RotationsPerSecond.of(90)),
                            m_intake.WaveIntakeCommand())
                        .beforeStarting(Commands.waitSeconds(2)),
                    shootCmd.withTimeout(6)),
                stopShootCmd));

    autoChooser.addCmd(
        "P3",
        () ->
            Commands.sequence(
                resetCmd,
                intakeCmd,
                p3_0Cmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                0.8837900161743164,
                                5.983599662780762,
                                Rotation2d.fromDegrees(180)))),
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                1.3281044721603394,
                                5.964677333831787,
                                Rotation2d.fromDegrees(180)))),
                p3_1Cmd,
                Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive),
                Commands.parallel(
                    Commands.race(
                            feeder.setFeederVelocityCommand(
                                () -> RotationsPerSecond.of(48), () -> RotationsPerSecond.of(90)),
                            m_intake.WaveIntakeCommand())
                        .beforeStarting(Commands.waitSeconds(2)),
                    shootCmd.withTimeout(6)),
                stopShootCmd,
                p3_2Cmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                1.505340428352356,
                                3.7539849281311035,
                                Rotation2d.fromDegrees(180))))));

    autoChooser.addCmd(
        "P4",
        () ->
            Commands.sequence(
                resetCmd,
                intakeCmd,
                shootAtPosition,
                Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()),
                m_intake.setPosCommand(() -> Degrees.of(0)),
                p4_0Cmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                7.664889812469482,
                                5.9223713874816895,
                                Rotation2d.fromDegrees(-90)))),
                p4_1Cmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                7.531137466430664,
                                2.8975112438201904,
                                Rotation2d.fromDegrees(-90)))),
                p4_2Cmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                2.695476770401001,
                                2.67116117477417,
                                Rotation2d.fromRadians(0.6565357014663004)))),
                Commands.parallel(
                        AimCommand.autoAimAtTarget(
                            this,
                            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint),
                            () -> 0.0,
                            () -> 0.0),
                        Commands.parallel(
                                feeder.setFeederVelocityCommand(
                                    () -> RotationsPerSecond.of(48),
                                    () -> RotationsPerSecond.of(90)),
                                m_intake.WaveIntakeCommand())
                            .beforeStarting(Commands.waitSeconds(1)))
                    .withTimeout(6),
                stopShootCmd));

    autoChooser.addCmd(
        "P6",
        () ->
            Commands.sequence(
                resetCmd,
                intakeCmd,
                shootAtPosition,
                Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()),
                m_intake.setPosCommand(() -> Degrees.of(0)),
                p6_0Cmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                7.8589396476745605 ,
                                7.029058456420898,
                                Rotation2d.fromDegrees(-90)))),
                p6_1Cmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                7.664889812469482 ,
                                5.1983232498168945 ,
                                Rotation2d.fromDegrees(-90)))),
                p6_2Cmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                7.8589396476745605 ,
                                7.029058456420898,
                                Rotation2d.fromRadians(-0.9667225715055064)))),
                Commands.runOnce(drive::stop, drive),
                Commands.parallel(
                        AimCommand.autoAimAtTarget(
                            this,
                            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint),
                            () -> 0.0,
                            () -> 0.0),
                        Commands.parallel(
                                feeder.setFeederVelocityCommand(
                                    () -> RotationsPerSecond.of(48),
                                    () -> RotationsPerSecond.of(90)),
                                m_intake.WaveIntakeCommand())
                            .beforeStarting(Commands.waitSeconds(1)))
                    .withTimeout(6),
                stopShootCmd));

    autoChooser.addCmd(
        "P6_mirror",
        () ->
            Commands.sequence(
                resetCmd,
                intakeCmd,
                shootAtPosition,
                Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()),
                m_intake.setPosCommand(() -> Degrees.of(0)),
                p6_0_mirrorCmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                7.664889812469482,
                                2.1466286125183114,
                                Rotation2d.fromDegrees(90)))),
                p6_1_mirrorCmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                7.664889812469482,
                                2.1466286125183114,
                                Rotation2d.fromDegrees(90)))),
                p6_2_mirrorCmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                3.5140202045440674,
                                2.4482624473571786,
                                Rotation2d.fromRadians(0.9667225715055064)))),
                Commands.runOnce(drive::stop, drive),
                Commands.parallel(
                        AimCommand.autoAimAtTarget(
                            this,
                            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint),
                            () -> 0.0,
                            () -> 0.0),
                        Commands.parallel(
                                feeder.setFeederVelocityCommand(
                                    () -> RotationsPerSecond.of(48),
                                    () -> RotationsPerSecond.of(90)),
                                m_intake.WaveIntakeCommand())
                            .beforeStarting(Commands.waitSeconds(1)))
                    .withTimeout(6),
                stopShootCmd));

    autoChooser.addCmd(
        "p2_1",
        () ->
            autoFactory
                .trajectoryCmd("t5", 0)
                .beforeStarting(autoFactory.resetOdometry("t5"))
                .andThen(autoFactory.trajectoryCmd("t5", 1)));
  }

  public Hood getHood() {
    return hood;
  }

  public Flywheel getFlywheel() {
    return flywheel;
  }

  public Feeder getFeeder() {
    return feeder;
  }

  public Intake getIntake() {
    return m_intake;
  }

  public Clamber getClamber() {
    return clamber;
  }

  public AimSubsystem getAimSubsystem() {
    return aimSubsystem;
  }

  public PassSubsystem getPassSubsystem() {
    return passSubsystem;
  }

  public Drive getDrive() {
    return drive;
  }

  /**
   * 使用此方法定义按键到指令的映射。可以实例化 {@link GenericHID} 或其子类 （如 {@link edu.wpi.first.wpilibj.Joystick} 或
   * {@link XboxController}），然后传递给 {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}
   * 来创建按钮。
   */
  private void configureButtonBindings() {
    // 默认指令：正常的场相对驾驶
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    // 有头
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
        .whileTrue(
            DriveCommands.snapToNearest30Degrees(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    // 按下 back 键时切换至 X 模式
    controller.back().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // 按下 start 键时将陀螺仪重置到 0°
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // 按住右肩键时启动 intake 吸球
    controller
        .rightBumper()
        .whileTrue(
            Commands.parallel(
                m_intake.setIntakeVelocityCommand(Volts.of(10)),
                m_intake.setPosCommand(() -> Degrees.of(0))))
        .onFalse(Commands.parallel(m_intake.stopCommand(), feeder.stopCommand()));

    // 按住a时自瞄目标
    controller
        .a()
        .whileTrue(
            Commands.parallel(
                AimCommand.autoAimAtTarget(
                    this,
                    () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint),
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX())))
        .onFalse(Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()));

    // 按下 x 键 或者 square 键时送球
    controller
        .x()
        .or(operator.square())
        .whileTrue(
            Commands.parallel(
                feeder.setFeederVelocityCommand(
                    () -> RotationsPerSecond.of(48), () -> RotationsPerSecond.of(90)),
                m_intake.WaveIntakeCommand()))
        .onFalse(
            Commands.parallel(
                feeder
                    .setFeederVelocityCommand(
                        () -> RotationsPerSecond.of(-20.0), () -> RotationsPerSecond.of(-80.0))
                    .withTimeout(1)
                    .andThen(feeder.stopCommand()),
                m_intake.stopCommand()));

    // 按住左肩键时爬升
    operator.L1().whileTrue(clamber.runPercentCommand(0.6));

    // 按住右肩键时反向爬升（放松）
    operator.R1().whileTrue(clamber.runPercentCommand(-0.6));

    // 按住 x 键时传球
    operator
        .cross()
        .whileTrue(
            PassCommand.passAtTarget(
                this,
                () -> {
                  var target = new Translation3d(FieldConstants.Hub.passPoint);
                  // 首先将传球基准点根据当前的联盟应用翻转，获取该点在全局(蓝方)绝对坐标系中的起始位置
                  target = AllianceFlipUtil.apply(target);

                  // 判断当前机器人全局 Y 坐标在哪半场，如果在另一半场，将传球点镜像过去
                  target =
                      AllianceFlipUtil.apply(drive.getPose()).getTranslation().getY()
                              > FieldConstants.fieldWidth / 2
                          ? target
                          : AllianceFlipUtil.mirror(target);

                  return target;
                },
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX()))
        .onFalse(Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()));

    // 按住 y 键时intake吐球
    controller
        .y()
        .whileTrue(
            Commands.parallel(
                m_intake.setIntakeVelocityCommand(Volts.of(-10)),
                feeder.setFeederVelocityCommand(
                    () -> RotationsPerSecond.of(-90), () -> RotationsPerSecond.of(-40))))
        .onFalse(m_intake.stopCommand().alongWith(feeder.stopCommand()));
    // DriveCommands.joystickDriveAtAngle(drive, () -> 0, () -> 0, () -> Rotation2d.k180deg));

    // 按住 b 键时自动跟随到 hub 的塔顶位置
    controller
        .b()
        .whileTrue(
            new FollowPoint(drive, () -> AllianceFlipUtil.apply(FieldConstants.Hub.tower))
                .onlyIf(
                    () ->
                        drive
                                .getPose()
                                .getTranslation()
                                .getDistance(
                                    AllianceFlipUtil.apply(
                                        FieldConstants.Hub.tower.getTranslation()))
                            < 1.6));

    controller
        .pov(90)
        .whileTrue(
            new FollowPoint(drive, () -> AllianceFlipUtil.apply(FieldConstants.Hub.blink))
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
                            this,
                            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint)))))
        .onFalse(Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()));
    controller
        .pov(270)
        .whileTrue(
            new FollowPoint(
                    drive,
                    () -> AllianceFlipUtil.apply(AllianceFlipUtil.mirror(FieldConstants.Hub.blink)))
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
                            this,
                            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint)))))
        .onFalse(Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()));
  }

  /**
   * 用此方法将自动阶段的指令传递给 {@link Robot} 主类。
   *
   * @return 自动阶段需要运行的指令
   */
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
